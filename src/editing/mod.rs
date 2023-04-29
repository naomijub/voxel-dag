use self::inner::{interior_from, NodeState};
use super::{
    basic_dag::OctVox,
    constants::{COLOR_TREE_LEVELS, LEAF_LEVEL, SUPPORTED_LEVELS},
    hash_table::basic::HashTable,
    shared_hash_dag::SharedHashDAG,
    tracking::Tracker,
    utils::{count_leaves, descend, vptr_to_lvl},
    validation::Node::Pass,
    HashDAG, HashDAGMut, Result,
};
use ::{nalgebra::Vector3, num_traits::identities::Zero};

pub mod inner;
pub mod shapes;

#[derive(Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
pub enum Operation {
    Link,
    Unlink,
}

pub trait Shape {
    type Edit;

    fn new(centroid: &Vector3<u32>, extent: u32) -> Self;
    fn collides(&self, edit: &Self::Edit) -> bool;
    fn will_be_full(&self, after: Operation, edit: &Self::Edit) -> bool;
    fn will_be_empty(&self, after: Operation, edit: &Self::Edit) -> bool;
}

pub trait Editor {
    fn edit<S>(&mut self, vptr: u32, operation: Operation, shape: &S) -> Result<u32>
    where
        S: Shape,
        S::Edit: From<OctVox>;
}

impl<T: Tracker> Editor for SharedHashDAG<HashTable<'_>, T> {
    #[inline]
    fn edit<S>(&mut self, vptr: u32, operation: Operation, shape: &S) -> Result<u32>
    where
        S: Shape,
        S::Edit: From<OctVox>,
    {
        let root_node = NodeState {
            level: vptr_to_lvl(vptr),
            vptr: Some(vptr),
            path: Vector3::zero(),
        };
        let vptr = if COLOR_TREE_LEVELS <= root_node.level {
            self.edit_deep((operation, shape), root_node)?.0
        } else {
            self.edit((operation, shape), root_node)?
        };
        Ok(vptr.ok_or("An empty DAG is invalid state.")?)
    }
}

impl<T: Tracker> SharedHashDAG<HashTable<'_>, T> {
    fn edit<S>(
        &mut self,
        (operation, shape): (Operation, &S),
        node: NodeState,
    ) -> Result<Option<u32>>
    where
        S: Shape,
        S::Edit: From<OctVox>,
    {
        let result = self.edit_interior(node, move |dag, node| {
            let vptr = *node;
            let edit_ptr = if !shape.collides(&node.edit_shape::<S>()) {
                *node
            } else if node.level == COLOR_TREE_LEVELS {
                dag.edit_deep((operation, shape), node)?.0
            } else {
                dag.edit((operation, shape), node)?
            };
            Ok((vptr != edit_ptr, edit_ptr, 0))
        });
        Ok(result?.0)
    }

    fn edit_deep<S>(
        &mut self,
        (operation, shape): (Operation, &S),
        node: NodeState,
    ) -> Result<(Option<u32>, u32)>
    where
        S: Shape,
        S::Edit: From<OctVox>,
    {
        let edit = node.edit_shape::<S>();
        if !shape.collides(&edit) {
            match *node {
                Some(vptr) => {
                    let voxel_count = if node.level == LEAF_LEVEL {
                        count_leaves(self.leaf(vptr)?)
                    } else {
                        self.get(vptr)? >> 8
                    };
                    Ok((Some(vptr), voxel_count))
                }
                _ => Ok((None, 0)),
            }
        } else if shape.will_be_empty(operation, &edit) {
            Ok((None, 0))
        } else if shape.will_be_full(operation, &edit) {
            Ok((
                Some(self.full_node_ptr(node.level)?),
                1 << (3 * (SUPPORTED_LEVELS - node.level)),
            ))
        } else if node.level == LEAF_LEVEL {
            self.edit_leaf((operation, shape), *node, &node.path)
        } else {
            self.edit_interior(node, move |dag, node| {
                let vptr = *node;
                let (edit_ptr, c) = dag.edit_deep((operation, shape), node)?;
                Ok((edit_ptr != vptr, edit_ptr, c))
            })
        }
    }

    pub fn edit_interior<F>(
        &mut self,
        NodeState { level, vptr, path }: NodeState,
        next: F,
    ) -> Result<(Option<u32>, u32)>
    where
        F: Fn(&mut Self, NodeState) -> Result<(bool, Option<u32>, u32)>,
    {
        // Edit exhaustively and push which children are affected and how. Any changes happening flips the `invalidated` bit.
        let (mut is_invalidated, mut children, mut voxel_count) = (false, [None; 8], 0);
        {
            let interior = match vptr {
                Some(vptr) => self.interior(vptr)?,
                None => &[0],
            }
            .to_vec();
            let mut interior = interior.into_iter();
            let child_mask = interior.next().unwrap() as u8;
            for child in 0..8 {
                let child_exists = child_mask & (1 << child) != 0;
                let node = NodeState {
                    level: level + 1,
                    vptr: if child_exists { interior.next() } else { None },
                    path: descend(&path, child),
                };
                let (invalidate, vptr, count) = next(self, node)?;
                is_invalidated |= invalidate;
                children[child as usize] = vptr;
                voxel_count += count;
            }
        }
        Ok(if !is_invalidated {
            (vptr, voxel_count)
        } else if let Some(interior) = interior_from(children, voxel_count) {
            (
                Some(self.find_or_add_interior(level, Pass(&interior))?),
                voxel_count,
            )
        } else {
            (None, 0)
        })
    }

    pub fn edit_leaf<S>(
        &mut self,
        (operation, shape): (Operation, &S),
        vptr: Option<u32>,
        path: &Vector3<u32>,
    ) -> Result<(Option<u32>, u32)>
    where
        S: Shape,
        S::Edit: From<OctVox>,
    {
        let mut leaf = match vptr {
            Some(vptr) => {
                let pool_idx = self.pool_idx(vptr)?;
                [self.pool[pool_idx], self.pool[pool_idx + 1]]
            }
            None => [0; 2],
        };
        let init_leaf = leaf;
        let mut edit_part = |leaf_idx, upper_idx| {
            let path: Vector3<_> = descend(path, upper_idx);
            let base_idx = upper_idx % 4 * 8;
            for bottom_idx in 0..8 {
                let path: Vector3<_> = descend(&path, bottom_idx);
                let child_bit = 1 << (base_idx + bottom_idx);
                if shape.collides(&S::Edit::from(OctVox::new(SUPPORTED_LEVELS, &path))) {
                    match operation {
                        Operation::Link => leaf[leaf_idx as usize] |= child_bit,
                        _ => leaf[leaf_idx as usize] &= !child_bit,
                    }
                }
            }
        };
        // The convention I'm now stuck with (good or bad) is to use an array for the leaf mask, so I need to split the loop:
        (0..4).for_each(|upper_idx| edit_part(0, upper_idx)); // One for the left 32 bits
        (4..8).for_each(|upper_idx| edit_part(1, upper_idx)); // One for the right 32 bits
        Ok(if leaf == [0; 2] {
            (None, 0)
        } else if leaf == init_leaf {
            (Some(vptr.unwrap()), count_leaves(&leaf))
        } else {
            (
                Some(self.find_or_add_leaf(Pass(&leaf))?),
                count_leaves(&leaf),
            )
        })
    }
}
