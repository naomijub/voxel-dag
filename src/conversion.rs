use super::{
    basic_dag::BasicDAG,
    constants::{COLOR_TREE_LEVELS, LEAF_LEVEL, SUPPORTED_LEVELS},
    hash_table::basic::HashTable,
    shared_hash_dag::SharedHashDAG,
    tracking::Tracker,
    utils::count_leaves,
    validation::Node::{self, Pass, Strict},
    HashDAG, HashDAGMut, Result,
};

// TODO [1] it turns out that the stop is not optimized away when none. Execution time has increased by 13%. Optimize this?
pub trait Converter {
    /// Imports a DAG into memory without creating duplications.
    /// During this process a lot of cache is allocated, if this proves to be problematic you may want to consider importing in batches.
    /// Arguments: stop: at which level to stop importing (relative to the root). Anything >= `LEAF_LEVEL` will error.
    fn import_strict(&mut self, dag: &BasicDAG, stop: Option<u32>) -> Result<u32>;
    /// Imports a DAG into memory without creating duplications. This without validation.
    /// During this process a lot of cache is allocated, if this proves to be problematic you may want to consider importing in batches.
    /// Arguments: stop: at which level to stop importing (relative to the root). Anything >= `LEAF_LEVEL` will error.
    fn import(&mut self, dag: &BasicDAG, stop: Option<u32>) -> Result<u32>;
    // TODO export(vptr) -> dyn (dag: &BasicDAG)
    // TODO export_serialized(vptr) -> "dyn (dag: &BasicDAG)::serialized()"
}

impl<T: Tracker> Converter for SharedHashDAG<HashTable<'_>, T> {
    #[inline]
    fn import_strict(&mut self, dag: &BasicDAG, stop: Option<u32>) -> Result<u32> {
        self.import(Strict(&[]), dag, stop)
    }
    #[inline]
    fn import(&mut self, dag: &BasicDAG, stop: Option<u32>) -> Result<u32> {
        self.import(Pass(&[]), dag, stop)
    }
}

impl<T: Tracker> SharedHashDAG<HashTable<'_>, T> {
    fn import(&mut self, how: Node, bd: &BasicDAG, mut stop: Option<u32>) -> Result<u32> {
        let root_level = SUPPORTED_LEVELS - bd.levels;
        if let Some(mut level) = stop {
            level += root_level;
            if LEAF_LEVEL <= level {
                return Err("The import stop is greater or equal to the leaf level.".into());
            } else {
                stop = Some(level);
            }
        }
        let mut visits = vec![!0; bd.pool.len()].into_boxed_slice();
        let node = match how {
            Strict(_) => Strict(&bd.pool),
            Pass(_) => Pass(&bd.pool),
        };
        // TODO [1] could create the choice between put with & without early stop (as it's not zero-cost atm)
        let (vptr, _) = self.put((node, &mut visits, stop), (root_level, bd.root_idx))?;
        Ok(vptr)
    }

    fn put(
        &mut self,
        (pool, visits, stop): (Node, &mut [u32], Option<u32>),
        (level, idx): (u32, usize),
    ) -> Result<(u32, Option<u32>)> {
        let is_visited = *visits.get(idx).ok_or(format!(
            "Node index out of bounds! Level: {level}, Index: {idx}"
        ))? != !0;
        // TODO [1] could move this out and have a function that invokes this one so there exists both one with and without early stopping
        if Some(level) == stop {
            let voxel_count = if COLOR_TREE_LEVELS <= level {
                Some(1 << (3 * (SUPPORTED_LEVELS - level)))
            } else {
                None
            };
            Ok((self.full_node_ptr(level)?, voxel_count))
        } else if is_visited {
            if level == LEAF_LEVEL {
                Ok((visits[idx], Some(count_leaves(self.leaf(visits[idx])?))))
            } else {
                Ok((visits[idx], Some(self.get(visits[idx])? >> 8)))
            }
        } else if level == LEAF_LEVEL {
            let &right = pool.get(idx + 1).ok_or(format!(
                "Least significant leaf mask index out of bounds! Leaf level, Pointer: {}",
                idx + 1
            ))?;
            let leaf = &[pool[idx], right];
            visits[idx] = self.find_or_add_leaf(match pool {
                Strict(_) => Strict(leaf),
                Pass(_) => Pass(leaf),
            })?;
            Ok((visits[idx], Some(count_leaves(leaf))))
        } else {
            let child_mask = pool[idx] & 0xff;
            let children = child_mask.count_ones() as usize;
            let mut interior = Vec::with_capacity(children + 1);
            interior.push(child_mask);
            let voxel_count = if COLOR_TREE_LEVELS <= level {
                let mut voxel_count = 0;
                for &idx in pool.iter().skip(idx + 1).take(children) {
                    // TODO [1] could make this generic so both a function that checks a stop and one who doesn't can be used
                    let (vptr, count) =
                        self.put((pool, visits, stop), (level + 1, idx as usize))?;
                    interior.push(vptr);
                    voxel_count += count.unwrap();
                }
                interior[0] |= voxel_count << 8;
                Some(voxel_count)
            } else {
                for &idx in pool.iter().skip(idx + 1).take(children) {
                    // TODO [1] could make this generic so both a function that checks a stop and one who doesn't can be used
                    let (vptr, _) = self.put((pool, visits, stop), (level + 1, idx as usize))?;
                    interior.push(vptr);
                }
                None
            };
            let node = match pool {
                Strict(_) => Strict(&interior),
                Pass(_) => Pass(&interior),
            };
            visits[idx] = self.find_or_add_interior(level, node)?;
            Ok((visits[idx], voxel_count))
        }
    }
}
