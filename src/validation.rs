use self::{
    utils::{validate_interior, validate_leaf},
    Node::{Pass, Strict},
    Validation::{Invalid, Valid},
};
use super::{
    constants::{COLOR_TREE_LEVELS, LEAF_LEVEL},
    hash_table::basic::HashTable,
    utils::{hash_interior, hash_leaf, vptr_to_lvl},
    HashDAG, Result,
};
use ::std::ops::Deref;

#[derive(Debug, Copy, Clone)]
pub struct LevelInfo {
    pub is_color_tree_level: bool,
    pub is_last_interior: bool,
}

impl LevelInfo {
    #[inline]
    #[must_use]
    pub const fn new(level: u32) -> Self {
        Self {
            is_color_tree_level: COLOR_TREE_LEVELS <= level,
            is_last_interior: level + 1 == LEAF_LEVEL,
        }
    }
}

/// A way to provide more context to the data. It determines if something should be read as possibly invalid or not,
/// but no guarantees are given about how it is actually processed.
#[derive(Copy, Clone, Debug)]
pub enum Node<'pool> {
    /// The data is not yet proven to be valid and should be treated as such.
    Strict(&'pool [u32]),
    /// The data has been validated or should be treated as such.
    Pass(&'pool [u32]),
}

impl<'pool> Deref for Node<'pool> {
    type Target = &'pool [u32];
    #[inline]
    fn deref(&self) -> &Self::Target {
        match self {
            Pass(data) | Strict(data) => data,
        }
    }
}

impl<'pool> Node<'pool> {
    #[inline]
    #[must_use]
    pub fn validated(self) -> Node<'pool> {
        Pass(*self)
    }
    #[inline]
    pub fn validated_as_leaf(self) -> Result<Node<'pool>> {
        match validate_leaf(self)? {
            Valid => Ok(self.validated()),
            Invalid(msg) => Err(msg),
        }
    }
    #[inline]
    pub fn validated_as_interior<DAG: HashDAG>(
        self,
        dag: &DAG,
        level_info: LevelInfo,
    ) -> Result<Node<'pool>> {
        match validate_interior(dag, self, level_info)? {
            Valid => Ok(self.validated()),
            Invalid(msg) => Err(msg),
        }
    }
    #[inline]
    #[must_use]
    pub fn hash_as_leaf(self) -> u32 {
        hash_leaf(*self)
    }
    #[inline]
    #[must_use]
    pub fn hash_as_interior(self) -> u32 {
        hash_interior(*self)
    }
}

#[derive(Debug, PartialOrd, PartialEq, Eq)]
pub enum Validation {
    Valid,
    Invalid(String),
}

pub trait Validator {
    fn validate(&self, vptr: u32) -> Result<Validation>;
}

impl Validator for HashTable<'_> {
    fn validate(&self, vptr: u32) -> Result<Validation> {
        let mut visited = vec![false; self.pool.len()].into_boxed_slice();
        let mut items = Vec::with_capacity(self.pool.len() / 16);
        items.push(vptr);
        for level_info in (vptr_to_lvl(vptr)..LEAF_LEVEL).map(LevelInfo::new) {
            let mut new_items = Vec::with_capacity(items.capacity());
            while let Some(vptr) = items.pop() {
                let interior = self.interior(vptr)?;
                match Strict(interior).validated_as_interior(self, level_info) {
                    Ok(node) => {
                        for &vptr in node.iter().skip(1) {
                            let pool_idx = self.pool_idx(vptr)?;
                            if !visited[pool_idx] {
                                visited[pool_idx] = true;
                                new_items.push(vptr);
                            }
                        }
                    }
                    Err(msg) => return Ok(Invalid(msg)),
                }
            }
            items = new_items;
        }
        while let Some(vptr) = items.pop() {
            if let Err(msg) = Strict(self.leaf(vptr)?).validated_as_leaf() {
                return Ok(Invalid(msg));
            }
        }
        Ok(Valid)
    }
}

pub mod utils {
    use super::{
        super::utils::{buckets_per_level, count_leaves, new_bucket_len, new_vptr},
        HashDAG, Invalid, LevelInfo,
        Node::{self, Pass, Strict},
        Result, Valid, Validation,
    };
    use std::cmp::Ordering;

    #[inline]
    pub fn validate_leaf(node: Node) -> Result<Validation> {
        match node {
            Strict(&[0, 0]) => Ok(Invalid(
                "Invalid node: The leaf mask contains no leaves!".into(),
            )),
            _ => Ok(Valid),
        }
    }
    pub fn validate_interior<DAG: HashDAG>(
        dag: &DAG,
        node: Node,
        LevelInfo {
            is_color_tree_level,
            is_last_interior,
        }: LevelInfo,
    ) -> Result<Validation> {
        match node {
            Pass(_) => Ok(Valid),
            Strict(interior) => {
                if interior.len() == 1 || 9 < interior.len() {
                    Ok(Invalid(
                        "Invalid node: The interior node's child mask is invalid.".into(),
                    ))
                } else {
                    let mut voxel_count = 0;
                    if is_color_tree_level {
                        if is_last_interior {
                            for &vp in interior.iter().skip(1) {
                                voxel_count += count_leaves(dag.leaf(vp)?);
                            }
                        } else {
                            for &vp in interior.iter().skip(1) {
                                voxel_count += dag.get(vp)? >> 8;
                            }
                        }
                    }
                    match (interior[0] >> 8).cmp(&voxel_count) {
                        Ordering::Less => {
                            Ok(Invalid("Invalid node: The voxel count is too low!".into()))
                        }
                        Ordering::Equal => Ok(Valid),
                        Ordering::Greater => {
                            Ok(Invalid("Invalid node: The voxel count is too high!".into()))
                        }
                    }
                }
            }
        }
    }
    /// Validates if the virtual pointer is a sane pointer and follows specified constraints.
    #[must_use]
    pub fn is_valid_vptr(
        vptr: u32,
        level: u32,
        bucket: Option<u32>,
        offset_bucket: Option<u32>,
    ) -> bool {
        bucket.map_or_else(
            || {
                let min = new_vptr(level, 0, 0).unwrap_or(!0);
                let max_offset_bucket = new_bucket_len(level) - 1;
                let max_bucket = buckets_per_level(level) - 1;
                let max = new_vptr(level, max_bucket, max_offset_bucket).unwrap_or(0);
                min <= vptr && vptr <= max
            },
            |bucket| {
                offset_bucket.map_or_else(
                    || {
                        let min = new_vptr(level, bucket, 0).unwrap_or(!0);
                        let max_offset_bucket = new_bucket_len(level) - 1;
                        let max = new_vptr(level, bucket, max_offset_bucket).unwrap_or(0);
                        min <= vptr && vptr <= max
                    },
                    |offset_bucket| new_vptr(level, bucket, offset_bucket).unwrap_or(!0) == vptr,
                )
            },
        )
    }
}
