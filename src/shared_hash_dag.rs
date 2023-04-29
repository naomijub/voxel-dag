use super::{
    constants::{COLOR_TREE_LEVELS, LEAF_LEVEL, PAGE_LEN, SUPPORTED_LEVELS},
    hash_table::basic::HashTable,
    tracking::Tracker,
    utils::hash_interior,
    validation::Node::{self, Pass},
    HashDAG, HashDAGMut, Result,
};
use ::std::ops::Deref;

/// `hash_dag` _must not_ implement any mutating trait. Incidentally invoking it would bypass the tracker.
pub struct SharedHashDAG<DAG: HashDAG, T: Tracker> {
    pub hash_dag: DAG,
    pub tracker: T,
}

impl<DAG: HashDAG, T: Tracker> Deref for SharedHashDAG<DAG, T> {
    type Target = DAG;
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.hash_dag
    }
}

/// Initializers
impl<T: Tracker + Default> SharedHashDAG<HashTable<'_>, T> {
    #[inline]
    pub fn blank(root: Option<&String>, capacity: usize, tracker: Option<T>) -> Result<Self> {
        Ok(Self {
            hash_dag: HashTable::blank(root, capacity)?,
            tracker: tracker.unwrap_or_default(),
        })
    }
    #[inline]
    pub fn with_capacity(root: Option<&String>, capacity: usize) -> Result<Self> {
        let mut dag = Self::blank(root, capacity, None)?;
        dag.add_full_leaf();
        for level in (0..LEAF_LEVEL).rev() {
            dag.add_full_interior(level);
        }
        Ok(dag)
    }
}

/// `HashDAG` mutation
impl<T: Tracker> SharedHashDAG<HashTable<'_>, T> {
    #[inline]
    pub fn pool_copy_from(&mut self, offset: usize, slice: &[u32]) {
        self.hash_dag.pool.copy_from(offset, slice);
    }
    #[inline]
    pub fn bucket_len_copy_from(&mut self, offset: usize, slice: &[u32]) {
        self.hash_dag.bucket_len.copy_from(offset, slice);
    }
    #[inline]
    pub fn bucket_len_add(&mut self, offset: usize, increase: u32) {
        self.bucket_len_copy_from(offset, &[self.bucket_len[offset] + increase]);
    }
    #[inline]
    pub fn allocate(&mut self, page: usize) -> Result<()> {
        self.hash_dag.lut.allocate(page);
        if self.hash_dag.pool.len() < (self.hash_dag.lut.hi() * PAGE_LEN) as usize {
            Err("No space is left to allocate! Consider resizing your pool.".into())
        } else {
            Ok(())
        }
    }
}

/// Full nodes
impl<T: Tracker> SharedHashDAG<HashTable<'_>, T> {
    pub fn add_full_leaf(&mut self) {
        const LEVEL: usize = LEAF_LEVEL as _;
        const NODE: Node = Pass(&[!0u32, !0u32]);
        let vptr = self.add_leaf(NODE, NODE.hash_as_leaf()).unwrap();
        self.hash_dag.full_node_pointers[LEVEL] = vptr;
    }
    pub fn add_full_interior(&mut self, level: u32) {
        debug_assert!(
            level < LEAF_LEVEL,
            "Trying to add a full node below leaf level."
        );
        let mut interior: [u32; 9] = [0; 9];
        interior[0] = 0xff;
        if COLOR_TREE_LEVELS <= level {
            let voxel_count = 1 << (3 * (SUPPORTED_LEVELS - level));
            debug_assert!(
                voxel_count < (1 << 24),
                "Voxel count overflow! Consider increasing COLOR_TREE_LEVELS."
            );
            interior[0] |= voxel_count << 8;
        }
        interior[1..9].copy_from_slice(&[self.full_node_pointers[level as usize + 1]; 8]);
        let hash = hash_interior(&interior);
        self.hash_dag.full_node_pointers[level as usize] =
            self.add_interior(level, Pass(&interior), hash).unwrap();
    }
}
