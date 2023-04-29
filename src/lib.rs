use {
    constants::{LEAF_LEVEL, PAGE_LEN},
    hash_table::basic::HashTable,
    shared_hash_dag::SharedHashDAG,
    tracking::Tracker,
    utils::{bucket_from_hash, new_bucket_len, new_bucket_len_idx, new_vptr},
    validation::{LevelInfo, Node},
};

pub mod basic_dag;
pub mod constants;
pub mod conversion;
pub mod editing;
pub mod hash_table;
pub mod prelude;
pub mod reporting;
pub mod shared_hash_dag;
pub mod shmem_config;
pub mod staging;
pub mod tracking;
pub mod utils;
pub mod validation;

#[cfg(test)]
mod tests;

pub type Result<T> = std::result::Result<T, String>;

pub trait HashDAG {
    /// Gets an item, usually this would be used for retrieving masks, but it retrieves anything.
    fn get(&self, vptr: u32) -> Result<u32>;
    /// Gets the entire leaf.
    fn leaf(&self, vptr: u32) -> Result<&[u32]>;
    /// Gets the mask and all children.
    fn interior(&self, vptr: u32) -> Result<&[u32]>;
    /// Gets a data dump of the pool and page table.
    fn dump(&self) -> (&[u32], &[u32]);
}

pub trait HashDAGMut {
    /// Provides a way to add a node with the guarantee that you don't add duplicates.
    fn find_or_add_leaf(&mut self, node: Node) -> Result<u32>;
    /// Provides a way to add a node with the guarantee that you don't add duplicates.
    fn find_or_add_interior(&mut self, level: u32, node: Node) -> Result<u32>;
    /// Adds a node without checking for duplicates.
    fn add_leaf(&mut self, node: Node, hash: u32) -> Result<u32>;
    /// Adds a node without checking for duplicates.
    fn add_interior(&mut self, level: u32, node: Node, hash: u32) -> Result<u32>;
}

// TODO Figure out a way to zero-cost refactor find_or_add_xx (maybe change node to cary more compile-time context)
// TODO do this however bottom-up, starting at utils
impl HashDAG for HashTable<'_> {
    #[inline]
    fn get(&self, vptr: u32) -> Result<u32> {
        Ok(self.pool[self.pool_idx(vptr)?])
    }
    #[inline]
    fn leaf(&self, vptr: u32) -> Result<&[u32]> {
        let pool_idx = self.pool_idx(vptr)?;
        Ok(&self.pool[pool_idx..=pool_idx + 1])
    }
    #[inline]
    fn interior(&self, vptr: u32) -> Result<&[u32]> {
        let pool_idx = self.pool_idx(vptr)?;
        let children = (self.pool[pool_idx] as u8).count_ones() as usize;
        Ok(&self.pool[pool_idx..=pool_idx + children])
    }
    #[inline]
    fn dump(&self) -> (&[u32], &[u32]) {
        (&self.pool, &self.lut)
    }
}

impl<T: Tracker> HashDAGMut for SharedHashDAG<HashTable<'_>, T> {
    fn find_or_add_leaf(&mut self, node: Node) -> Result<u32> {
        let node = node.validated_as_leaf()?;
        let hash = node.hash_as_leaf();
        let bucket = bucket_from_hash(LEAF_LEVEL, hash);
        let find = |bucket_len| self.find_leaf(bucket, bucket_len, *node);
        let full_node_ptr = self.full_node_ptr(LEAF_LEVEL).unwrap();
        Ok(if *node == self.leaf(full_node_ptr).unwrap() {
            full_node_ptr
        } else if !self.is_allocated((new_vptr(LEAF_LEVEL, bucket, 0)? / PAGE_LEN) as _)? {
            // TODO Lock on bucket
            self.add_leaf(node, hash)?
        } else if let Some(vptr) = find(self.bucket_len(LEAF_LEVEL, bucket))? {
            vptr
        } else {
            // TODO Lock on bucket
            // Take a second peek after locking
            if let Some(vptr) = find(self.bucket_len(LEAF_LEVEL, bucket))? {
                vptr
            } else {
                self.add_leaf(node, hash)?
            }
        })
    }
    fn find_or_add_interior(&mut self, level: u32, node: Node) -> Result<u32> {
        let node = node.validated_as_interior(&self.hash_dag, LevelInfo::new(level))?;
        let hash = node.hash_as_interior();
        let bucket = bucket_from_hash(level, hash);
        let find = |bucket_len| self.find_interior(level, bucket, bucket_len, *node);
        let full_node_ptr = self.full_node_ptr(level)?;
        Ok(if *node == self.interior(full_node_ptr).unwrap() {
            full_node_ptr
        } else if !self.is_allocated((new_vptr(level, bucket, 0)? / PAGE_LEN) as _)? {
            // TODO Lock on bucket
            self.add_interior(level, node, hash)?
        } else if let Some(vptr) = find(self.bucket_len(level, bucket))? {
            vptr
        } else {
            // TODO Lock on bucket
            // Take a second peek after locking
            if let Some(vptr) = find(self.bucket_len(level, bucket))? {
                vptr
            } else {
                self.add_interior(level, node, hash)?
            }
        })
    }
    #[inline]
    fn add_leaf(&mut self, node: Node, hash: u32) -> Result<u32> {
        let node = node.validated_as_leaf()?;
        let bucket = bucket_from_hash(LEAF_LEVEL, hash);
        let bucket_len_idx = new_bucket_len_idx(LEAF_LEVEL, bucket);
        let bucket_len = self.bucket_len[bucket_len_idx];
        let vptr = new_vptr(LEAF_LEVEL, bucket, bucket_len)?;
        if bucket_len % PAGE_LEN == 0 {
            self.allocate((vptr / PAGE_LEN) as _)?;
        }
        let pool_idx = self.pool_idx(vptr)?;
        let range = pool_idx..pool_idx + 2;
        self.pool_copy_from(range.start, *node);
        self.bucket_len_add(bucket_len_idx, 2);
        if self.bucket_len[bucket_len_idx] < new_bucket_len(LEAF_LEVEL) {
            self.tracker.register(vptr, range)?;
            Ok(vptr)
        } else {
            Err("Overflowing bucket on leaf level!".into())
        }
    }
    #[inline]
    fn add_interior(&mut self, level: u32, node: Node, hash: u32) -> Result<u32> {
        let node = node.validated_as_interior(&self.hash_dag, LevelInfo::new(level))?;
        let node_len = node.len() as u32;
        let bucket = bucket_from_hash(level, hash);
        let bucket_len_idx = new_bucket_len_idx(level, bucket);
        let mut bucket_len = self.bucket_len[bucket_len_idx];
        let vptr = {
            let page_space_left = PAGE_LEN - bucket_len % PAGE_LEN;
            let would_overflow = page_space_left < node_len;
            if PAGE_LEN == page_space_left || would_overflow {
                if would_overflow {
                    bucket_len += page_space_left;
                }
                let vptr = new_vptr(level, bucket, bucket_len)?;
                self.allocate((vptr / PAGE_LEN) as _)?;
                vptr
            } else {
                let vptr = new_vptr(level, bucket, bucket_len)?;
                debug_assert!(self.is_allocated((vptr / PAGE_LEN) as _)?);
                vptr
            }
        };
        let pool_idx = self.pool_idx(vptr)?;
        debug_assert!(pool_idx + node.len() < self.pool.len());
        let range = pool_idx..pool_idx + node.len();
        self.pool_copy_from(range.start, *node);
        self.bucket_len_copy_from(bucket_len_idx, &[bucket_len + node_len]);
        if self.bucket_len[bucket_len_idx] < new_bucket_len(level) {
            self.tracker.register(vptr, range)?;
            Ok(vptr)
        } else {
            Err(format!("Overflowing bucket on level {level}!"))
        }
    }
}
