use super::super::{
    constants::{LEAF_LEVEL, PAGE_LEN, TOTAL_BUCKETS, TOTAL_PAGES, TOTAL_VIRT_SPACE},
    utils::{new_bucket_len, new_bucket_len_idx, new_vptr, shmem::ShmemArray},
    Result,
};
use ::{
    shared_memory::ShmemError,
    std::{cmp::Ordering, ops::Deref, pin::Pin},
};

const LUT_LEN: usize = TOTAL_PAGES as usize;

pub struct PageLUT<'shmem>(Pin<Box<ShmemArray<'shmem, u32>>>);

impl Deref for PageLUT<'_> {
    type Target = [u32];
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0[..LUT_LEN] // exclude hi
    }
}

impl PageLUT<'_> {
    #[inline]
    pub fn new(root: Option<&String>) -> std::result::Result<Self, ShmemError> {
        Ok(Self({
            let root = root.map(|root| format!("{root}page_table.flink"));
            let mut mem = ShmemArray::new(LUT_LEN + 1, root)?;
            mem.copy_from(0, &[!0; LUT_LEN]);
            mem.copy_from(LUT_LEN, &[0]); // set hi to 0
            mem
        }))
    }
    #[inline]
    pub fn allocate(&mut self, page: usize) {
        // TODO lock on a page in current scope
        debug_assert!(
            !self.is_allocated(page).expect("Page does not exist."),
            "Trying to allocate an allocated page."
        );
        let hi = self.hi();
        self.0.copy_from(page, &[hi * PAGE_LEN]);
        self.0.copy_from(LUT_LEN, &[hi + 1]); // write hi
    }
    #[inline]
    #[must_use]
    pub fn hi(&self) -> u32 {
        self.0[LUT_LEN]
    }
    #[inline]
    pub fn is_allocated(&self, page: usize) -> Result<bool> {
        match self.get(page) {
            Some(&vptr) => Ok(vptr != !0),
            None => Err("Trying to lookup a non-existing page.".into()),
        }
    }
}

pub struct HashTable<'shmem> {
    /// The virtual pointers of each level's full node.
    pub full_node_pointers: [u32; LEAF_LEVEL as usize + 1],
    /// The page table spanning the full virtual space.
    pub lut: PageLUT<'shmem>,
    /// The free store which tells you how full a given bucket is.
    pub bucket_len: Pin<Box<ShmemArray<'shmem, u32>>>,
    /// The pool containing **all** nodes.
    pub pool: Pin<Box<ShmemArray<'shmem, u32>>>,
}

impl HashTable<'_> {
    /// Initializes the pool to a multiple of 128 pages to prevent UB.
    pub fn blank(root: Option<&String>, mut capacity: usize) -> Result<Self> {
        #[inline]
        fn map(error: ShmemError) -> String {
            match error {
                ShmemError::LinkExists => {
                    "A HashDAG with the same file link already exists.".into()
                }
                _ => error.to_string(),
            }
        }

        const BLOCK_LEN: usize = PAGE_LEN as usize * 128;
        capacity += (BLOCK_LEN - capacity % BLOCK_LEN) % BLOCK_LEN;
        if (TOTAL_VIRT_SPACE as usize) < capacity || capacity == 0 {
            Err(format!("Cannot allocate {capacity} words to a pool!"))
        } else {
            Ok(Self {
                full_node_pointers: [!0; LEAF_LEVEL as usize + 1],
                lut: PageLUT::new(root).map_err(map)?,
                bucket_len: {
                    let root = root.map(|root| format!("{root}free_store.flink"));
                    let mut mem = ShmemArray::new(TOTAL_BUCKETS as _, root).map_err(map)?;
                    mem.copy_from(0, &[0; TOTAL_BUCKETS as _]);
                    mem
                },
                pool: {
                    let root = root.map(|root| format!("{root}data_pool.flink"));
                    ShmemArray::new(capacity, root).map_err(map)?
                },
            })
        }
    }
}

impl HashTable<'_> {
    #[inline]
    pub fn is_allocated(&self, page: usize) -> Result<bool> {
        self.lut.is_allocated(page)
    }
    #[inline]
    #[must_use]
    pub fn bucket_len(&self, level: u32, bucket: u32) -> u32 {
        self.bucket_len[new_bucket_len_idx(level, bucket)]
    }
    #[inline]
    pub fn pool_idx(&self, vptr: u32) -> Result<usize> {
        let (page, offset) = ((vptr / PAGE_LEN) as _, vptr % PAGE_LEN);
        if self.lut.is_allocated(page)? {
            let idx = self.lut[page] + offset;
            if idx < self.lut.hi() * PAGE_LEN {
                Ok(idx as _)
            } else {
                Err("Virtual pointer points to out of bound memory.".into())
            }
        } else {
            Err("Virtual pointer points to unallocated memory.".into())
        }
    }
}

impl HashTable<'_> {
    #[inline]
    pub fn full_node_ptr(&self, level: u32) -> Result<u32> {
        match self.full_node_pointers.get(level as usize) {
            Some(&vptr) => Ok(vptr),
            None => Err("Trying a full node lookup with a non-existing level.".into()),
        }
    }
    /// Does a sequential search for the specified node.
    pub fn find_leaf(&self, bucket: u32, bucket_len: u32, leaf: &[u32]) -> Result<Option<u32>> {
        if new_bucket_len(LEAF_LEVEL) < bucket_len {
            Err("Trying to find a leaf with an overflowing bucket size.".into())
        } else {
            let vptr = new_vptr(LEAF_LEVEL, bucket, 0)?;
            let pool_idx = self.pool_idx(vptr)?;
            Ok((0..bucket_len).step_by(2).find_map(|offset| {
                let pool_idx = pool_idx + offset as usize;
                match leaf.cmp(&self.pool[pool_idx..=pool_idx + 1]) {
                    Ordering::Equal => Some(vptr + offset),
                    _ => None,
                }
            }))
        }
    }
    /// Does a sequential search for the specified node.
    pub fn find_interior(
        &self,
        level: u32,
        bucket: u32,
        bucket_len: u32,
        interior: &[u32],
    ) -> Result<Option<u32>> {
        let node_len = interior.len() as u32;
        if new_bucket_len(level) < bucket_len {
            Err("Trying to find an interior node with an overflowing bucket size.".into())
        } else {
            let base_ptr = new_vptr(level, bucket, 0)?;
            let base_idx = self.pool_idx(base_ptr)?;
            let find_in_page = |lower_bound, upper_bound| {
                let mut offset = lower_bound;
                while offset < upper_bound {
                    let idx = base_idx + offset as usize;
                    match interior.cmp(&self.pool[idx..idx + interior.len()]) {
                        Ordering::Equal => return Some(base_ptr + offset),
                        _ => offset += (self.pool[idx] as u8).count_ones() + 1,
                    }
                }
                None
            };

            let overflow = bucket_len % PAGE_LEN;
            let clipped = bucket_len - overflow;
            let mut walker = (0..clipped).step_by(PAGE_LEN as _);
            let found = walker.find_map(|offset| find_in_page(offset, offset + PAGE_LEN));
            Ok(if found.is_none() && node_len <= overflow {
                find_in_page(clipped, bucket_len)
            } else {
                found
            })
        }
    }
}
