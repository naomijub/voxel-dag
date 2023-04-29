use super::{
    constants::{PAGE_LEN, TOTAL_PAGES},
    hash_table::basic::HashTable,
    Result, SharedHashDAG,
};
use ::std::ops::Range;

pub trait Tracker {
    fn register(&mut self, vptr: u32, range: Range<usize>) -> Result<()>;
    fn clear(&mut self);
}

pub mod basic {
    use super::{HashTable, Range, Result, SharedHashDAG, Tracker, PAGE_LEN, TOTAL_PAGES};

    pub type BasicHashDAG<'shmem> = SharedHashDAG<HashTable<'shmem>, BasicTracker>;

    pub type PoolMask = u8;
    pub type PageTableMask = u8;
    pub const POOL_MASK_BITS: usize = PoolMask::BITS as usize;
    pub const LUT_MASK_BITS: usize = PageTableMask::BITS as usize;
    pub const POOL_MASK_BIT_LEN: usize = PAGE_LEN as usize;
    pub const LUT_MASK_BIT_LEN: usize = TOTAL_PAGES as usize / LUT_MASK_BITS;

    pub struct BasicTracker {
        /// The pool mask is a collection of words with each bit representing a complete page.
        pub pool_mask: Box<[PoolMask]>,
        /// The page table mask is a single word with each bit representing a partition of the page table.
        pub page_table_mask: PageTableMask,
    }

    impl Default for BasicTracker {
        #[inline]
        fn default() -> Self {
            Self {
                pool_mask: vec![0; TOTAL_PAGES as usize / POOL_MASK_BITS].into_boxed_slice(),
                page_table_mask: 0,
            }
        }
    }

    impl Tracker for BasicTracker {
        #[inline]
        fn register(&mut self, vptr: u32, range: Range<usize>) -> Result<()> {
            let idx = range.start / POOL_MASK_BIT_LEN;
            if idx != (range.end - 1) / POOL_MASK_BIT_LEN {
                return Err("Cannot register a range spanning beyond a page.".into());
            }
            self.pool_mask[idx / POOL_MASK_BITS] |= 1 << (idx % POOL_MASK_BITS);
            self.page_table_mask |= 1 << (vptr / PAGE_LEN / LUT_MASK_BIT_LEN as u32);
            Ok(())
        }
        #[inline]
        fn clear(&mut self) {
            self.pool_mask = vec![0; TOTAL_PAGES as usize / POOL_MASK_BITS].into_boxed_slice();
            self.page_table_mask = 0;
        }
    }
}

pub mod dummy {
    use super::{HashTable, Range, Result, SharedHashDAG, Tracker};

    pub type HostOnlyHashDAG<'shmem> = SharedHashDAG<HashTable<'shmem>, DummyTracker>;

    #[inline]
    pub fn blank<'shmem>(
        root: Option<&String>,
        capacity: usize,
    ) -> Result<HostOnlyHashDAG<'shmem>> {
        HostOnlyHashDAG::blank(root, capacity, None)
    }

    #[derive(Debug, Default)]
    pub struct DummyTracker;

    impl Tracker for DummyTracker {
        #[inline]
        fn register(&mut self, _: u32, _: Range<usize>) -> Result<()> {
            Ok(())
        }
        #[inline]
        fn clear(&mut self) {}
    }
}
