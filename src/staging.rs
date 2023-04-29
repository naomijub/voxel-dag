use super::tracking::basic::{
    BasicHashDAG, PageTableMask, PoolMask, LUT_MASK_BIT_LEN, POOL_MASK_BIT_LEN,
};
use std::{mem::size_of, ops::Range};

pub trait Staging {
    fn stage<WP, WL>(&self, write_pool: WP, write_lut: WL)
    where
        WP: FnMut(Range<usize>, Range<usize>),
        WL: FnMut(Range<usize>, Range<usize>);
}

macro_rules! write_if_end {
    ($src_idx: expr, $dst_idx: expr, $len: expr, $write: expr) => {
        if $len != 0 {
            $write($src_idx..$src_idx + $len, $dst_idx..$dst_idx + $len);
            $src_idx += $len;
            $dst_idx += $len;
            $len = 0;
        }
    };
}

impl Staging for BasicHashDAG<'_> {
    fn stage<WP, WL>(&self, mut write_pool: WP, mut write_lut: WL)
    where
        WP: FnMut(Range<usize>, Range<usize>),
        WL: FnMut(Range<usize>, Range<usize>),
    {
        // Page table (check each bit in each byte)
        {
            let (mut src_idx, mut dst_idx, mut len) = (0, 0, 0);
            for &mask in &self.tracker.page_table_mask.to_le_bytes() {
                for shift in 0..8 {
                    if mask & (1 << shift) != 0 {
                        len += LUT_MASK_BIT_LEN;
                    } else {
                        write_if_end!(src_idx, dst_idx, len, write_lut);
                        dst_idx += LUT_MASK_BIT_LEN;
                    }
                }
            }
            if len != 0 {
                write_lut(src_idx..src_idx + len, dst_idx..dst_idx + len);
            }
        }
        // Pool (like previous algorithm, but optimized for a larger mask)
        {
            let (mut src_idx, mut dst_idx, mut len) = (0, 0, 0);
            // SAFETY: (in HashTable) the pool length is a multiple of 128 pages and each bit is a page.
            for mask in unsafe { self.tracker.pool_mask.align_to::<u128>() }.1 {
                if *mask == !0 {
                    len += 128 * POOL_MASK_BIT_LEN;
                } else if *mask == 0 {
                    write_if_end!(src_idx, dst_idx, len, write_pool);
                    dst_idx += 128 * POOL_MASK_BIT_LEN;
                } else {
                    for &mask in &mask.to_le_bytes() {
                        for shift in 0..8 {
                            if mask & (1 << shift) != 0 {
                                len += POOL_MASK_BIT_LEN;
                            } else {
                                write_if_end!(src_idx, dst_idx, len, write_pool);
                                dst_idx += POOL_MASK_BIT_LEN;
                            }
                        }
                    }
                }
            }
            if len != 0 {
                write_pool(src_idx..src_idx + len, dst_idx..dst_idx + len);
            }
        }
    }
}

impl BasicHashDAG<'_> {
    #[inline]
    #[must_use]
    pub fn staging_specs(&self) -> StagingCache {
        // SAFETY: `TOTAL_PAGES` is a multiple of 128 bits.
        // SAFETY: (in HashTable) the pool length is a multiple of 128 pages and each bit is a page.
        unsafe { StagingCache::new(&self.tracker.pool_mask, self.tracker.page_table_mask) }
    }
}

pub struct StagingCache {
    pub pool_items: usize,
    pub pages: usize,
}

impl StagingCache {
    #[inline]
    /// `pool_mask` is a mask with each bit representing a page.
    /// `page_table_mask` is a mask with each bit representing a partition of the page table.
    ///
    /// # Safety
    ///
    /// It is assumed that the pool length is a multiple of 128 pages.
    /// Not following this assumption leads to UB.
    #[must_use]
    pub unsafe fn new(pool_mask: &[PoolMask], page_table_mask: PageTableMask) -> Self {
        let iter = pool_mask.align_to::<u128>().1.iter();
        let pool_set = iter.map(|mask| mask.count_ones()).sum::<u32>() as usize;
        let page_table_set = page_table_mask.count_ones() as usize;
        Self {
            pool_items: pool_set * POOL_MASK_BIT_LEN,
            pages: LUT_MASK_BIT_LEN * page_table_set,
        }
    }
    #[inline]
    #[must_use]
    pub const fn pool_size(&self) -> usize {
        self.pool_items * size_of::<u32>()
    }
    #[inline]
    #[must_use]
    pub const fn page_table_size(&self) -> usize {
        self.pages * size_of::<u32>()
    }
    #[inline]
    #[must_use]
    pub const fn total_size(&self) -> usize {
        (self.pool_items + self.pages) * size_of::<u32>()
    }
}
