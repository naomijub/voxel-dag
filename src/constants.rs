use serde_derive::{Deserialize, Serialize};

pub const SEED: u32 = 0;
pub const PAGE_LEN: u32 = 512;
pub const SUPPORTED_LEVELS: u32 = 17;
pub const COLOR_TREE_LEVELS: u32 = SUPPORTED_LEVELS - 7;
pub const LEAF_LEVELS: u32 = 2;
pub const LEAF_LEVEL: u32 = SUPPORTED_LEVELS - LEAF_LEVELS;
pub const HI_BUCKET_LEN: u32 = 1024;
pub const LO_BUCKET_LEN: u32 = 4096;
pub const HI_PAGES_PER_BUCKET: u32 = HI_BUCKET_LEN / PAGE_LEN;
pub const LO_PAGES_PER_BUCKET: u32 = LO_BUCKET_LEN / PAGE_LEN;
pub const BUCKETS_PER_HI_LEVEL: u32 = 1 << 10;
pub const BUCKETS_PER_LO_LEVEL: u32 = 1 << 16;
pub const HI_LEVELS: u32 = 9;
pub const LO_LEVELS: u32 = SUPPORTED_LEVELS - HI_LEVELS;
pub const TOTAL_HI_BUCKETS: u32 = HI_LEVELS * BUCKETS_PER_HI_LEVEL;
pub const TOTAL_LO_BUCKETS: u32 = LO_LEVELS * BUCKETS_PER_LO_LEVEL;
pub const TOTAL_BUCKETS: u32 = TOTAL_HI_BUCKETS + TOTAL_LO_BUCKETS;
pub const TOTAL_PAGES: u32 =
    TOTAL_HI_BUCKETS * HI_PAGES_PER_BUCKET + TOTAL_LO_BUCKETS * LO_PAGES_PER_BUCKET;
pub const TOTAL_VIRT_SPACE: u32 = TOTAL_PAGES * PAGE_LEN;

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct SparseVoxelsSequential {
    pub nodes: Vec<u32>,
    pub levels: u32,
}
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct SparseVoxelsSegmented {
    pub nodes: Vec<Vec<u32>>,
    pub levels: u32,
}

mod tests {
    #[allow(unused_imports)]
    use super::{
        BUCKETS_PER_HI_LEVEL, BUCKETS_PER_LO_LEVEL, HI_BUCKET_LEN, LO_BUCKET_LEN, PAGE_LEN,
        TOTAL_PAGES,
    };

    #[test]
    fn verify_settings() {
        #![allow(clippy::assertions_on_constants)]
        assert_eq!(HI_BUCKET_LEN % PAGE_LEN, 0);
        assert_eq!(LO_BUCKET_LEN % PAGE_LEN, 0);
        assert!(32 <= PAGE_LEN);
        assert_eq!((PAGE_LEN & (PAGE_LEN - 1)), 0);
        assert_eq!((HI_BUCKET_LEN & (HI_BUCKET_LEN - 1)), 0);
        assert_eq!((BUCKETS_PER_HI_LEVEL & (BUCKETS_PER_HI_LEVEL - 1)), 0);
        assert_eq!((BUCKETS_PER_LO_LEVEL & (BUCKETS_PER_LO_LEVEL - 1)), 0);
        assert!(16 < PAGE_LEN, "Paging is too small to hold full nodes!");
        assert!(TOTAL_PAGES < !0u32);
        // Below is a little restrictive, but it may be assumed. Not following this assumption may lead to UB.
        assert_eq!((TOTAL_PAGES / 128) * 128, TOTAL_PAGES);
    }
}
