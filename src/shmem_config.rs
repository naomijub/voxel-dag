use super::constants::{
    BUCKETS_PER_HI_LEVEL, BUCKETS_PER_LO_LEVEL, COLOR_TREE_LEVELS, HI_BUCKET_LEN, HI_LEVELS,
    HI_PAGES_PER_BUCKET, LEAF_LEVEL, LEAF_LEVELS, LO_BUCKET_LEN, LO_LEVELS, LO_PAGES_PER_BUCKET,
    PAGE_LEN, SUPPORTED_LEVELS, TOTAL_BUCKETS, TOTAL_HI_BUCKETS, TOTAL_LO_BUCKETS, TOTAL_PAGES,
    TOTAL_VIRT_SPACE,
};
use std::{
    fs::{remove_file, File},
    io::{Result, Write},
};

#[allow(dead_code)]
pub const VERSION: u32 = 0;

#[derive(Debug)]
pub struct ShmemConfig {
    pub path: String,
    pub class: String,
}

impl ShmemConfig {
    pub fn write(self) -> Result<Self> {
        let mut file = File::create(self.path.as_str())?;

        let bytes = [
            b"{",
            format!("\"{}\":{},", "VERSION", VERSION).as_bytes(),
            format!("\"{}\":\"{}\",", "CLASS", self.class).as_bytes(),
            format!("\"{}\":{},", "PAGE_LEN", PAGE_LEN).as_bytes(),
            format!("\"{}\":{},", "SUPPORTED_LEVELS", SUPPORTED_LEVELS).as_bytes(),
            format!("\"{}\":{},", "COLOR_TREE_LEVELS", COLOR_TREE_LEVELS).as_bytes(),
            format!("\"{}\":{},", "LEAF_LEVELS", LEAF_LEVELS).as_bytes(),
            format!("\"{}\":{},", "LEAF_LEVEL", LEAF_LEVEL).as_bytes(),
            format!("\"{}\":{},", "HI_BUCKET_LEN", HI_BUCKET_LEN).as_bytes(),
            format!("\"{}\":{},", "LO_BUCKET_LEN", LO_BUCKET_LEN).as_bytes(),
            format!("\"{}\":{},", "HI_PAGES_PER_BUCKET", HI_PAGES_PER_BUCKET).as_bytes(),
            format!("\"{}\":{},", "LO_PAGES_PER_BUCKET", LO_PAGES_PER_BUCKET).as_bytes(),
            format!("\"{}\":{},", "BUCKETS_PER_HI_LEVEL", BUCKETS_PER_HI_LEVEL).as_bytes(),
            format!("\"{}\":{},", "BUCKETS_PER_LO_LEVEL", BUCKETS_PER_LO_LEVEL).as_bytes(),
            format!("\"{}\":{},", "HI_LEVELS", HI_LEVELS).as_bytes(),
            format!("\"{}\":{},", "LO_LEVELS", LO_LEVELS).as_bytes(),
            format!("\"{}\":{},", "TOTAL_HI_BUCKETS", TOTAL_HI_BUCKETS).as_bytes(),
            format!("\"{}\":{},", "TOTAL_LO_BUCKETS", TOTAL_LO_BUCKETS).as_bytes(),
            format!("\"{}\":{},", "TOTAL_BUCKETS", TOTAL_BUCKETS).as_bytes(),
            format!("\"{}\":{},", "TOTAL_PAGES", TOTAL_PAGES).as_bytes(),
            // last one has no comma
            format!("\"{}\":{}", "TOTAL_VIRT_SPACE", TOTAL_VIRT_SPACE).as_bytes(),
            b"}",
        ]
        .concat();
        file.write_all(&bytes)?;
        Ok(self)
    }
}

impl Drop for ShmemConfig {
    #[inline]
    fn drop(&mut self) {
        remove_file(self.path.as_str()).unwrap_or_default();
    }
}
