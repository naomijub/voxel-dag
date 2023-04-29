use super::{constants::PAGE_LEN, hash_table::basic::HashTable};
use ::std::mem::size_of;

#[derive(Debug)]
pub struct HashTableReport {
    pub allocated_pages_in_mb: f32,
    pub page_table_in_mb: f32,
    pub pool_in_mb: f32,
    pub total_pages: u32,
    pub allocated_pages: u32,
}

pub trait Reporter {
    fn allocated_pages_in_mb(&self) -> f32;
    fn page_table_in_mb(&self) -> f32;
    fn pool_in_mb(&self) -> f32;
    fn total_pages(&self) -> u32;
    fn allocated_pages(&self) -> u32;
    fn report(&self) -> HashTableReport;
}

impl Reporter for HashTable<'_> {
    #[inline]
    fn allocated_pages_in_mb(&self) -> f32 {
        (size_of::<u32>() * self.allocated_pages() as usize) as f32 / 1e6f32
    }
    #[inline]
    fn page_table_in_mb(&self) -> f32 {
        (size_of::<u32>() * self.lut.len()) as f32 / 1e6f32
    }
    #[inline]
    fn pool_in_mb(&self) -> f32 {
        (size_of::<u32>() * self.pool.len()) as f32 / 1e6f32
    }
    #[inline]
    fn total_pages(&self) -> u32 {
        self.pool.len() as u32 / PAGE_LEN
    }
    #[inline]
    fn allocated_pages(&self) -> u32 {
        self.lut.hi()
    }
    #[inline]
    fn report(&self) -> HashTableReport {
        HashTableReport {
            allocated_pages_in_mb: self.allocated_pages_in_mb(),
            page_table_in_mb: self.page_table_in_mb(),
            pool_in_mb: self.pool_in_mb(),
            total_pages: self.total_pages(),
            allocated_pages: self.allocated_pages(),
        }
    }
}
