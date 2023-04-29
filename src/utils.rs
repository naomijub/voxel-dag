use super::{
    constants::{
        BUCKETS_PER_HI_LEVEL, BUCKETS_PER_LO_LEVEL, HI_BUCKET_LEN, HI_LEVELS, LO_BUCKET_LEN, SEED,
        SUPPORTED_LEVELS, TOTAL_BUCKETS, TOTAL_HI_BUCKETS, TOTAL_VIRT_SPACE,
    },
    Result,
};
use nalgebra::Vector3;
use std::num::Wrapping;

/// This module contains self-referential structs which basically wrap around shared memory.
pub mod shmem {
    #![allow(clippy::cast_ptr_alignment)]
    use ::{
        shared_memory::{Shmem, ShmemConf, ShmemError},
        std::{
            marker::PhantomPinned, mem::size_of, ops::Deref, path::Path, pin::Pin,
            slice::from_raw_parts_mut,
        },
    };

    pub struct ShmemArray<'shmem, T> {
        inner: Shmem,
        slice: &'shmem mut [T],
        _pin: PhantomPinned,
    }

    impl<T> Deref for ShmemArray<'_, T> {
        type Target = [T];
        fn deref(&self) -> &Self::Target {
            self.slice
        }
    }

    impl<T: Copy> ShmemArray<'_, T> {
        pub fn new<S: AsRef<Path>>(
            len: usize,
            flink: Option<S>,
        ) -> Result<Pin<Box<Self>>, ShmemError> {
            let conf = flink.map_or_else(ShmemConf::new, |path| ShmemConf::new().flink(path));
            let mut raw = Self {
                inner: conf.size(len * size_of::<T>()).create()?,
                slice: &mut [],
                _pin: PhantomPinned,
            };
            // SAFETY: the data pointed to will be pinned without move
            // SAFETY: the pool is one allocated object
            raw.slice = unsafe { from_raw_parts_mut(raw.inner.as_ptr().cast::<T>(), len) };
            Ok(Box::pin(raw))
        }
        #[inline]
        /// # Safety
        ///
        /// - The mutable slice must alway point to inner shared memory.
        /// - Shared memory is one allocated object in contiguous memory.
        #[must_use]
        pub unsafe fn slice_mut(self: Pin<&mut Self>) -> &mut [T] {
            Pin::get_unchecked_mut(self).slice
        }
        #[inline]
        pub fn mut_copy_from(self: Pin<&mut Self>, offset: usize, slice: &[T]) {
            // SAFETY: only copy operations take place, nothing moves or changes reference
            (unsafe { self.slice_mut() })[offset..offset + slice.len()].copy_from_slice(slice);
        }
        #[inline]
        pub fn copy_from(self: &mut Pin<Box<Self>>, offset: usize, slice: &[T]) {
            self.as_mut().mut_copy_from(offset, slice);
        }
    }
}

fn murmur_hash_64(h: u64) -> u64 {
    let mut h = Wrapping(h);
    h ^= h >> 33;
    h *= Wrapping(0xff51_afd7_ed55_8ccd);
    h ^= h >> 33;
    h *= Wrapping(0xc4ce_b9fe_1a85_ec53);
    h ^= h >> 33;
    h.0
}

fn murmur_hash_32(node: &[u32]) -> u32 {
    let mut h = Wrapping(SEED);
    let n = Wrapping(node.len());
    for &k in node {
        let mut k = Wrapping(k);
        k *= Wrapping(0xcc9e_2d51);
        k = (k << 15) | (k >> 17);
        k *= Wrapping(0x1b87_3593);
        h ^= k;
        h = (h << 13) | (h >> 19);
        h = h * Wrapping(5) + Wrapping(0xe654_6b64);
    }

    h ^= Wrapping(n.0 as u32);
    h ^= h >> 16;
    h *= Wrapping(0x85eb_ca6b);
    h ^= h >> 13;
    h *= Wrapping(0xc2b2_ae35);
    h ^= h >> 16;
    h.0
}

#[must_use]
pub fn new_bucket_len_idx_u32(level: u32, bucket: u32) -> u32 {
    debug_assert!(level < SUPPORTED_LEVELS);
    let idx = if level < HI_LEVELS {
        debug_assert!(bucket < BUCKETS_PER_HI_LEVEL);
        level * BUCKETS_PER_HI_LEVEL
    } else {
        debug_assert!(bucket < BUCKETS_PER_LO_LEVEL);
        TOTAL_HI_BUCKETS + (level - HI_LEVELS) * BUCKETS_PER_LO_LEVEL
    } + bucket;
    debug_assert!(idx < TOTAL_BUCKETS);
    idx
}

#[inline]
#[must_use]
pub fn new_bucket_len_idx(level: u32, bucket: u32) -> usize {
    new_bucket_len_idx_u32(level, bucket) as usize
}

#[inline]
#[must_use]
pub fn buckets_per_level(level: u32) -> u32 {
    debug_assert!(level < SUPPORTED_LEVELS);
    if level < HI_LEVELS {
        BUCKETS_PER_HI_LEVEL
    } else {
        BUCKETS_PER_LO_LEVEL
    }
}

#[inline]
#[must_use]
pub fn new_bucket_len(level: u32) -> u32 {
    debug_assert!(level < SUPPORTED_LEVELS);
    if level < HI_LEVELS {
        HI_BUCKET_LEN
    } else {
        LO_BUCKET_LEN
    }
}

#[inline]
/// The supplied hash is bounded to the range [0-b[ with b := correct number of buckets at this level
#[must_use]
pub fn bucket_from_hash(level: u32, hash: u32) -> u32 {
    hash & (buckets_per_level(level) - 1) // TODO could use modulo too, but is more expensive, curious about distribution though
}

const HI: u32 = TOTAL_HI_BUCKETS * HI_BUCKET_LEN;
/// This returns the virtual pointer composed exclusively out of all three given parameters
pub fn new_vptr(level: u32, bucket: u32, offset_bucket: u32) -> Result<u32> {
    debug_assert!(level < SUPPORTED_LEVELS);
    let idx = if level < HI_LEVELS {
        if HI_BUCKET_LEN <= offset_bucket {
            return Err("Creating pointer: Bucket offset out of bounds! (Top level)".into());
        } else {
            new_bucket_len_idx_u32(level, bucket) * HI_BUCKET_LEN
        }
    } else if LO_BUCKET_LEN <= offset_bucket {
        return Err("Creating pointer: Bucket offset out of bounds! (Low level)".into());
    } else if BUCKETS_PER_LO_LEVEL <= bucket {
        return Err("Creating pointer: Bucket too large!".into());
    } else {
        let lo = ((level - HI_LEVELS) * BUCKETS_PER_LO_LEVEL + bucket) * LO_BUCKET_LEN;
        HI + lo
    } + offset_bucket;
    if idx < TOTAL_VIRT_SPACE {
        Ok(idx)
    } else {
        Err("Creating pointer: Virtual pointer is out of bounds!".into())
    }
}

#[inline]
#[must_use]
pub const fn vptr_to_lvl(vptr: u32) -> u32 {
    let hi = vptr / (BUCKETS_PER_HI_LEVEL * HI_BUCKET_LEN);
    let lo = vptr.overflowing_sub(HI).0 / (BUCKETS_PER_LO_LEVEL * LO_BUCKET_LEN) + HI_LEVELS;
    let is_hi_lvl = (vptr < HI) as u32;
    is_hi_lvl * hi + (1 - is_hi_lvl) * lo
}

#[inline]
#[must_use]
pub fn hash_leaf(leaf: &[u32]) -> u32 {
    murmur_hash_64(as_leaf_mask(leaf)) as u32
}

#[inline]
const fn as_leaf_mask(leaf: &[u32]) -> u64 {
    (leaf[1] as u64) << 32 | leaf[0] as u64
}

#[inline]
#[must_use]
pub fn hash_interior(node: &[u32]) -> u32 {
    murmur_hash_32(node)
}

#[inline]
#[must_use]
pub const fn count_leaves(leaf: &[u32]) -> u32 {
    as_leaf_mask(leaf).count_ones()
}

#[inline]
#[must_use]
pub fn descend(path: &Vector3<u32>, child: u32) -> Vector3<u32> {
    path.map_with_location(|r, _, v| (v << 1) | (child >> (2 - r)) & 1)
}

#[inline]
#[must_use]
pub const fn upper_child_mask(leaf: &[u32]) -> u8 {
    ((leaf[0] & 0x0000_00ff != 0) as u8)
        | (((leaf[0] & 0x0000_ff00 != 0) as u8) << 1)
        | (((leaf[0] & 0x00ff_0000 != 0) as u8) << 2)
        | (((leaf[0] & 0xff00_0000 != 0) as u8) << 3)
        | (((leaf[1] & 0x0000_00ff != 0) as u8) << 4)
        | (((leaf[1] & 0x0000_ff00 != 0) as u8) << 5)
        | (((leaf[1] & 0x00ff_0000 != 0) as u8) << 6)
        | (((leaf[1] & 0xff00_0000 != 0) as u8) << 7)
}

#[inline]
#[must_use]
pub const fn bottom_child_mask(leaf: &[u32], child: u32) -> u8 {
    (leaf[(4 <= child) as usize] >> ((child & 3) * 8)) as u8
}

pub mod serialization {
    use serde::de::DeserializeOwned;
    use std::{
        fs::File,
        io::{BufReader, Read},
        mem::size_of,
    };

    #[inline]
    pub fn load_ron<T: DeserializeOwned>(name: &str) -> Result<T, String> {
        ron::de::from_reader(&mut BufReader::new(
            File::open(format!("assets/{name}.ron")).unwrap(),
        ))
        .map_err(|e| format!("Failed to parse {name}.ron, reason: {e}"))
    }

    #[inline]
    pub fn read_size<R: Read>(file: &mut R) -> Option<usize> {
        let mut bytes = [0; 8];
        file.read_exact(&mut bytes).ok()?;
        Some(u64::from_le_bytes(bytes) as usize)
    }

    #[inline]
    pub fn read_word<R: Read>(file: &mut R) -> Option<u32> {
        let mut bytes = [0; 4];
        file.read_exact(&mut bytes).ok()?;
        Some(u32::from_le_bytes(bytes))
    }

    #[inline]
    pub fn read_boxed_slice<R: Read, T: Sized + Clone>(file: &mut R) -> Option<Box<[T]>> {
        let mut bytes = vec![0; size_of::<T>() * read_size(file)?];
        file.read_exact(&mut bytes).ok()?;
        // SAFETY: bytes is a single allocated object with the correct length for alignment.
        Some(unsafe { bytes.align_to::<T>() }.1.iter().cloned().collect())
    }

    #[inline]
    pub fn read_exact_slice<R: Read, T: Sized + Clone>(
        file: &mut R,
        size: usize,
    ) -> Option<Box<[T]>> {
        let mut bytes = vec![0; size_of::<T>() * size];
        file.read_exact(&mut bytes).ok()?;
        // SAFETY: bytes is a single allocated object with the correct length for alignment.
        Some(unsafe { bytes.align_to::<T>() }.1.iter().cloned().collect())
    }
}

mod tests {
    #[allow(unused_imports)]
    use super::{murmur_hash_32, murmur_hash_64};

    #[test]
    fn valid_murmur_hash_32() {
        assert_ne!(murmur_hash_32(&[123, 123]), murmur_hash_32(&[123]));
    }
}
