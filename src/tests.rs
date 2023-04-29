#![allow(unused_imports, dead_code)]
use super::{
    constants::{
        SparseVoxelsSegmented, SparseVoxelsSequential, BUCKETS_PER_HI_LEVEL, COLOR_TREE_LEVELS,
        HI_BUCKET_LEN, HI_LEVELS, LEAF_LEVEL, PAGE_LEN, SUPPORTED_LEVELS, TOTAL_PAGES,
        TOTAL_VIRT_SPACE,
    },
    conversion::Converter,
    editing::{
        inner::interior_from,
        shapes::{Sphere, AABB},
        Editor, Operation,
        Operation::{Link, Unlink},
        Shape,
    },
    hash_table::basic::HashTable,
    prelude::*,
    shared_hash_dag::SharedHashDAG,
    staging::Staging,
    tracking::{
        basic::{BasicHashDAG, POOL_MASK_BITS, POOL_MASK_BIT_LEN},
        dummy::{blank, HostOnlyHashDAG},
        Tracker,
    },
    utils::{
        bucket_from_hash, hash_interior, hash_leaf, new_bucket_len_idx, new_vptr,
        serialization::{load_ron, read_exact_slice, read_word},
        vptr_to_lvl,
    },
    validation::{Validation::Valid, Validator},
    HashDAG, HashDAGMut, Result,
};
use utils::{
    add_lantern, basic_blank, basic_with_capacity, full_dag, host_only_blank,
    host_only_with_capacity, import_matches, stage, verify_full_interior, verify_full_leaf,
};
use ::{
    nalgebra::Vector3,
    num_traits::identities::Zero,
    std::{cmp::Ordering, fs::File, io::Read, path::Path},
};

const SVDAG_STORE: &str = ".local/svdags/";
const DAG_SUFFIX: &str = ".basic_dag.dag.bin";

mod utils {
    use super::*;

    #[inline]
    pub fn full_dag<'lut>() -> HostOnlyHashDAG<'lut> {
        host_only_with_capacity(0xffff * (PAGE_LEN as usize)).unwrap()
    }
    #[inline]
    pub fn host_only_blank<'shmem>(capacity: usize) -> Result<HostOnlyHashDAG<'shmem>> {
        blank(None, capacity)
    }
    #[inline]
    pub fn basic_blank<'shmem>(capacity: usize) -> Result<BasicHashDAG<'shmem>> {
        BasicHashDAG::blank(None, capacity, None)
    }
    #[inline]
    pub fn host_only_with_capacity<'shmem>(capacity: usize) -> Result<HostOnlyHashDAG<'shmem>> {
        HostOnlyHashDAG::with_capacity(None, capacity)
    }
    #[inline]
    pub fn basic_with_capacity<'shmem>(capacity: usize) -> Result<BasicHashDAG<'shmem>> {
        BasicHashDAG::with_capacity(None, capacity)
    }

    #[inline]
    pub fn verify_full_leaf(dag: &HostOnlyHashDAG, vptr: u32) {
        assert_ne!(vptr, 0);
        for &leaf in dag.leaf(vptr).unwrap().iter() {
            assert_eq!(leaf, !0u32);
        }
    }

    pub fn verify_full_interior(dag: &HostOnlyHashDAG, level: u32, vptr: u32) -> u32 {
        let child_ptr = dag.full_node_ptr(level + 1).unwrap();
        assert_ne!(vptr, 0);
        assert_ne!(child_ptr, 0);
        let node = dag.interior(vptr).unwrap();
        assert_eq!(node.len(), 9);
        child_ptr
    }

    pub fn add_lantern<T: Tracker>(
        dag: &mut SharedHashDAG<HashTable, T>,
    ) -> (Result<u32>, BasicDAG) {
        let bd = {
            let mut file = File::open("assets/lantern.comp.bin").unwrap();
            file.read_exact(&mut [0; 8 * 6]).unwrap();
            let levels = read_word(&mut file).unwrap();
            let num_nodes = read_word(&mut file).unwrap() as usize;
            file.read_exact(&mut [0; 4 * 6]).unwrap();
            let pool = read_exact_slice(&mut file, num_nodes).unwrap();
            BasicDAG::new(levels, pool)
        };
        (dag.import_strict(&bd, None), bd)
    }

    pub fn import_matches(bd: &BasicDAG, dag: &HostOnlyHashDAG, optimized: bool, vptr: u32) {
        let mut ptrs1: Vec<u32> = Vec::with_capacity(bd.pool.len() / 4);
        let mut ptrs2: Vec<u32> = Vec::with_capacity(bd.pool.len() / 4);
        ptrs1.push(0u32);
        ptrs2.push(vptr);
        for _level in (SUPPORTED_LEVELS - bd.levels)..LEAF_LEVEL {
            let mut n_ptrs1: Vec<u32> = Vec::with_capacity(ptrs1.capacity());
            let mut n_ptrs2: Vec<u32> = Vec::with_capacity(ptrs2.capacity());
            while let Some(ptr) = ptrs1.pop() {
                let interior: Vec<u32> = {
                    let ptr = ptr as usize;
                    let children = (bd.pool[ptr] as u8).count_ones() as usize;
                    bd.pool[ptr..=ptr + children].to_vec()
                };
                if optimized {
                    for &vptr in interior.iter().skip(1) {
                        if !n_ptrs1.contains(&vptr) {
                            n_ptrs1.push(vptr);
                        }
                    }
                } else {
                    for &vptr in interior.iter().skip(1) {
                        n_ptrs1.push(vptr);
                    }
                }
            }
            while let Some(vptr) = ptrs2.pop() {
                let interior = dag.interior(vptr).unwrap();
                if optimized {
                    for vptr in interior.iter().skip(1) {
                        if !n_ptrs2.contains(vptr) {
                            n_ptrs2.push(*vptr);
                        }
                    }
                } else {
                    interior.iter().skip(1).for_each(|&vp| n_ptrs2.push(vp));
                }
            }
            ptrs1 = n_ptrs1;
            ptrs2 = n_ptrs2;
            assert_eq!(ptrs1.len(), ptrs2.len());
        }
    }

    pub fn stage(dag: &mut BasicHashDAG, pool_dst: &mut [u32], lut_dst: &mut [u32]) {
        let specs = dag.staging_specs();
        let mut pool_src = vec![0; specs.pool_items as _].into_boxed_slice();
        let mut lut_src = vec![0; specs.pages as _].into_boxed_slice();
        let (pool, lut) = dag.hash_dag.dump();
        // Create a staging buffer
        dag.stage(
            |src_range, dst_range| pool_src[src_range].copy_from_slice(&pool[dst_range]),
            |src_range, dst_range| lut_src[src_range].copy_from_slice(&lut[dst_range]),
        );
        // Stage to device buffer
        dag.stage(
            |src_range, dst_range| pool_dst[dst_range].copy_from_slice(&pool_src[src_range]),
            |src_range, dst_range| lut_dst[dst_range].copy_from_slice(&lut_src[src_range]),
        );
    }
}

mod basic_dag {
    use super::*;
    #[test]
    /// Should return None. It is assumed you want to look up a child-node.
    fn find_root() {
        let file = format!("lantern1k{DAG_SUFFIX}");
        let bd = BasicDAG::from_file(Path::new(SVDAG_STORE).join(file)).expect("File missing.");
        let root_level = SUPPORTED_LEVELS - bd.levels;
        assert!(bd
            .find_node(&OctVox::new(root_level, &Vector3::zero()))
            .is_none());
    }

    #[test]
    fn find_non_root() {
        let file = format!("lantern1k{DAG_SUFFIX}");
        let bd = BasicDAG::from_file(Path::new(SVDAG_STORE).join(file)).expect("File missing.");
        let root_level = SUPPORTED_LEVELS - bd.levels;
        let node_found = bd
            .find_node(&OctVox::new(root_level + 1, &Vector3::zero()))
            .unwrap();
        assert_ne!(node_found, bd.root_idx);
        assert_ne!(node_found, bd.pool[bd.root_idx] as _);
        // A more rigorous test would be to now pointer-chase and to then determine if it is the correct node.
    }
}

mod hash_table {
    use super::*;
    mod blank {
        use super::*;
        const BLOCK_LEN: usize = PAGE_LEN as usize * 128;
        #[test]
        fn blank_hash_table() {
            let dag = host_only_blank(TOTAL_VIRT_SPACE as _).unwrap();
            assert!(dag.pool.len() <= TOTAL_VIRT_SPACE as usize);
            assert_eq!(dag.pool.len() % BLOCK_LEN, 0);
        }
        #[test]
        fn blank_hash_table_zero_size() {
            assert_eq!(
                host_only_blank(0).err(),
                Some(format!("Cannot allocate {} words to a pool!", 0))
            );
        }
        #[test]
        fn blank_hash_table_single_page() {
            let dag = host_only_blank(1).unwrap();
            assert_eq!(BLOCK_LEN, dag.pool.len());
            assert_eq!(dag.pool.len() % BLOCK_LEN, 0);
        }
        #[test]
        fn blank_hash_table_size_too_large() {
            const MAX_LEN: usize = TOTAL_VIRT_SPACE as _;
            const EXPECTED: usize = MAX_LEN + BLOCK_LEN - MAX_LEN % BLOCK_LEN;
            assert_eq!(
                host_only_blank(TOTAL_VIRT_SPACE as usize + 1).err(),
                Some(format!("Cannot allocate {EXPECTED} words to a pool!"))
            );
        }
    }

    mod allocate {
        use super::*;
        #[test]
        fn allocate_single() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.allocate(1).unwrap();
            assert_eq!(dag.is_allocated(1), Ok(true));
        }
        #[test]
        fn allocate_multiple() {
            const COUNT: u32 = TOTAL_PAGES;
            let mut dag = host_only_blank((COUNT * PAGE_LEN) as _).unwrap();
            (0..COUNT).for_each(|page| dag.allocate(page as _).unwrap());
        }
        #[test]
        #[cfg(debug_assertions)]
        #[should_panic(expected = "Page does not exist.")]
        fn allocate_one_to_many() {
            const COUNT: u32 = TOTAL_PAGES;
            let mut dag = host_only_blank((COUNT * PAGE_LEN) as _).unwrap();
            (0..COUNT).for_each(|page| dag.allocate(page as _).unwrap());
            dag.allocate(COUNT as _).unwrap();
        }
        #[test]
        #[cfg(not(debug_assertions))]
        #[should_panic]
        fn allocate_one_to_many() {
            const COUNT: u32 = TOTAL_PAGES;
            let mut dag = host_only_blank((COUNT * PAGE_LEN) as _).unwrap();
            (0..COUNT).for_each(|page| dag.allocate(page as _).unwrap());
            dag.allocate(COUNT as _).unwrap();
        }
        #[test]
        #[cfg(debug_assertions)]
        #[should_panic(expected = "Trying to allocate an allocated page.")]
        fn allocate_twice() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.allocate(0).unwrap();
            dag.allocate(0).unwrap();
        }
        #[test]
        #[cfg(debug_assertions)]
        #[should_panic(expected = "Page does not exist.")]
        fn allocate_out_of_bound_page() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.allocate(TOTAL_PAGES as _).unwrap();
        }
    }

    mod pool_idx {
        use super::*;
        #[test]
        fn pool_idx_sanity_check() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let vptr =
                new_vptr(HI_LEVELS - 1, BUCKETS_PER_HI_LEVEL - 1, HI_BUCKET_LEN - 1).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();
            dag.pool_idx(vptr).unwrap();
        }
        #[test]
        fn pool_idx_lower_bound() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.allocate(0).unwrap();
            assert_eq!(dag.pool_idx(0).unwrap(), 0);
        }
        #[test]
        fn pool_idx_upper_bound() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const VPTR: u32 = TOTAL_VIRT_SPACE - 1;
            dag.allocate((VPTR / PAGE_LEN) as _).unwrap();
            let idx = dag.pool_idx(VPTR).unwrap();
            assert_eq!(idx, PAGE_LEN as usize - 1);
        }
        #[test]
        fn pool_idx_beyond_upper_bound() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const VPTR: u32 = TOTAL_VIRT_SPACE - 1;
            dag.allocate((VPTR / PAGE_LEN) as _).unwrap();
            assert_eq!(
                dag.pool_idx(VPTR + 1),
                Err("Trying to lookup a non-existing page.".into())
            );
        }
        #[test]
        fn pool_idx_vptr_to_unallocated() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const VPTR: u32 = TOTAL_VIRT_SPACE - 1;
            dag.allocate((VPTR / PAGE_LEN) as _).unwrap();
            // Offsetting by, say just one, may not work as it may still point in the "correct" page
            assert_eq!(
                dag.pool_idx(VPTR - PAGE_LEN),
                Err("Virtual pointer points to unallocated memory.".into())
            );
        }
    }

    mod add_full {
        use super::*;
        #[test]
        /// This must not fail as it is not "`find_or_add_full_leaf`".
        fn add_full_leaf_twice() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.add_full_leaf();
            dag.add_full_leaf();
        }
        #[test]
        fn add_full_leaf_and_find() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.add_full_leaf();

            // Simple node index fetch
            let vptr = dag.full_node_ptr(LEAF_LEVEL).unwrap();
            verify_full_leaf(&dag, vptr);

            // Search as if there was no full node table
            const LEAF: &[u32] = &[!0, !0];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let bucket_len = dag.bucket_len(LEAF_LEVEL, bucket);
            let ptr = dag.find_leaf(bucket, bucket_len, LEAF).unwrap();
            assert_eq!(vptr, ptr.unwrap());
        }
        #[test]
        /// Out of order meaning you work top-down instead of bottom-up. It must not fail (dumb function).
        fn add_full_interior_out_of_order() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.add_full_interior(0);
        }
        #[test]
        fn add_full_interior_page_overflow() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            (0..(PAGE_LEN as f32 / 9f32) as usize + 1).for_each(|_| {
                dag.add_full_interior(0);
            });
            let vptr = dag.full_node_ptr(0).unwrap();
            let page1 = vptr / PAGE_LEN;
            assert!(1 < page1);
            assert!(dag.is_allocated(page1 as _).unwrap());
            assert_eq!((vptr - PAGE_LEN) / PAGE_LEN, page1 - 1);
            let page0 = page1 - 1;
            assert!(dag.is_allocated(page0 as _).unwrap());
        }
        #[test]
        fn add_full_interior_and_find() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            dag.add_full_interior(1);
            dag.add_full_interior(0);
            // Simple node index fetch
            let vptr = dag.full_node_ptr(0).unwrap();
            let child_ptr = verify_full_interior(&dag, 0, vptr);
            // Search as if there was no full node table
            let interior = [
                0xff, child_ptr, child_ptr, child_ptr, child_ptr, child_ptr, child_ptr, child_ptr,
                child_ptr,
            ];
            let hash = hash_interior(&interior);
            let bucket = bucket_from_hash(0, hash);
            let bucket_len = dag.bucket_len(0, bucket);
            let ptr = dag.find_interior(0, bucket, bucket_len, &interior);
            assert_eq!(Ok(Some(vptr)), ptr);
        }

        mod initialized_dag {
            use super::*;
            use crate::{utils::new_vptr, HashDAGMut};

            #[test]
            /// Meaning all levels have a full node (including root).
            fn add_full_node_array_and_verify() {
                let dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
                for level in 0..LEAF_LEVEL {
                    let vptr = dag.full_node_ptr(level).unwrap();
                    let _ = verify_full_interior(&dag, level, vptr);
                }
                verify_full_leaf(&dag, dag.full_node_ptr(LEAF_LEVEL).unwrap());
                assert!(dag.full_node_pointers.iter().all(|idx| *idx != 0));
            }

            #[test]
            fn add_full_leaf_to_fully_initialized() {
                let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
                let old_ptr = dag.full_node_ptr(LEAF_LEVEL);
                let new_ptr = dag.find_or_add_leaf(Pass(&[!0, !0]));
                assert_eq!(old_ptr, new_ptr);
            }

            #[test]
            fn add_full_leaf_after_new() {
                let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
                let hash = hash_leaf(&[!0, !0]);
                let bucket = bucket_from_hash(LEAF_LEVEL, hash);
                let vptr = new_vptr(LEAF_LEVEL, bucket, 0).unwrap();
                let page = vptr / PAGE_LEN;
                assert!(dag.is_allocated(page as _).unwrap());
                let new_ptr = dag.find_or_add_leaf(Pass(&[!0, !0]));
                assert_eq!(new_ptr.unwrap(), vptr);
            }

            #[test]
            fn add_full_interior_to_fully_initialized() {
                let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
                let old_ptr = dag.full_node_ptr(0).unwrap();
                let child_ptr = dag.full_node_ptr(1).unwrap();
                let interior = [
                    0xff, child_ptr, child_ptr, child_ptr, child_ptr, child_ptr, child_ptr,
                    child_ptr, child_ptr,
                ];
                let new_ptr = dag.find_or_add_interior(0, Pass(&interior));
                assert_eq!(old_ptr, new_ptr.unwrap());
            }

            #[test]
            fn add_full_interior_after_new() {
                let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
                const LEVEL: u32 = 0;
                let interior = [1u32, 0];
                let hash = hash_interior(&interior);
                let bucket = bucket_from_hash(LEVEL, hash);
                let vptr = new_vptr(LEVEL, bucket, 0).unwrap();
                let page = vptr / PAGE_LEN;
                assert!(!dag.is_allocated(page as _).unwrap());
                let new_ptr = dag.find_or_add_interior(LEVEL, Pass(&interior));
                assert_eq!(new_ptr.unwrap(), vptr);
            }
        }
    }

    mod add {
        use super::*;
        #[test]
        fn add_empty_interior() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let hash = hash_interior(&[]);
            // This used to fail, but it should just add, not verify, then add
            assert!(dag.add_interior(0, Pass(&[]), hash).is_ok());
        }
        #[test]
        fn add_incomplete_interior() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEVEL: u32 = 0;
            let interior = [0b0000_0000];
            let hash = hash_interior(&interior);
            // This used to fail, but it should just add, not verify, then add
            assert!(dag.add_interior(LEVEL, Pass(&interior), hash).is_ok());
        }
    }

    mod find {
        use super::*;
        #[test]
        fn find_existing_leaf_in_zero_sized_bucket_window() {
            let dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[!0, !0];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let vptr = dag.find_leaf(bucket, 0, LEAF).unwrap();
            assert!(vptr.is_none());
        }
        #[test]
        fn find_existing_leaf_in_tiny_smaller_than_page_window() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[0, 1];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let bucket_len = 8;
            // pretend 4 empty leaves are added which (in)conveniently hash the same way
            dag.bucket_len_copy_from(new_bucket_len_idx(LEAF_LEVEL, bucket), &[bucket_len]);
            let vptr = new_vptr(LEAF_LEVEL, bucket, bucket_len).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();

            assert!(dag.add_leaf(Pass(LEAF), hash).is_ok());
            assert_eq!(dag.bucket_len(LEAF_LEVEL, bucket), bucket_len + 2);
            assert!(dag.find_leaf(bucket, bucket_len, LEAF).unwrap().is_none());
        }
        #[test]
        fn find_existing_leaf_in_tiny_larger_than_page_window() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[0, 1];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let bucket_len = PAGE_LEN + 8;
            // pretend 4 empty leaves are added which (in)conveniently hash the same way
            dag.bucket_len_copy_from(new_bucket_len_idx(LEAF_LEVEL, bucket), &[bucket_len]);
            let vptr = new_vptr(LEAF_LEVEL, bucket, bucket_len).unwrap();
            dag.allocate(((vptr - PAGE_LEN) / PAGE_LEN) as _).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();

            assert!(dag.add_leaf(Pass(LEAF), hash).is_ok());
            assert_eq!(dag.bucket_len(LEAF_LEVEL, bucket), bucket_len + 2);
            assert!(dag.find_leaf(bucket, bucket_len, LEAF).unwrap().is_none());
        }
        #[test]
        fn find_leaf_in_larger_than_page() {
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[0, 1];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let bucket_len = PAGE_LEN + 8;
            // pretend 4 empty leaves are added which (in)conveniently hash the same way
            dag.bucket_len_copy_from(new_bucket_len_idx(LEAF_LEVEL, bucket), &[bucket_len]);
            let vptr = new_vptr(LEAF_LEVEL, bucket, bucket_len).unwrap();
            dag.allocate(((vptr - PAGE_LEN) / PAGE_LEN) as _).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();

            let vptr = dag.add_leaf(Pass(LEAF), hash).unwrap();
            assert_eq!(dag.bucket_len(LEAF_LEVEL, bucket), bucket_len + 2);
            let ptr = dag.find_leaf(bucket, bucket_len + 2, LEAF).unwrap();
            assert_eq!(ptr.unwrap(), vptr);
        }
        #[test]
        fn find_leaf_unallocated() {
            let dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[!0, !0];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            assert_eq!(
                dag.find_leaf(bucket, 0, LEAF),
                Err("Virtual pointer points to unallocated memory.".into())
            );
        }
        #[test]
        fn find_interior_unallocated() {
            let dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEVEL: u32 = 0;
            let node = [0xff; 9];
            let hash = hash_interior(&node);
            let bucket = bucket_from_hash(LEVEL, hash);
            assert_eq!(
                dag.find_interior(LEVEL, bucket, 0, &node),
                Err("Virtual pointer points to unallocated memory.".into())
            );
        }
        #[test]
        fn find_existing_interior_in_zero_sized_bucket_window() {
            let dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEVEL: u32 = 0;
            let vptr = dag.full_node_ptr(LEVEL).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            let node = dag.pool[pool_idx..pool_idx + 9].to_vec();
            let hash = hash_interior(&node);
            let bucket = bucket_from_hash(LEVEL, hash);
            let vptr = dag.find_interior(LEVEL, bucket, 0, &node);
            assert!(vptr.unwrap().is_none());
        }
        #[test]
        fn find_existing_interior_in_tiny_smaller_than_page_window() {
            const LEVEL: u32 = 0;
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let interior = [0b0100_1011, 1, 1, 1, 1];
            let hash = hash_interior(&interior);
            let bucket = bucket_from_hash(LEVEL, hash);
            let vptr = new_vptr(LEVEL, bucket, 0).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[0xff]);
            let bucket_len = 9;
            dag.bucket_len_copy_from(new_bucket_len_idx(LEVEL, bucket), &[bucket_len]);

            assert!(dag.add_interior(LEVEL, Pass(&interior), hash).is_ok());
            let expected_bucket_len = bucket_len + interior.len() as u32;
            assert_eq!(dag.bucket_len(LEVEL, bucket), expected_bucket_len);
            let found = dag.find_interior(LEVEL, bucket, bucket_len, &interior);
            assert!(found.unwrap().is_none());
        }
        #[test]
        fn find_existing_interior_in_tiny_larger_than_page_window() {
            const LEVEL: u32 = 0;
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let interior = [0b0100_1011, 1, 1, 1, 1];
            let hash = hash_interior(&interior);
            let bucket = bucket_from_hash(LEVEL, hash);
            let vptr = new_vptr(LEVEL, bucket, 0).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();
            dag.allocate(((vptr + PAGE_LEN) / PAGE_LEN) as _).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            let bucket_len = 9 + PAGE_LEN;
            for offset in (0..PAGE_LEN).step_by(9) {
                dag.pool_copy_from(pool_idx + offset as usize, &[0xff]);
            }
            dag.pool_copy_from(pool_idx + PAGE_LEN as usize, &[0xff]);
            dag.bucket_len_copy_from(new_bucket_len_idx(LEVEL, bucket), &[bucket_len]);

            assert!(dag.add_interior(LEVEL, Pass(&interior), hash).is_ok());
            let expected_bucket_len = bucket_len + interior.len() as u32;
            assert_eq!(dag.bucket_len(LEVEL, bucket), expected_bucket_len);
            let found = dag.find_interior(LEVEL, bucket, bucket_len, &interior);
            assert!(found.unwrap().is_none());
        }
        #[test]
        fn find_interior_in_larger_than_page() {
            const LEVEL: u32 = 0;
            let mut dag = host_only_blank((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let interior = [0b0100_1011, 1, 1, 1, 1];
            let hash = hash_interior(&interior);
            let bucket = bucket_from_hash(LEVEL, hash);
            let vptr = new_vptr(LEVEL, bucket, 0).unwrap();
            dag.allocate((vptr / PAGE_LEN) as _).unwrap();
            dag.allocate(((vptr + PAGE_LEN) / PAGE_LEN) as _).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            let bucket_len = 9 + PAGE_LEN;
            for offset in (0..PAGE_LEN).step_by(9) {
                dag.pool_copy_from(pool_idx + offset as usize, &[0xff]);
            }
            dag.pool_copy_from(pool_idx + PAGE_LEN as usize, &[0xff]);
            dag.bucket_len_copy_from(new_bucket_len_idx(LEVEL, bucket), &[bucket_len]);

            let vptr = dag.add_interior(LEVEL, Pass(&interior), hash);
            let expected_bucket_len = bucket_len + interior.len() as u32;
            assert_eq!(dag.bucket_len(LEVEL, bucket), expected_bucket_len);
            let ptr = dag.find_interior(LEVEL, bucket, expected_bucket_len, &interior);
            assert_eq!(ptr, Ok(Some(vptr.unwrap())));
        }
    }

    mod find_or_add {
        use super::*;
        #[test]
        fn find_or_add_existing_full_leaf() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            const LEAF: &[u32] = &[!0, !0];
            let hash = hash_leaf(LEAF);
            let bucket = bucket_from_hash(LEAF_LEVEL, hash);
            let old_full = dag.full_node_ptr(LEAF_LEVEL).unwrap();
            let old_bucket_len = dag.bucket_len(LEAF_LEVEL, bucket);
            assert!(dag.find_or_add_leaf(Pass(LEAF)).is_ok());
            let new_full = dag.full_node_ptr(LEAF_LEVEL).unwrap();
            let new_bucket_len = dag.bucket_len(LEAF_LEVEL, bucket);
            assert_eq!(old_full, new_full);
            assert_eq!(old_bucket_len, new_bucket_len);
        }
        #[test]
        fn find_or_add_existing_full_interior() {
            const LEVEL: u32 = 0;
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let old_full = dag.full_node_ptr(LEVEL).unwrap();
            let pool_idx = dag.pool_idx(old_full).unwrap();
            let interior: Vec<u32> = dag.pool[pool_idx..pool_idx + 9].into();
            let hash = hash_interior(&interior);
            let bucket = bucket_from_hash(LEVEL, hash);
            let old_bucket_len = dag.bucket_len(LEVEL, bucket);
            assert!(dag.find_or_add_interior(LEVEL, Pass(&interior)).is_ok());
            let new_full = dag.full_node_ptr(LEVEL).unwrap();
            let new_bucket_len = dag.bucket_len(LEVEL, bucket);
            assert_eq!(old_full, new_full);
            assert_eq!(old_bucket_len, new_bucket_len);
        }
    }
    mod validation {
        use super::*;
        use crate::{
            utils::{bucket_from_hash, hash_interior},
            validation::{utils::is_valid_vptr, Validation::Valid, Validator},
        };
        #[test]
        fn is_valid_after_new() {
            let dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root = dag.full_node_ptr(0).unwrap();
            assert_eq!(dag.validate(root), Ok(Valid));
        }
        #[test]
        fn is_invalid_after_leaf_tampering() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root_vptr = dag.full_node_ptr(0).unwrap();
            let vptr = dag.full_node_ptr(LEAF_LEVEL).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[0, 0]);
            assert_ne!(dag.validate(root_vptr), Ok(Valid));
        }
        #[test]
        fn is_invalid_after_voxel_count_tampering() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root_vptr = dag.full_node_ptr(0).unwrap();
            let vptr = dag.full_node_ptr(COLOR_TREE_LEVELS).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[0xff]);
            assert_ne!(dag.validate(root_vptr), Ok(Valid));
        }
        #[test]
        fn is_invalid_after_child_mask_tampering() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root_vptr = dag.full_node_ptr(0).unwrap();
            let vptr = dag.full_node_ptr(COLOR_TREE_LEVELS).unwrap();
            let pool_idx = dag.pool_idx(vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[(dag.pool[pool_idx] >> 8) << 8]);
            assert_ne!(dag.validate(root_vptr), Ok(Valid));
        }
        #[test]
        fn is_invalid_after_nullifying_root_vptr() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root_vptr = dag.full_node_ptr(0).unwrap();
            let pool_idx = dag.pool_idx(root_vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[0]);
            assert_ne!(dag.validate(root_vptr), Ok(Valid));
        }
        #[test]
        fn is_invalid_after_overflow_voxel_count() {
            let mut dag = host_only_with_capacity((SUPPORTED_LEVELS * PAGE_LEN) as _).unwrap();
            let root_vptr = dag.full_node_ptr(0).unwrap();
            let pool_idx = dag.pool_idx(root_vptr).unwrap();
            dag.pool_copy_from(pool_idx, &[0xff00]);
            assert_ne!(dag.validate(root_vptr), Ok(Valid));
        }
        #[test]
        fn is_valid_vptr_on_valid_exact() {
            let dag = full_dag();
            {
                const LEVEL: u32 = HI_LEVELS - 1;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(is_valid_vptr(vptr, LEVEL, Some(bucket), Some(0)));
            }
            {
                const LEVEL: u32 = HI_LEVELS;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(is_valid_vptr(vptr, LEVEL, Some(bucket), Some(0)));
            }
        }
        #[test]
        fn is_valid_vptr_on_invalid_exact() {
            let dag = full_dag();
            {
                const LEVEL: u32 = HI_LEVELS - 1;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(!is_valid_vptr(vptr, LEVEL, Some(bucket), Some(1)));
            }
            {
                const LEVEL: u32 = HI_LEVELS;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(!is_valid_vptr(vptr, LEVEL, Some(bucket), Some(1)));
            }
        }
        #[test]
        fn is_valid_vptr_on_valid_no_offset() {
            let dag = full_dag();
            {
                const LEVEL: u32 = HI_LEVELS - 1;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(is_valid_vptr(vptr, LEVEL, Some(bucket), None));
            }
            {
                const LEVEL: u32 = HI_LEVELS;
                let vptr = dag.full_node_ptr(LEVEL).unwrap();
                let bucket = bucket_from_hash(LEVEL, hash_interior(dag.interior(vptr).unwrap()));
                assert!(is_valid_vptr(vptr, LEVEL, Some(bucket), None));
            }
        }
        #[test]
        fn is_valid_vptr_on_valid_no_bucket() {
            let dag = full_dag();
            {
                let vptr = dag.full_node_ptr(HI_LEVELS - 1).unwrap();
                assert!(is_valid_vptr(vptr, HI_LEVELS - 1, None, None));
            }
            {
                let vptr = dag.full_node_ptr(HI_LEVELS).unwrap();
                assert!(is_valid_vptr(vptr, HI_LEVELS, None, None));
            }
        }
        #[test]
        fn is_valid_vptr_on_exceeding_virt_mem() {
            assert!(!is_valid_vptr(TOTAL_VIRT_SPACE, 0, None, None));
            assert!(!is_valid_vptr(TOTAL_VIRT_SPACE, LEAF_LEVEL, None, None));
        }
    }

    mod conversion {
        use super::*;
        #[test]
        fn import_nodes_ron() {
            let toy_dag: SparseVoxelsSequential = load_ron("nodes").unwrap();
            let bd = BasicDAG {
                pool: toy_dag.nodes.into_boxed_slice(),
                levels: toy_dag.levels,
                root_idx: 0,
            };
            let mut dag = full_dag();
            let vptr = dag.import_strict(&bd, None).unwrap();
            assert_eq!(dag.validate(vptr).unwrap(), Valid);
            import_matches(&bd, &dag, true, vptr);
        }
        #[test]
        fn import_nodes_svo_ron() {
            let mut toy_dag: SparseVoxelsSegmented = load_ron("nodes-svo").unwrap();
            let bd = BasicDAG {
                pool: {
                    let mut pool = Vec::new();
                    let children = toy_dag.nodes[0].len() as u32;
                    let mut iter = toy_dag.nodes[0].iter();
                    pool.push(*iter.next().unwrap());
                    iter.for_each(|&i| pool.push(i + children));
                    pool.append(&mut toy_dag.nodes[1]);
                    pool
                }
                .into_boxed_slice(),
                levels: toy_dag.levels,
                root_idx: 0,
            };
            let mut dag = full_dag();
            let vptr = dag.import_strict(&bd, None).unwrap();
            assert_eq!(dag.validate(vptr).unwrap(), Valid);
            import_matches(&bd, &dag, false, vptr);
        }
        #[test]
        /// This should pass as it doesn't actually read it as a cyclical graph. At some point it will read the interior as leaves.
        /// Spotting this error is something a more rigorous/paranoid implementation should do.
        fn import_cyclical_graph() {
            let bd = BasicDAG {
                pool: Box::new([
                    0b0110_1001,
                    5,
                    5,
                    5,
                    7,
                    0b0000_0001,
                    9,
                    0b0000_0001,
                    0,
                    0xffff_ffff,
                    0xffff_ffff,
                ]),
                levels: 4,
                root_idx: 0,
            };
            let mut dag = full_dag();
            let vptr = dag.import_strict(&bd, None).unwrap();
            assert_eq!(dag.validate(vptr).unwrap(), Valid);
            import_matches(&bd, &dag, true, vptr);
        }
        #[test]
        fn import_lantern() {
            let mut dag = host_only_with_capacity(32_000_000 / 4).unwrap();
            let (import, bd) = add_lantern(&mut dag);
            let vptr = import.unwrap();
            assert_eq!(dag.validate(vptr).unwrap(), Valid);
            import_matches(&bd, &dag, true, vptr);
        }
        #[test]
        fn import_too_much() {
            let mut dag = host_only_with_capacity(1).unwrap();
            let (import, _) = add_lantern(&mut dag);
            assert_eq!(
                import.err(),
                Some("No space is left to allocate! Consider resizing your pool.".into())
            );
        }
        const MAX_OFFSET: u32 = HI_BUCKET_LEN - 1;
        const MAX_BUCKET: u32 = BUCKETS_PER_HI_LEVEL - 1;
        #[test]
        fn vptr_to_lvl_0() {
            let vptr = new_vptr(0, 0, 0);
            assert_eq!(0, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(0, 0, MAX_OFFSET);
            assert_eq!(0, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(0, MAX_BUCKET, MAX_OFFSET);
            assert_eq!(0, vptr_to_lvl(vptr.unwrap()));
        }
        #[test]
        fn vptr_to_hi_lvl() {
            let vptr = new_vptr(HI_LEVELS - 1, 0, 0);
            assert_eq!(HI_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(HI_LEVELS - 1, 0, MAX_OFFSET);
            assert_eq!(HI_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(HI_LEVELS - 1, MAX_BUCKET, MAX_OFFSET);
            assert_eq!(HI_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
        }
        #[test]
        fn vptr_to_lo_lvl() {
            let vptr = new_vptr(HI_LEVELS, 0, 0);
            assert_eq!(HI_LEVELS, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(HI_LEVELS, 0, MAX_OFFSET);
            assert_eq!(HI_LEVELS, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(HI_LEVELS, MAX_BUCKET, MAX_OFFSET);
            assert_eq!(HI_LEVELS, vptr_to_lvl(vptr.unwrap()));
        }
        #[test]
        fn vptr_to_max_lvl() {
            let vptr = new_vptr(SUPPORTED_LEVELS - 1, 0, 0);
            assert_eq!(SUPPORTED_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(SUPPORTED_LEVELS - 1, 0, MAX_OFFSET);
            assert_eq!(SUPPORTED_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
            let vptr = new_vptr(SUPPORTED_LEVELS - 1, MAX_BUCKET, MAX_OFFSET);
            assert_eq!(SUPPORTED_LEVELS - 1, vptr_to_lvl(vptr.unwrap()));
        }
    }

    mod oct_vox {
        use super::*;
        #[test]
        fn is_child_true_when_deep_child() {
            let parent = OctVox::new(3, &[2, 1, 0].into());
            assert!(parent.is_child(&parent.descended(5).descended(1).descended(7).descended(3)));
            assert!(parent.is_child(&parent.descended(3).descended(7).descended(1).descended(5)));
        }
        #[test]
        fn is_child_true_when_direct_child() {
            let parent = OctVox::new(3, &[2, 1, 0].into());
            assert!(parent.is_child(&parent.descended(5)));
            assert!(parent.is_child(&parent.descended(1)));
            assert!(parent.is_child(&parent.descended(3)));
            assert!(parent.is_child(&parent.descended(7)));
        }
        #[test]
        fn is_child_false_when_sibling() {
            let parent = OctVox::new(3, &Vector3::zero());
            assert!(!parent.is_child(&OctVox::new(3, &[1; 3].into())));
        }
        #[test]
        fn is_child_false_when_self() {
            let parent = OctVox::new(3, &Vector3::zero());
            assert!(!parent.is_child(&OctVox::new(3, &Vector3::zero())));
        }
        #[test]
        fn is_child_false_when_shallow_branch() {
            let parent = OctVox::new(3, &[2, 1, 0].into());
            let sibling = OctVox::new(3, &[0, 1, 2].into());
            assert!(!parent.is_child(&sibling.descended(1)));
            assert!(!parent.is_child(&sibling.descended(3)));
            assert!(!parent.is_child(&sibling.descended(5)));
            assert!(!parent.is_child(&sibling.descended(7)));
        }
        #[test]
        fn is_child_false_when_deep_branch() {
            let parent = OctVox::new(3, &[2, 1, 0].into());
            let sibling = OctVox::new(3, &[0, 1, 2].into());
            assert!(!parent.is_child(&sibling.descended(1).descended(3).descended(5).descended(7)));
            assert!(!parent.is_child(&sibling.descended(7).descended(5).descended(3).descended(1)));
        }
    }

    mod editing {
        use super::*;
        #[test]
        fn unlinking_bunch_of_tests() {
            let mut dag = full_dag();
            let path = [0, 1, 0].into();
            let vptr = dag.full_node_ptr(0).unwrap();
            {
                let shape = AABB::from(OctVox::new(COLOR_TREE_LEVELS - 1, &path));
                assert_ne!(dag.edit(vptr, Unlink, &shape), Ok(vptr));
            }
            {
                let shape = AABB::from(OctVox::new(COLOR_TREE_LEVELS, &path));
                assert_ne!(dag.edit(vptr, Unlink, &shape), Ok(vptr));
            }
            {
                let shape = AABB::from(OctVox::new(COLOR_TREE_LEVELS + 1, &path));
                assert_ne!(dag.edit(vptr, Unlink, &shape), Ok(vptr));
            }
            {
                let shape = AABB::from(OctVox::new(LEAF_LEVEL, &path));
                assert_ne!(dag.edit(vptr, Unlink, &shape), Ok(vptr));
            }
            {
                let shape = AABB::from(OctVox::new(LEAF_LEVEL + 1, &path));
                assert_ne!(dag.edit(vptr, Unlink, &shape), Ok(vptr));
            }
        }
        #[test]
        fn voxel_count_should_not_overflow() {
            let mut dag = full_dag();
            let vptr = dag.full_node_ptr(0).unwrap();
            let shape = Sphere::new(&Vector3::zero(), 400);
            let result = dag.edit(vptr, Unlink, &shape);
            assert!(result.is_ok());
        }

        mod interior_from {
            use super::*;
            #[test]
            fn node_from_empty() {
                assert!(interior_from([None; 8], 0).is_none());
            }
            #[test]
            fn node_from_full() {
                let mut node = interior_from([Some(0); 8], 0).unwrap();
                assert_eq!(node[0], 0xff);
                assert_eq!(node.len(), 9);
                node.dedup();
                assert_eq!(node.len(), 2);
            }
            #[test]
            fn node_from_sparse() {
                let children = [Some(0), None, Some(0), None, Some(0), None, Some(0), None];
                let mut node = interior_from(children, 0).unwrap();
                assert_eq!(node[0], 0b101_0101);
                assert_eq!(node.len(), 5);
                node.dedup();
                assert_eq!(node.len(), 2);
            }
        }

        /// This is internal and has certain assumptions:
        /// - The intent is to (un)link, with unlinking as the default operation.
        /// - It is assumed that any other shapes than AABB accurately describe themselves. The shape (AABB) is always explicitly mentioned.
        /// * The guide (`editing_masks.md`) can help a lot here. It's a lot faster than keeping track of the numbers. *
        mod edit_leaf {
            use super::*;
            use crate::{tests::full_dag, utils::hash_leaf, HashDAGMut};
            use nalgebra::Vector3;
            #[test]
            fn produce_empty_leaf_using_shape_aabb_operation_unlink() {
                let mut dag = full_dag();
                let interior: Vector3<u32> = [3, 2, 1].into();
                let leaf = &[0b0000_0100_0000_0000, 0]; // +(0, 1, 2)
                let unit_voxel = interior.map(|v| v << 2) + Vector3::new(0, 1, 2);
                let shape = AABB::from(OctVox::new(SUPPORTED_LEVELS, &unit_voxel));
                let single_leaf: Option<u32> = dag.add_leaf(Pass(leaf), hash_leaf(leaf)).ok();
                let edit = dag
                    .edit_leaf((Unlink, &shape), single_leaf, &interior)
                    .unwrap();
                assert!(edit.0.is_none());
                // Sanity check for the reader/maintainer. Proof that the scope (supposedly spanning a single voxel) affects exactly one bit.
                let full_leaf = dag.full_node_ptr(LEAF_LEVEL).ok();
                let edit = dag
                    .edit_leaf((Unlink, &shape), full_leaf, &interior)
                    .unwrap();
                let (_, voxel_count) = edit;
                assert_eq!(voxel_count, 63);
            }
            #[test]
            fn produce_full_leaf_using_shape_aabb_operation_link() {
                let mut dag = full_dag();
                let interior: Vector3<u32> = [1, 1, 2].into();
                let leaf = &[0b1011_1111_1111_1111_1111_1111_1111_1111, !0]; // +(1, 3, 2)
                let unit_voxel = interior.map(|v| v << 2) + Vector3::new(1, 3, 2);
                let scope = (
                    Link,
                    &AABB::from(OctVox::new(SUPPORTED_LEVELS, &unit_voxel)),
                );
                let single_leaf: Option<u32> = dag.add_leaf(Pass(leaf), hash_leaf(leaf)).ok();
                let edit = dag.edit_leaf(scope, single_leaf, &interior).unwrap();
                let (_, voxel_count) = edit;
                assert_eq!(voxel_count, 64);
            }
            #[test]
            fn produce_sparse_leaf_using_shape_aabb_operation_unlink() {
                let mut dag = full_dag();
                let interior: Vector3<u32> = [3, 2, 1].into();
                for extent in 1..4u32 {
                    let scope = (Unlink, &AABB::new(&interior.map(|v| v << 2), extent));
                    let full_leaf = dag.full_node_ptr(LEAF_LEVEL).ok();
                    let edit = dag.edit_leaf(scope, full_leaf, &interior).unwrap();
                    let (vptr, voxel_count) = edit;
                    // "Hits" the corner, thus extends in only 3 directions instead of 6
                    assert_eq!(voxel_count, 64 - extent.pow(3), "extent: {extent}");
                    let leaf = dag.leaf(vptr.unwrap()).unwrap();
                    assert_eq!(
                        leaf[0] & 1,
                        0,
                        "This bit should never be set (it is the 'hit' corner)"
                    );
                    assert_eq!(
                        leaf[1] & (1 << 31),
                        1 << 31,
                        "This bit should always be set (the furthest voxel from 'hit')"
                    );
                }
            }
            #[test]
            fn produce_sparse_leaf_using_shape_aabb_operation_link() {
                let mut dag = full_dag();
                let interior: Vector3<u32> = [3, 2, 1].into();
                for extent in 1..4u32 {
                    let scope = (Link, &AABB::new(&interior.map(|v| v << 2), extent));
                    let edit = dag.edit_leaf(scope, None, &interior).unwrap();
                    let (vptr, voxel_count) = edit;
                    // "Hits" the corner, thus extends in only 3 directions instead of 6
                    assert_eq!(voxel_count, extent.pow(3), "extent: {extent}");
                    let leaf = dag.leaf(vptr.unwrap()).unwrap();
                    assert_eq!(
                        leaf[0] & 1,
                        1,
                        "This bit should always be set (it is the 'hit' corner)"
                    );
                    assert_eq!(
                        leaf[1] & (1 << 31),
                        0,
                        "This bit should never be set (the furthest voxel from 'hit')"
                    );
                }
            }
        }

        mod edit_interior {
            use super::*;
            use crate::{
                editing::inner::NodeState,
                tests::full_dag,
                utils::{descend, hash_interior},
                HashDAGMut,
            };
            #[test]
            fn produce_empty_interior() {
                let mut dag = full_dag();
                const LEVEL: u32 = COLOR_TREE_LEVELS - 2; // A random level; logic about color trees is supposed to be inside the closure
                let interior = [0b0010_0000, dag.full_node_ptr(LEVEL + 1).unwrap()];
                let single_interior = dag
                    .add_interior(LEVEL, Pass(&interior), hash_interior(&interior))
                    .unwrap();
                let node = NodeState {
                    level: LEVEL,
                    vptr: Some(single_interior),
                    path: [3, 2, 1].into(),
                };
                let child_path = descend(&node.path, (interior[0] - 1).count_ones());
                let result = dag.edit_interior(node, |_, node| {
                    assert_eq!(node.level, LEVEL + 1);
                    if node.path == child_path {
                        assert_eq!(*node, Some(interior[1]));
                        Ok((true, None, 0))
                    } else {
                        assert!(node.is_none());
                        Ok((false, None, 0))
                    }
                });
                assert_eq!(result, Ok((None, 0)));
            }
            #[test]
            fn produce_existing_full_interior() {
                let mut dag = full_dag();
                const LEVEL: u32 = COLOR_TREE_LEVELS - 2; // A random level; logic about color trees is supposed to be inside the closure
                let interior = [0];
                let single_interior = dag
                    .add_interior(LEVEL, Pass(&interior), hash_interior(&interior))
                    .unwrap();
                let node = NodeState {
                    level: LEVEL,
                    vptr: Some(single_interior),
                    path: [3, 2, 1].into(),
                };
                let full = dag.full_node_ptr(LEVEL + 1).unwrap();
                let result = dag.edit_interior(node, |_, node| {
                    assert!(node.is_none());
                    Ok((true, Some(full), 0))
                });
                assert_eq!(result, Ok((dag.full_node_ptr(LEVEL).ok(), 0)));
            }
        }

        mod edit {
            use super::*;
            // TODO (un)link leaf
            // TODO (un)link below color tree
            // TODO (un)link on color tree
            // TODO (un)link above color tree
        }
    }

    mod tracking {
        use super::*;
        #[test]
        fn registers_fine() {
            let mut dag = basic_blank(32_000_000 / 4).unwrap();
            let pool_len = dag.pool.len();
            let idx = (pool_len - 1) / POOL_MASK_BIT_LEN;
            let (mask_idx, bit_idx) = (idx / POOL_MASK_BITS, idx % POOL_MASK_BITS);
            assert_eq!(
                bit_idx,
                POOL_MASK_BITS - 1,
                "Something is wrong with block alignment."
            );

            // First check if blank() does what it's supposed to.
            let mut mask = dag.tracker.pool_mask[mask_idx];
            assert_eq!(mask, 0, "`blank()` performs an allocation.");

            let range = pool_len - 1..pool_len;
            dag.tracker.register(0, range).unwrap();
            mask = dag.tracker.pool_mask[mask_idx];
            let mut expected = 1 << (POOL_MASK_BITS - 1);
            assert_eq!(mask, expected);

            let range = pool_len - POOL_MASK_BIT_LEN - 1..pool_len - POOL_MASK_BIT_LEN;
            dag.tracker.register(0, range).unwrap();
            mask = dag.tracker.pool_mask[mask_idx];
            expected |= 1 << (POOL_MASK_BITS - 2);
            assert_eq!(mask, expected);
        }
        #[test]
        fn import_stage_edit_stage() {
            let mut dag = basic_with_capacity((32_000_000 / 4) as _).unwrap();
            let mut dev_pool = vec![0; dag.pool.len()].into_boxed_slice();
            let mut dev_lut = vec![0; dag.lut.len()].into_boxed_slice();
            // Import & stage
            let vptr = add_lantern(&mut dag).0.unwrap();
            stage(&mut dag, &mut dev_pool, &mut dev_lut);
            dag.tracker.clear();
            assert_eq!(dev_pool.as_ref().cmp(&dag.pool), Ordering::Equal);
            assert_eq!(dev_lut.as_ref().cmp(&dag.lut), Ordering::Equal);
            // Edit & stage
            dag.edit(vptr, Operation::Link, &AABB::new(&Vector3::zero(), 200))
                .unwrap();
            stage(&mut dag, &mut dev_pool, &mut dev_lut);
            dag.tracker.clear();
            assert_eq!(dev_pool.as_ref().cmp(&dag.pool), Ordering::Equal);
            assert_eq!(dev_lut.as_ref().cmp(&dag.lut), Ordering::Equal);
        }
    }
}
