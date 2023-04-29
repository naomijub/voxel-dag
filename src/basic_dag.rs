use super::{
    constants::SUPPORTED_LEVELS,
    utils::{
        descend,
        serialization::{read_boxed_slice, read_word},
    },
    Result,
};
use ::{
    nalgebra::Vector3,
    num_traits::identities::Zero,
    std::{
        fs::File,
        io::{BufReader, Read},
        path::Path,
    },
};

#[derive(Debug, PartialOrd, Eq, PartialEq)]
/// An octree-aligned voxel.
pub struct OctVox {
    pub depth: u32,
    pub path: Vector3<u32>,
}

impl OctVox {
    #[inline]
    #[must_use]
    pub const fn new(level: u32, path: &Vector3<u32>) -> Self {
        let depth = SUPPORTED_LEVELS - level;
        let path: Vector3<_> = *path;
        Self { depth, path }
    }
    #[inline]
    #[must_use]
    pub fn descended(&self, child: u32) -> Self {
        let depth = self.depth - 1;
        let path = descend(&self.path, child);
        Self { depth, path }
    }
    #[inline]
    #[must_use]
    pub fn is_child(&self, other: &Self) -> bool {
        if self.depth <= other.depth {
            false
        } else {
            let diff = self.depth - other.depth;
            other.path.map(|v| v >> diff) == self.path
        }
    }
}

#[derive(Debug)]
pub struct BasicDAG {
    pub pool: Box<[u32]>,
    pub levels: u32,
    pub root_idx: usize,
}

impl BasicDAG {
    #[inline]
    #[must_use]
    pub fn new(levels: u32, pool: Box<[u32]>) -> Self {
        Self {
            pool,
            levels,
            root_idx: 0,
        }
    }
    #[inline]
    pub fn from_file<P: AsRef<Path>>(path: P) -> Option<Self> {
        let mut file = BufReader::new(File::open(path).ok()?);
        file.read_exact(&mut [0; 8 * 6]).ok()?;
        Some(Self {
            levels: read_word(&mut file)?,
            pool: read_boxed_slice(&mut file)?,
            root_idx: 0,
        })
    }
    #[inline]
    #[must_use]
    pub fn find_node(&self, target: &OctVox) -> Option<usize> {
        if target.depth < 2 {
            None
        } else {
            let init_state = OctVox::new(SUPPORTED_LEVELS - self.levels, &Vector3::zero());
            self.seek(target, (init_state, self.root_idx)).ok()?
        }
    }
    fn seek(&self, target: &OctVox, (state, idx): (OctVox, usize)) -> Result<Option<usize>> {
        let &mask = self.pool.get(idx).ok_or(format!(
            "Node index out of bounds! Depth: {}, Index: {}",
            state.depth, idx
        ))?;
        let child_mask = mask as u8;
        let mut offset = 1;
        for child in 0..8 {
            if child_mask & (1 << child) != 0 {
                let state = state.descended(child);
                let idx = self.pool[idx + offset] as usize;
                if target == &state {
                    return Ok(Some(idx));
                } else if !state.is_child(target) || state.depth == 2 {
                    return Ok(None);
                } else if let Some(idx) = self.seek(target, (state, idx))? {
                    return Ok(Some(idx));
                }
                offset += 1;
            }
        }
        Ok(None)
    }
}
