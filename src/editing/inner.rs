use super::{super::basic_dag::OctVox, Shape};
use ::{nalgebra::Vector3, std::ops::Deref};

#[must_use]
pub fn interior_from(children: [Option<u32>; 8], voxel_count: u32) -> Option<Vec<u32>> {
    let mut node = Vec::with_capacity(9);
    node.push(voxel_count << 8);
    for (child, &vptr) in children.iter().enumerate() {
        if let Some(vptr) = vptr {
            node.push(vptr);
            node[0] |= 1 << child;
        }
    }
    if node[0] as u8 != 0 {
        Some(node)
    } else {
        None
    }
}

pub struct NodeState {
    pub level: u32,
    pub vptr: Option<u32>,
    pub path: Vector3<u32>,
}

impl NodeState {
    #[inline]
    #[must_use]
    pub fn edit_shape<S>(&self) -> S::Edit
    where
        S: Shape,
        S::Edit: From<OctVox>,
    {
        S::Edit::from(OctVox::new(self.level, &self.path))
    }
}

impl Deref for NodeState {
    type Target = Option<u32>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.vptr
    }
}
