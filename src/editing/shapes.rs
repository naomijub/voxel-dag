pub use self::{aabb::AABB, sphere::Sphere};
use super::{Operation, Shape};

mod aabb {
    use super::{
        super::{OctVox, Vector3},
        Operation, Shape,
    };

    #[derive(Debug)]
    pub struct AABB {
        pub min: Vector3<i64>,
        pub max: Vector3<i64>,
    }

    impl From<OctVox> for AABB {
        #[inline]
        fn from(OctVox { depth, path }: OctVox) -> Self {
            let min: Vector3<_> = path.map(|v| i64::from(v << depth));
            let max: Vector3<_> = min.map(|v| v + (1 << depth));
            Self { min, max }
        }
    }

    impl Shape for AABB {
        type Edit = Self;

        #[inline]
        fn new(centroid: &Vector3<u32>, extent: u32) -> Self {
            let min = centroid.map(|v| i64::from(v) - i64::from(extent));
            let max = centroid.map(|v| i64::from(v) + i64::from(extent));
            Self { min, max }
        }
        #[inline]
        fn collides(&self, edit: &Self::Edit) -> bool {
            !(edit.max.x <= self.min.x
                || self.max.x <= edit.min.x
                || edit.max.y <= self.min.y
                || self.max.y <= edit.min.y
                || edit.max.z <= self.min.z
                || self.max.z <= edit.min.z)
        }
        #[inline]
        fn will_be_full(&self, after: Operation, edit: &Self::Edit) -> bool {
            after == Operation::Link
                && self.min.x <= edit.min.x
                && self.max.x >= edit.max.x
                && self.min.y <= edit.min.y
                && self.max.y >= edit.max.y
                && self.min.z <= edit.min.z
                && self.max.z >= edit.max.z
        }
        #[inline]
        fn will_be_empty(&self, after: Operation, edit: &Self::Edit) -> bool {
            after == Operation::Unlink
                && self.min.x <= edit.min.x
                && self.max.x >= edit.max.x
                && self.min.y <= edit.min.y
                && self.max.y >= edit.max.y
                && self.min.z <= edit.min.z
                && self.max.z >= edit.max.z
        }
    }
}

mod sphere {
    use super::{
        super::{OctVox, Vector3},
        Operation, Shape, AABB,
    };

    #[derive(Debug)]
    pub struct Sphere {
        centroid: Vector3<i64>,
        r_sqr: i64,
    }

    impl From<OctVox> for Sphere {
        #[inline]
        fn from(OctVox { depth, path }: OctVox) -> Self {
            let centroid: Vector3<_> = path.map(|v| i64::from(v << (depth - 1)));
            let r_sqr = i64::from((1u32 << (depth - 1)).pow(2));
            Self { centroid, r_sqr }
        }
    }

    impl Shape for Sphere {
        type Edit = AABB;

        #[inline]
        fn new(centroid: &Vector3<u32>, extent: u32) -> Self {
            let centroid: Vector3<_> = centroid.map(i64::from);
            let r_sqr = i64::from(extent) * i64::from(extent);
            Self { centroid, r_sqr }
        }
        fn collides(&self, edit: &Self::Edit) -> bool {
            let x = if self.centroid.x < edit.min.x {
                self.centroid.x - edit.min.x
            } else if edit.max.x < self.centroid.x {
                self.centroid.x - edit.max.x
            } else {
                0
            };
            let y = if self.centroid.y < edit.min.y {
                self.centroid.y - edit.min.y
            } else if edit.max.y < self.centroid.y {
                self.centroid.y - edit.max.y
            } else {
                0
            };
            let z = if self.centroid.z < edit.min.z {
                self.centroid.z - edit.min.z
            } else if edit.max.z < self.centroid.z {
                self.centroid.z - edit.max.z
            } else {
                0
            };
            0 < self.r_sqr - x * x - y * y - z * z
        }
        fn will_be_full(&self, after: Operation, edit: &Self::Edit) -> bool {
            let x =
                ((edit.min.x - self.centroid.x).pow(2)).max((edit.max.x - self.centroid.x).pow(2));
            let y =
                ((edit.min.y - self.centroid.y).pow(2)).max((edit.max.y - self.centroid.y).pow(2));
            let z =
                ((edit.min.z - self.centroid.z).pow(2)).max((edit.max.z - self.centroid.z).pow(2));
            after == Operation::Link && (x + y + z < self.r_sqr)
        }
        fn will_be_empty(&self, after: Operation, edit: &Self::Edit) -> bool {
            let x =
                ((edit.min.x - self.centroid.x).pow(2)).max((edit.max.x - self.centroid.x).pow(2));
            let y =
                ((edit.min.y - self.centroid.y).pow(2)).max((edit.max.y - self.centroid.y).pow(2));
            let z =
                ((edit.min.z - self.centroid.z).pow(2)).max((edit.max.z - self.centroid.z).pow(2));
            after == Operation::Unlink && (x + y + z < self.r_sqr)
        }
    }
}
