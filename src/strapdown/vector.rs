// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::{Mul, Div, Add, Sub};
use std::cmp::PartialEq;
use std::vec;
use derive_more;
use std::f64::consts::PI;
use approx::RelativeEq;

// Crate

// ----------------------------------------------------------------------------
// Vectors
// ----------------------------------------------------------------------------
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    PartialOrd,
    derive_more::Add,
    derive_more::Sub,
    derive_more::Mul,
)]
pub struct Vector{
    pub x: f64,
    pub y: f64,
    pub z: f64
}

// Pg 3-1
impl Vector{

    pub fn new(x: f64, y: f64, z: f64) -> Vector{
        return Vector {
            x,
            y,
            z
        }
    }

    pub fn of(num: f64) -> Vector{
        return Vector::new(num, num, num)
    }

    pub fn to_array(self) -> [f64; 3]{
        // Eq: 3.1-10, Pg 3-3
        return [self.x, self.y, self.z]
    }

    pub fn mag(self) -> f64{
        // Eq: 3.1.1-4, Pg 3-8
        return (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0)).sqrt()
    }

    pub fn dot(self, vec: Vector) -> f64{
        // Eq 3.1.1-5, Pg 3-8
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)
    }

    pub fn cross(self, vec: Vector) -> Vector{
        // Eq 3.1.1-6, Pg 3-8
        return Vector::new(
            (self.y * vec.z) - (self.z * vec.y),
            (self.z * vec.x) - (self.x * vec.z),
            (self.x * vec.y) - (self.y * vec.x)
        )
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn vec_dot(){
        // Arbitrary Vector
        let vec = Vector::new(1.0, 2.0, 3.0);
        let vec2 = Vector::new(2.0, 1.0, 3.0);

        let vec_dot_vec2 = vec.dot(vec2);
        let vec2_dot_vec = vec2.dot(vec);

    }

    #[test]
    fn vec_cross(){
        // Arbitrary Vector
        let vec = Vector::new(1.0, 2.0, 3.0);
        let vec2 = Vector::new(2.0, 1.0, 3.0);

        let vec_cross_vec2 = vec.cross(vec2);
        let vec2_cross_vec = vec2.cross(vec);

        let deviation_error = Vector::of(1e-6);

        // Eq 3.1.1-8, Pg 3-8
        assert_relative_eq!(
            vec_cross_vec2,
            -1 * vec2_cross_vec,
            max_relative=1e-6
        );
    }
}