// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::{Mul, Div, Add, Sub};
use std::cmp::PartialEq;
use std::vec;
use derive_more;
use std::f64::consts::PI;

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
    derive_more::Sub
)]
struct Vector{
    x: f64,
    y: f64,
    z: f64
}

// Pg 3-1
impl Vector{

    fn new(x: f64, y: f64, z: f64) -> Vector{
        return Vector {
            x,
            y,
            z
        }
    }

    fn of(num: f64) -> Vector{
        return Vector::new(num, num, num)
    }

    fn as_array(self) -> [f64; 3]{
        // Eq: 3.1-10, Pg 3-3
        return [self.x, self.y, self.z]
    }

    fn mag(self) -> f64{
        // Eq: 3.1.1-4, Pg 3-8
        return (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0)).sqrt()
    }

    fn dot(self, vec: Vector) -> f64{
        // Eq 3.1.1-5, Pg 3-8
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)
    }

    fn cross(self, vec: Vector) -> Vector{
        // Eq 3.1.1-6, Pg 3-8
        return Vector::new(
            (self.y * vec.z) - (self.z * vec.y),
            (self.z * vec.x) - (self.x * vec.z),
            (self.x * vec.y) - (self.y * vec.x)
        )
    }
}

// -------------
// DCMs
// -------------

// Pg 3-3
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    PartialOrd,
    derive_more::Add,
    derive_more::Sub,
    derive_more::Mul
)]
struct DCM{
    c11: f64,
    c12: f64,
    c13: f64,
    c21: f64,
    c22: f64,
    c23: f64,
    c31: f64,
    c32: f64,
    c33: f64,
}

impl DCM{

    fn new(
        c11: f64, c12: f64, c13: f64,
        c21: f64, c22: f64, c23: f64,
        c31: f64, c32: f64, c33: f64,
    ) -> DCM{
        // Eq: 3.1-10, Pg 3-3
        return DCM {
            c11, c12, c13,
            c21, c22, c23,
            c31, c32, c33
        }
    }

    fn of(num: f64) -> DCM{
        return DCM::new(
            num, num, num,
            num, num, num,
            num, num, num
        )
    }

    fn identity() -> DCM{
        return DCM::new(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        )
    }

    fn as_array(self) -> [[f64; 3]; 3]{
        // Eq: 3.1-10, Pg 3-3
        return [
            [self.c11, self.c12, self.c13],
            [self.c21, self.c22, self.c23],
            [self.c31, self.c32, self.c33],
        ]
    }

    fn transpose(self) -> DCM{
        // Source:
        //     https://en.wikipedia.org/wiki/Transpose
        return DCM::new(
            self.c11, self.c21, self.c31,
            self.c12, self.c22, self.c32,
            self.c13, self.c23, self.c33,
        )
    }

}

impl Mul<DCM> for DCM{
    type Output = DCM;

    fn mul(self, b: DCM) -> DCM{
        // 3x3 Matrix Multiplication
        let _c11 = (self.c11 * b.c11) + (self.c12 * b.c21) + (self.c13 * b.c31);
        let _c12 = (self.c11 * b.c12) + (self.c12 * b.c22) + (self.c13 * b.c32);
        let _c13 = (self.c11 * b.c13) + (self.c12 * b.c23) + (self.c13 * b.c33);

        let _c21 = (self.c21 * b.c11) + (self.c22 * b.c21) + (self.c23 * b.c31);
        let _c22 = (self.c21 * b.c12) + (self.c22 * b.c22) + (self.c23 * b.c32);
        let _c23 = (self.c21 * b.c13) + (self.c22 * b.c23) + (self.c23 * b.c33);

        let _c31 = (self.c31 * b.c11) + (self.c32 * b.c21) + (self.c33 * b.c31);
        let _c32 = (self.c31 * b.c12) + (self.c32 * b.c22) + (self.c33 * b.c32);
        let _c33 = (self.c31 * b.c13) + (self.c32 * b.c23) + (self.c33 * b.c33);

        return DCM::new(
            _c11, _c12, _c13,
            _c21, _c22, _c23,
            _c31, _c32, _c33,
        )

    }
}

impl Mul<Vector> for DCM{
    // Eq 3.1.1-2, Pg 3-15
    type Output = Vector;

    fn mul(self, vec: Vector) -> Vector{
        // 3x3 times a 3x1 matrix
        return Vector::new(
            (self.c11 * vec.x) + (self.c12 * vec.y) + (self.c13 * vec.z),
            (self.c21 * vec.x) + (self.c22 * vec.y) + (self.c23 * vec.z),
            (self.c31 * vec.x) + (self.c32 * vec.y) + (self.c33 * vec.z)
        )
    }
}

// -------------
// Euler Angles
// -------------

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    PartialOrd,
    derive_more::Add,
    derive_more::Sub,
    derive_more::Mul,
    derive_more::Div
)]
struct Euler{
    yaw: f64,
    pitch: f64,
    roll: f64
}

impl Euler{
    fn new(yaw: f64, pitch: f64, roll: f64) -> Euler{
        return Euler {yaw, pitch, roll}
    }

    fn of(num: f64) -> Euler{
        return Euler::new(num, num, num)
    }

    fn zeros() -> Euler{
        return Euler::new(0.0, 0.0, 0.0)
    }

    fn as_dcm(self) -> DCM{
        // Eq 3.2.3.1-1, Pg 3-33
        let _c11 = self.pitch.cos() * self.yaw.cos();
        let _c12 =
            (-self.roll.cos() * self.yaw.sin()) +
            (self.roll.sin() * self.pitch.sin() * self.yaw.sin());
        let _c13 =
            (self.roll.sin() * self.yaw.sin()) +
            (self.roll.cos() * self.pitch.sin() * self.yaw.cos());

        let _c21 = self.pitch.cos() * self.yaw.cos();
        let _c22 =
            (self.roll.sin() * self.yaw.cos()) +
            (self.roll.sin() * self.pitch.sin() * self.yaw.sin());
        let _c23 =
            (-self.roll.sin() * self.yaw.cos()) +
            (self.roll.cos() * self.pitch.sin() * self.yaw.sin());

        let _c31 = -self.pitch.sin();
        let _c32 =


        return DCM::new(
        )
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec_dot(){
        // Arbitrary Vector
        let vec = Vector::new(1.0, 2.0, 3.0);
        let vec2 = Vector::new(2.0, 1.0, 3.0);

        let vec_dot_vec2 = vec.dot(vec2);
        let vec2_dot_vec = vec2.dot(vec);

        // Eq 3.1.1-7, Pg3-8
        assert_eq!(
            vec_dot_vec2 - vec2_dot_vec < 1e-6,
            true
        )
    }

    #[test]
    fn vec_cross(){
        // Arbitrary Vector
        let vec = Vector::new(1.0, 2.0, 3.0);
        let vec2 = Vector::new(2.0, 1.0, 3.0);

        let vec_cross_vec2 = vec.cross(vec2);
        let vec2_cross_vec = vec2.cross(vec);

        let deviation_error = Vector::of(1e-6);

        // Eq 3.1.1-8
        assert_eq!(
            vec_cross_vec2 + vec2_cross_vec < deviation_error,
            true
        )
    }

    #[test]
    fn dcm_transpose(){
        // Arbitrary dcm
        let dcm = DCM::new(
            0.5, 0.2, 0.0,
            0.0, 1.0, 0.0,
            0.2, 0.0, 1.0,
        );
        let transpose_dcm = dcm.transpose();
        let identity_dcm = DCM::identity();
        let matmul = dcm * transpose_dcm;

        let deviation_error = DCM::of(1e-6);

        // Eq 3.1-12, with some floating poitn allowance
        assert_eq!(
            matmul - identity_dcm  < deviation_error,
            true
        )

    }
}