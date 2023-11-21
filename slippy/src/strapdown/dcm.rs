// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::{Mul, Div, Add, Sub};
use std::cmp::PartialEq;
use std::vec;
use derive_more;
use std::f64::consts::PI;

// Crate

use crate::strapdown::vector::Vector;
use crate::strapdown::euler::Euler;

// ----------------------------------------------------------------------------
// Direction Cosines [3.2.1] Pg 3-15
// ----------------------------------------------------------------------------

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
// Eq 3.2.1-1, Pg 3-15
pub struct DCM{
    pub c11: f64,
    pub c12: f64,
    pub c13: f64,
    pub c21: f64,
    pub c22: f64,
    pub c23: f64,
    pub c31: f64,
    pub c32: f64,
    pub c33: f64,
}

impl DCM{

    pub fn new(
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

    pub fn of(num: f64) -> DCM{
        return DCM::new(
            num, num, num,
            num, num, num,
            num, num, num
        )
    }

    pub fn identity() -> DCM{
        return DCM::new(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        )
    }

    pub fn to_array(self) -> [[f64; 3]; 3]{
        return [
            [self.c11, self.c12, self.c13],
            [self.c21, self.c22, self.c23],
            [self.c31, self.c32, self.c33]
        ]
    }

    pub fn to_euler(self) -> Euler{
        // Eq 3.2.3.2-1, Pg 3-34
        let theta =
            (-self.c31 / (self.c32.powf(2.0) + self.c33.powf(2.0)).sqrt()).atan();

        let psi = 0.0;
        let phi = 0.0;

        if self.c31 < 0.999{
            let phi = (self.c32 / self.c33).atan();
            let psi = (self.c21 / self.c11).atan();
        } else if self.c31 <= 0.999{
            let psi = ((self.c23 - self.c12) / (self.c13 + self.c22)).atan();
        } else{
            let psi =
                PI + ((self.c23 + self.c21) / (self.c13 - self.c22)).atan();
        };

        return Euler::new(psi, theta, phi);
    }

    pub fn transpose(self) -> DCM{
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

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn dcm_to_euler(){

    }
}