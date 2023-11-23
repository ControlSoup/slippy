// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::Mul;
use derive_more;
use std::f64::consts::PI;

// Crate

use crate::strapdown::vector::Vector3;

// ----------------------------------------------------------------------------
// Direction Cosines [3.2.1] Pg 3-15
// ----------------------------------------------------------------------------

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    derive_more::Add,
    derive_more::AddAssign,
    derive_more::Sub,
    derive_more::SubAssign,
    derive_more::Mul,
    derive_more::Neg
)]
// Eq 3.2.1-1, Pg 3-15
pub struct Matrix3x3{
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

impl Matrix3x3{

    pub fn new(
        c11: f64, c12: f64, c13: f64,
        c21: f64, c22: f64, c23: f64,
        c31: f64, c32: f64, c33: f64,
    ) -> Matrix3x3{
        // Eq: 3.1-10, Pg 3-3
        return Matrix3x3 {
            c11, c12, c13,
            c21, c22, c23,
            c31, c32, c33
        }
    }

    pub fn of(num: f64) -> Matrix3x3{
        return Matrix3x3::new(
            num, num, num,
            num, num, num,
            num, num, num
        )
    }

    pub fn identity() -> Matrix3x3{
        return Matrix3x3::new(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        )
    }

    pub fn from_array(array: [f64; 9]) -> Matrix3x3{
        return Matrix3x3::new(
            array[0], array[1], array[2],
            array[3], array[4], array[5],
            array[6], array[7], array[8]
        )
    }

    pub fn norm(self) -> f64{
        return(
            self.c11.powf(2.0) + self.c12.powf(2.0) + self.c13.powf(2.0) + 
            self.c21.powf(2.0) + self.c22.powf(2.0) + self.c23.powf(2.0) + 
            self.c31.powf(2.0) + self.c32.powf(2.0) + self.c33.powf(2.0) 
        ).sqrt()
    }

    pub fn to_array(self) -> [f64; 9]{
        return [
            self.c11, self.c12, self.c13,
            self.c21, self.c22, self.c23,
            self.c31, self.c32, self.c33
        ]
    }

    pub fn to_euler(self) -> Vector3{
        // Eq 3.2.3.2-1, Pg 3-34
        let mut euler = Vector3::zeros();
        euler.y =
            (-self.c31 / (self.c32.powf(2.0) + self.c33.powf(2.0)).sqrt()).atan();

        if self.c31 < 0.999{
            euler.z = (self.c32 / self.c33).atan();
            euler.x = (self.c21 / self.c11).atan();
        } else if self.c31 <= 0.999{
            euler.x = ((self.c23 - self.c12) / (self.c13 + self.c22)).atan();
        } else{
            euler.x =
                PI + ((self.c23 + self.c21) / (self.c13 - self.c22)).atan();
        };

        return euler
    }

    pub fn transpose(self) -> Matrix3x3{
        // Source:
        //     https://en.wikipedia.org/wiki/Transpose
        return Matrix3x3::new(
            self.c11, self.c21, self.c31,
            self.c12, self.c22, self.c32,
            self.c13, self.c23, self.c33,
        )
    }

    pub fn transform(self, vec: Vector3) -> Vector3{
        return self * vec
    }

    pub fn derivative(self, vec: Vector3) -> Matrix3x3{
        // Cross product operator for the angular rate vector
        let scew_sym = Matrix3x3::new(
               0.0, -vec.z, vec.y,
             vec.z,    0.0, vec.x,
            -vec.y, -vec.x,   0.0,
        );

        return self * scew_sym
    }

}

impl Mul<Matrix3x3> for Matrix3x3{
    type Output = Matrix3x3;

    fn mul(self, b: Matrix3x3) -> Matrix3x3{
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

        return Matrix3x3::new(
            _c11, _c12, _c13,
            _c21, _c22, _c23,
            _c31, _c32, _c33,
        )

    }
}

impl Mul<Vector3> for Matrix3x3{
    // Eq 3.1.1-2, Pg 3-15
    type Output = Vector3;

    fn mul(self, vec: Vector3) -> Vector3{
        // 3x3 times a 3x1 matrix
        return Vector3::new(
            (self.c11 * vec.x) + (self.c12 * vec.y) + (self.c13 * vec.z),
            (self.c21 * vec.x) + (self.c22 * vec.y) + (self.c23 * vec.z),
            (self.c31 * vec.x) + (self.c32 * vec.y) + (self.c33 * vec.z)
        )
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::test::almost_equal_array; 

    #[test]
    fn dcm_transpose(){
        // Arbitrary dcm
        let dcm = Matrix3x3::identity();
        let transpose_dcm = dcm.transpose();
        let identity_dcm = Matrix3x3::identity();
        let matmul = dcm * transpose_dcm;


        // Eq 3.1-12, with some floating point allowance
        almost_equal_array(
            &matmul.to_array(),
            &identity_dcm.to_array()
        )

    }

    #[test]
    fn dcm_to_euler(){
        // Identity check
        let dcm = Matrix3x3::identity();
        let euler = Vector3::zeros();

        let dcm_to_euler = dcm.to_euler();

        almost_equal_array(
            &dcm_to_euler.to_array(),
            &euler.to_array()
        );
        
    }

    #[test]
    fn dcm_rate(){
        let mut dcm = Matrix3x3::identity();
        let rate = Vector3::new(0.1, 0.0, 0.0);
        
        let increment = 1e-6;
        let amount = (10.0 / increment) as usize;

        for _ in 0..amount{
            dcm += dcm.derivative(rate) * increment;
        }

        almost_equal_array(
            &dcm.to_euler().to_array(), 
            &[1.0, 0.0, 0.0]
        );
    }
}