// ----------------------------------------------------------------------------
// Strapdown Analytics
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::Mul;
use derive_more;

// Crate
use super::{Vector3, Matrix3x3};

// ----------------------------------------------------------------------------
// Quaternions [3.24, Pg 3-38]
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
    derive_more::Div,
    derive_more::Neg
)]
pub struct Quaternion{
    // Eq 3.2.4-28, Pg 3-44
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl Quaternion{

    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Quaternion{
        return Quaternion{a, b, c, d}
    }

    pub fn of(num: f64) -> Quaternion{
        return Quaternion::new(num, num, num, num)
    }

    pub fn identity() -> Quaternion{
        return Quaternion::new(1.0, 0.0, 0.0, 0.0)
    }

    pub fn from_array(array: [f64; 4]) -> Quaternion{
        return Quaternion::new(array[0], array[1], array[2], array[3])
    }

    pub fn to_array(&self) -> [f64; 4]{
        return [self.a, self.b, self.c, self.d]
    }

    pub fn conjugate(&self) -> Quaternion{
        // Source:
        //    https://en.wikipedia.org/wiki/Quaternion
        return Quaternion::new(self.a, -self.b, -self.c, -self.d)
    }

    pub fn transform(self, vec: Vector3) -> Vector3{
        // w = uvu*
        let quat = (self * vec) * self.conjugate();
        return Vector3::new(quat.b, quat.c, quat.d)
    }

    pub fn derivative(self, vec: Vector3) -> Quaternion{
        // q_dot = q * w / 2.0
        return self * vec / 2.0
    }

    pub fn error(&self, target: Quaternion) -> Quaternion{
        return target * self.conjugate()
    }

    pub fn to_dcm(&self) -> Matrix3x3{
        let _c11 =
            self.a.powf(2.0)
            + self.b.powf(2.0)
            - self.c.powf(2.0)
            - self.d.powf(2.0);
        let _c12 = 2.0 * ((self.b * self.c) - (self.a * self.d));
        let _c13 = 2.0 * ((self.b * self.d) + (self.a * self.c));

        let _c21 = 2.0 * ((self.b * self.c) + (self.a * self.d));
        let _c22 =
            self.a.powf(2.0)
            - self.b.powf(2.0)
            + self.c.powf(2.0)
            - self.d.powf(2.0);
        let _c23 = 2.0 * ((self.c * self.d) - (self.a * self.b));

        let _c31 = 2.0 * ((self.b * self.d) - (self.a * self.c));
        let _c32 = 2.0 * ((self.c * self.d) + (self.a * self.b));
        let _c33 =
            self.a.powf(2.0)
            - self.b.powf(2.0)
            - self.c.powf(2.0)
            + self.d.powf(2.0);

        return Matrix3x3::new(
            _c11, _c12, _c13,
            _c21, _c22, _c23,
            _c31, _c32, _c33,
        )

    }

    pub fn to_euler(&self) -> Vector3{
        return self.to_dcm().to_euler()
    }

}

impl Mul<Vector3> for Quaternion{
    type Output = Quaternion;
    fn mul(self, vec: Vector3) -> Quaternion{
        // Eq 3.2.4-10, Pg 3-41 (Simplifed form)
        return Quaternion::new(
            (-self.b * vec.i) + (-self.c * vec.j) + (-self.d * vec.k),
            (self.a * vec.i) + (-self.d * vec.j) + (self.c * vec.k),
            (self.d * vec.i) + (self.a * vec.j) + (-self.b * vec.k),
            (-self.c * vec.i) + (self.b * vec.j) + (self.a * vec.k)
        )
    }
}

impl Mul<Quaternion> for Quaternion{
    type Output = Quaternion;
    fn mul(self, quat: Quaternion) -> Quaternion{
        // Eq 3.2.4-10, Pg 3-41
        return Quaternion::new(
          (self.a * quat.a) + (-self.b * quat.b) + (-self.c * quat.c) + (-self.d * quat.d),
          (self.b * quat.a) + (self.a * quat.b) + (-self.d * quat.c) + (self.c * quat.d),
          (self.c * quat.a) + (self.d * quat.b) + (self.a * quat.c) + (-self.b * quat.d),
          (self.d * quat.a) + (-self.c * quat.b) + (self.b * quat.c) + (self.a * quat.d)
        )
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {

    use super::*;
    use crate::test::almost_equal_array;

    // Math
    # [test]
    fn quat_90_transform(){
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let quat = Quaternion::new(
            0.7071067811865476,
            0.0,
            0.7071067811865475,
            0.0
        );

        let transformation = quat.transform(vec).to_array();
        almost_equal_array(
            &transformation,
            &[3.0, 2.0, -1.0]
        );
    }

    // Conversions

    #[test]
    fn quat_to_euler(){
        // Identity
        let quat = Quaternion::identity();

        almost_equal_array(
            &quat.to_euler().to_array(),
            &[0.0, 0.0, 0.0]
        );

    }

    #[test]
    fn quat_to_dcm(){
        // Identity
        let quat = Quaternion::identity();
        let dcm = Matrix3x3::identity();

        almost_equal_array(
            &quat.to_dcm().to_array(),
            &dcm.to_array()
        );

        let quat = Quaternion{
            a: 0.9641015011871702,
            b: 0.02351519745119192,
            c: 0.2506948010244541,
            d:0.0843056797421489
        };

        almost_equal_array(
            &quat.to_dcm().to_quat().to_array(),
            &quat.to_array()
        );
    }

    // Derivative

    #[test]
    fn quat_derivative_x(){
        let mut quat = Quaternion::identity();
        let rate = Vector3::new(0.1, 0.0, 0.0);

        let increment = 1e-6;
        let amount = (10.0 / increment) as usize;

        for _ in 0..amount{
            quat += quat.derivative(rate) * increment;
        }
        almost_equal_array(
            &quat.to_euler().to_array(),
            &[1.0, 0.0, 0.0]
        );


    }
    #[test]
    fn quat_derivative_y(){
        let mut quat = Quaternion::identity();
        let rate = Vector3::new(0.0, 0.1, 0.0);

        let increment = 1e-6;
        let amount = (10.0 / increment) as usize;

        for _ in 0..amount{
            quat += quat.derivative(rate) * increment;
        }
        almost_equal_array(
            &quat.to_euler().to_array(),
            &[0.0, 1.0, 0.0]
        );


    }
    #[test]
    fn quat_derivative_z(){
        let mut quat = Quaternion::identity();
        let rate = Vector3::new(0.0, 0.0, 0.1);

        let increment = 1e-6;
        let amount = (10.0 / increment) as usize;

        for _ in 0..amount{
            quat += quat.derivative(rate) * increment;
        }
        almost_equal_array(
            &quat.to_euler().to_array(),
            &[0.0, 0.0, 1.0]
        );


    }
}