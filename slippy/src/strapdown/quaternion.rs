// ----------------------------------------------------------------------------
// Strapdown Analytics
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::Mul;
use derive_more;
use std::f64::consts::PI;

// Crate
use super::{vector::Vector3, matrix::Matrix3x3};

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
        let _c12 = 2.0 * (self.b * self.c - (self.a * self.d));
        let _c13 = 2.0 * (self.b * self.d + (self.a * self.c));

        let _c21 = 2.0 * (self.b * self.c + (self.a * self.d));
        let _c22 =
            self.a.powf(2.0)
            - self.b.powf(2.0)
            + self.c.powf(2.0)
            - self.d.powf(2.0);
        let _c23 = 2.0 * (self.c * self.d - (self.a * self.d));

        let _c31 = 2.0 * (self.b * self.d - (self.a * self.c));
        let _c32 = 2.0 * (self.c * self.d + (self.a * self.b));
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
        let mut euler = Vector3::zeros();
        let  sqw = self.a * self.a;
        let  sqx = self.b * self.b;
        let  sqy = self.c * self.c;
        let  sqz = self.d * self.d;
        let  unit = sqx + sqy + sqz + sqw;
        let  test = self.b * self.c + self.d * self.a;

        if test > 0.499 * unit { // singularity at north pole
            euler.y = 2.0 * self.b.atan2(self.a);
            euler.x = PI / 2.0;
            return euler
        }
        if test < -0.499 * unit { // singularity at south pole
            euler.y = -2.0 * self.b.atan2(self.a);
            euler.z = - PI / 2.0;
            euler.x = 0.0;
            return euler
        }

        euler.y = (2.0 * self.c * self.a - 2.0 * self.b * self.d).atan2(sqx - sqy - sqz + sqw);
        euler.z = (2.0 * test / unit).asin();
        euler.x = (2.0 * self.b * self.a - 2.0 * self.c * self.d).atan2(-sqx + sqy - sqz + sqw);

        return euler
    }

}

impl Mul<Vector3> for Quaternion{
    type Output = Quaternion;
    fn mul(self, vec: Vector3) -> Quaternion{
        // Eq 3.2.4-10, Pg 3-41 (Simplifed form)
        return Quaternion::new(
            (-self.b * vec.x) + (-self.c * vec.y) + (-self.d * vec.z),
            (self.a * vec.x) + (-self.d * vec.y) + (self.c * vec.z),
            (self.d * vec.x) + (self.a * vec.y) + (-self.b * vec.z),
            (-self.c * vec.x) + (self.b * vec.y) + (self.a * vec.z)
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

        // From example
        // https://www.andre-gaschler.com/rotationconverter/

        let quat = Quaternion::new(
            0.25, 2.0, -0.2, 0.442699
        );
        almost_equal_array(
            &quat.to_euler().to_array(),
            &[2.7354038, 0.8700035, -2.7020736]
        )

    }

    #[test]
    fn quat_to_dcm(){
        // Identity
        let quat = Quaternion::identity();
        let dcm = Matrix3x3::identity();

        almost_equal_array(
            &quat.to_dcm().to_array(),
            &dcm.to_array()
        )
    }

    // Derivative

    #[test]
    fn quat_derivative(){
        let mut quat = Quaternion::identity();

        let increment = 1e-6;
        let amount = (10.0 / increment) as usize;

        for _ in 0..amount{
            let rate = quat.transform(Vector3::new(0.1, 0.0, 0.0));
            quat += quat.derivative(rate) * increment;
        }
        almost_equal_array(
            &quat.to_euler().to_array(),
            &[1.0, 0.0, 0.0]
        );


    }
}