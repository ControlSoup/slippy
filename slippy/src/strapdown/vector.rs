// ----------------------------------------------------------------------------
// Strapdown Analytics
// ----------------------------------------------------------------------------

// 3rd Party
use derive_more;

use super::{quaternion::Quaternion, matrix::Matrix3x3};

// Crate

// ----------------------------------------------------------------------------
// Vectors [3.1, pg 3-1]
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
pub struct Vector3{
    pub x: f64,
    pub y: f64,
    pub z: f64
}

// Pg 3-1
impl Vector3{

    pub fn new(x: f64, y: f64, z: f64) -> Vector3{
        return Vector3 {x,y,z}
    }

    pub fn of(num: f64) -> Vector3{
        return Vector3::new(num, num, num)
    }

    pub fn zeros() -> Vector3{
        return Vector3::of(0.0)
    }

    pub fn from_array(array: [f64; 3]) -> Vector3{
        return Vector3::new(array[0], array[1], array[2])
    }

    pub fn to_array(self) -> [f64; 3]{
        // Eq: 3.1-10, Pg 3-3
        return [self.x, self.y, self.z]
    }

    pub fn quat_form(self) -> Quaternion{
        // Eq 3.2.3.1-3, Pg 3-44
        return Quaternion::new(
            0.0, self.x, self.y, self.z
        )
    }

    pub fn norm(self) -> f64{
        // Eq: 3.1.1-4, Pg 3-8
        return (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0)).sqrt()
    }

    pub fn dot(self, vec: &Vector3) -> f64{
        // Eq 3.1.1-5, Pg 3-8
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)
    }

    pub fn cross(self, vec: &Vector3) -> Vector3{
        // Eq 3.1.1-6, Pg 3-8
        return Vector3::new(
            (self.y * vec.z) - (self.z * vec.y),
            (self.z * vec.x) - (self.x * vec.z),
            (self.x * vec.y) - (self.y * vec.x)
        )
    }

    pub fn error(self, target: Vector3) -> Vector3{
        return target - self
    }

    pub fn to_dcm(self) -> Matrix3x3{
        // Eq 3.2.3.1-1, Pg 3-33
        let _c11 = self.y.cos() * self.x.cos();
        let _c12 =
            (-self.z.cos() * self.x.sin())
            + (self.z.sin() * self.y.sin() * self.x.cos());
        let _c13 =
            (self.z.sin() * self.x.sin())
            + (self.z.cos() * self.y.sin() * self.x.cos());

        let _c21 = self.y.cos() * self.x.sin();
        let _c22 =
            (self.z.cos() * self.x.cos())
            + (self.z.sin() * self.y.sin() * self.x.sin());
        let _c23 =
            (-self.z.sin() * self.x.cos())
            + (self.z.cos() * self.y.sin() * self.x.sin());

        let _c31 = -self.y.sin();
        let _c32 = self.z.sin() * self.y.cos();
        let _c33 = self.z.cos() * self.y.cos();


        return Matrix3x3::new(
            _c11, _c12, _c12,
            _c21, _c22, _c23,
            _c31, _c32, _c33,
        )
    }

    pub fn to_quat(self)-> Quaternion{
        let cr = (self.x * 0.5).cos();
        let sr = (self.x * 0.5).sin();
        let cp = (self.y * 0.5).cos();
        let sp = (self.y * 0.5).sin();
        let cy = (self.z * 0.5).cos();
        let sy = (self.z * 0.5).sin();

        return Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        )

    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use crate::test::almost_equal_array;

    #[test]
    fn vec_dot(){
        // Arbitrary Vector3
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let vec2 = Vector3::new(2.0, 1.0, 3.0);

        let vec_dot_vec2 = vec.dot(&vec2);
        let vec2_dot_vec = vec2.dot(&vec);

        assert_relative_eq!(
            vec_dot_vec2,
            vec2_dot_vec,
            max_relative=1e-6
        )
    }

    #[test]
    fn vec_cross(){
        // Arbitrary Vector3
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let vec2 = Vector3::new(2.0, 1.0, 3.0);

        let vec_cross_vec2 = vec.cross(&vec2);
        let vec2_cross_vec = vec2.cross(&vec);

        // Eq 3.1.1-8, Pg 3-8s
        almost_equal_array(
            &vec_cross_vec2.to_array(),
            &(-vec2_cross_vec).to_array()
        )
    }

    // Euler conversion
    #[test]
    fn euler_to_dcm(){

        // Identity check
        let euler = Vector3::zeros();
        let dcm = Matrix3x3::identity();
        let euler_to_dcm = euler.to_dcm();
        almost_equal_array(
            &dcm.to_array(),
            &euler_to_dcm.to_array()
        )

    }

    #[test]
    fn euler_to_quat(){
        // Identity check
        let euler = Vector3::new(0.0, 0.0, 0.0);
        let quat = Quaternion::identity();

        almost_equal_array(
            &euler.to_quat().to_array(),
            &quat.to_array()
        )
    }
}