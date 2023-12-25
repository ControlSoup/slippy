// ----------------------------------------------------------------------------
// Strapdown Analytics
// ----------------------------------------------------------------------------

// 3rd Party
use derive_more;

use crate::geo::{Vector2, self};

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
    pub i: f64,
    pub j: f64,
    pub k: f64
}

// Pg 3-1
impl Vector3{

    pub fn new(i: f64, j: f64, k: f64) -> Vector3{
        return Vector3 {i,j,k}
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

    pub fn from_spherical(
        r: f64, 
        theta: f64, 
        phi: f64
    ) -> Vector3{
        // Alternative spherical coordinates based on x and y axis
        // Source: https://en.wikipedia.org/wiki/Spherical_coordinate_system
        return Vector3::new(
            r * theta.sin() * phi.cos(),
            r * theta.sin() * phi.sin(),
            r * theta.cos(),
        )
    }

    pub fn to_array(self) -> [f64; 3]{
        // Eq: 3.1-10, Pg 3-3
        return [self.i, self.j, self.k]
    }

    pub fn to_unit(self) -> Vector3{
        return self.clone() / self.norm()
    }

    pub fn to_dcm(self) -> Matrix3x3{
        // Eq 3.2.3.1-1, Pg 3-33
        let _c11 = self.j.cos() * self.i.cos();
        let _c12 =
            (-self.k.cos() * self.i.sin())
            + (self.k.sin() * self.j.sin() * self.i.cos());
        let _c13 =
            (self.k.sin() * self.i.sin())
            + (self.k.cos() * self.j.sin() * self.i.cos());

        let _c21 = self.j.cos() * self.i.sin();
        let _c22 =
            (self.k.cos() * self.i.cos())
            + (self.k.sin() * self.j.sin() * self.i.sin());
        let _c23 =
            (-self.k.sin() * self.i.cos())
            + (self.k.cos() * self.j.sin() * self.i.sin());

        let _c31 = -self.j.sin();
        let _c32 = self.k.sin() * self.j.cos();
        let _c33 = self.k.cos() * self.j.cos();


        return Matrix3x3::new(
            _c11, _c12, _c12,
            _c21, _c22, _c23,
            _c31, _c32, _c33,
        )
    }

    pub fn to_quat(self)-> Quaternion{
        return self.to_dcm().to_quat()
    }

    pub fn quat_form(self) -> Quaternion{
        // Eq 3.2.3.1-3, Pg 3-44
        return Quaternion::new(
            0.0, self.i, self.j, self.k
        )
    }

    pub fn norm(self) -> f64{
        // Eq: 3.1.1-4, Pg 3-8
        return (self.i.powf(2.0) + self.j.powf(2.0) + self.k.powf(2.0)).sqrt()
    }

    pub fn dot(self, vec: &Vector3) -> f64{
        // Eq 3.1.1-5, Pg 3-8
        return (self.i * vec.i) + (self.j * vec.j) + (self.k * vec.k)
    }

    pub fn cross(self, vec: &Vector3) -> Vector3{
        // Eq 3.1.1-6, Pg 3-8
        return Vector3::new(
            (self.j * vec.k) - (self.k * vec.j),
            (self.k * vec.i) - (self.i * vec.k),
            (self.i * vec.j) - (self.j * vec.i)
        )
    }

    pub fn error(self, target: Vector3) -> Vector3{
        return target - self
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
        );
    }

    #[test]
    fn euler_to_quat(){
        // Identity check
        let euler = Vector3::new(0.0, 0.0, 0.0);
        let quat = Quaternion::identity();

        almost_equal_array(
            &euler.to_quat().to_array(),
            &quat.to_array()
        );
    }

    #[test]
    fn from_spherical(){
        // Identity check
        let vector3 = Vector3::from_spherical(1.0, 0.0, 0.0);

        almost_equal_array(
            &vector3.to_array(),
            &[0.0, 0.0, 1.0]
        )
    }
}