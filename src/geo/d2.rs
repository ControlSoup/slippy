use std::ops::Mul;
use derive_more;

pub struct Point2{
    pub x: f64,
    pub y: f64
}


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
pub struct Vector2{
    pub i: f64,
    pub j: f64
}


impl Vector2{
    pub fn new(i: f64, j:f64) -> Vector2{
        return Vector2 {i, j}
    }

    pub fn from_points(x1: f64, y1: f64, x2: f64, y2: f64) -> Vector2{
        return Vector2::new(x2 - x1, y2 - y1)
    }

    pub fn from_angle_rad(norm:f64, angle_rad: f64) -> Vector2{
        return Vector2::new(
            norm * angle_rad.cos(),
            norm * angle_rad.sin()
        )
    }

    pub fn from_array(coords: [f64; 2]) -> Vector2{
        return Vector2::new(coords[0], coords[1])
    }

    pub fn to_array(&self) -> [f64; 2]{
        return [self.i, self.j]
    }

    pub fn to_unit(&self) -> Vector2{
        return self.clone() / self.norm()
    }

    pub fn get_perpendicular(&self) -> Vector2{
        // Returns a perpindicular vector
        return Vector2::new(self.j, -self.i)
    }

    pub fn dot(&self, vec: &Vector2) -> f64{
        return (self.i * vec.i) + (self.j * vec.j)
    }

    pub fn norm(&self) -> f64{
        return (self.i.powf(2.0) + self.j.powf(2.0)).sqrt()
    }

    pub fn norm_sqr(&self) -> f64{
        return self.norm().powf(2.0)
    }

    pub fn angle_rad(&self, vec: &Vector2) -> f64{
        return (self.dot(vec) / (self.norm() * vec.norm())).acos()
    }

    pub fn angle_x_rad(&self) -> f64{
        // Postive angle in radians from the i axis
        return (self.i / (self.norm() * 2.0_f64.sqrt())).acos()
    }

    pub fn angle_y_rad(&self) -> f64{
        // Postive angle in radians from the j axis
        return (self.j / (self.norm() * 2.0_f64.sqrt())).acos()
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn geometric_definitions(){
        // Source:
        //    https://en.wikipedia.org/wiki/Dot_product

        // a • b = 0 for orthogonal

        let a = Vector2::new(1.0, 0.0);
        let b = Vector2::new(0.0, 1.0);

        assert_relative_eq!(
            a.dot(&b),
            0.0,
            max_relative = 1e-6
        );

        // a • a = ||a||^2
        assert_relative_eq!(
            a.dot(&a),
            a.norm().powf(2.0),
            max_relative = 1e-6
        );
    }

    #[test]
    fn from_example(){
        // Source:
        // https://www.calculatorsoup.com/calculators/algebra/dot-product-calculator.php

        let a = Vector2::new(1.0, 2.0);
        let b = Vector2::new(6.0, 7.0);
        // a • a = ||a||^2
        assert_relative_eq!(
            a.dot(&b),
            20.0,
            max_relative = 1e-6
        );
    }

    #[test]
    fn unit_circle_Q1(){
        let angle = PI / 4.0;
        let a = Vector2::from_angle_rad(1.0, angle);

        // Unit circle
        assert_relative_eq!(
            a.norm(),
            1.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.i,
            2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.j,
            2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
    }

    #[test]
    fn unit_circle_Q2(){
        let angle = 3.0 * PI / 4.0 ;
        let a = Vector2::from_angle_rad(1.0, angle);

        // Unit circle
        assert_relative_eq!(
            a.norm(),
            1.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.i,
            -2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.j,
            2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
    }

    #[test]
    fn unit_circle_Q3(){
        let angle = -3.0 * PI / 4.0 ;
        let a = Vector2::from_angle_rad(1.0, angle);

        // Unit circle
        assert_relative_eq!(
            a.norm(),
            1.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.i,
            -2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.j,
            -2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
    }

    #[test]
    fn unit_circle_Q4(){
        let angle = - PI / 4.0 ;
        let a = Vector2::from_angle_rad(1.0, angle);

        // Unit circle
        assert_relative_eq!(
            a.norm(),
            1.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.i,
            2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.j,
            -2.0_f64.sqrt() / 2.0,
            max_relative = 1e-6
        );
    }

    #[test]
    fn ops(){
        let a = Vector2::new(1.0, 2.0);
        let b = Vector2::new(3.0, 2.0);
        let c = a + b;

        // Elment wise operations
        assert_relative_eq!(
            c.i,
            4.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            c.j,
            4.0,
            max_relative = 1e-6
        );


    }
}