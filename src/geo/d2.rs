use std::f64::consts::PI;
use derive_more;

use super::PI_DOUBLE;

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

// ----------------------------------------------------------------------------
// Vector
// ----------------------------------------------------------------------------

impl Vector2{
    pub fn new(i: f64, j:f64) -> Vector2{
        return Vector2 {i, j}
    }

    pub fn unit() -> Vector2{
        return Vector2::new(0.0, 0.0)
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

    pub fn to_line2(&self, start_x_m: f64, start_y_m: f64) -> Line2{
        return Line2::new(
            start_x_m,
            start_y_m,
            start_x_m + self.i,
            start_y_m + self.j
        )
    } 

    pub fn to_line2_from_origin(&self) -> Line2{
        return Line2::new(
            0.0,
            0.0,
            self.i,
            self.j
        )
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
        return  self.angle_rad(&Vector2::new(1.0, 0.0))
    }

    pub fn angle_y_rad(&self) -> f64{
        // Postive angle in radians from the j axis
        return self.angle_rad(&Vector2::new(0.0, 1.0))
    }
}


// ----------------------------------------------------------------------------
// Line2
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
pub struct Line2{
    pub start_x_m: f64,
    pub start_y_m: f64,
    pub end_x_m: f64,
    pub end_y_m: f64
}

impl Line2{
    pub fn new(start_x_m: f64, start_y_m:f64, end_x_m: f64, end_y_m: f64) -> Line2{
        return Line2{
            start_x_m,
            start_y_m,
            end_x_m,
            end_y_m,
        }
    }

    pub fn unit() -> Line2{
        return Line2::new_from_origin(1.0, 1.0)
    }

    pub fn new_from_origin(end_x_m: f64, end_y_m: f64) -> Line2{
        return Line2::new(0.0, 0.0, end_x_m, end_y_m)
    }

    pub fn from_array(array: [f64; 4]) -> Line2{
        return Line2::new(array[0], array[1], array[2], array[3])
    }

    pub fn from_angle_rad(start_x_m: f64, start_y_m: f64, length_m: f64, angle_rad:f64) -> Line2{
        let vector2 = Vector2::from_angle_rad(length_m, angle_rad);

        return Line2::from_vector2(start_x_m, start_y_m, vector2)
    }


    pub fn from_vector2(start_x_m: f64, start_y_m: f64, vector2: Vector2) -> Line2{
        return Line2::new(
            start_x_m, 
            start_y_m, 
            start_x_m + vector2.i, 
            start_y_m + vector2.j
        )
    }

    pub fn to_array(&self) -> [f64; 4]{
        return [self.start_x_m, self.start_y_m, self.end_x_m, self.end_y_m]
    }

    pub fn to_vector2(&self) -> Vector2{
        return Vector2::from_points(
            self.start_x_m, 
            self.start_y_m, 
            self.end_x_m, 
            self.end_y_m
        )
    }

    pub fn length_m(&self) -> f64{
        return self.to_vector2().norm()
    }

    pub fn angle_rad(&self, line22: &Line2) -> f64{
        return self.to_vector2().angle_rad(&line22.to_vector2())
    }

    pub fn angle_x_rad(&self) -> f64{
        return self.to_vector2().angle_x_rad()
    }

    pub fn angle_y_rad(&self) -> f64{
        return self.to_vector2().angle_y_rad()
    }

}

// ----------------------------------------------------------------------------
// Circle 
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
pub struct Circle{
    pub center_x_m: f64,
    pub center_y_m: f64,
    pub radius_m: f64
}

impl Circle{
    pub fn new(center_x_m: f64, center_y_m: f64, radius_m: f64) -> Circle{

        let mut radius_m = radius_m;
        if radius_m < 0.0{
            radius_m = -radius_m;
        }
        else if radius_m == 0.0{
            panic!("ERROR| radius_m cannont be 0.0")
        }

        return Circle{
            center_x_m,
            center_y_m,
            radius_m 
        }
    }

    pub fn unit() -> Circle{
        return Circle::new(0.0, 0.0, 1.0)
    }

    pub fn from_array(array: [f64; 3]) -> Circle{
        return Circle::new(array[0], array[1], array[2])
    }

    pub fn from_vector2(vector2: Vector2) -> Circle{
        return Circle::new(
            0.0,
            0.0,
            vector2.norm()
        )
    }

    pub fn center_to_vector2(&self) -> Vector2{
        return Vector2::new(
            self.center_x_m,
            self.center_y_m
        )
    } 

    pub fn center_to_line2(&self) -> Line2{
        return Line2::new_from_origin(
            self.center_x_m,
            self.center_y_m
        )
    }

    pub fn diameter_m(&self) -> f64{
        return self.radius_m * 2.0
    }

    pub fn area(&self) -> f64{
        return self.radius_m.powf(2.0) * PI 
    }

    pub fn circumference(&self) -> f64{
        return PI_DOUBLE * self.radius_m
    }

    pub fn intersect_circle(&self, circle2: &Circle) -> Option<Vector2>{

        // Source:
        // Intersection of Linear and Circular Components in 2D David Eberly
        // Geometric Tools, Redmond WA 98052
        let r0 = self.radius_m;
        let r1 = circle2.radius_m;


        let u = circle2.center_to_vector2() - self.center_to_vector2();
        let v = u.get_perpendicular();
        let s = (((r0.powf(2.0) - r1.powf(2.0)) / u.norm_sqr()) + 1.0) / 2.0;
        let t = ((r0.powf(2.0) / u.norm_sqr()) - s.powf(2.0)).sqrt();

        // Edge cases 
        if u.norm() <= (r0 + r1) && u.norm() >= (r0 - r1) {
            // If |U| = |r0 + r1| there is one solution and c1 is outside c0
            // If |U| = |r0 - r1| there is one solution and c1 is inside c0
            return Some(self.center_to_vector2() + (u * s) + (v * t));
        }

        return None
    
    }

}


// ----------------------------------------------------------------------------
// Vector Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use crate::test::almost_equal_array;

    use super::*;
    use crate::geo::PI_QUARTER;
    use approx::assert_relative_eq;

    #[test]
    fn vector2_geometric_definitions(){
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
    fn vector2_from_example(){
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
    fn vector2_unit_circle_Q1(){
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
    fn vector2_unit_circle_Q2(){
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
    fn vector2_unit_circle_Q3(){
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
    fn vector2_unit_circle_Q4(){
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
    fn vector2_angle_check(){
        let a = Vector2::new(1.0, 1.0);
        let b = Vector2::new(0.0, 1.0);

        assert_relative_eq!(
            a.angle_x_rad(),
            PI_QUARTER
        );
        assert_relative_eq!(
            a.angle_y_rad(),
            PI_QUARTER
        );
        assert_relative_eq!(
            a.angle_rad(&b),
            PI_QUARTER
        );
    }

    #[test]
    fn vecto2_to_line2(){
        let a = Vector2::new(1.0, 1.0);

        almost_equal_array(
            &a.to_line2(1.0, 1.0).to_array(),
            &[1.0,1.0,2.0,2.0] 
        );

    } 

// ----------------------------------------------------------------------------
// Line2 Tests
// ----------------------------------------------------------------------------

    #[test]
    fn new_line2_check(){

        let a = Line2::new_from_origin(1.0, 0.0);

        assert_relative_eq!(
            a.start_x_m,
            0.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.start_y_m,
            0.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.end_x_m,
            1.0,
            max_relative = 1e-6
        );
        assert_relative_eq!(
            a.end_y_m,
            0.0,
            max_relative = 1e-6
        );
    }

    # [test]
    fn line2_to_vector2(){
        let a = Line2::new(1.0, 1.0, 2.0, 2.0);

        almost_equal_array(
            &a.to_vector2().to_array(),
            &[1.0, 1.0] 
        );
    }

    #[test]
    fn line2_angle_check(){
        let a = Line2::new_from_origin(1.0, 1.0);
        let b = Line2::new_from_origin(0.0, 1.0);

        assert_relative_eq!(
            a.angle_x_rad(),
            PI_QUARTER
        );
        assert_relative_eq!(
            a.angle_y_rad(),
            PI_QUARTER
        );
        assert_relative_eq!(
            a.angle_rad(&b),
            PI_QUARTER
        );
    }

// ----------------------------------------------------------------------------
// Circle Tests
// ----------------------------------------------------------------------------
    # [test]
    fn circle2_new(){
        let a = Circle::new(0.5, 0.5, 1.0);

        assert_relative_eq!(
            a.center_x_m,
            0.5
        );
        assert_relative_eq!(
            a.center_y_m,
            0.5
        );
        assert_relative_eq!(
            a.radius_m,
            1.0
        );
    }

    # [test]
    fn circle2_center_vector(){
        let a = Circle::new(10.0, -5.0, 1.0);

        let center = a.center_to_vector2();
        assert_relative_eq!(
            center.i,
            10.0
        );
        assert_relative_eq!(
            center.j,
            -5.0
        );

    }

    # [test]
    fn circle2_intersect(){
        let a = Circle::new(0.5, 0.0, 1.0);
        let b = Circle::new(-0.5, 0.0, 1.0);

        let intersect = a.intersect_circle(&b).unwrap();

        assert_relative_eq!(
            intersect.j, 
            0.866,
            max_relative=1e-2 
        );
        assert_relative_eq!(
            intersect.i, 
            0.0,
            max_relative=1e-2 
        );

    }
}