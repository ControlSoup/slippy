use std::f64::consts::PI;

pub fn deg_to_rad(x: f64) -> f64{
    return   x * PI / 180.0
}

pub fn rad_to_deg(x: f64) -> f64{
    return   x * 180.0 / PI
}