use std::f64::consts::PI;

// MISC Functions and constants for 2d

pub const PI_THREE_HALFS:f64 = 3.0 * PI / 2.0;

pub fn length_from_points(x1: &f64, y1: &f64, x2: &f64, y2: &f64) -> f64{
    return (y2 - y1) / (x2 - x1)
}

pub fn angle_from_y_rad(x1: &f64, y1: &f64, x2: &f64, y2: &f64) -> f64{
    let y = y2 - y1;
    let x = x2 - x1;

    return y.atan2(x)
}