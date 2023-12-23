use std::f64::consts::PI;

pub const PI_DOUBLE: f64 = 2.0 * PI;
pub const PI_THREE_HALFS: f64 = 3.0 * PI / 2.0;
pub const PI_HALF: f64 = PI / 2.0;
pub const PI_QUARTER: f64 = PI / 4.0;

pub mod d2;
pub use d2::Vector2;
pub use d2::Line2;
pub use d2::Circle;

pub mod d3;
pub use d3::Vector3;
pub use d3::Matrix3x3;
pub use d3::Quaternion;