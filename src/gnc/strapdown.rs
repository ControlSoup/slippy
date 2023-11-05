// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use nalgebra::{Matrix3, Vector4, Vector3, Matrix1x4};

// ----------------------------------------------------------------------------
// Chapter 3
// ----------------------------------------------------------------------------

//  EXTERNAL SOURCE
pub fn euler_radps_to_quat(
    euler_radps: Vector3<f64>
)-> Vector4<f64>{
    let qx =
        (euler_radps[0] / 2.0).sin() *
        (euler_radps[1] / 2.0).cos() *
        (euler_radps[2] / 2.0).cos() -
        (euler_radps[0] / 2.0).cos() *
        (euler_radps[1] / 2.0).sin() *
        (euler_radps[2] / 2.0).sin();

    let qy =
        (euler_radps[0] / 2.0).cos() *
        (euler_radps[1] / 2.0).sin() *
        (euler_radps[2] / 2.0).cos() +
        (euler_radps[0] / 2.0).sin() *
        (euler_radps[1] / 2.0).cos() *
        (euler_radps[2] / 2.0).sin();

    let qz =
        (euler_radps[0] / 2.0).cos() *
        (euler_radps[1] / 2.0).cos() *
        (euler_radps[2] / 2.0).sin() -
        (euler_radps[0] / 2.0).sin() *
        (euler_radps[1] / 2.0).sin() *
        (euler_radps[2] / 2.0).cos();

    let qw =
        (euler_radps[0] / 2.0).cos() *
        (euler_radps[1] / 2.0).cos() *
        (euler_radps[2] / 2.0).cos() +
        (euler_radps[0] / 2.0).sin() *
        (euler_radps[1] / 2.0).sin() *
        (euler_radps[2] / 2.0).sin();

    return Vector4::from_row_slice(
        &[qx, qy, qz, qw]
    )
}

// Eq: 3.3.3-4, Pg: 3-52
pub fn gryo_to_scewsym(wx: f64, wy: f64, wz: f64) -> Matrix3<f64>{
    return Matrix3::from_row_slice(&[
        0., -wz, wy,
        wz, 0., -wx,
        -wy, wx, 0.,
    ])
}

// Eq: 3.3.2-9, Pg: 3-53
pub fn dcm_rate(
    current_dcm: Matrix3<f64>,
    scew_symetric: Matrix3<f64>
) -> Matrix3<f64>{

   return current_dcm * scew_symetric;
}

// Eq: 3.3.4-13, Pg: 3-62
pub fn quat_rate(
    current_quat: Vector4<f64>,
    gyro_radps: Vector3<f64>
) -> Vector4<f64>{

    let four_vec: Matrix1x4<f64> = Matrix1x4::from_row_slice(
        &[0.0, gyro_radps[0], gyro_radps[1], gyro_radps[2]]
    );

    let quat_dot =  current_quat * four_vec / 2.0;

    return Vector4::from_row_slice(
        &[quat_dot[0], quat_dot[1], quat_dot[2], quat_dot[3]]
    )
}