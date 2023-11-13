// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use nalgebra::{Matrix3, Vector4, Vector3, Matrix1x4, Matrix4};
use std::f64::consts::PI;

pub fn flip_4(vec: Vector4<f64>)-> Matrix1x4<f64>{
    return Matrix1x4::from_row_slice(vec.as_slice())
}

pub fn unflip_4(matrix: Matrix1x4<f64>)->Vector4<f64>{
    return Vector4::from_row_slice(matrix.as_slice())
}

// ----------------------------------------------------------------------------
// Chapter 3
// ----------------------------------------------------------------------------

//  EXTERNAL SOURCE
//  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
pub fn euler_rad_to_quat(
    euler_radps: Vector3<f64>
)-> Vector4<f64>{
    // Abbreviations for the various angular functions

    let cr = (euler_radps[0] * 0.5).cos();
    let sr = (euler_radps[0] * 0.5).sin();
    let cp = (euler_radps[1] * 0.5).cos();
    let sp = (euler_radps[1] * 0.5).sin();
    let cy = (euler_radps[2] * 0.5).cos();
    let sy = (euler_radps[2] * 0.5).sin();

    return Vector4::from_row_slice(&[
        (cr * cp * cy + sr * sp * sy),
        (sr * cp * cy - cr * sp * sy),
        (cr * sp * cy + sr * cp * sy),
        (cr * cp * sy - sr * sp * cy)
    ])

}

//  EXTERNAL SOURCE
//  https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
pub fn quat_to_euler_rad(
    quat: Vector4<f64>
)-> Vector3<f64>{

    // q1 can be non-normalised quaternion
    let mut euler = Vector3::zeros();
    let  sqw = quat[0] * quat[0];
    let  sqx = quat[1] * quat[1];
    let  sqy = quat[2] * quat[2];
    let  sqz = quat[3] * quat[3];
	let  unit = sqx + sqy + sqz + sqw;
	let  test = quat[1] * quat[2] + quat[3] * quat[0];

	if test > 0.499 * unit { // singularity at north pole
		euler[1] = 2.0 * quat[1].atan2(quat[0]);
		euler[0] = PI / 2.0;
		return euler
	}
	if test < -0.499 * unit { // singularity at south pole

		euler[1] = -2.0 * quat[1].atan2(quat[0]);
		euler[2] = -PI / 2.0;
		euler[0] = 0.0;
		return euler
	}

    euler[1] = (2.0 * quat[2] * quat[0] - 2.0 * quat[1] * quat[3]).atan2(sqx - sqy - sqz + sqw);
	euler[2] = (2.0 * test / unit).asin();
	euler[0] = (2.0 * quat[1] * quat[0]- 2.0 * quat[2] * quat[3]).atan2(-sqx + sqy - sqz + sqw);

    return euler
}

// Eq: 3.2.4-11, PG: 3-40
pub fn quat_matrix_form(
    four_vec: &Vector4<f64>
) -> Matrix4<f64>{

    let a = four_vec[0];
    let b = four_vec[1];
    let c = four_vec[2];
    let d = four_vec[3];

    return Matrix4::from_row_slice(&[
        a, -b, -c,  -d,
        b,  a, -d,   c,
        c,  d,  a,  -b,
        d, -c,  b,   a
    ])
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

// Eq: 3.2.4.1-3, Pg 3-44
pub fn to_quat_vec(
    vector: &Vector3<f64>
)-> Vector4<f64>{
    return Vector4::from_row_slice(&[
        0.0, vector[0], vector[1], vector[2]
    ])
}

// Eq: 3.2.4-15, Pg: 3-41
pub fn quat_conjugate(
    quat: &Vector4<f64>
) -> Matrix4<f64>{
    let a = quat[0];
    let b = quat[1];
    let c = quat[2];
    let d = quat[3];
    return Matrix4::from_row_slice(&[
         a,  b,  c,  d,
        -b,  a, -d,  c,
        -c,  d,  a, -b,
        -d, -c, b,   a,
    ])
}

// Eq 3.2.4-16 Pg: 3-44
pub fn quat_transformation(
    quat: &Vector4<f64>,
    vector: &Vector3<f64>
) -> Vector3<f64>{
    // w = uvu*
    // return = [quat_matrix][quat_vector][quat_conjugate]
    let new_quat_vec =
        flip_4(quat_matrix_form(quat) * to_quat_vec(vector))
        * quat_conjugate(quat);

    return Vector3::from_row_slice(&[
        new_quat_vec[1], new_quat_vec[2], new_quat_vec[3]
    ])
}

// Eq: 3.3.4-19 an Eq: 3.3.4, Pg: 3-62
pub fn quat_rate(
    current_quat: &Vector4<f64>,
    gyro_radps: &Vector3<f64>
) -> Vector4<f64>{

    // Small dt approximation for quaternion deriviative

    let quat_four_vec = quat_matrix_form(current_quat);

    return quat_four_vec * to_quat_vec(gyro_radps) / 2.0
}


#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn quat_conversions(){
        let quat = Vector4::from_row_slice(&[1.0, 0.0, 0.0, 0.0]);
        let euler = quat_to_euler_rad(quat.clone());

        assert_relative_eq!(
            quat,
            euler_rad_to_quat(euler),
            max_relative=1.0e-6
        );
        assert_relative_eq!(
            euler,
            quat_to_euler_rad(quat),
            max_relative=1.0e-6
        );
    }

    #[test]
    fn tranformations(){
        let vec = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);
        let quat = Vector4::from_row_slice(&[1.0, 0.0, 0.0, 0.0]);
        let transform =  quat_transformation(&quat, &vec);

        assert_relative_eq!(
            vec,
            transform,
            max_relative=1e-6
        )

    }
}