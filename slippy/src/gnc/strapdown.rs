// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use nalgebra::{Matrix3, Vector3, Matrix4, Matrix4x1, Quaternion};
use std::f64::consts::PI;

// Convention constants

pub const X: usize = 0;
pub const Y: usize = 1;
pub const Z: usize = 2;
pub const W: usize = 3;

// ----------------------------------------------------------------------------
// Chapter 3
// ----------------------------------------------------------------------------
//  EXTERNAL SOURCE
//  https://en.wikipedia.org/wiki/Conversion_between_UnitQuaternions_and_Euler_angles

pub fn euler_rad_to_quat(
    euler_radps: Vector3<f64>
)-> Quaternion<f64>{
    // Abbreviations for the various angular functions

    let cr = (euler_radps[X] * 0.5).cos();
    let sr = (euler_radps[X] * 0.5).sin();
    let cp = (euler_radps[Y] * 0.5).cos();
    let sp = (euler_radps[Y] * 0.5).sin();
    let cy = (euler_radps[Z] * 0.5).cos();
    let sy = (euler_radps[Z] * 0.5).sin();

    return Quaternion::new(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    )

}

//  EXTERNAL SOURCE
//  https://www.euclideanspace.com/maths/geometry/rotations/conversions/UnitQuaternionToEuler/
pub fn quat_to_euler_rad(
    quat: Quaternion<f64>
)-> Vector3<f64>{

    // q1 can be non-normalised Quaternion
    let mut euler = Vector3::zeros();
    let  sqw = quat[W] * quat[W];
    let  sqx = quat[X] * quat[X];
    let  sqy = quat[Y] * quat[Y];
    let  sqz = quat[Z] * quat[Z];
	let  unit = sqx + sqy + sqz + sqw;
	let  test = quat[X] * quat[Y] + quat[Z] * quat[W];

	if test > 0.499 * unit { // singularity at north pole
		euler[Y] = 2.0 * quat[X].atan2(quat[W]);
		euler[X] = PI / 2.0;
		return euler
	}
	if test < -0.499 * unit { // singularity at south pole

		euler[Y] = -2.0 * quat[X].atan2(quat[W]);
		euler[Z] = -PI / 2.0;
		euler[X] = 0.0;
		return euler
	}

    euler[Y] = (2.0 * quat[Y] * quat[W] - 2.0 * quat[X] * quat[Z]).atan2(sqx - sqy - sqz + sqw);
	euler[Z] = (2.0 * test / unit).asin();
	euler[X] = (2.0 * quat[X] * quat[W]- 2.0 * quat[Y] * quat[Z]).atan2(-sqx + sqy - sqz + sqw);

    return euler
}

// Eq: 3.2.4.1-4, PG: 3-44
pub fn quat_matrix_form(
    quat: &Quaternion<f64>
) -> Matrix4<f64>{

    let a = quat[W];
    let b = quat[X];
    let c = quat[Y];
    let d = quat[Z];

    return Matrix4::new(
        a, -b, -c,  -d,
        b,  a, -d,   c,
        c,  d,  a,  -b,
        d, -c,  b,   a
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

// Eq: 3.2.4.1-3, Pg 3-44
pub fn to_quat_vec(
    vector: &Vector3<f64>
)-> Matrix4x1<f64>{
    return Matrix4x1::new(0.0, vector[0], vector[1], vector[2])
}

// Eq 3.2.4-16 Pg: 3-44
pub fn quat_transformation(
    quat: &Quaternion<f64>,
    vector: &Vector3<f64>
) -> Vector3<f64>{
    // w = uvu*
    // return = [quat_matrix][quat_vector][quat_conjugate]
    let uv = Quaternion::from_vector(
        quat_matrix_form(quat) * to_quat_vec(vector)
    );
    let uvu_star = quat_matrix_form(&uv) * quat.conjugate().as_vector();

    return Vector3::new(uvu_star[X], uvu_star[Y], uvu_star[Z])
}


// Eq: 3.3.4-19 an Eq: 3.3.4, Pg: 3-62
pub fn quat_rate(
    current_quat: &Quaternion<f64>,
    gyro_radps: &Vector3<f64>
) -> Quaternion<f64>{

    // Small dt approximation for Quaternion deriviative

    return Quaternion::from_vector(
        quat_matrix_form(current_quat) * to_quat_vec(gyro_radps) / 2.0
    )
}


#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn quat_conversions(){
        let quat = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let euler = Vector3::new(0.0, 0.0, 0.0);

        assert_relative_eq!(
            euler,
            quat_to_euler_rad(quat),
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
        let vec = Vector3::new(1.0, 1.0, 1.0);
        let quat = Quaternion::identity();
        let transform =  quat_transformation(&quat, &vec);

        assert_relative_eq!(
            vec,
            transform,
            max_relative=1e-6
        )

    }
}