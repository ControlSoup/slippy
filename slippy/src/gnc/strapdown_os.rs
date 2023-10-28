// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use nalgebra::Matrix3;

// ----------------------------------------------------------------------------
// Chapter 3
// ----------------------------------------------------------------------------



// Eq: 3.3.3-4, Pg: 3.52
pub fn gryo_to_scewsym(wx: f64, wy: f64, wz: f64) -> Matrix3<f64>{
    return Matrix3::from_row_slice(&[
        0., -wz, wy,
        wz, 0., -wx,
        -wy, wx, 0.,
    ])
}

// Eq: 3.3.2-9, Pg: 3.53
pub fn dcm_rate(
    current_dcm: Matrix3<f64>,
    scew_symetric: Matrix3<f64>
) -> Matrix3<f64>{

   return current_dcm * scew_symetric;
}