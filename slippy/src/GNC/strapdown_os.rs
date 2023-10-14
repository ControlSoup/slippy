// ----------------------------------------------------------------------------
// Strapdown Analytics for OS (not embedded systems)
// ----------------------------------------------------------------------------

use ndarray::Array2;
use ndarray::Dot;

// ----------------------------------------------------------------------------
// Chapter 3
// ----------------------------------------------------------------------------

// Eq: 3.3.3-4, Pg: 3.52
fn gryo_to_dcm(wx: f64, wy: f64, yz: f64){
    return arr2([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0],
    ])
}

// Eq: 3.3.2-9, Pg: 3.53
fn dcm_rate(current_dcm: Array2, scew_symetric: Array2){
    return Dot::dot(current_dcm, scew_symetric)
}