use ndarray::{Array1, Array2};
use ndarray_linalg::solve::Inverse;

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

#[allow(dead_code)]
#[derive(Debug)]
pub struct State{
    pub position_m: Array1<f64>,
    pub velocity_mps: Array1<f64>,
    pub cb2i: Array2<f64>,
    pub angular_rate_radps: Array1<f64>,
}

pub impl State{
    pub fn new(
        position_m: Array1<f64>,
        velocity_mps: Array1<f64>,
        cb2i: Array2<f64>,
        angular_rate_radps: Array1<f64>,
    ) -> State{
            return State {
                position_m,
                velocity_mps,
                cb2i,
                angular_rate_radps,
            }
    }

    pub fn zeros() -> State{
        return State {
            position_m: Array1::zeros(3),
            velocity_mps: Array1::zeros(3),
            cb2i: Array2::eye(3),
            angular_rate_radps: Array1::zeros(3),
        }
    }

    pub fn get_state() -> Array1{
        return Array1::new()
    }
}

// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct Inputs{
    pub force_n: Array1<f64>,
    pub moment_nm: Array1<f64>
}

pub impl Inputs{
    pub fn new(
        force_n: Array1<f64>,
        moment_nm: Array1<f64>,
    ) -> Inputs {
            return Inputs{
                force_n: force_n,
                moment_nm: moment_nm,
            }
    }

    pub fn zeros() -> Inputs{
        return Inputs{
            force_n: Array1::zeros(3),
            moment_nm: Array1::zeros(3),
        }
    }
}

// ----------------------------------------------------------------------------
// Mass Properties
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct MassProperties{
    pub mass_cg_kg: f64,
    pub i_tensor_cg_kgpm2: Array2<f64>
}

pub impl MassProperties{
    pub fn new(
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: Array2<f64>
    ) -> MassProperties{
        return MassProperties {
            mass_cg_kg: mass_cg_kg,
            i_tensor_cg_kgpm2: i_tensor_cg_kgpm2
        }
    }
}