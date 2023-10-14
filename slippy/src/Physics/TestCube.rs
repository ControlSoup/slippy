use ndarray::Array1;

use crate::Physics::Basic;
use crate::RunTime::Integration;

struct TestCube{
    state: Basic::State,
    inputs: Basic::Inputs,
    mass_props: Basic::MassProperties
}

impl Default for TestCube{
    fn default() -> Self{
        return Self{
            state: Basic::State::zeros(),
            inputs: Basic::Inputs::zeros(),
            mass_props: Basic::MassProperties::zeros()
        }
    }
}

impl Integration::Integrate for TestCube{
    fn get_deriviative(&self) -> Array1<f64>{
        let velocity_mps: Array1<f64> = self.state.velocity_mps;

        // a = f / m
        let acceleration_mps2 =
            self.inputs.force_n / self.mass_props.mass_cg_kg;

        return velocity_mps
    }

    fn update() {

    }
}