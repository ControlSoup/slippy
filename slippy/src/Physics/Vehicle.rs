
use crate::Physics::Basic;

pub struct Vehicle {
    state: Basic::State,
    inputs: Basic::Inputs,
    mass_props: Basic::MassProperties
}

impl Vehicle {
    pub fn new(
        state: Basic::State,
        mass_props: Basic::MassProperties,
        inputs: Basic::Inputs
    )-> Vehicle {
        return Vehicle{
            state,
            mass_props,
            inputs
        }
    }

    pub fn get_deriviative(&mut self){

        // a = F / m
        let acceleration_mps2 =
            self.inputs.force_n / self.mass_props.mass_cg_kg;

        // grav


    }
}