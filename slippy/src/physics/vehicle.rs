use crate::physics::basic::{State, MassProperties, Inputs};
use crate::sim::integration::Integrate;

pub struct Vehicle {
    state: State,
    inputs: Inputs,
    mass_props: MassProperties
}

impl Vehicle {
    pub fn new(
        state: State,
        mass_props: MassProperties,
        inputs: Inputs
    )-> Vehicle {
        return Vehicle{
            state,
            mass_props,
            inputs
        }
    }

    pub fn init() -> Vehicle{
        return Vehicle::new(
            State.init(),
            MassProperties.init(),
            Inputs.init()
        )
    }
}