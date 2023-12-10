use rand_distr::{Normal, Distribution};
use rand::thread_rng;

use crate::sim;


pub struct BasicSensor{
    std: f64,
    measured_value: f64,
    output_slope: f64,
    output_offset: f64
}

impl BasicSensor{
    pub fn new_std(
        std: f64,
        output_slope: f64,
        output_offset: f64
    ) -> BasicSensor{
        return BasicSensor {
            std,
            measured_value: 0.0,
            output_slope,
            output_offset
        }
    }

    pub fn new_simple_from_std(std:f64) -> BasicSensor{
        return BasicSensor::new_std(std, 1.0, 0.0)
    }

    pub fn new_simple_from_variance(variance: f64) -> BasicSensor{
        return BasicSensor::new_std(variance.sqrt(), 1.0, 0.0)
    }

    pub fn output(&mut self, actual_value: f64) -> f64{
        let distr = Normal::new(actual_value, self.std).expect(
            "Could not create normal distribution from BasicSensor output"
        );
        self.measured_value =  distr.sample(&mut thread_rng());

        return self.measured_value
    }
}

impl sim::Save for BasicSensor{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.measured_value [-]").as_str(),
            self.measured_value
        );
    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        self.save_data(node_name, runtime);

        runtime.add_or_set(format!(
            "{node_name}.std [-]").as_str(),
            self.std
        );
        runtime.add_or_set(format!(
            "{node_name}.output_slope [-]").as_str(),
            self.output_slope
        );
        runtime.add_or_set(format!(
            "{node_name}.output_offset [-]").as_str(),
            self.output_offset
        );
    }
}
