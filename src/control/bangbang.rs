use crate::sim;

#[derive(
    Debug,
    Clone
)]

pub struct BangBang{
    pub setpoint: f64,
    error: f64,
    output: bool,
}

impl BangBang{
    pub fn new(
        setpoint: f64
    ) -> BangBang{
        return BangBang{
            setpoint,
            error: 0.0,
            output: false
        }
    }

    pub fn output(&mut self, process_value: f64, dt: f64) -> bool{

        // Simple Bang Bang
        self.error = self.setpoint - process_value;

        if self.error < 0.0{
            self.output = true
        } else{
            self.output = false
        }

        return self.output
    }

}

impl sim::Save for BangBang{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.error [-]").as_str(),
            self.error,
        );
    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.setpoint [-]").as_str(),
            self.setpoint,
        );
        runtime.add_or_set(format!(
            "{node_name}.output [-]").as_str(),
            self.output as u8 as f64,
        );

    }
}