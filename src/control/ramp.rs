use crate::sim::runtime::{Save,Runtime};

pub struct Ramp{
   pub target: f64,
   rate: f64,
   current_value: f64
}

impl Ramp{
    pub fn new(start_value: f64, target: f64, rate: f64) -> Ramp{
        return Ramp{
            target,
            rate,
            current_value: start_value
        }
    }

    pub fn output(&mut self, dt: f64) -> f64{
        if self.current_value > self.target{
            self.current_value =  self.current_value - (self.rate * dt)
        } else {
            self.current_value = self.current_value + (self.rate * dt)
        };

        return self.current_value
    }
}

impl Save for Ramp{
    fn save_data(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.current_value [-]").as_str(),
            self.current_value
        );
    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {
        self.save_data(node_name, runtime);
        runtime.add_or_set(format!(
            "{node_name}.target [-]").as_str(),
            self.target
        );
        runtime.add_or_set(format!(
            "{node_name}.rate [-]").as_str(),
            self.rate
        );

    }
}