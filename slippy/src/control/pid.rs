use crate::sim::runtime::{Runtime, Save};

#[derive(
    Debug,
    Clone
)]

pub struct PID{
    pub setpoint: f64,
    error: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    p_term: f64,
    i_term: f64,
    d_term: f64,
    ouput: f64,
    last_error: f64
}

impl PID{
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        setpoint: f64
    ) -> PID{
        return PID{
            setpoint,
            error: 0.0,
            kp,
            ki,
            kd,
            p_term: 0.0,
            i_term: 0.0,
            d_term: 0.0,
            ouput: 0.0,
            last_error: 0.0
        }
    }

    pub fn ouput(&mut self, process_value: f64, dt: f64) -> f64{

        // Simple PID
        self.error = self.setpoint - process_value;
        self.p_term = self.kp * self.error;
        self.i_term += self.ki * self.error * dt;
        self.d_term = self.kd * (self.error - self.last_error / dt);
        self.last_error = self.error;

        self.ouput = self.p_term + self.i_term + self.d_term;
        return self.ouput
    }
}

impl Save for PID{
    fn save_data(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {

        runtime.add_or_set(format!(
            "{node_name}.setpoint [-]").as_str(),
            self.setpoint,
        );
        runtime.add_or_set(format!(
            "{node_name}.output [-]").as_str(),
            self.ouput,
        );
        runtime.add_or_set(format!(
            "{node_name}.error [-]").as_str(),
            self.error,
        );
    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {
        self.save_data(node_name, runtime);

        // Gains
        runtime.add_or_set(format!(
            "{node_name}.kp [-]").as_str(),
            self.kp,
        );
        runtime.add_or_set(format!(
            "{node_name}.ki [-]").as_str(),
            self.ki,
        );
        runtime.add_or_set(format!(
            "{node_name}.kd [-]").as_str(),
            self.kd,
        );

        // Indiviual Contributions
        runtime.add_or_set(format!(
            "{node_name}.p_term [-]").as_str(),
            self.p_term,
        );
        runtime.add_or_set(format!(
            "{node_name}.i_term [-]").as_str(),
            self.i_term,
        );
        runtime.add_or_set(format!(
            "{node_name}.d_term [-]").as_str(),
            self.d_term,
        );

    }
}