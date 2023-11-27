use crate::sim::{runtime::{Runtime, Save}};

#[derive(
    Debug,
    Clone
)]

// Verbose PID for gain tuning and debuggin
pub struct PID{
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
    ) -> PID{
        return PID{
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

    pub fn ouput(&mut self, error: f64, dt: f64) -> f64{

        // Simple PID
        self.error = error;
        self.p_term = self.kp * error;
        self.i_term += self.ki * error * dt;
        self.d_term = self.kd * (error - self.last_error / dt);
        self.last_error = error;

        self.ouput = self.p_term + self.i_term + self.d_term;
        return self.ouput
    }
}

impl Save for PID{
    fn save_data(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {
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

        runtime.add_or_set(format!(
            "{node_name}.output [-]").as_str(),
            self.ouput,
        );
        runtime.add_or_set(format!(
            "{node_name}.error [-]").as_str(),
            self.error,
        );
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