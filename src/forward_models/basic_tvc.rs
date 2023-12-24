use crate::{geo, sim, control};

pub struct BasicTVC{
    pos_joint_m: geo::Vector3,
    thrust_vec_n: geo::Vector3,
    moment_vec_nm: geo::Vector3,
    thrust_n: f64,
    angle_1_rad: f64,
    angle_2_rad: f64,
    max_angle_rad: f64
}

impl BasicTVC{
    pub fn new(
        thrust_n: f64, 
        pos_joint_m: [f64; 3], 
        angle_1_rad: f64, 
        angle_2_rad: f64, 
        max_angle_rad: f64
    ) -> BasicTVC{
        let pos_joint_m = geo::Vector3::from_array(pos_joint_m);
        let thrust_vec_n = geo::Vector3::from_spherical(
            1.0, 
            angle_1_rad, 
            angle_2_rad
        );
        let moment_vec_nm = pos_joint_m.cross(&thrust_vec_n);
        return BasicTVC{
            pos_joint_m,
            thrust_vec_n,
            moment_vec_nm,
            thrust_n,
            angle_1_rad,
            angle_2_rad,
            max_angle_rad
        }
    }

    pub fn set_angle_1_rad(&mut self, angle_1_rad: f64){
        self.angle_1_rad = control::clamp(angle_1_rad, self.max_angle_rad, -self.max_angle_rad);
        self.thrust_vec_n = geo::Vector3::from_spherical(self.thrust_n, self.angle_1_rad, self.angle_2_rad);
        self.moment_vec_nm = self.pos_joint_m.cross(&self.thrust_vec_n);
    }

    pub fn set_angle_2_rad(&mut self, angle_2_rad: f64){
        self.angle_2_rad = control::clamp(angle_2_rad, self.max_angle_rad, -self.max_angle_rad);
        self.thrust_vec_n = geo::Vector3::from_spherical(self.thrust_n, self.angle_1_rad, self.angle_2_rad);
        self.moment_vec_nm = self.pos_joint_m.cross(&self.thrust_vec_n);
    }

    pub fn set_thrust_n(&mut self, thrust_n: f64){
        self.thrust_n = thrust_n;
        self.thrust_vec_n = self.thrust_vec_n.to_unit() * self.thrust_n;
        self.moment_vec_nm = self.pos_joint_m.cross(&self.thrust_vec_n);
    }

    pub fn get_thrust_vec_n(&self) -> geo::Vector3{
        return self.thrust_vec_n
    }

    pub fn get_moment_vec_nm(&self) -> geo::Vector3{
        return self.moment_vec_nm
    }
}

// ----------------------------------------------------------------------------
// Data recording
// ----------------------------------------------------------------------------

impl sim::Save for BasicTVC{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.angle_1 [rad]").as_str(),self.angle_1_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.angle_2 [rad]").as_str(),self.angle_2_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.total_thrust [N]").as_str(),self.thrust_vec_n.k,
        );
    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        self.save_data(node_name, runtime);
        runtime.add_or_set(format!(
            "{node_name}.thrust.x [N]").as_str(),self.thrust_vec_n.i,
        );
        runtime.add_or_set(format!(
            "{node_name}.thrust.y [N]").as_str(),self.thrust_vec_n.j,
        );
        runtime.add_or_set(format!(
            "{node_name}.thrust.z [N]").as_str(),self.thrust_vec_n.k,
        );

        runtime.add_or_set(format!(
            "{node_name}.moment.x [Nm]").as_str(),self.moment_vec_nm.i
        );
        runtime.add_or_set(format!(
            "{node_name}.moment.y [Nm]").as_str(),self.moment_vec_nm.j
        );
        runtime.add_or_set(format!(
            "{node_name}.moment.z [Nm]").as_str(),self.moment_vec_nm.k
        );

        runtime.add_or_set(format!(
            "{node_name}.pos_joint.x [m]").as_str(),self.pos_joint_m.i
        );
        runtime.add_or_set(format!(
            "{node_name}.pos_joint.y [m]").as_str(),self.pos_joint_m.j
        );
        runtime.add_or_set(format!(
            "{node_name}.pos_joint.z [m]").as_str(),self.pos_joint_m.k
        );

    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::sim::Save;
    use super::*;
    use approx::assert_relative_eq;
    #[test]
    fn sin_sweep(){
        let mut runtime = sim::Runtime::new(PI * 2.0, 1e-2, "angle [rad]");
        let mut tvc = BasicTVC::new(1.0, [0.0,0.0,0.0], 0.0, 0.0, 2.0*PI);

        while runtime.is_running{
            tvc.save_data_verbose("tvc", &mut runtime);

            if runtime.get_x() >= runtime.get_max_x(){
                break
            }

            tvc.set_angle_1_rad(runtime.get_x());
            runtime.increment();
        }

        runtime.export_to_csv("results/data/tvc_sinsweep.csv")
    }
}