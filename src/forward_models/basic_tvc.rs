use crate::{geo, sim, control};

pub struct BasicTVC{
    pos_joint_m: geo::Vector3,
    thrust_vec_n: geo::Vector3,
    moment_vec_nm: geo::Vector3,
    thrust_n: f64,
    theta_rad: f64,
    phi_rad: f64,
    max_angle_rad: f64,
    max_thrust_n: f64
}

impl BasicTVC{
    pub fn new(
        thrust_n: f64, 
        pos_joint_m: [f64; 3], 
        theta_rad: f64, 
        phi_rad: f64, 
        max_angle_rad: f64,
        max_thrust_n: f64
    ) -> BasicTVC{
        let pos_joint_m = geo::Vector3::from_array(pos_joint_m);

        let xyz_axis = geo::Matrix3x3::from_xyz_euler(phi_rad, -theta_rad, 0.0);
        let thrust_vec_n = geo::Vector3::new(xyz_axis.c31, xyz_axis.c32, xyz_axis.c33) * thrust_n;

        let moment_vec_nm = pos_joint_m.cross(&thrust_vec_n);
        return BasicTVC{
            pos_joint_m,
            thrust_vec_n,
            moment_vec_nm,
            thrust_n,
            theta_rad,
            phi_rad,
            max_angle_rad,
            max_thrust_n
        }
    }

    fn update_params(&mut self){
        let xyz_axis = geo::Matrix3x3::from_xyz_euler(self.phi_rad, -self.theta_rad, 0.0);
        self.thrust_vec_n = geo::Vector3::new(xyz_axis.c31, xyz_axis.c32, xyz_axis.c33) * self.thrust_n;
        self.moment_vec_nm = self.pos_joint_m.cross(&self.thrust_vec_n);
    }

    pub fn set_theta_rad(&mut self, theta_rad: f64){
        self.theta_rad = control::clamp(theta_rad, self.max_angle_rad, -self.max_angle_rad);
        self.update_params();
    }

    pub fn set_phi_rad(&mut self, phi_rad: f64){
        self.phi_rad = control::clamp(phi_rad, self.max_angle_rad, -self.max_angle_rad);
        self.update_params();
    }

    pub fn set_thrust_n(&mut self, thrust_n: f64){
        self.thrust_n = control::clamp(thrust_n, self.max_thrust_n, 0.0);
        self.update_params();
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
            "{node_name}.theta [rad]").as_str(),self.theta_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.phi [rad]").as_str(),self.phi_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.total_thrust [N]").as_str(),self.thrust_n
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

    use crate::geo::PI_QUARTER;
    use crate::sim::Save;
    use crate::test::almost_equal_array;
    use approx::assert_relative_eq;
    use super::*;

    #[test]
    fn forty_five(){
        let mut tvc = BasicTVC::new(1.0, [0.0, 0.0, 0.0], PI_QUARTER, 0.0, 5.0, 1.0);

        tvc.set_thrust_n(10.0);
        tvc.set_theta_rad(PI_QUARTER);

        assert_relative_eq!(
            tvc.get_thrust_vec_n().i,
            2_f64.sqrt() / 2.0        
        );

        tvc.set_theta_rad(0.0);
        assert_relative_eq!(
            tvc.get_thrust_vec_n().i,
            0.0
        );

        tvc.set_phi_rad(PI_QUARTER);

        assert_relative_eq!(
            tvc.get_thrust_vec_n().j,
            2_f64.sqrt() / 2.0        
        );

        tvc.set_phi_rad(0.0);
        assert_relative_eq!(
            tvc.get_thrust_vec_n().j,
            0.0
        );


    }

    #[test]
    fn sin_sweep(){
        let mut runtime = sim::Runtime::new(PI * 2.0 + 1e-2, 1e-2, "angle [rad]");
        let mut tvc = BasicTVC::new(1.0, [0.0,0.0,-1.0], 0.0, 0.0, 2.0*PI, 1.0);

        while runtime.is_running{
            tvc.save_data_verbose("tvc", &mut runtime);

            if runtime.get_x() >= runtime.get_max_x(){
                break
            }

            tvc.set_thrust_n(10.0);
            tvc.set_theta_rad(runtime.get_x());
            tvc.set_phi_rad(runtime.get_x());
            runtime.increment();
        }

        runtime.export_to_csv("results/data/tvc_sinsweep.csv");

        almost_equal_array(
            &[-0.0, -0.0, 1.0],
            &tvc.get_thrust_vec_n().to_array(),
        );
        almost_equal_array(
            &tvc.get_moment_vec_nm().to_array(),
            &[0.0, 0.0, 0.0]
        );

    }
}