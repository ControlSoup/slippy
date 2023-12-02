
use derive_more;

use crate::strapdown::{
    matrix::Matrix3x3,
    vector::Vector3,
    quaternion::Quaternion
};

use crate::sim::{integration::Integrate, runtime::{Runtime, Save}};

#[derive(
    Debug,
    Clone,
    PartialEq,
    derive_more::Add,
    derive_more::AddAssign,
    derive_more::Sub,
    derive_more::SubAssign,
    derive_more::Mul,
    derive_more::Div,
    derive_more::Neg
)]

pub struct RigidBody{
    // Forces and Moments
    pub nav_force_n: Vector3,
    pub nav_moment_nm: Vector3,
    pub body_force_n: Vector3,
    pub body_moment_nm: Vector3,

    // State
    pos_m: Vector3,
    vel_mps: Vector3,
    accel_mps2: Vector3,
    quat: Quaternion,
    ang_vel_radps: Vector3,
    ang_accel_radps2: Vector3,

    // Mass Properties
    pub mass_cg_kg: f64,
    i_tensor_cg_kgpm2: Matrix3x3,
    inv_i_tensor_cg_kgpm2: Matrix3x3
}

impl RigidBody{
    pub fn new(
        nav_force_n: [f64; 3],
        nav_moment_nm: [f64; 3],
        body_force_n: [f64; 3],
        body_moment_nm: [f64; 3],
        pos_m: [f64; 3],
        vel_mps: [f64; 3],
        accel_mps2: [f64; 3],
        quat: [f64; 4],
        ang_vel_radps: [f64; 3],
        ang_accel_radps2: [f64; 3],
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: [f64; 9]
    ) -> RigidBody{

        // Precompute inverse of Inertia tensor
        let i_tensor_cg_kgpm2 = Matrix3x3::from_array(i_tensor_cg_kgpm2);
        let inv_i_tensor_cg_kgpm2 = i_tensor_cg_kgpm2.inv()
            .expect("i_tensor_cg_kgpm2 was not invertible");


        return RigidBody {
            nav_force_n: Vector3::from_array(nav_force_n),
            nav_moment_nm: Vector3::from_array(nav_moment_nm),
            body_force_n: Vector3::from_array(body_force_n),
            body_moment_nm: Vector3::from_array(body_moment_nm),
            pos_m: Vector3::from_array(pos_m),
            vel_mps: Vector3::from_array(vel_mps),
            accel_mps2: Vector3::from_array(accel_mps2),
            quat: Quaternion::from_array(quat),
            ang_vel_radps: Vector3::from_array(ang_vel_radps),
            ang_accel_radps2: Vector3::from_array(ang_accel_radps2),
            mass_cg_kg,
            i_tensor_cg_kgpm2,
            inv_i_tensor_cg_kgpm2
        }
    }

    pub fn identity() -> RigidBody{
        return RigidBody {
            nav_force_n: Vector3::zeros(),
            nav_moment_nm: Vector3::zeros(),
            body_force_n: Vector3::zeros(),
            body_moment_nm: Vector3::zeros(),
            pos_m: Vector3::zeros(),
            vel_mps: Vector3::zeros(),
            accel_mps2: Vector3::zeros(),
            quat: Quaternion::identity(),
            ang_vel_radps: Vector3::zeros(),
            ang_accel_radps2: Vector3::zeros(),
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2: Matrix3x3::identity()
        }
    }

    fn zeros() -> RigidBody{
        return RigidBody {
            nav_force_n: Vector3::zeros(),
            nav_moment_nm: Vector3::zeros(),
            body_force_n: Vector3::zeros(),
            body_moment_nm: Vector3::zeros(),
            pos_m: Vector3::zeros(),
            vel_mps: Vector3::zeros(),
            accel_mps2: Vector3::zeros(),
            quat: Quaternion::of(0.0),
            ang_vel_radps: Vector3::zeros(),
            ang_accel_radps2: Vector3::zeros(),
            mass_cg_kg: 0.0,
            i_tensor_cg_kgpm2: Matrix3x3::of(0.0),
            inv_i_tensor_cg_kgpm2: Matrix3x3::of(0.0)
        }
    }

    pub fn get_pos_m(&self) -> Vector3{
        return self.pos_m
    }

    pub fn get_vel_mps(&self) -> Vector3{
        return self.vel_mps
    }

    pub fn get_accel_mps2(&self) -> Vector3{
        return self.accel_mps2
    }

    pub fn get_quat(&self) -> Quaternion{
        return self.quat
    }

    pub fn get_ang_vel_radps(&self) -> Vector3{
        return self.ang_vel_radps
    }

    pub fn get_ang_accel_radps2(&self) -> Vector3{
        return self.ang_accel_radps2
    }
}

impl Integrate for RigidBody{

    fn effects(&mut self) {
        // Rigidbody Dynamics
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics
        //
        // Notes:
        //     accelerations act about nav frame

        let total_forces_n =
            self.nav_force_n +
            self.quat.transform(self.body_force_n);

        let total_moments_nm =
            self.nav_moment_nm +
            self.quat.transform(self.body_moment_nm);

        // F = ma
        self.accel_mps2 = total_forces_n / self.mass_cg_kg;

        // I * w
        let i_dot_w =
            self.i_tensor_cg_kgpm2 * self.ang_vel_radps;

        // w x (I * w)
        let w_cross_i_dot_w = self.ang_vel_radps.cross(&i_dot_w);

        // M - (w x (I * w))
        let m_minus_w_cross_i_dot_w = total_moments_nm - w_cross_i_dot_w;

        // alpha = I^-1(M-(w × (I * w))
        self.ang_accel_radps2 =
            self.inv_i_tensor_cg_kgpm2 * m_minus_w_cross_i_dot_w;

    }

    fn get_derivative(&self)-> Self {
        let mut d = RigidBody::zeros();

        // State Derviative
        d.pos_m = self.vel_mps.clone();
        d.vel_mps = self.accel_mps2.clone();
        d.quat = self.quat.derivative(self.ang_vel_radps).clone();
        d.ang_vel_radps = self.ang_accel_radps2.clone();

        return d

    }
}


impl Save for RigidBody{
    fn save_data(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {
        // State
        runtime.add_or_set(format!(
            "{node_name}.pos.x [m]").as_str(),
            self.pos_m.x.clone()
        );
        runtime.add_or_set(format!(
            "{node_name}.pos.y [m]").as_str(),
            self.pos_m.y.clone()
        );
        runtime.add_or_set(format!(
            "{node_name}.pos.z [m]").as_str(),
            self.pos_m.z.clone()
        );

        runtime.add_or_set(format!(
            "{node_name}.vel.x [m/s]").as_str(),
            self.vel_mps.x
        );
        runtime.add_or_set(format!(
            "{node_name}.vel.y [m/s]").as_str(),
            self.vel_mps.y
        );
        runtime.add_or_set(format!(
            "{node_name}.vel.z [m/s]").as_str(),
            self.vel_mps.z
        );

        runtime.add_or_set(format!(
            "{node_name}.accel.x [m/s^2]").as_str(),
                self.accel_mps2.x
        );
        runtime.add_or_set(format!(
            "{node_name}.accel.y [m/s^2]").as_str(),
                self.accel_mps2.y
        );
        runtime.add_or_set(format!(
            "{node_name}.accel.z [m/s^2]").as_str(),
                self.accel_mps2.z
        );

        runtime.add_or_set(format!(
            "{node_name}.quat.a [-]").as_str(),
            self.quat.a
        );
        runtime.add_or_set(format!(
            "{node_name}.quat.b [-]").as_str(),
            self.quat.b
        );
        runtime.add_or_set(format!(
            "{node_name}.quat.c [-]").as_str(),
            self.quat.c
        );
        runtime.add_or_set(format!(
            "{node_name}.quat.d [-]").as_str(),
            self.quat.d
        );


        runtime.add_or_set(format!(
            "{node_name}ang_vel.x [rad/s]").as_str(),
            self.ang_vel_radps.x
        );
        runtime.add_or_set(format!(
            "{node_name}ang_vel.y [rad/s]").as_str(),
            self.ang_vel_radps.y
        );
        runtime.add_or_set(format!(
            "{node_name}ang_vel.z [rad/s]").as_str(),
            self.ang_vel_radps.z
        );

        runtime.add_or_set(format!(
            "{node_name}.ang_accel.x [rad/s^2]").as_str(),
            self.ang_accel_radps2.x
        );
        runtime.add_or_set(format!(
            "{node_name}.ang_accel.y [rad/s^2]").as_str(),
            self.ang_accel_radps2.y
        );
        runtime.add_or_set(format!(
            "{node_name}.ang_accel.z [rad/s^2]").as_str(),
            self.ang_accel_radps2.z
        );

        // Force and Moments
        runtime.add_or_set(format!(
            "{node_name}.nav_moment.x [N]").as_str(),
            self.nav_force_n.x
        );
        runtime.add_or_set(format!(
            "{node_name}.nav_moment.y [N]").as_str(),
            self.nav_force_n.y
        );
        runtime.add_or_set(format!(
            "{node_name}.nav_moment.z [N]").as_str(),
            self.nav_force_n.z
        );

        runtime.add_or_set(format!(
            "{node_name}.nav_moment.x [Nm]").as_str(),
            self.nav_moment_nm.x
        );
        runtime.add_or_set(format!(
            "{node_name}.nav_moment.y [Nm]").as_str(),
            self.nav_moment_nm.y
        );
        runtime.add_or_set(format!(
            "{node_name}.nav_moment.z [Nm]").as_str(),
            self.nav_moment_nm.z
        );

        runtime.add_or_set(format!(
            "{node_name}.body_force.x [N]").as_str(),
            self.body_force_n.x
        );
        runtime.add_or_set(format!(
            "{node_name}.body_force.y [N]").as_str(),
            self.body_force_n.y
        );
        runtime.add_or_set(format!(
            "{node_name}.body_force.z [N]").as_str(),
            self.body_force_n.z
        );

        runtime.add_or_set(format!(
            "{node_name}.body_moment.x [Nm]").as_str(),
            self.body_moment_nm.x
        );
        runtime.add_or_set(format!(
            "{node_name}.body_moment.y [Nm]").as_str(),
            self.body_moment_nm.y
        );
        runtime.add_or_set(format!(
            "{node_name}.body_moment.z [Nm]").as_str(),
            self.body_moment_nm.z
        );

        // Mass properties
        runtime.add_or_set("mass_cg [kg]", self.mass_cg_kg);

    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut Runtime) where Self: Sized {

        self.save_data(node_name, runtime);

        // Moment of intertia
        runtime.add_or_set(format!(
            "{node_name}.Ixx [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c11
        );
        runtime.add_or_set(format!(
            "{node_name}.Ixy [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c12
        );
        runtime.add_or_set(format!(
            "{node_name}.Ixz [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c13
        );
        runtime.add_or_set(format!(
            "{node_name}.Iyx [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c21
        );
        runtime.add_or_set(format!(
            "{node_name}.Iyy [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c22
        );
        runtime.add_or_set(format!(
            "{node_name}.Iyz [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c23
        );
        runtime.add_or_set(format!(
            "{node_name}.Izx [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c31
        );
        runtime.add_or_set(format!(
            "{node_name}.Izx [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c32
        );
        runtime.add_or_set(format!(
            "{node_name}.Izz [kg/m^2]").as_str(),
            self.i_tensor_cg_kgpm2.c33
        );

        // Attitude conversion
        let euler = self.quat.to_euler();
        runtime.add_or_set(format!(
            "{node_name}.euler.x [rad]").as_str(),
            euler.x
        );
        runtime.add_or_set(format!(
            "{node_name}.euler.y [rad]").as_str(),
            euler.y
        );
        runtime.add_or_set(format!(
            "{node_name}.euler.z [rad]").as_str(),
            euler.z
        );

        let dcm = self.quat.to_dcm();
        runtime.add_or_set(format!(
            "{node_name}.dcm.c11 [-]").as_str(),
            dcm.c11
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c12 [-]").as_str(),
            dcm.c12
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c13 [-]").as_str(),
            dcm.c13
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c21 [-]").as_str(),
            dcm.c21
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c22 [-]").as_str(),
            dcm.c22
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c23 [-]").as_str(),
            dcm.c23
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c31 [-]").as_str(),
            dcm.c31
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c32 [-]").as_str(),
            dcm.c32
        );
        runtime.add_or_set(format!(
            "{node_name}.dcm.c33 [-]").as_str(),
            dcm.c33
        );
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::test::almost_equal_array;
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn body_translation(){
        let mut object = RigidBody::identity();

        // Set Forces
        object.quat = Vector3::new(0.0, PI / 2.0,0.0).to_quat();
        object.pos_m = Vector3::new(0.0, 1.0, 2.0);
        object.body_force_n = Vector3::new(1.0, 1.0, 1.0);

        let dt = 1e-4;
        let max_int = (5.0 / dt) as usize;

        for _ in 0..max_int{
            object = object.rk4(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        almost_equal_array(
            &object.vel_mps.to_array(),
            &[5.0, 5.0, -5.0]
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        // But oreintation is rotated 90 degrees with x facing down
        almost_equal_array(
            &object.pos_m.to_array(),
            &[12.5, 13.5, -10.5]
        );
    }
    # [test]
    fn nav_force_n(){
        let mut object = RigidBody::identity();

        // Set Forces
        object.quat = Vector3::new(0.0, PI / 2.0,0.0).to_quat();
        object.pos_m = Vector3::new(0.0, 1.0, 2.0);
        object.nav_force_n = Vector3::new(1.0, 1.0, 1.0);

        let dt = 1e-4;
        let max_int = (5.0 / dt) as usize;

        for _ in 0..max_int{
            object = object.rk4(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        almost_equal_array(
            &object.vel_mps.to_array(),
            &[5.0, 5.0, 5.0]
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        // Oreintation should not effect this
        almost_equal_array(
            &object.pos_m.to_array(),
            &[12.5,13.5, 14.5]
        );
    }

    #[test]
    fn spin_cone_simulator(){
        // SPIN-CONE SIMULATOR from Strapdown Analytics
        // Section: 11.2.1, Pg 11-12

        let beta = 0.25; // Precessional axis angle
        let omega_s = 1.0; // Intertial rotation rate
        let omega_c = 1.5; // Intertial precessional rate

        // Use an identity intertia tensor and mass
        let mut uut = RigidBody{
            nav_force_n: Vector3::zeros(),
            nav_moment_nm: Vector3::zeros(),
            body_force_n: Vector3::zeros(),
            body_moment_nm: Vector3::zeros(),
            pos_m: Vector3::zeros(),
            vel_mps: Vector3::zeros(),
            accel_mps2: Vector3::zeros(),
            quat: Vector3::new(beta, 0.0, 0.0).to_quat(),
            ang_accel_radps2: Vector3::zeros(),
            ang_vel_radps: Vector3::new(0.0, 0.0, omega_s + omega_s),
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2: Matrix3x3::identity()
        };

        let mut runtime = Runtime::new(10.0, 1e-3, "time [s]");
        let dt = runtime.get_dx();
        let init_euler = uut.quat.to_euler();

        while runtime.is_running{
            // Update spin cone velocity
            let omega_s_vec =
                uut.quat.transform(Vector3::new(0.0, 0.0, omega_s));
            let omega_c_vec = Vector3::new(0.0, 0.0, omega_c);
            uut.ang_vel_radps = omega_s_vec + omega_c_vec;

            uut = uut.rk4(dt);
            uut.save_data("uut", &mut runtime);
            runtime.increment();
        }

        runtime.export_to_csv("spin_cone", "results/data");

        let end_psi = -omega_c * runtime.get_max_x();
        let end_theta = (PI / 2.0) - beta;
        let end_phi = (omega_s - (omega_c * beta.cos()))
            * runtime.get_max_x()
            + init_euler.z;

        // Eq 11.2.1.1-7
        almost_equal_array(
            &Vector3::new(end_psi, end_theta, end_phi).to_dcm().to_array(),
            &uut.quat.to_dcm().to_array()
        );
    }

    #[test]
    fn spin_rock_size_simulator(){
        // SPIN-ROCK-SIZE SIMULATOR
        // Section 11.2.3, Pg 11-27 from strapdown analytics

    }
}