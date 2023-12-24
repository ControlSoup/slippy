
use derive_more;

use crate::geo;

use crate::sim;

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
    pub intertial_force_n: geo::Vector3,
    pub intertial_moment_nm: geo::Vector3,
    pub body_force_n: geo::Vector3,
    pub body_moment_nm: geo::Vector3,

    // State
    inertial_pos_m: geo::Vector3,
    inertial_vel_mps: geo::Vector3,
    inertial_accel_mps2: geo::Vector3,
    quat_b2i: geo::Quaternion,
    body_ang_vel_radps: geo::Vector3,
    body_ang_accel_radps2: geo::Vector3,

    // Mass Properties
    pub mass_cg_kg: f64,
    i_tensor_cg_kgpm2: geo::Matrix3x3,
    inv_i_tensor_cg_kgpm2: geo::Matrix3x3
}

impl RigidBody{
    pub fn new(
        intertial_force_n: [f64; 3],
        intertial_moment_nm: [f64; 3],
        body_force_n: [f64; 3],
        body_moment_nm: [f64; 3],
        inertial_pos_m: [f64; 3],
        inertial_vel_mps: [f64; 3],
        inertial_accel_mps2: [f64; 3],
        quat_b2i: [f64; 4],
        body_ang_vel_radps: [f64; 3],
        body_ang_accel_radps2: [f64; 3],
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: [f64; 9]
    ) -> RigidBody{

        // Precompute inverse of Inertia tensor
        let i_tensor_cg_kgpm2 = geo::Matrix3x3::from_array(i_tensor_cg_kgpm2);
        let inv_i_tensor_cg_kgpm2 = i_tensor_cg_kgpm2.inv()
            .expect("i_tensor_cg_kgpm2 was not invertible");


        return RigidBody {
            intertial_force_n: geo::Vector3::from_array(intertial_force_n),
            intertial_moment_nm: geo::Vector3::from_array(intertial_moment_nm),
            body_force_n: geo::Vector3::from_array(body_force_n),
            body_moment_nm: geo::Vector3::from_array(body_moment_nm),
            inertial_pos_m: geo::Vector3::from_array(inertial_pos_m),
            inertial_vel_mps: geo::Vector3::from_array(inertial_vel_mps),
            inertial_accel_mps2: geo::Vector3::from_array(inertial_accel_mps2),
            quat_b2i: geo::Quaternion::from_array(quat_b2i),
            body_ang_vel_radps: geo::Vector3::from_array(body_ang_vel_radps),
            body_ang_accel_radps2: geo::Vector3::from_array(body_ang_accel_radps2),
            mass_cg_kg,
            i_tensor_cg_kgpm2,
            inv_i_tensor_cg_kgpm2
        }
    }

    pub fn identity() -> RigidBody{
        return RigidBody {
            intertial_force_n: geo::Vector3::zeros(),
            intertial_moment_nm: geo::Vector3::zeros(),
            body_force_n: geo::Vector3::zeros(),
            body_moment_nm: geo::Vector3::zeros(),
            inertial_pos_m: geo::Vector3::zeros(),
            inertial_vel_mps: geo::Vector3::zeros(),
            inertial_accel_mps2: geo::Vector3::zeros(),
            quat_b2i: geo::Quaternion::identity(),
            body_ang_vel_radps: geo::Vector3::zeros(),
            body_ang_accel_radps2: geo::Vector3::zeros(),
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: geo::Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2: geo::Matrix3x3::identity()
        }
    }

    fn zeros() -> RigidBody{
        return RigidBody {
            intertial_force_n: geo::Vector3::zeros(),
            intertial_moment_nm: geo::Vector3::zeros(),
            body_force_n: geo::Vector3::zeros(),
            body_moment_nm: geo::Vector3::zeros(),
            inertial_pos_m: geo::Vector3::zeros(),
            inertial_vel_mps: geo::Vector3::zeros(),
            inertial_accel_mps2: geo::Vector3::zeros(),
            quat_b2i: geo::Quaternion::of(0.0),
            body_ang_vel_radps: geo::Vector3::zeros(),
            body_ang_accel_radps2: geo::Vector3::zeros(),
            mass_cg_kg: 0.0,
            i_tensor_cg_kgpm2: geo::Matrix3x3::of(0.0),
            inv_i_tensor_cg_kgpm2: geo::Matrix3x3::of(0.0)
        }
    }

    pub fn get_pos_m(&self) -> geo::Vector3{
        return self.inertial_pos_m
    }

    pub fn get_vel_mps(&self) -> geo::Vector3{
        return self.inertial_vel_mps
    }

    pub fn get_accel_mps2(&self) -> geo::Vector3{
        return self.inertial_accel_mps2
    }

    pub fn get_quat(&self) -> geo::Quaternion{
        return self.quat_b2i
    }

    pub fn get_body_ang_vel_radps(&self) -> geo::Vector3{
        return self.body_ang_vel_radps
    }

    pub fn get_body_ang_accel_radps2(&self) -> geo::Vector3{
        return self.body_ang_accel_radps2
    }
}

impl sim::Integrate for RigidBody{

    fn effects(&mut self) {
        // Rigidbody Dynamics
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics
        //
        // Notes:
        //     accelerations act about nav frame

        let total_forces_n =
            self.intertial_force_n +
            self.quat_b2i.transform(self.body_force_n);

        let total_moments_nm =
            self.intertial_moment_nm +
            self.quat_b2i.transform(self.body_moment_nm);

        // F = ma
        self.inertial_accel_mps2 = total_forces_n / self.mass_cg_kg;

        // I * w
        let i_dot_w =
            self.i_tensor_cg_kgpm2 * self.body_ang_vel_radps;

        // w x (I * w)
        let w_cross_i_dot_w = self.body_ang_vel_radps.cross(&i_dot_w);

        // M - (w x (I * w))
        let m_minus_w_cross_i_dot_w = total_moments_nm - w_cross_i_dot_w;

        // alpha = I^-1(M-(w × (I * w))
        self.body_ang_accel_radps2 =
            self.inv_i_tensor_cg_kgpm2 * m_minus_w_cross_i_dot_w;

    }

    fn get_derivative(&self)-> Self {
        let mut d = RigidBody::zeros();

        // State Derviative
        d.inertial_pos_m = self.inertial_vel_mps.clone();
        d.inertial_vel_mps = self.inertial_accel_mps2.clone();
        d.quat_b2i = self.quat_b2i.derivative(self.body_ang_vel_radps).clone();
        d.body_ang_vel_radps = self.body_ang_accel_radps2.clone();

        return d

    }
}

// ----------------------------------------------------------------------------
// Data Recording
// ----------------------------------------------------------------------------

impl sim::Save for RigidBody{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        // State
        runtime.add_or_set(format!(
            "{node_name}.inertial_pos.x [m]").as_str(),
            self.inertial_pos_m.i.clone()
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_pos.y [m]").as_str(),
            self.inertial_pos_m.j.clone()
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_pos.z [m]").as_str(),
            self.inertial_pos_m.k.clone()
        );

        runtime.add_or_set(format!(
            "{node_name}.inertial_vel.x [m/s]").as_str(),
            self.inertial_vel_mps.i
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_vel.y [m/s]").as_str(),
            self.inertial_vel_mps.j
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_vel.z [m/s]").as_str(),
            self.inertial_vel_mps.k
        );

        runtime.add_or_set(format!(
            "{node_name}.inertial_accel.x [m/s^2]").as_str(),
                self.inertial_accel_mps2.i
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_accel.y [m/s^2]").as_str(),
                self.inertial_accel_mps2.j
        );
        runtime.add_or_set(format!(
            "{node_name}.inertial_accel.z [m/s^2]").as_str(),
                self.inertial_accel_mps2.k
        );

        runtime.add_or_set(format!(
            "{node_name}.quat_b2i.a [-]").as_str(),
            self.quat_b2i.a
        );
        runtime.add_or_set(format!(
            "{node_name}.quat_b2i.b [-]").as_str(),
            self.quat_b2i.b
        );
        runtime.add_or_set(format!(
            "{node_name}.quat_b2i.c [-]").as_str(),
            self.quat_b2i.c
        );
        runtime.add_or_set(format!(
            "{node_name}.quat_b2i.d [-]").as_str(),
            self.quat_b2i.d
        );


        runtime.add_or_set(format!(
            "{node_name}.body_ang_vel.x [rad/s]").as_str(),
            self.body_ang_vel_radps.i
        );
        runtime.add_or_set(format!(
            "{node_name}.body_ang_vel.y [rad/s]").as_str(),
            self.body_ang_vel_radps.j
        );
        runtime.add_or_set(format!(
            "{node_name}.body_ang_vel.z [rad/s]").as_str(),
            self.body_ang_vel_radps.k
        );

        runtime.add_or_set(format!(
            "{node_name}.body_ang_accel.x [rad/s^2]").as_str(),
            self.body_ang_accel_radps2.i
        );
        runtime.add_or_set(format!(
            "{node_name}.body_ang_accel.y [rad/s^2]").as_str(),
            self.body_ang_accel_radps2.j
        );
        runtime.add_or_set(format!(
            "{node_name}.body_ang_accel.z [rad/s^2]").as_str(),
            self.body_ang_accel_radps2.k
        );

        // Force and Moments
        runtime.add_or_set(format!(
            "{node_name}.intertial_force.x [N]").as_str(),
            self.intertial_force_n.i
        );
        runtime.add_or_set(format!(
            "{node_name}.intertial_force.y [N]").as_str(),
            self.intertial_force_n.j
        );
        runtime.add_or_set(format!(
            "{node_name}.intertial_force.z [N]").as_str(),
            self.intertial_force_n.k
        );

        runtime.add_or_set(format!(
            "{node_name}.intertial_moment.x [Nm]").as_str(),
            self.intertial_moment_nm.i
        );
        runtime.add_or_set(format!(
            "{node_name}.intertial_moment.y [Nm]").as_str(),
            self.intertial_moment_nm.j
        );
        runtime.add_or_set(format!(
            "{node_name}.intertial_moment.z [Nm]").as_str(),
            self.intertial_moment_nm.k
        );

        runtime.add_or_set(format!(
            "{node_name}.body_force.x [N]").as_str(),
            self.body_force_n.i
        );
        runtime.add_or_set(format!(
            "{node_name}.body_force.y [N]").as_str(),
            self.body_force_n.j
        );
        runtime.add_or_set(format!(
            "{node_name}.body_force.z [N]").as_str(),
            self.body_force_n.k
        );

        runtime.add_or_set(format!(
            "{node_name}.body_moment.x [Nm]").as_str(),
            self.body_moment_nm.i
        );
        runtime.add_or_set(format!(
            "{node_name}.body_moment.y [Nm]").as_str(),
            self.body_moment_nm.j
        );
        runtime.add_or_set(format!(
            "{node_name}.body_moment.z [Nm]").as_str(),
            self.body_moment_nm.k
        );

        // Mass properties
        runtime.add_or_set("mass_cg [kg]", self.mass_cg_kg);

    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {

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

        // Dcm representation
        let dcm = self.quat_b2i.to_dcm();
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

        // Attitude conversion
        let euler = dcm.to_euler();
        runtime.add_or_set(format!(
            "{node_name}.euler.i [rad]").as_str(),
            euler.i
        );
        runtime.add_or_set(format!(
            "{node_name}.euler.j [rad]").as_str(),
            euler.j
        );
        runtime.add_or_set(format!(
            "{node_name}.euler.k [rad]").as_str(),
            euler.k
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
    use crate::sim::{Save, Integrate};

    use super::*;

    #[test]
    fn body_translation(){
        let mut object = RigidBody::identity();

        // Set Forces
        object.quat_b2i = geo::Matrix3x3::new(
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0
        ).to_quat();
        object.inertial_pos_m = geo::Vector3::new(0.0, 1.0, 2.0);
        object.body_force_n = geo::Vector3::new(1.0, 1.0, 1.0);

        let dt = 1e-4;
        let max_int = (5.0 / dt) as usize;

        for _ in 0..max_int{
            object = object.rk4(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        almost_equal_array(
            &object.inertial_vel_mps.to_array(),
            &[5.0, 5.0, -5.0]
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        // But oreintation is rotated 90 degrees with x facing down
        almost_equal_array(
            &object.inertial_pos_m.to_array(),
            &[12.5, 13.5, -10.5]
        );
    }
    # [test]
    fn intertial_force_n(){
        let mut object = RigidBody::identity();

        // Set Forces
        object.quat_b2i = geo::Vector3::new(0.0, PI / 2.0,0.0).to_quat();
        object.inertial_pos_m = geo::Vector3::new(0.0, 1.0, 2.0);
        object.intertial_force_n = geo::Vector3::new(1.0, 1.0, 1.0);

        let dt = 1e-4;
        let max_int = (5.0 / dt) as usize;

        for _ in 0..max_int{
            object = object.rk4(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        almost_equal_array(
            &object.inertial_vel_mps.to_array(),
            &[5.0, 5.0, 5.0]
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        // Oreintation should not effect this
        almost_equal_array(
            &object.inertial_pos_m.to_array(),
            &[12.5,13.5, 14.5]
        );
    }

    #[test]
    fn spin_cone_simulator(){
        // SPIN-CONE SIMULATOR from Strapdown Analytics
        // Section: 11.2.1, Pg 11-12

        let beta = 0.25; // Precessional axis angle
        let omega_s = 0.025; // Intertial rotation rate
        let omega_c = 0.03; // Intertial precessional rate

        // Use an identity intertia tensor and mass
        let mut uut = RigidBody{
            intertial_force_n: geo::Vector3::zeros(),
            intertial_moment_nm: geo::Vector3::zeros(),
            body_force_n: geo::Vector3::zeros(),
            body_moment_nm: geo::Vector3::zeros(),
            inertial_pos_m: geo::Vector3::zeros(),
            inertial_vel_mps: geo::Vector3::zeros(),
            inertial_accel_mps2: geo::Vector3::zeros(),
            quat_b2i: geo::Vector3::new(beta, 0.0, 0.0).to_quat(),
            body_ang_accel_radps2: geo::Vector3::zeros(),
            body_ang_vel_radps: geo::Vector3::new(0.0, 0.0, 0.0),
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: geo::Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2: geo::Matrix3x3::identity()
        };

        let mut runtime = sim::Runtime::new(10.0, 1e-3, "time [s]");
        let dt = runtime.get_dx();
        let init_euler = uut.quat_b2i.to_euler();

        while runtime.is_running{
            uut.save_data_verbose("uut", &mut runtime);

            // Update spin cone velocity
            let omega_s_vec = geo::Vector3::new(0.0, 0.0, omega_s);
            let omega_c_vec = uut.get_quat()
                .conjugate()
                .transform(geo::Vector3::new(0.0, 0.0, omega_c));

            uut.body_ang_vel_radps = omega_s_vec + omega_c_vec;

            uut = uut.rk4(dt);
            runtime.increment();
        }

        runtime.export_to_csv("results/data/spin_cone.csv");

        // Strapdown uses Z-Y-X, I use X-Y-Z
        let end_psi = -omega_c * runtime.get_x();
        let end_theta = (PI / 2.0) - beta;
        let end_phi = (omega_s - (omega_c * beta.cos()))
            * runtime.get_max_x()
            + init_euler.k;

        // Eq 11.2.1.1-7
        almost_equal_array(
            &uut.quat_b2i.to_euler().to_array(),
            &geo::Vector3::new(end_phi, end_theta, end_psi).to_array()
        );
    }

    fn spin_rock_size_simulator(){
        // SPIN-ROCK-SIZE SIMULATOR
        // Section 11.2.3, Pg 11-27 from strapdown analytics

    }
}