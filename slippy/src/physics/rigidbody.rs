
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
    pub local_level_force_n: Vector3,
    pub local_level_moment_nm: Vector3,
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
        local_level_force_n: [f64; 3],
        local_level_moment_nm: [f64; 3],
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
            local_level_force_n: Vector3::from_array(local_level_force_n),
            local_level_moment_nm: Vector3::from_array(local_level_moment_nm),
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
            local_level_force_n: Vector3::zeros(),
            local_level_moment_nm: Vector3::zeros(),
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
            local_level_force_n: Vector3::zeros(),
            local_level_moment_nm: Vector3::zeros(),
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
} 

impl Integrate for RigidBody{
    
    fn effects(&mut self) {
        // Rigidbody Dynamics
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics
        //
        // Notes:
        //     Forces and moments act about the body frame

        let total_forces_n = 
            self.local_level_force_n + 
            self.quat.transform(self.body_force_n);

        let total_moments_nm =  
            self.local_level_moment_nm + 
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
        d.ang_vel_radps = self.ang_accel_radps2;

        return d

    }
}


impl Save for RigidBody{
    fn save(&self, node_name: String, runtime: &mut Runtime) where Self: Sized {
        // State
        runtime.add_or_set("pos.x [m]", self.pos_m.x.clone());
        runtime.add_or_set("pos.y [m]", self.pos_m.y.clone());
        runtime.add_or_set("pos.z [m]", self.pos_m.z.clone());

        runtime.add_or_set("vel.x [m/s]", self.vel_mps.x);
        runtime.add_or_set("vel.y [m/s]", self.vel_mps.y);
        runtime.add_or_set("vel.z [m/s]", self.vel_mps.z);

        runtime.add_or_set("accel.x [m/s^2]", self.accel_mps2.x);
        runtime.add_or_set("accel.y [m/s^2]", self.accel_mps2.y);
        runtime.add_or_set("accel.z [m/s^2]", self.accel_mps2.z);

        runtime.add_or_set(".quat.a [-]", self.quat.a);
        runtime.add_or_set(".quat.b [-]", self.quat.b);
        runtime.add_or_set(".quat.c [-]", self.quat.c);
        runtime.add_or_set(".quat.d [-]", self.quat.d);

        runtime.add_or_set("ang_vel.x [rad/s]", self.ang_vel_radps.x);
        runtime.add_or_set("ang_vel.y [rad/s]", self.ang_vel_radps.y);
        runtime.add_or_set("ang_vel.z [rad/s]", self.ang_vel_radps.z);

        runtime.add_or_set(
            "ang_accel.x [rad/s^2]", self.ang_accel_radps2.x
        );
        runtime.add_or_set(
            "ang_accel.y [rad/s^2]", self.ang_accel_radps2.y
        );
        runtime.add_or_set(
            "ang_accel.z [rad/s^2]", self.ang_accel_radps2.z
        );

        // Force and Moments
        runtime.add_or_set(
            "local_level_force.x [N]", self.local_level_force_n.x
        );
        runtime.add_or_set(
            "local_level_force.y [N]", self.local_level_force_n.y
        );
        runtime.add_or_set(
            "local_level_force.z [N]", self.local_level_force_n.z
        );

        runtime.add_or_set(
            "local_level_moment.x [Nm]", self.local_level_moment_nm.x 
        );
        runtime.add_or_set(
            "local_level_moment.y [Nm]", self.local_level_moment_nm.y 
        );
        runtime.add_or_set(
            "local_level_moment.z [Nm]", self.local_level_moment_nm.z 
        );

        runtime.add_or_set(
            "body_force.x [N]", self.body_force_n.x
        );
        runtime.add_or_set(
            "body_force.y [N]", self.body_force_n.y
        );
        runtime.add_or_set(
            "body_force.z [N]", self.body_force_n.z
        );

        runtime.add_or_set(
            "body_moment.x [Nm]", self.body_moment_nm.x 
        );
        runtime.add_or_set(
            "body_moment.y [Nm]", self.body_moment_nm.y 
        );
        runtime.add_or_set(
            "body_moment.z [Nm]", self.body_moment_nm.z 
        );

        // Mass properties
        runtime.add_or_set("mass_cg [kg]", self.mass_cg_kg);

        runtime.add_or_set(
            "Ixx [kg/m^2]", self.i_tensor_cg_kgpm2.c11
        );
        runtime.add_or_set(
            "Ixy [kg/m^2]", self.i_tensor_cg_kgpm2.c12
        );
        runtime.add_or_set(
            "Ixz [kg/m^2]", self.i_tensor_cg_kgpm2.c13
        );
        runtime.add_or_set(
            "Iyx [kg/m^2]", self.i_tensor_cg_kgpm2.c21
        );
        runtime.add_or_set(
            "Iyy [kg/m^2]", self.i_tensor_cg_kgpm2.c22
        );
        runtime.add_or_set(
            "Iyz [kg/m^2]", self.i_tensor_cg_kgpm2.c23
        );
        runtime.add_or_set(
            "Izx [kg/m^2]", self.i_tensor_cg_kgpm2.c31
        );
        runtime.add_or_set(
            "Izx [kg/m^2]", self.i_tensor_cg_kgpm2.c32
        );
        runtime.add_or_set(
            "Izz [kg/m^2]", self.i_tensor_cg_kgpm2.c33
        );
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use crate::test::almost_equal_array;

    use super::*;

    #[test]
    fn local_translation(){
        let mut object = RigidBody::identity();

        // Set Forces
        object.pos_m = Vector3::new(0.0, 1.0, 2.0);
        object.local_level_force_n = Vector3::new(1.0, 1.0, 1.0);

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
        almost_equal_array(
            &object.pos_m.to_array(),
            &[12.5, 13.5, 14.5]
        );
    }

    #[test]
    fn local_rotation(){

        for i in 0..2{
            let mut object = RigidBody::identity();
            let mut final_w = [0.0, 0.0, 0.0];
            let mut final_theta = [0.0, 0.0, 0.0];
            let mut local_level_moments_nm = [0.0, 0.0, 0.0];


            final_w[i] = 0.5;
            final_theta[i] = 1.25;
            local_level_moments_nm[i] = 0.1;


            object.quat = Quaternion::identity();

            object.local_level_moment_nm = Vector3::from_array(
                local_level_moments_nm
            );

            let dt = 0.25;
            let max_int = (5.0 / dt) as i64;

            for _ in 0..max_int{
                object = object.rk4(dt);
            }

            // w  = w_0 + alpha*t
            almost_equal_array(
                &object.ang_vel_radps.to_array(),
                &final_w
            );

            // theta = w_0 * t + alpht * t^2 / 2
            almost_equal_array(
                &object.quat.to_euler().to_array(),
                &final_theta
            );
        }
    }
}