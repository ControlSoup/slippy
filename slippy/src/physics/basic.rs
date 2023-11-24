#[allow(dead_code, unused)]
// Standard
use std::ops::{Mul, Div};

// 3d Party

use crate::strapdown::matrix::Matrix3x3;
// Local
use crate::strapdown::vector::Vector3;
use crate::strapdown::quaternion::Quaternion;
use crate::sim::integration::Integrate;
use crate::sim::runtime::{BasicRunTime, Save};

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    derive_more::Add,
    derive_more::AddAssign,
    derive_more::Sub,
    derive_more::SubAssign,
    derive_more::Mul,
    derive_more::Div,
    derive_more::Neg
)]
pub struct State{
    pub pos_m: Vector3,
    pub vel_mps: Vector3,
    pub accel_mps2: Vector3,
    pub quat: Quaternion,
    pub ang_vel_radps: Vector3,
    pub ang_accel_radps2: Vector3,
}

impl State{
    pub fn new(
        pos_m: [f64; 3],
        vel_mps: [f64; 3],
        accel_mps2: [f64; 3],
        quat: [f64; 4],
        ang_vel_radps: [f64; 3],
        ang_accel_radps2: [f64; 3],
    ) -> State{
        return State {
            pos_m: Vector3::from_array(pos_m),
            vel_mps: Vector3::from_array(vel_mps),
            accel_mps2: Vector3::from_array(accel_mps2),
            quat: Quaternion::from_array(quat),
            ang_vel_radps: Vector3::from_array(ang_vel_radps),
            ang_accel_radps2: Vector3::from_array(ang_accel_radps2),
        }
    }

    pub fn init() -> State{
        return State {
            pos_m: Vector3::zeros(),
            vel_mps: Vector3::zeros(),
            accel_mps2: Vector3::zeros(),
            quat: Quaternion::identity(),
            ang_vel_radps: Vector3::zeros(),
            ang_accel_radps2: Vector3::zeros(),
        }
    }
}

impl Integrate for State{
    fn get_derivative(&self)-> Self {
        // The State object is in the body frame
        let mut d = State::init();

        d.pos_m = self.vel_mps.clone();
        d.vel_mps = self.accel_mps2.clone();
        d.quat = self.quat.derivative(self.ang_vel_radps);
        d.ang_vel_radps = self.ang_accel_radps2.clone();

        return d
    }
}

impl Save for State{
    fn save(self, mut runtime: BasicRunTime) where Self: Sized {
        runtime.add_or_set("State.pos.x [m]", self.pos_m.x);
        runtime.add_or_set("State.pos.y [m]", self.pos_m.y);
        runtime.add_or_set("State.pos.z [m]", self.pos_m.z);

        runtime.add_or_set("State.vel.x [m/s]", self.vel_mps.x);
        runtime.add_or_set("State.vel.y [m/s]", self.vel_mps.y);
        runtime.add_or_set("State.vel.z [m/s]", self.vel_mps.z);

        runtime.add_or_set("State.accel.x [m/s^2]", self.accel_mps2.x);
        runtime.add_or_set("State.accel.y [m/s^2]", self.accel_mps2.y);
        runtime.add_or_set("State.accel.z [m/s^2]", self.accel_mps2.z);

        runtime.add_or_set("State.quat.a [-]", self.quat.a);
        runtime.add_or_set("State.quat.b [-]", self.quat.b);
        runtime.add_or_set("State.quat.c [-]", self.quat.c);
        runtime.add_or_set("State.quat.d [-]", self.quat.d);

        runtime.add_or_set("State.ang_vel.x [rad/s]", self.ang_vel_radps.x);
        runtime.add_or_set("State.ang_vel.y [rad/s]", self.ang_vel_radps.y);
        runtime.add_or_set("State.ang_vel.z [rad/s]", self.ang_vel_radps.z);

        runtime.add_or_set(
            "State.ang_accel.x [rad/s^2]", self.ang_accel_radps2.x
        );
        runtime.add_or_set(
            "State.ang_accel.y [rad/s^2]", self.ang_accel_radps2.y
        );
        runtime.add_or_set(
            "State.ang_accel.z [rad/s^2]", self.ang_accel_radps2.z
        );

    }
}

// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct Inputs{
    pub local_level_force_n: Vector3,
    pub local_level_moment_nm: Vector3,
    pub body_force_n: Vector3,
    pub body_moment_nm: Vector3
}


// TODO: Update Forces to act in the body frame
impl Inputs{
    pub fn new(
        local_level_force_n: [f64; 3],
        local_level_moment_nm: [f64; 3],
        body_force_n: [f64; 3],
        body_moment_nm: [f64; 3],
    ) -> Inputs {
            return Inputs{
                local_level_force_n:
                    Vector3::from_array(local_level_force_n),
                local_level_moment_nm:
                    Vector3::from_array(local_level_moment_nm),
                body_force_n:
                    Vector3::from_array(body_force_n),
                body_moment_nm:
                    Vector3::from_array(body_moment_nm),
            }
    }

    pub fn zeros() -> Inputs{
        return Inputs{
            local_level_force_n: Vector3::zeros(),
            body_force_n: Vector3::zeros(),
            local_level_moment_nm: Vector3::zeros(),
            body_moment_nm: Vector3::zeros()

        }
    }

    pub fn effect_state(&self, state: &mut State, mass_props: &MassProperties){
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics
        //
        // Notes:
        //     Forces and moments act about the body frame

        let total_forces_n = 
            self.local_level_force_n + 
            state.quat.transform(self.body_force_n);

        let total_moments_nm =  
            self.local_level_moment_nm + 
            state.quat.transform(self.body_moment_nm);

        // F = ma
        state.accel_mps2 = total_forces_n / mass_props.mass_cg_kg;

        // I * w
        let i_dot_w =
            mass_props.i_tensor_cg_kgpm2 * state.ang_vel_radps;

        // w x (I * w)
        let w_cross_i_dot_w = state.ang_vel_radps.cross(&i_dot_w);

        // M - (w x (I * w))
        let m_minus_w_cross_i_dot_w = total_moments_nm - w_cross_i_dot_w;

        // alpha = I^-1(M-(w × (I * w))
        state.ang_accel_radps2 =
            mass_props.inv_i_tensor_cg_kgpm2 * m_minus_w_cross_i_dot_w;

    }

}

// ----------------------------------------------------------------------------
// Mass Properties
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct MassProperties{
    pub mass_cg_kg: f64,
    pub i_tensor_cg_kgpm2: Matrix3x3,
    pub inv_i_tensor_cg_kgpm2: Matrix3x3
}

impl MassProperties{
    pub fn new(
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: [f64; 9]
    ) -> MassProperties{

        let i_tensor: Matrix3x3 = Matrix3x3::from_array(i_tensor_cg_kgpm2);

        let inv: Matrix3x3 = i_tensor.inv()
            .expect("i_tensor_cg_kgpm2 was not invertible");

        return MassProperties {
            mass_cg_kg: mass_cg_kg,
            i_tensor_cg_kgpm2: i_tensor,
            inv_i_tensor_cg_kgpm2: inv
        }
    }

    pub fn zeros() -> MassProperties{
        return MassProperties {
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2:Matrix3x3::identity()
        }
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use crate::test::almost_equal_array;

    use super::*;
    struct TestObject{
        state: State,
        inputs: Inputs,
        mass_props: MassProperties,
    }

    impl TestObject{
        fn zeros()->TestObject{
            return TestObject {
                state: State::init(),
                inputs: Inputs::zeros(),
                mass_props: MassProperties::zeros()
            }
        }
        fn update(&mut self, dt: f64){
            self.inputs.effect_state(&mut self.state, &self.mass_props);

            self.state = self.state.rk4(dt);
        }
    }

    #[allow(unused)]
    #[test]
    fn zeros(){
        let inputs = Inputs::zeros();
        let state = State::init();
        let mass_props = MassProperties::zeros();
    }

    #[allow(unused)]
    #[test]
    fn new(){
        let inputs = Inputs::new(
            [1.0, 1.0, 1.1],
            [1.0, 1.0, 1.1],
            [1.0, 1.0, 1.1],
            [1.0, 1.0, 1.1],
        );
        let state = State::new(
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0]
        );

        let mass_props = MassProperties::new(
            10.0,
            [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ],

        );

        assert_eq!(
            mass_props.inv_i_tensor_cg_kgpm2,
            mass_props.i_tensor_cg_kgpm2
        );
    }

    #[allow(unused)]
    #[test]
    fn local_translation(){
        let mut object = TestObject::zeros();

        // Set Forces
        object.state.pos_m = Vector3::new(0.0, 1.0, 2.0);
        object.inputs.local_level_force_n = Vector3::new(1.0, 1.0, 1.0);

        let dt = 1e-4;
        let max_int = (5.0 / dt) as usize;

        let mut time = 0.0;
        for i in 0..max_int{
            object.update(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        almost_equal_array(
            &object.state.vel_mps.to_array(),
            &[5.0, 5.0, 5.0]
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        almost_equal_array(
            &object.state.pos_m.to_array(),
            &[12.5, 13.5, 14.5]
        );
    }

    #[test]
    fn local_rotation(){

        for i in 0..2{
            let mut object = TestObject::zeros();
            let mut final_w = [0.0, 0.0, 0.0];
            let mut final_theta = [0.0, 0.0, 0.0];
            let mut local_level_moments_nm = [0.0, 0.0, 0.0];


            final_w[i] = 0.5;
            final_theta[i] = 1.25;
            local_level_moments_nm[i] = 0.1;


            object.state.quat = Quaternion::identity();

            object.inputs.local_level_moment_nm = Vector3::from_array(
                local_level_moments_nm
            );

            let dt = 0.25;
            let max_int = (5.0 / dt) as i64;

            for _ in 0..max_int{
                object.update(dt);
            }

            // w  = w_0 + alpha*t
            almost_equal_array(
                &object.state.ang_vel_radps.to_array(),
                &final_w
            );

            // theta = w_0 * t + alpht * t^2 / 2
            almost_equal_array(
                &object.state.quat.to_euler().to_array(),
                &final_theta
            );
        }
    }
}