#[allow(dead_code, unused)]

// Standard
use std::ops::{Mul, Div};

// 3d Party
use nalgebra as na;
use na::{Vector3, Vector4, Matrix3};
use derive_more::Add;

// Local
use crate::gnc::strapdown;
use crate::sim::integration::Integrate;

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

#[derive(Debug, Add, Clone)]
pub struct State{
    pub pos_m: Vector3<f64>,
    pub vel_mps: Vector3<f64>,
    pub accel_mps2: Vector3<f64>,
    pub quat: Vector4<f64>,
    pub ang_vel_radps: Vector3<f64>,
    pub ang_accel_radps2: Vector3<f64>,
}

impl State{
    pub fn new(
        pos_m: &[f64; 3],
        vel_mps: &[f64; 3],
        accel_mps2: &[f64; 3],
        quat: &[f64; 4],
        ang_vel_radps: &[f64; 3],
        ang_accel_radps2: &[f64; 3],
    ) -> State{
        return State {
            pos_m: Vector3::from_row_slice(pos_m),
            vel_mps: Vector3::from_row_slice(vel_mps),
            accel_mps2: Vector3::from_row_slice(accel_mps2),
            quat: Vector4::from_row_slice(quat),
            ang_vel_radps: Vector3::from_row_slice(ang_vel_radps),
            ang_accel_radps2: Vector3::from_row_slice(ang_accel_radps2),
        }
    }

    pub fn zeros() -> State{
        return State {
            pos_m: Vector3::zeros(),
            vel_mps: Vector3::zeros(),
            accel_mps2: Vector3::zeros(),
            quat: Vector4::from_row_slice(
                &[1.0, 1.0, 1.0, 1.0]
            ),
            ang_vel_radps: Vector3::zeros(),
            ang_accel_radps2: Vector3::zeros(),
        }
    }
}

impl Mul<f64> for State{
    type Output = State;
    fn mul(self, rhs: f64) -> State{
        return State {
            pos_m: self.pos_m * rhs,
            vel_mps: self.vel_mps * rhs,
            accel_mps2: self.accel_mps2 * rhs,
            quat: self.quat * rhs,
            ang_vel_radps: self.ang_vel_radps * rhs,
            ang_accel_radps2: self.ang_accel_radps2 * rhs
        }
    }
}

impl Div<f64> for State{
    type Output = State;
    fn div(self, rhs: f64) -> State{
        return State {
            pos_m: self.pos_m / rhs,
            vel_mps: self.vel_mps / rhs,
            accel_mps2: self.accel_mps2 * rhs,
            quat: self.quat / rhs,
            ang_vel_radps: self.ang_vel_radps / rhs,
            ang_accel_radps2: self.ang_accel_radps2 / rhs
        }
    }
}

impl Integrate for State{
    fn get_derivative(&self)-> Self {
        let mut d = State::zeros();

        d.pos_m = self.vel_mps.clone();
        d.vel_mps = self.accel_mps2.clone();

        // Quat update
        d.quat = strapdown::quat_rate(
            self.quat,
            self.ang_vel_radps
        );

        d.ang_vel_radps = self.ang_accel_radps2.clone();
        return d
    }
}

// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct Inputs{
    pub force_n: Vector3<f64>,
    pub moment_nm: Vector3<f64>
}


// TODO: Update Forces to act in the body frame
impl Inputs{
    pub fn new(
        force_n: &[f64; 3],
        moment_nm: &[f64; 3],
    ) -> Inputs {
            return Inputs{
                force_n: Vector3::from_row_slice(force_n),
                moment_nm: Vector3::from_row_slice(moment_nm),
            }
    }

    pub fn zeros() -> Inputs{
        return Inputs{
            force_n: Vector3::zeros(),
            moment_nm: Vector3::zeros(),
        }
    }


    pub fn effect_state(&self, state: &mut State, mass_props: &MassProperties){
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics

        // F = ma
        state.accel_mps2 = &self.force_n / mass_props.mass_cg_kg;

        // I * w
        let i_dot_w: Vector3<f64> =
            mass_props.i_tensor_cg_kgpm2 * state.ang_vel_radps;

        // w x (I * w)
        let w_cross_i_dot_w: Vector3<f64> = state.ang_vel_radps.cross(&i_dot_w);

        // M - (w x (I * w))
        let m_minus_w_cross_i_dot_w: Vector3<f64> =
            &self.moment_nm - w_cross_i_dot_w;

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
    pub i_tensor_cg_kgpm2: Matrix3<f64>,
    pub inv_i_tensor_cg_kgpm2: Matrix3<f64>
}

impl MassProperties{
    pub fn new(
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: &[[f64; 3]; 3]
    ) -> MassProperties{

        let i_tensor: Matrix3<f64> = Matrix3::from_row_slice(&[
            i_tensor_cg_kgpm2[0][0],i_tensor_cg_kgpm2[0][1],i_tensor_cg_kgpm2[0][2],
            i_tensor_cg_kgpm2[1][0],i_tensor_cg_kgpm2[1][1],i_tensor_cg_kgpm2[1][2],
            i_tensor_cg_kgpm2[2][0],i_tensor_cg_kgpm2[2][1],i_tensor_cg_kgpm2[2][2]
        ]);

        let inv: Matrix3<f64> = i_tensor
            .try_inverse()
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
            i_tensor_cg_kgpm2: Matrix3::identity(),
            inv_i_tensor_cg_kgpm2:Matrix3::identity()
        }
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    struct TestObject{
        state: State,
        inputs: Inputs,
        mass_props: MassProperties,
    }

    impl TestObject{
        fn zeros()->TestObject{
            return TestObject {
                state: State::zeros(),
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
    fn test_zeros(){
        let inputs = Inputs::zeros();
        let state = State::zeros();
        let mass_props = MassProperties::zeros();
    }

    #[allow(unused)]
    #[test]
    fn test_new(){
        let inputs = Inputs::new(
            &[1.0, 1.0, 1.1],
            &[1.0, 1.0, 1.1],
        );
        let state = State::new(
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0],
            &[0.0, 0.0, 0.0, 0.0],
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0]
        );

        let mass_props = MassProperties::new(
            10.0,
            &[
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ],

        );

        assert_eq!(
            mass_props.inv_i_tensor_cg_kgpm2,
            mass_props.i_tensor_cg_kgpm2
        );
    }

    #[allow(unused)]
    #[test]
    fn test_translation(){
        let mut object = TestObject::zeros();
        // Set Forces
        object.state.pos_m = Vector3::from_row_slice(&[0.0, 1.0, 2.0]);
        object.inputs.force_n = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);

        for _ in 0..5{
            object.update(1.0);
        }

        assert_eq!(
            object.state.vel_mps,
            Vector3::from_row_slice(&[5.0, 5.0, 5.0])
        );
        assert_eq!(
            object.state.pos_m,
            Vector3::from_row_slice(&[25.0, 26.0, 27.0])
        );
    }

    #[test]
    fn test_rotation(){
        let mut object = TestObject::zeros();

        object.state.ang_vel_radps = Vector3::from_row_slice(&[0.0, 1.0, 2.0]);
        // Set Forces
        object.inputs.moment_nm = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);

        for _ in 0..5{
            object.update(1.0);
        }
        assert_eq!(
            object.state.ang_vel_radps,
            Vector3::from_row_slice(&[5.0, 6.0, 7.0])
        );
        assert_ne!(
            object.state.quat,
            Vector4::from_row_slice(&[1.0, 1.0, 1.0, 1.0])
        );
    }

    #[test]
    fn test_rotation_and_translation(){
        let mut object = TestObject::zeros();

        // Set Forces
        object.inputs.moment_nm = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);
        object.inputs.force_n = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);

        for _ in 0..5{
            object.update(1.0);
        }

        assert_eq!(
            object.state.ang_vel_radps,
            Vector3::from_row_slice(&[5.0, 5.0, 5.0])
        );
        assert_eq!(
            object.state.vel_mps,
            Vector3::from_row_slice(&[5.0, 5.0, 5.0])
        );
    }
}