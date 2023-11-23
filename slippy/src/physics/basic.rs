#[allow(dead_code, unused)]
// Standard
use std::ops::{Mul, Div};

// 3d Party

// Local
use crate::strapdown::vector::Vector;
use crate::strapdown::quaternion::Quaternion;
use crate::sim::integration::Integrate;

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
    pub pos_m: Vector,
    pub vel_mps: Vector,
    pub accel_mps2: Vector,
    pub quat: Quaternion,
    pub ang_vel_radps: Vector,
    pub ang_accel_radps2: Vector,
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
            pos_m: Vector::from_array(pos_m),
            vel_mps: Vector::from_array(vel_mps),
            accel_mps2: Vector::from_array(accel_mps2),
            quat: Quaternion::from_array(quat),
            ang_vel_radps: Vector::from_array(ang_vel_radps),
            ang_accel_radps2: Vector::from_array(ang_accel_radps2),
        }
    }

    pub fn init() -> State{
        return State {
            pos_m: Vector::zeros(),
            vel_mps: Vector::zeros(),
            accel_mps2: Vector::zeros(),
            quat: Quaternion::identity(),
            ang_vel_radps: Vector::zeros(),
            ang_accel_radps2: Vector::zeros(),
        }
    }
}

impl Integrate for State{
    fn get_derivative(&self)-> Self {
        // The State object is in the body frame
        let mut d = State::init();

        d.pos_m = self.vel_mps.clone();
        d.vel_mps = self.accel_mps2.clone();
        d.quat = self.quat.rate(self.ang_vel_radps);
        d.ang_vel_radps = self.ang_accel_radps2.clone();

        return d
    }
}

// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct Inputs{
    pub local_level_force_n: Vector,
    pub local_level_moment_nm: Vector,
    pub body_force_n: Vector,
    pub body_moment_nm: Vector
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
                    Vector::from_array(local_level_force_n),
                local_level_moment_nm:
                    Vector::from_array(local_level_moment_nm),
                body_force_n:
                    Vector::from_array(body_force_n),
                body_moment_nm:
                    Vector::from_array(body_moment_nm),
            }
    }

    pub fn zeros() -> Inputs{
        return Inputs{
            local_level_force_n: Vector::zeros(),
            body_force_n: Vector::zeros(),
            local_level_moment_nm: Vector::zeros(),
            body_moment_nm: Vector::zeros()

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
    fn test_zeros(){
        let inputs = Inputs::zeros();
        let state = State::init();
        let mass_props = MassProperties::zeros();
    }

    #[allow(unused)]
    #[test]
    fn test_new(){
        let inputs = Inputs::new(
            &[1.0, 1.0, 1.1],
            &[1.0, 1.0, 1.1],
            &[1.0, 1.0, 1.1],
            &[1.0, 1.0, 1.1],
        );
        let state = State::new(
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0],
            &[1.0, 0.0, 0.0, 0.0],
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
        object.inputs.local_level_force_n = Vector3::from_row_slice(&[1.0, 1.0, 1.0]);

        let dt = 0.001;
        let max_int = (5.0 / dt) as i64;

        let mut time = 0.0;
        for i in 0..max_int{

            object.update(dt);
        }

        // vf = vi + (f/m)t = [5.0, 5.0, 5.0]
        assert_relative_eq!(
            object.state.vel_mps,
            Vector3::from_row_slice(&[5.0, 5.0, 5.0]),
            max_relative = 1.0e-6
        );

        // x = vi * t + a * t^2 /2  = [12.5, 13.5, 14.5]
        assert_relative_eq!(
            object.state.pos_m,
            Vector3::from_row_slice(&[12.5, 13.5, 14.5]),
            max_relative = 1.0e-6
        );
    }

    #[test]
    fn test_pure_rotation(){
        let mut object = TestObject::zeros();

        object.state.quat = strapdown::euler_rad_to_quat(
            Vector3::from_row_slice(&[
                0.0,
                0.0,
                0.0
            ])
        );

        object.inputs.local_level_moment_nm = Vector3::from_row_slice(&[0.1,0.0,0.0]);

        let dt = 0.25;
        let max_int = (5.0 / dt) as i64;

        for _ in 0..max_int{
            println!("\n{}\n", strapdown::quat_to_euler_rad(object.state.quat));
            object.update(dt);
        }

        // w  = w_0 + alpha*t
        assert_relative_eq!(
            object.state.ang_vel_radps,
            Vector3::from_row_slice(&[
                0.5,
                0.0,
                0.0

            ]),
            max_relative=1e-6
        );

        // theta = w_0 * t + alpht * t^2 / 2
        assert_relative_eq!(
            strapdown::quat_to_euler_rad(object.state.quat),
            Vector3::from_row_slice(&[
                1.25,
                0.0,
                0.0
            ]),
            max_relative=1e-6
        )
    }
}