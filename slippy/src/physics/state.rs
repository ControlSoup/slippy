
use derive_more;
use crate::strapdown::{vector::Vector3, quaternion::Quaternion};
use crate::sim::{integration::Integrate, runtime::{Runtime,Save}};
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
    pos_m: Vector3,
    vel_mps: Vector3,
    pub accel_mps2: Vector3,
    quat: Quaternion,
    ang_vel_radps: Vector3,
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
    fn save(self, mut runtime: Runtime) where Self: Sized {
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

pub trait EffectState{
    fn effect_state(self, mut state: &State) where Self: Sized{

    }
}