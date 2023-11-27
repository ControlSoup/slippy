use std::{f64::consts::PI, convert::identity};

use sim::{runtime::{Runtime, Save}, integration::Integrate};
use control::{pid::PID, ramp::Ramp};
use physics::rigidbody::RigidBody;
use strapdown::{vector::Vector3, quaternion::Quaternion, matrix::Matrix3x3};

mod strapdown;
mod sim;
mod test;
mod units;
mod physics;
mod control;

fn main() {
    let mut runtime = Runtime::new(20.0, 1e-3, "time [s]");

    let mut test_object = RigidBody::new(
        [0.0, 0.0, -9.8],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );

    test_object.body_moment_nm = Vector3::new(0.1, 0.1, 0.1);
    let dt = runtime.get_dx();
    let i_dcm = Matrix3x3::identity();
    let i_quat = Quaternion::identity();

    // PID
    let mut altitude_ramp = Ramp::new(0.0, 5.0, 0.25);
    let mut pid_alt = PID::new(0.4, 0.00, 0.0);
    let mut pid_x = PID::new(0.25, 0.001, 0.0);
    let mut pid_y = PID::new(0.25, 0.001, 0.0);
    let mut pid_z = PID::new(0.1, 0.001, 0.0);


    while runtime.is_running{


        test_object.save_data("hopper", &mut runtime);
        altitude_ramp.save_data("target_position", &mut runtime);
        pid_alt.save_data("pid_alt", &mut runtime);
        pid_x.save_data("pid_x", &mut runtime);
        pid_y.save_data("pid_y", &mut runtime);
        pid_z.save_data("pid_z", &mut runtime);

        test_object = test_object.rk4(dt);

        // Ramp to altitude
        let alt_error = altitude_ramp.output(dt) - test_object.get_pos_m().z;
        test_object.body_force_n.z = pid_alt.ouput(alt_error, dt) + 9.8;

        // Update controllers
        let euler_error = (i_quat * test_object.get_quat().conjugate()).to_euler();
        test_object.body_moment_nm.x = pid_x.ouput(euler_error.x, dt);
        test_object.body_moment_nm.y = pid_y.ouput(euler_error.y, dt);
        test_object.body_moment_nm.z = pid_z.ouput(euler_error.z, dt);


        runtime.increment();
    }

    runtime.export_to_csv("test", "results/data");
}
