use sim::{Runtime, Save, Integrate};

mod strapdown;
mod sim;
mod test;
mod units;
mod physics;
mod control;
mod instrumentation;
mod geo;
mod forward_models;

fn main() {
    let mut runtime = sim::Runtime::new(20.0, 1e-3, "time [s]");

    let mut test_object = physics::RigidBody::new(
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

    test_object.body_moment_nm = strapdown::Vector3::new(0.1, 0.1, 0.1);

    // Useful vars
    let dt = runtime.get_dx();
    let i_quat = strapdown::Quaternion::identity();

    // PID
    let mut altitude_ramp = control::Ramp::new(0.0, 5.0, 0.25);
    let mut pid_alt = control::PID::new(0.4, 0.00, 0.0, 0.0);
    let mut pid_x = control::PID::new(0.25, 0.001, 0.0, 0.0);
    let mut pid_y = control::PID::new(0.25, 0.001, 0.0, 0.0);
    let mut pid_z = control::PID::new(0.1, 0.001, 0.0, 0.0);

    // Servo TVC
    let tvc = forward_models::ServoTVC::from_points([0.0, -0.04], [0.021, 0.0], [0.021, -0.04]);

    // Instrumentation
    let mut test_sensor = instrumentation::BasicSensor::new_simple_from_variance(0.01,"m");

    while runtime.is_running{

        // Instrumentation update
        test_sensor.output(test_object.get_pos_m().z);

        // Save Data
        test_object.save_data_verbose("hopper", &mut runtime);
        test_sensor.save_data_verbose("test_sensor", &mut runtime);
        altitude_ramp.save_data_verbose("target_position", &mut runtime);
        pid_alt.save_data_verbose("pid_alt", &mut runtime);
        pid_x.save_data_verbose("pid_x", &mut runtime);
        pid_y.save_data_verbose("pid_y", &mut runtime);
        pid_z.save_data_verbose("pid_z", &mut runtime);

        // Pid Controllers
        pid_alt.setpoint = altitude_ramp.output(dt);

        let euler_error = (test_object.get_quat().error(i_quat)).to_euler();

        test_object.body_moment_nm.x = pid_x.ouput(euler_error.x, dt);
        test_object.body_moment_nm.y = pid_y.ouput(euler_error.y, dt);
        test_object.body_moment_nm.z = pid_z.ouput(euler_error.z, dt);
        test_object.body_force_n.z = pid_alt.ouput(test_object.get_pos_m().z, dt) + 9.8;


        // Integrate and increment sim
        test_object = test_object.rk4(dt);
        runtime.increment();
    }

    runtime.export_to_csv("test", "results/data");
}
