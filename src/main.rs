use geo::{PI_HALF, Vector2, Vector3};
use sim::{Runtime, Save, Integrate};

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
        [0.0, 0.0, 0.0],
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

    test_object.body_moment_nm = geo::Vector3::new(0.1, 0.1, 0.1);

    // Useful vars
    let dt = runtime.get_dx();
    let i_quat = geo::Quaternion::identity();
    let thrust = 10.0;

    // PID
    let mut altitude_ramp = control::Ramp::new(0.0, 5.0, 0.25);
    let mut pid_alt = control::PID::new(0.4, 0.00, 0.0, 0.0);
    let mut pid_x = control::PID::new(0.25, 0.001, 0.0, 0.0);
    let mut pid_y = control::PID::new(0.25, 0.001, 0.0, 0.0);
    let mut pid_z = control::PID::new(0.1, 0.001, 0.0, 0.0);

    // Servo TVC
    let mut tvc_x = forward_models::FourBarLinkage::from_points([0.0, -0.04], [0.021, 0.0], [0.021, -0.021], PI_HALF);
    let mut tvc_y = forward_models::FourBarLinkage::from_points([0.0, -0.04], [0.021, 0.0], [0.021, -0.021], PI_HALF);

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
        tvc_x.save_data_verbose("tvc_x", &mut runtime);
        tvc_y.save_data_verbose("tvc_x", &mut runtime);

        // Pid Controllers
        pid_alt.setpoint = altitude_ramp.output(dt);

        let euler_error = (test_object.get_quat().error(i_quat)).to_euler();


        // Tvc
        tvc_x.set_servo_angle_rad(pid_x.ouput(euler_error.x, dt));
        tvc_y.set_servo_angle_rad(pid_y.ouput(euler_error.y, dt));

        let thrust_x = tvc_x.get_thrust_vector();
        let thrust_y = tvc_x.get_thrust_vector();

        test_object.body_force_n = 
            (Vector3::new(thrust_x.i, 0.0, thrust_x.j) + Vector3::new(0.0, thrust_y.i, thrust_y.j)).to_unit() * thrust
        ;


        // Integrate and increment sim
        test_object = test_object.rk4(dt);
        runtime.increment();
    }

    runtime.export_to_csv("test", "results/data");
}
