use std::f64::consts::PI;

use sim::{runtime::{Runtime, Save}, integration::Integrate};
use physics::rigidbody::RigidBody;
use strapdown::{vector::Vector3, quaternion::Quaternion};

mod strapdown;
mod sim;
mod test;
mod units;
mod physics;

fn main() {
    let mut runtime = Runtime::new(10.0, 1e-3, "time [s]");

    let mut test_object = RigidBody::new(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        Vector3::new(0.0 , 0.1, 0.2).to_quat().to_array(),
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let mut test_object = RigidBody::identity();

    test_object.body_moment_nm = Vector3::new(0.1, 0.1, 0.1);
    let dt = runtime.get_dx();

    let index =(10.0 / 1e-3) as usize;
    for _ in  0..index{
        test_object = test_object.rk4(dt.clone());
        test_object.save_data("Test".to_string(), &mut runtime);
        runtime.increment();
    }

    runtime.export_to_csv("test", "results/data");
}
