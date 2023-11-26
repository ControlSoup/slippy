use sim::{runtime::{Runtime, Save}, integration::Integrate};
use physics::rigidbody::RigidBody;
use strapdown::vector::Vector3;

mod strapdown;
mod sim;
mod test;
mod physics;

fn main() {
    let mut runtime = Runtime::new(10.0, 1e-3, "time [s]");

    let mut test_object = RigidBody::identity();
    test_object.body_moment_nm = Vector3::new(0.1, 0.1, 0.1);
    test_object.body_force_n = Vector3::new(0.3, 0.01, 0.25);
    let dt = runtime.get_dx();

    let index =(10.0 / 1e-3) as usize;
    for _ in  0..index{
        test_object = test_object.rk4(dt.clone());
        test_object.save("Test".to_string(), &mut runtime);
        runtime.increment();

        if runtime.get_x() > 3.0{
            test_object.body_force_n = Vector3::new(-0.2, -0.2, -0.25);
        };
    }

    runtime.export_to_csv("test", "results/data");
}
