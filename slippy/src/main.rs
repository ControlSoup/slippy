use sim::{runtime::{Runtime, self, Save}, integration::Integrate};
use physics::rigidbody::RigidBody;
use strapdown::vector::Vector3;

mod strapdown;
mod sim;
mod test;
mod physics;

fn main() {
    let mut runtime = Runtime::new(10.0, 1e-3, "time [s]");

    let mut test_object = RigidBody::identity();
    test_object.local_level_force_n = Vector3::new(1.0, 1.0, 1.0);
    let dt = runtime.get_dx();

    let index =(10.0 / 1e-3) as usize;
    for i in  0..index{
        test_object = test_object.rk4(dt.clone());
        test_object.save("Test".to_string(), &mut runtime);
        runtime.increment();
    }

    runtime.export_to_csv("test", "");
}
