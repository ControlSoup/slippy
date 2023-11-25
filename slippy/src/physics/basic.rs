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
                mass_props: MassProperties::identity()
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
        let mass_props = MassProperties::identity();
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