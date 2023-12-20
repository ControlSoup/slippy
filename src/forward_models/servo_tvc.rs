use std::f64::consts::PI;

use crate::geo;
use crate::sim;
use crate::units::deg_to_rad;


/// Simulates a single axis tvc mechanism as a four bar linkage
pub struct ServoTVC{
    a: geo::Line2, // Input link (Servo)
    b: geo::Line2, // Ouput link (Thrust Axis)
    g: geo::Line2, // Ground link (Servo Origin to Thrust Axis Origin)
    l: geo::Line2, // Free link (TVC Connection arm)
    servo_angle_rad: f64,
    tvc_angle_rad: f64
}

impl ServoTVC{
    fn new(
        a: geo::Line2,
        b: geo::Line2,
        g: geo::Line2,
        l: geo::Line2  
    ) -> ServoTVC{
        return ServoTVC{
            a,
            b,
            g,
            l,
            servo_angle_rad: a.angle_x_rad(),
            tvc_angle_rad: b.angle_x_rad()
        }
    }

    fn from_points(
        p2: [f64; 2],
        p3: [f64; 2],
        p4: [f64; 2]
    ) -> ServoTVC{
        return ServoTVC::new(
            geo::Line2::new(p3[0], p3[1], p4[0], p4[1]),
            geo::Line2::new(0.0, 0.0, p2[0], p2[1]),
            geo::Line2::new(0.0, 0.0, p3[0], p3[1]),
            geo::Line2::new(p2[0], p2[1], p4[0], p4[1])
        )
    }

    fn new_basic(
        servo_start_y_m: f64,
        servo_radius_m: f64,
        connection_length_m: f64
    ) -> ServoTVC{
        return ServoTVC::from_points(
            [0.0, servo_start_y_m - servo_radius_m],
            [connection_length_m, servo_start_y_m],
            [connection_length_m, servo_start_y_m - servo_radius_m]
        )
    }

    fn set_servo_angle_rad(&mut self, servo_angle_rad: f64){
        self.servo_angle_rad = servo_angle_rad;
        let alpha = geo::PI_THREE_HALFS + servo_angle_rad;

        // Define new servo vector
        self.a = geo::Line2::from_angle_rad(
            self.a.start_x_m, 
            self.a.start_y_m, 
            self.a.length_m(), 
            alpha
        );

        // Calculate the intersection between the circles defind by tvc and linkage
        let c0 = geo::Circle::new(
            self.a.end_x_m,
            self.a.end_y_m,
            self.l.length_m()
        );

        let c1 = geo::Circle::new(
            self.b.start_x_m, self.b.start_y_m, self.b.length_m()
        );

        let intersect_l_b = match c1.intersect_circle(&c0){
            None => panic!("Bad: \n b: {:?}\n Angle: {:?}\n a: {:?}\n l: {:?}\n c0: {:?}\n c1:{:?}", self.b, self.servo_angle_rad, self.a, self.l, c0, c1),
            Some(vector) => vector
        };


        self.b.end_x_m = intersect_l_b.i;
        self.b.end_y_m = intersect_l_b.j;
        self.l = geo::Line2::new(
            self.b.end_x_m,
            self.b.end_y_m,
            self.a.end_x_m,
            self.a.end_y_m
        );


        self.get_tvc_angle_rad();
    }

    fn get_tvc_angle_rad(&mut self) -> f64{
        let beta = self.b.angle_x_rad();
        self.tvc_angle_rad = beta - geo::PI_THREE_HALFS;

        return self.tvc_angle_rad
    }
}
// ----------------------------------------------------------------------------
// Data recording
// ----------------------------------------------------------------------------

impl sim::Save for ServoTVC{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.tvc_angle [rad]").as_str(),self.tvc_angle_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.servo_angle [rad]").as_str(),self.servo_angle_rad
        );

    }

    fn save_data_verbose(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        self.save_data(node_name, runtime);
        
        // Points
        runtime.add_or_set(format!(
            "{node_name}.a.start_x [m]").as_str(), self.a.start_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.a.start_y [m]").as_str(), self.a.start_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.a.end_x [m]").as_str(), self.a.end_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.a.end_y [m]").as_str(), self.a.end_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.b.start_x [m]").as_str(), self.b.start_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.b.start_y [m]").as_str(), self.b.start_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.b.end_x [m]").as_str(), self.b.end_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.b.end_y [m]").as_str(), self.b.end_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.g.start_x [m]").as_str(), self.g.start_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.g.start_y [m]").as_str(), self.g.start_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.g.end_x [m]").as_str(), self.g.end_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.g.end_y [m]").as_str(), self.g.end_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.l.start_x [m]").as_str(), self.l.start_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.l.start_y [m]").as_str(), self.l.start_y_m
        );
        runtime.add_or_set(format!(
            "{node_name}.l.end_x [m]").as_str(), self.l.end_x_m
        );
        runtime.add_or_set(format!(
            "{node_name}.l.end_y [m]").as_str(), self.l.end_y_m
        );

        // Lengths
        runtime.add_or_set(format!(
            "{node_name}.a.length [m]").as_str(), self.a.length_m()
        );
        runtime.add_or_set(format!(
            "{node_name}.b.length [m]").as_str(), self.b.length_m()
        );
        runtime.add_or_set(format!(
            "{node_name}.g.length [m]").as_str(), self.g.length_m()
        );
        runtime.add_or_set(format!(
            "{node_name}.l.length [m]").as_str(), self.l.length_m()
        );
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::geo::{PI_HALF,PI_QUARTER};
    use crate::sim::Save;
    use super::*;
    use approx::assert_relative_eq;
    use crate::test::almost_equal_array;

    #[test]
    fn sin_sweep(){
        let mut runtime = sim::Runtime::new(PI * 2.0, 1e-3, "angle [rad]");
        let mut tvc = ServoTVC::new_basic(-1.5, 0.5, 1.0);

        // Ensure TVC axis
        assert_relative_eq!(
            tvc.b.start_x_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.b.end_x_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.b.start_y_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.b.end_y_m,
            -2.0,
            max_relative=1e-6
        );

        // Ensure TVC arm is intialized correctly
        assert_relative_eq!(
            tvc.a.start_x_m,
            1.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.a.end_x_m,
            1.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.a.start_y_m,
            -1.5,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.a.end_y_m,
            -2.0,
            max_relative=1e-6
        );

        // Ensure ground is intialized correctly
        assert_relative_eq!(
            tvc.g.start_x_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.g.end_x_m,
            1.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.g.start_y_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.g.end_y_m,
            -1.5,
            max_relative=1e-6
        );

        // Ensure TVC arm is intialized correctly
        assert_relative_eq!(
            tvc.l.start_x_m,
            0.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.l.end_x_m,
            1.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.l.start_y_m,
            -2.0,
            max_relative=1e-6
        );
        assert_relative_eq!(
            tvc.l.end_y_m,
            -2.0,
            max_relative=1e-6
        );

        while runtime.is_running{
        tvc.save_data_verbose("tvc", &mut runtime);
            tvc.set_servo_angle_rad(runtime.get_x());
            runtime.increment();
        };

        runtime.export_to_csv("tvc_sinsweep", "results/data")
    }
}
