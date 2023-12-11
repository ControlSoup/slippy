use std::f64::consts::PI;

use crate::geo;
use crate::geo::PI_THREE_HALFS;
use crate::sim;


/// Simulates a single axis tvc mechanism as a four bar linkage
pub struct ServoTVC{
    a: geo::Vector2, // Input link (Servo)
    b: geo::Vector2, // Ouput link (Thrust Axis)
    g: geo::Vector2, // Ground link (Servo Origin to Thrust Axis Origin)
    l: geo::Vector2, // Free link (TVC Connection arm)
    servo_angle_rad: f64,
    tvc_angle_rad: f64
}

impl ServoTVC{
    fn new(
        a: geo::Vector2,
        b: geo::Vector2,
        g: geo::Vector2,
        l: geo::Vector2
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
            geo::Vector2::from_points(p3[0], p3[1], p4[0], p4[1]),
            geo::Vector2::from_points(0.0, 0.0, p2[0], p2[1]),
            geo::Vector2::from_points(0.0, 0.0, p3[0], p3[1]),
            geo::Vector2::from_points(p2[0], p2[1], p4[0], p4[1])
        )
    }

    fn to_points_array(&self) -> [f64; 8]{
        let _u = self.g + self.a;
        return [0.0, 0.0, self.b.i, self.b.j, self.g.i, self.g.j, _u.i, _u.j]
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
        self.a = geo::Vector2::from_angle_rad(self.a.norm(), alpha);

        // Precompute a couple things duplicates for readability

        // Calculate the intersection between the circles defind by tvc and linkage
        let _u =  self.g + self.a;
        let _v = _u.get_perpendicular();

        let _s = (1.0 + ((self.l.norm_sqr() - self.b.norm_sqr()) / _u.norm_sqr())) / 2.0;
        let _t = ((self.l.norm_sqr() / _u.norm_sqr()) - _s.powf(2.0)).sqrt();

        let _new_coord = _u + (_u * _s) + (_v * _t);
        self.b = geo::Vector2::new(_new_coord.i, _new_coord.j);
        self.l = geo::Vector2::from_points(self.b.i, self.b.j, _u.i, _u.j);


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

        // Defining points
        let points = self.to_points_array();
        runtime.add_or_set(format!(
            "{node_name}.p1_x [m]").as_str(),points[0]
        );
        runtime.add_or_set(format!(
            "{node_name}.p1_y [m]").as_str(),points[1]
        );
        runtime.add_or_set(format!(
            "{node_name}.p2_x [m]").as_str(),points[2]
        );
        runtime.add_or_set(format!(
            "{node_name}.p2_y [m]").as_str(),points[3]
        );
        runtime.add_or_set(format!(
            "{node_name}.p3_x [m]").as_str(),points[4]
        );
        runtime.add_or_set(format!(
            "{node_name}.p3_y [m]").as_str(),points[5]
        );
        runtime.add_or_set(format!(
            "{node_name}.p4_x [m]").as_str(),points[6]
        );
        runtime.add_or_set(format!(
            "{node_name}.p4_y [m]").as_str(),points[7]
        );

        // Vectors
        runtime.add_or_set(format!(
            "{node_name}.a_norm [-]").as_str(),self.a.norm()
        );
        runtime.add_or_set(format!(
            "{node_name}.b_norm [-]").as_str(),self.a.norm()
        );
        runtime.add_or_set(format!(
            "{node_name}.g_norm [-]").as_str(),self.g.norm()
        );
        runtime.add_or_set(format!(
            "{node_name}.l_norm [-]").as_str(),self.l.norm()
        );
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use approx::assert_relative_eq;
    use crate::{test::almost_equal_array, sim::{self, Save}};

    #[test]
    fn sin_sweep(){
        let mut runtime = sim::Runtime::new(2.0 * PI, 1e-3, "angle [rad]");
        let mut tvc = ServoTVC::new_basic(-1.0, 1.0, 1.0);

        assert_relative_eq!(
            tvc.a.j,
            -1.0,
            max_relative = 1e-6
        );
        while runtime.is_running{
            tvc.save_data_verbose("tvc", &mut runtime);
            tvc.set_servo_angle_rad(runtime.get_x());
            runtime.increment();

        };

        runtime.export_to_csv("tvc_sinsweep", "results/data")
    }
}
