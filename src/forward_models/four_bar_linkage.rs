use crate::{geo, sim, control};


pub struct FourBarLinkage{
    a: geo::Line2, // Input link 
    b: geo::Line2, // Ouput link 
    g: geo::Line2, // Ground link 
    l: geo::Line2, // Free link 
    input_angle_rad: f64,
    output_angle_rad: f64,
    max_input_angle_rad: f64
}

impl FourBarLinkage{
    pub fn new(
        a: geo::Line2,
        b: geo::Line2,
        g: geo::Line2,
        l: geo::Line2,
        max_input_angle_rad: f64
    ) -> FourBarLinkage{
        return FourBarLinkage{
            a,
            b,
            g,
            l,
            input_angle_rad: a.angle_x_rad(),
            output_angle_rad: b.angle_x_rad(),
            max_input_angle_rad
        }
    }

    pub fn from_points(
        p2: [f64; 2],
        p3: [f64; 2],
        p4: [f64; 2],
        max_input_angle_rad:f64
    ) -> FourBarLinkage{
        return FourBarLinkage::new(
            geo::Line2::new(p3[0], p3[1], p4[0], p4[1]),
            geo::Line2::new(0.0, 0.0, p2[0], p2[1]),
            geo::Line2::new(0.0, 0.0, p3[0], p3[1]),
            geo::Line2::new(p2[0], p2[1], p4[0], p4[1]),
            max_input_angle_rad
        )
    }

    pub fn new_basic(
        servo_start_y_m: f64,
        servo_radius_m: f64,
        connection_length_m: f64,
        max_input_angle_rad: f64
    ) -> FourBarLinkage{
        return FourBarLinkage::from_points(
            [0.0, servo_start_y_m - servo_radius_m],
            [connection_length_m, servo_start_y_m],
            [connection_length_m, servo_start_y_m - servo_radius_m],
            max_input_angle_rad
        )
    }

    pub fn set_servo_angle_rad(&mut self, input_angle_rad: f64){

        self.input_angle_rad = control::clamp(input_angle_rad, self.max_input_angle_rad, -self.max_input_angle_rad);

        let alpha = geo::PI_THREE_HALFS + self.input_angle_rad;

        // Define new servo vector
        self.a = geo::Line2::from_angle_rad(
            self.a.start_x_m, 
            self.a.start_y_m, 
            self.a.length_m(), 
            alpha
        );

        // Calculate the intersection between the circles defind by four_bar and linkage
        let c0 = geo::Circle::new(
            self.a.end_x_m,
            self.a.end_y_m,
            self.l.length_m()
        );

        let c1 = geo::Circle::new(
            self.b.start_x_m, self.b.start_y_m, self.b.length_m()
        );

        let intersect_l_b = match c1.intersect_circle(&c0){
            None => panic!("Bad Intersect"),
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

    pub fn get_tvc_angle_rad(&mut self) -> f64{
        let beta = self.b.angle_x_rad();
        self.output_angle_rad = beta - geo::PI_THREE_HALFS;

        return self.output_angle_rad
    }

    pub fn get_thrust_vector(&mut self) -> geo::Vector2{
        let beta = self.b.angle_x_rad();
        self.output_angle_rad = beta - geo::PI_THREE_HALFS;

        return geo::Vector2::from_angle_rad(self.b.length_m(), self.output_angle_rad)
    }
}

// ----------------------------------------------------------------------------
// Data recording
// ----------------------------------------------------------------------------

impl sim::Save for FourBarLinkage{
    fn save_data(&self, node_name: &str, runtime: &mut sim::Runtime) where Self: Sized {
        runtime.add_or_set(format!(
            "{node_name}.tvc_angle [rad]").as_str(),self.output_angle_rad
        );
        runtime.add_or_set(format!(
            "{node_name}.servo_angle [rad]").as_str(),self.input_angle_rad
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

    use crate::sim::Save;
    use super::*;
    use approx::assert_relative_eq;
    #[test]
    fn sin_sweep(){
        let mut runtime = sim::Runtime::new(PI * 2.0, 1e-2, "angle [rad]");
        let mut four_bar = FourBarLinkage::new_basic(-1.5, 0.5, 1.0, 3.0 * PI);

        // Ensure Main axis
        assert_relative_eq!(
            four_bar.b.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.end_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.start_y_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        // Ensure arm is intialized correctly
        assert_relative_eq!(
            four_bar.a.start_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.start_y_m,
            -1.5,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        // Ensure ground is intialized correctly
        assert_relative_eq!(
            four_bar.g.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.start_y_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.end_y_m,
            -1.5,
            max_relative=1e-2
        );

        // Ensure linkage is intialized correctly
        assert_relative_eq!(
            four_bar.l.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.start_y_m,
            -2.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        while runtime.is_running{
        four_bar.save_data_verbose("fourbar", &mut runtime);

            if runtime.get_x() >= runtime.get_max_x(){
                break
            }

            four_bar.set_servo_angle_rad(runtime.get_x());
            runtime.increment();
        };

        assert_relative_eq!(
            four_bar.b.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.end_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.start_y_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.b.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        assert_relative_eq!(
            four_bar.a.start_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.start_y_m,
            -1.5,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.a.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        assert_relative_eq!(
            four_bar.g.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.start_y_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.g.end_y_m,
            -1.5,
            max_relative=1e-2
        );

        assert_relative_eq!(
            four_bar.l.start_x_m,
            0.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.end_x_m,
            1.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.start_y_m,
            -2.0,
            max_relative=1e-2
        );
        assert_relative_eq!(
            four_bar.l.end_y_m,
            -2.0,
            max_relative=1e-2
        );

        runtime.export_to_csv("results/data/four_bar.csv")
    }
}
