use crate::geometry::d2::{PI_THREE_HALFS, length_from_points, angle_from_y_rad};

struct ServoTVC{
    servo_start_x: f64,
    servo_start_y: f64,
    servo_end_x : f64,
    servo_end_y : f64,
    servo_angle_rad: f64,
    servo_joint_radius: f64,
    tvc_start_x: f64,
    tvc_start_y: f64,
    tvc_end_x: f64,
    tvc_end_y: f64,
    tvc_angle_rad: f64,
    tvc_joint_radius: f64,
    connection_length: f64
}

impl ServoTVC{

    fn new_from_points(
        servo_start_x: f64,
        servo_start_y: f64,
        servo_end_x: f64,
        servo_end_y: f64,
        tvc_start_x: f64,
        tvc_start_y: f64,
        tvc_end_x: f64,
        tvc_end_y: f64,
    ) -> ServoTVC{
        return ServoTVC {
            servo_start_x,
            servo_start_y,
            servo_end_x,
            servo_end_y,
            servo_angle_rad:
                angle_from_y_rad(
                    &servo_start_x,
                    &servo_start_y,
                    &servo_end_x,
                    &servo_end_y
                ),
            servo_joint_radius:
                length_from_points(
                    &servo_start_x,
                    &servo_start_y,
                    &servo_end_x,
                    &servo_end_y
                ),
            tvc_start_x,
            tvc_start_y,
            tvc_end_x,
            tvc_end_y,
            tvc_angle_rad:
                angle_from_y_rad(
                    &tvc_start_x,
                    &tvc_start_y,
                    &tvc_end_x,
                    &tvc_end_y
                ),
            tvc_joint_radius:
                length_from_points(
                    &tvc_start_x,
                    &tvc_start_y,
                    &tvc_end_x,
                    &tvc_end_y
                ),
            connection_length:
                length_from_points(
                    &servo_end_x,
                    &servo_end_y,
                    &tvc_end_x,
                    &tvc_end_y
                ),
        }


    }

    fn new_from_ratio(
        servo_start_y: f64,
        servo_joint_radius: f64,
        tvc_joint_radius: f64,
        connection_length: f64
    ) -> ServoTVC{
        return ServoTVC {
            servo_start_x: connection_length,
            servo_start_y,
            servo_end_x: connection_length,
            servo_end_y: -servo_joint_radius,
            servo_angle_rad: 0.0,
            servo_joint_radius,
            tvc_start_x: 0.0,
            tvc_start_y: 0.0,
            tvc_end_x: 0.0,
            tvc_end_y: -tvc_joint_radius,
            tvc_angle_rad: 0.0,
            tvc_joint_radius,
            connection_length
        }
    }

    fn get_tvc_angle(&mut self, servo_angle_rad: f64){
        self.servo_angle_rad = servo_angle_rad;

        // Update end points of servo
        self.servo_end_x = self.servo_end_x + (self.servo_joint_radius * self.servo_angle_rad.cos());
        self.servo_end_y = self.servo_end_y + (self.servo_joint_radius * self.servo_angle_rad.sin());

        // Calculate distance between servo end position and tvc pivot
        let distance_between_centeres =
            (self.servo_end_y - self.tvc_start_y)
            / (self.servo_end_x - self.tvc_start_x);

        let ditance_between_centers = length_from_points(
            &self.tvc_start_x,
            &self.tvc_end_y,
            &self.servo_end_x,
            &self.servo_end_y
        );

        // Law of cosines and solve for missing tvc angle
        let numer = self.tvc_joint_radius.powf(2.0) + self.connection_length.powf(2.0) - distance_between_centeres.powf(2.0);
        self.tvc_angle_rad = PI_THREE_HALFS - (
            numer / (2.0 * self.tvc_joint_radius * self.connection_length)
        );

        // Update end points of tvc
        self.tvc_end_x = self.tvc_end_y + (self.tvc_joint_radius * self.tvc_angle_rad.cos());
        self.tvc_end_y = self.tvc_end_y + (self.tvc_joint_radius * self.tvc_angle_rad.sin());
    }
}
