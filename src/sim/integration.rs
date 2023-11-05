use std::ops::{Mul, Div, Add};
use ::core::fmt::Debug;


pub trait Integrate{
    fn get_derivative(&self)-> Self;

    fn rk4(&mut self, dt: f64) -> Self
        where
            Self:
                Sized +
                Clone +
                Debug +
                Add<Self, Output = Self> +
                Mul<f64, Output = Self> +
                Div<f64, Output = Self>,
            // &Self:
            //     Sized +
            //     Clone +
            //     Debug +
            //     Add<Self, Output = Self> +
            //     Mul<f64, Output = Self> +
            //     Div<f64, Output = Self>,
    {

        let k1 = self.get_derivative();
        let k2 = (self.clone() + (k1.clone() / 2.0) * dt / 2.0).get_derivative();
        let k3 = (self.clone() + (k2.clone() / 2.0)).get_derivative();
        let k4 = (self.clone() + k3.clone()).get_derivative();

        return self.clone() + ((k1 + (k2 * 2.0) + (k3 * 2.0) + k4) * dt / 6.0)
    }

    fn euler(&self, dt: f64)-> Self
        where
            Self:
                Sized +
                Clone +
                Add<Self, Output = Self> +
                Mul<f64, Output = Self>
    {
        let euler =  self.clone() + (self.get_derivative() * dt);
        return euler
    }
}

#[cfg(test)]
mod tests {


    #[test]
    fn test1(){
        use super::*;
        use derive_more::Add;
        #[derive(Add, Debug,Clone)]

        struct Location{
            position: f64,
            velocity: f64,
            acceleration: f64
        }

        impl Mul<f64> for Location {
            type Output = Location;

            fn mul(self, rhs: f64) -> Location {
                Location {
                    position: self.position * rhs,
                    velocity: self.velocity * rhs,
                    acceleration: self.acceleration * rhs,
                }
            }
        }

        impl Div<f64> for Location{
            type Output = Location;

            fn div(self, rhs: f64) -> Location{
                return Location{
                    position: self.position / rhs,
                    velocity: self.velocity / rhs,
                    acceleration: self.acceleration / rhs,
                }
            }
        }

        impl Integrate for Location{
            fn get_derivative(&self)-> Self {
                let mut derivative = self.clone();
                derivative.position = self.velocity;
                derivative.velocity = self.acceleration;
                derivative.acceleration = 0.0;

                return derivative
            }
        }

        let mut test_vehicle = Location{
            position: 0.0,
            velocity: 0.0,
            acceleration: 1.0
        };

        for _ in 0..5{

            test_vehicle = test_vehicle.euler(1.0);
        }
        assert_eq!(test_vehicle.velocity, 5.0);


        for _ in 0..5{
            test_vehicle = test_vehicle.rk4(1.0);
        }
        assert_eq!(test_vehicle.velocity, 10.0);

    }
}