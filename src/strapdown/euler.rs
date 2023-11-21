// ----------------------------------------------------------------------------
// Strapdown Analytics for non embedded applications
// ----------------------------------------------------------------------------

// 3rd Party
use std::ops::{Mul, Div, Add, Sub};
use std::cmp::PartialEq;
use derive_more;
use std::f64::consts::PI;

// Crate
use crate::strapdown::dcm::DCM;

// ----------------------------------------------------------------------------
// Euler Angles [3.2.3] Pg 3-31
// ----------------------------------------------------------------------------

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    PartialOrd,
    derive_more::Add,
    derive_more::Sub,
    derive_more::Mul,
    derive_more::Div
)]
pub struct Euler{
    pub psi: f64,
    pub theta: f64,
    pub phi: f64
}

impl Euler{
    pub fn new(psi: f64, theta: f64, phi: f64) -> Euler{
        return Euler {psi, theta, phi}
    }

    pub fn of(num: f64) -> Euler{
        return Euler::new(num, num, num)
    }

    pub fn zeros() -> Euler{
        return Euler::new(0.0, 0.0, 0.0)
    }

    pub fn to_array(self) -> [f64; 3]{
        return [self.psi, self.theta, self.phi]
    }

    pub fn to_dcm(self) -> DCM{
        // Eq 3.2.3.1-1, Pg 3-33
        let _c11 = self.theta.cos() * self.psi.cos();
        let _c12 =
            (-self.phi.cos() * self.psi.sin()) +
            (self.phi.sin() * self.theta.sin() * self.psi.cos());
        let _c13 =
            (self.phi.sin() * self.psi.sin()) +
            (self.phi.cos() * self.theta.sin() * self.psi.cos());

        let _c21 = self.theta.cos() * self.psi.sin();
        let _c22 =
            (self.phi.cos() * self.psi.cos()) +
            (self.phi.sin() * self.theta.sin() * self.psi.sin());
        let _c23 =
            (-self.phi.sin() * self.psi.cos()) +
            (self.phi.cos() * self.theta.sin() * self.psi.sin());

        let _c31 = -self.theta.sin();
        let _c32 = self.phi.sin() * self.theta.cos();
        let _c33 = self.phi.cos() * self.theta.cos();


        return DCM::new(
            _c11, _c12, _c12,
            _c21, _c22, _c22,
            _c31, _c32, _c32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_to_dcm(){

        // Identity check
        let euler = Euler::zeros();
        let dcm = DCM::identity();
        let euler_to_dcm = euler.to_dcm();

        // Convert back check

        let euler = Euler::new(1.0,0.5,0.25);
        let euler_to_dcm = euler.to_dcm();
        let dcm_to_euler = euler_to_dcm.to_euler();

        let error = Euler::of(1e-6);
    }
}