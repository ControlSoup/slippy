use crate::strapdown::vector::Vector3;
use super::mass_properties::MassProperties;
use super::state::State;


// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct Inputs{
    pub local_level_force_n: Vector3,
    pub local_level_moment_nm: Vector3,
    pub body_force_n: Vector3,
    pub body_moment_nm: Vector3
}


// TODO: Update Forces to act in the body frame
impl Inputs{
    pub fn new(
        local_level_force_n: [f64; 3],
        local_level_moment_nm: [f64; 3],
        body_force_n: [f64; 3],
        body_moment_nm: [f64; 3],
    ) -> Inputs {
            return Inputs{
                local_level_force_n:
                    Vector3::from_array(local_level_force_n),
                local_level_moment_nm:
                    Vector3::from_array(local_level_moment_nm),
                body_force_n:
                    Vector3::from_array(body_force_n),
                body_moment_nm:
                    Vector3::from_array(body_moment_nm),
            }
    }

    pub fn zeros() -> Inputs{
        return Inputs{
            local_level_force_n: Vector3::zeros(),
            body_force_n: Vector3::zeros(),
            local_level_moment_nm: Vector3::zeros(),
            body_moment_nm: Vector3::zeros()

        }
    }

    pub fn effect_state(&self, state: &mut State, mass_props: &MassProperties){
        // Source:
        //   https://en.wikipedia.org/wiki/Rigid_body_dynamics
        //
        // Notes:
        //     Forces and moments act about the body frame

        let total_forces_n = 
            self.local_level_force_n + 
            state.quat.transform(self.body_force_n);

        let total_moments_nm =  
            self.local_level_moment_nm + 
            state.quat.transform(self.body_moment_nm);

        // F = ma
        state.accel_mps2 = total_forces_n / mass_props.mass_cg_kg;

        // I * w
        let i_dot_w =
            mass_props.i_tensor_cg_kgpm2 * state.ang_vel_radps;

        // w x (I * w)
        let w_cross_i_dot_w = state.ang_vel_radps.cross(&i_dot_w);

        // M - (w x (I * w))
        let m_minus_w_cross_i_dot_w = total_moments_nm - w_cross_i_dot_w;

        // alpha = I^-1(M-(w × (I * w))
        state.ang_accel_radps2 =
            mass_props.inv_i_tensor_cg_kgpm2 * m_minus_w_cross_i_dot_w;

    }

}

impl Save for Inputs{

    fn save(self, mut runtime: Runtime) where Self: Sized {
        runtime.add_or_set(
            "Inputs.local_level_force.x [N]", self.local_level_force_n.x
        );
        runtime.add_or_set(
            "Inputs.local_level_force.y [N]", self.local_level_force_n.y
        );
        runtime.add_or_set(
            "Inputs.local_level_force.z [N]", self.local_level_force_n.z
        );

        runtime.add_or_set(
            "Inputs.local_level_moment.x [Nm]", self.local_level_moment_nm.x 
        );
        runtime.add_or_set(
            "Inputs.local_level_moment.y [Nm]", self.local_level_moment_nm.y 
        );
        runtime.add_or_set(
            "Inputs.local_level_moment.z [Nm]", self.local_level_moment_nm.z 
        );

        runtime.add_or_set(
            "Inputs.body_force.x [N]", self.body_force_n.x
        );
        runtime.add_or_set(
            "Inputs.body_force.y [N]", self.body_force_n.y
        );
        runtime.add_or_set(
            "Inputs.body_force.z [N]", self.body_force_n.z
        );

        runtime.add_or_set(
            "Inputs.body_moment.x [Nm]", self.body_moment_nm.x 
        );
        runtime.add_or_set(
            "Inputs.body_moment.y [Nm]", self.body_moment_nm.y 
        );
        runtime.add_or_set(
            "Inputs.body_moment.z [Nm]", self.body_moment_nm.z 
        );
    }
}