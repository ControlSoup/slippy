
use crate::strapdown::{matrix::Matrix3x3, vector::Vector3};
use crate::sim::runtime::{Runtime, Save};

// ----------------------------------------------------------------------------
// Mass Properties
// ----------------------------------------------------------------------------

#[derive(Debug)]
pub struct MassProperties{
    pub mass_cg_kg: f64,
    i_tensor_cg_kgpm2: Matrix3x3,
    inv_i_tensor_cg_kgpm2: Matrix3x3
}

impl MassProperties{
    pub fn new(
        mass_cg_kg: f64,
        i_tensor_cg_kgpm2: [f64; 9]
    ) -> MassProperties{

        let i_tensor: Matrix3x3 = Matrix3x3::from_array(i_tensor_cg_kgpm2);

        let inv: Matrix3x3 = i_tensor.inv()
            .expect("i_tensor_cg_kgpm2 was not invertible");

        return MassProperties {
            mass_cg_kg: mass_cg_kg,
            i_tensor_cg_kgpm2: i_tensor,
            inv_i_tensor_cg_kgpm2: inv
        }
    }

    pub fn identity() -> MassProperties{
        return MassProperties {
            mass_cg_kg: 1.0,
            i_tensor_cg_kgpm2: Matrix3x3::identity(),
            inv_i_tensor_cg_kgpm2:Matrix3x3::identity()
        }
    }
}


impl Save for MassProperties{
    fn save(self, mut runtime: Runtime) where Self: Sized {
        runtime.add_or_set("MassProperties.mass_cg [kg]", self.mass_cg_kg);

        runtime.add_or_set(
            "MassProperties.Ixx [kg/m^2]", self.i_tensor_cg_kgpm2.c11
        );
        runtime.add_or_set(
            "MassProperties.Ixy [kg/m^2]", self.i_tensor_cg_kgpm2.c12
        );
        runtime.add_or_set(
            "MassProperties.Ixz [kg/m^2]", self.i_tensor_cg_kgpm2.c13
        );
        runtime.add_or_set(
            "MassProperties.Iyx [kg/m^2]", self.i_tensor_cg_kgpm2.c21
        );
        runtime.add_or_set(
            "MassProperties.Iyy [kg/m^2]", self.i_tensor_cg_kgpm2.c22
        );
        runtime.add_or_set(
            "MassProperties.Iyz [kg/m^2]", self.i_tensor_cg_kgpm2.c23
        );
        runtime.add_or_set(
            "MassProperties.Izx [kg/m^2]", self.i_tensor_cg_kgpm2.c31
        );
        runtime.add_or_set(
            "MassProperties.Izx [kg/m^2]", self.i_tensor_cg_kgpm2.c32
        );
        runtime.add_or_set(
            "MassProperties.Izz [kg/m^2]", self.i_tensor_cg_kgpm2.c33
        );
    }
}