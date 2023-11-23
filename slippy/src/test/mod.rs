use approx::assert_relative_eq;

pub const FLOAT_DEV: f64  = 1e-6;

pub fn almost_equal_array(array1: &[f64], array2: &[f64]){

    if array1.len() != array2.len(){
        panic!("ERROR| not equal in length to array2")
    }

    for (i, value) in array1.iter().enumerate(){
        assert_relative_eq!(
            *value,
            array2[i],
            max_relative=FLOAT_DEV
        )
    }
}