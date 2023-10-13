// ----------------------------------------------------------------------------
// Generic RK4 Integration Method
// Source: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
// ----------------------------------------------------------------------------

fn rk4(
    fn_dot: fn(Array1<f64>) -> Array1<f64>,
    current_values: Array1<f64>,
    dt: f64
){
    // k1
    let k1 = fn_dot(current_values);

    // k2
    let k2 = fn_dot(
        current_values + (k1 * dt / 2)
    );

    // k3
    let k3 = fn_dot(
        current_values + (k2 * dt / 2)
    );

    // k4
    let k4 = fn_dot(
        current_values + (k3 * dt)
    );

    // yn + 1
    return current_values + ((k1 + (2 * k2) + (2 * k3) + k4) * dt / 6.0)

}