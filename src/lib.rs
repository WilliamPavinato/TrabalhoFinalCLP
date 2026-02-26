/// missile_sim - Rust ballistic physics engine
/// 
///
/// This library is compiled as a C-compatible shared library (.so/.dll)
/// and called from Python via ctypes (FFI - Foreign Function Interface).
///
/// Physics model:
///   - Flat-Earth 2D projectile motion with atmospheric drag
///   - Gravity: g = 9.81 m/s²
///   - Drag: F_drag = 0.5 * Cd * rho * A * v²  (simplified: drag_coeff * v²)
///   - Integration: Runge-Kutta 4th order (RK4) for accuracy
///
/// The `#[no_mangle]` attribute and `extern "C"` calling convention allow
/// Python's ctypes to call these functions directly from the shared library.

use std::os::raw::c_double;
use std::os::raw::c_int;

/// State of a single missile at one moment in time
#[repr(C)]
pub struct MissileState {
    /// Horizontal position (meters)
    pub x: c_double,
    /// Vertical position (meters)
    pub y: c_double,
    /// Horizontal velocity (m/s)
    pub vx: c_double,
    /// Vertical velocity (m/s)
    pub vy: c_double,
    /// Time elapsed (seconds)
    pub t: c_double,
    /// 1 if still in flight, 0 if landed/exploded
    pub active: c_int,
}

/// Simulation parameters passed from Python
#[repr(C)]
pub struct SimParams {
    /// Launch angle in degrees (0=horizontal, 90=vertical)
    pub angle_deg: c_double,
    /// Launch speed in m/s
    pub speed: c_double,
    /// Drag coefficient (combined: 0.5 * Cd * rho * A / mass)
    pub drag_coeff: c_double,
    /// Gravitational acceleration (m/s²), typically 9.81
    pub gravity: c_double,
    /// Integration time step (seconds)
    pub dt: c_double,
}

/// Trajectory point for returning full path to Python
#[repr(C)]
pub struct TrajectoryPoint {
    pub x: c_double,
    pub y: c_double,
    pub t: c_double,
}

/// Maximum number of trajectory points we will store per missile.
/// At dt=0.1s and a ~600s flight, 8000 points is plenty.
const MAX_POINTS: usize = 8000;


/// Internal derivative computation for RK4.
/// Returns (dx/dt, dy/dt, dvx/dt, dvy/dt) given current state and params.
fn derivatives(
    _x: f64, _y: f64, vx: f64, vy: f64,
    drag: f64, gravity: f64,
) -> (f64, f64, f64, f64) {
    let v = (vx * vx + vy * vy).sqrt();
    // Drag decelerates along velocity vector: a_drag = -drag_coeff * v * v_unit
    let ax = -drag * v * vx;
    let ay = -gravity - drag * v * vy;
    (vx, vy, ax, ay)
}

/// Advance state by one RK4 step of size dt.
/// RK4: k1 = f(t,y), k2 = f(t+dt/2, y+k1*dt/2), ...
///      y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
fn rk4_step(x: f64, y: f64, vx: f64, vy: f64, dt: f64, drag: f64, gravity: f64)
    -> (f64, f64, f64, f64)
{
    let (dx1, dy1, dvx1, dvy1) = derivatives(x, y, vx, vy, drag, gravity);

    let x2  = x  + 0.5 * dt * dx1;
    let y2  = y  + 0.5 * dt * dy1;
    let vx2 = vx + 0.5 * dt * dvx1;
    let vy2 = vy + 0.5 * dt * dvy1;
    let (dx2, dy2, dvx2, dvy2) = derivatives(x2, y2, vx2, vy2, drag, gravity);

    let x3  = x  + 0.5 * dt * dx2;
    let y3  = y  + 0.5 * dt * dy2;
    let vx3 = vx + 0.5 * dt * dvx2;
    let vy3 = vy + 0.5 * dt * dvy2;
    let (dx3, dy3, dvx3, dvy3) = derivatives(x3, y3, vx3, vy3, drag, gravity);

    let x4  = x  + dt * dx3;
    let y4  = y  + dt * dy3;
    let vx4 = vx + dt * dvx3;
    let vy4 = vy + dt * dvy3;
    let (dx4, dy4, dvx4, dvy4) = derivatives(x4, y4, vx4, vy4, drag, gravity);

    let xn  = x  + dt / 6.0 * (dx1  + 2.0*dx2  + 2.0*dx3  + dx4);
    let yn  = y  + dt / 6.0 * (dy1  + 2.0*dy2  + 2.0*dy3  + dy4);
    let vxn = vx + dt / 6.0 * (dvx1 + 2.0*dvx2 + 2.0*dvx3 + dvx4);
    let vyn = vy + dt / 6.0 * (dvy1 + 2.0*dvy2 + 2.0*dvy3 + dvy4);

    (xn, yn, vxn, vyn)
}

/// Compute the complete trajectory of a single missile.
///
/// # Safety
/// `out_points` must point to a buffer of at least `max_points` TrajectoryPoint elements.
/// `out_count` must point to a valid c_int.
///
/// Called from Python as:
///   lib.compute_trajectory(byref(params), out_buf, MAX_POINTS, byref(count))
#[no_mangle]
pub unsafe extern "C" fn compute_trajectory(
    params: *const SimParams,
    out_points: *mut TrajectoryPoint,
    max_points: c_int,
    out_count: *mut c_int,
) {
    // Dereference params pointer (safe because Python guarantees valid struct)
    let p = &*params;

    let angle_rad = p.angle_deg.to_radians();
    let mut vx = p.speed * angle_rad.cos();
    let mut vy = p.speed * angle_rad.sin();
    let mut x: f64 = 0.0;
    let mut y: f64 = 0.0;
    let mut t: f64 = 0.0;

    let limit = (max_points as usize).min(MAX_POINTS);
    let mut count = 0usize;

    // Integrate until missile hits ground (y < 0) or point buffer full
    loop {
        if count >= limit { break; }

        // Store current point
        let pt = out_points.add(count);
        (*pt).x = x;
        (*pt).y = y;
        (*pt).t = t;
        count += 1;

        // Advance by one RK4 step
        let (xn, yn, vxn, vyn) = rk4_step(x, y, vx, vy, p.dt, p.drag_coeff, p.gravity);

        // Check ground collision (y crosses zero)
        if yn < 0.0 && y >= 0.0 {
            // Linear interpolation to find exact impact point
            let frac = y / (y - yn);
            let xi = x + frac * (xn - x);
            if count < limit {
                let pt = out_points.add(count);
                (*pt).x = xi;
                (*pt).y = 0.0;
                (*pt).t = t + frac * p.dt;
                count += 1;
            }
            break;
        }

        x = xn; y = yn; vx = vxn; vy = vyn;
        t += p.dt;

        // Safety: if missile goes far underground (numerical error), stop
        if y < -1.0 { break; }
    }

    *out_count = count as c_int;
}

/// Compute maximum range for a given speed (no drag approximation as reference).
/// Returns analytical range: R = v² * sin(2θ) / g
/// Exported so Python can display the theoretical vs simulated comparison.
#[no_mangle]
pub extern "C" fn theoretical_range(speed: c_double, angle_deg: c_double, gravity: c_double) -> c_double {
    let angle_rad = angle_deg.to_radians();
    speed * speed * (2.0 * angle_rad).sin() / gravity
}

/// Return library version string pointer (static lifetime, safe to read from Python).
#[no_mangle]
pub extern "C" fn lib_version() -> *const u8 {
    b"missile_sim_rust v1.0\0".as_ptr()
}
