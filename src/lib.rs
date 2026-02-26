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

