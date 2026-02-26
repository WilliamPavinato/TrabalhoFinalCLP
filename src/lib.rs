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