#!/usr/bin/env python3
"""
run_study.py — Headless case-of-study for the Makefile `study` target.

Computes and prints trajectories for several launch angles without needing
a graphical display.  Output is also saved to trajectory_output.csv.

This script demonstrates the Rust FFI working correctly from Python
without any GUI dependency.
"""

import ctypes
import os
import sys
import math

# ── Load Rust library ─────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
candidates = [
    os.path.join(script_dir, "target", "release", "libmissile_sim.so"),
    os.path.join(script_dir, "libmissile_sim.so"),
    os.path.join(script_dir, "target", "release", "missile_sim.dll"),
    os.path.join(script_dir, "missile_sim.dll"),
]
lib = None
for path in candidates:
    if os.path.exists(path):
        lib = ctypes.CDLL(path)
        print(f"[INFO] Loaded: {path}")
        break
if lib is None:
    print("[ERROR] Rust library not found. Run: make build")
    sys.exit(1)

# ── ctypes structs (must match Rust #[repr(C)] exactly) ─────────────────────

class SimParams(ctypes.Structure):
    _fields_ = [
        ("angle_deg",  ctypes.c_double),
        ("speed",      ctypes.c_double),
        ("drag_coeff", ctypes.c_double),
        ("gravity",    ctypes.c_double),
        ("dt",         ctypes.c_double),
    ]

class TrajectoryPoint(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("t", ctypes.c_double),
    ]

MAX_PTS = 8000
lib.compute_trajectory.argtypes = [
    ctypes.POINTER(SimParams),
    ctypes.POINTER(TrajectoryPoint),
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
]
lib.compute_trajectory.restype = None
lib.theoretical_range.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.theoretical_range.restype  = ctypes.c_double

# ── Run study cases ───────────────────────────────────────────────────────────

SPEED      = 7_500.0   # m/s  (~ICBM range)
DRAG_COEFF = 2e-5      # simplified atmospheric drag coefficient
GRAVITY    = 9.81

angles = [20, 30, 45, 60, 75]

print("\n" + "="*72)
print("  ICBM Ballistic Simulation — Case of Study")
print(f"  Launch speed: {SPEED:.0f} m/s   Drag coeff: {DRAG_COEFF:.1e}")
print("="*72)
print(f"  {'Angle':>6}  {'Range(km)':>10}  {'MaxAlt(km)':>10}  "
      f"{'Flight(s)':>10}  {'Theor(km)':>10}  {'Error%':>7}")
print("-"*72)

csv_rows = ["angle_deg,range_km,max_alt_km,flight_s,theoretical_km,error_pct"]

for angle in angles:
    params  = SimParams(angle_deg=angle, speed=SPEED,
                        drag_coeff=DRAG_COEFF, gravity=GRAVITY, dt=0.5)
    out_buf   = (TrajectoryPoint * MAX_PTS)()
    out_count = ctypes.c_int(0)

    # ← FFI call into Rust ←
    lib.compute_trajectory(ctypes.byref(params), out_buf, MAX_PTS, ctypes.byref(out_count))

    n = out_count.value
    pts = [(out_buf[i].x, out_buf[i].y, out_buf[i].t) for i in range(n)]

    range_km  = pts[-1][0] / 1000
    max_alt   = max(p[1] for p in pts) / 1000
    flight_t  = pts[-1][2]
    theor_km  = lib.theoretical_range(SPEED, float(angle), GRAVITY) / 1000
    error_pct = (theor_km - range_km) / theor_km * 100 if theor_km else 0

    print(f"  {angle:>6}°  {range_km:>10.1f}  {max_alt:>10.1f}  "
          f"{flight_t:>10.1f}  {theor_km:>10.1f}  {error_pct:>6.1f}%")
    csv_rows.append(f"{angle},{range_km:.2f},{max_alt:.2f},{flight_t:.1f},{theor_km:.2f},{error_pct:.2f}")

print("="*72)
print("\nNote: 'Error%' shows drag reduction vs vacuum (theoretical) range.\n"
      "      Larger drag → shorter actual range.\n")

# Save CSV
csv_path = os.path.join(script_dir, "trajectory_output.csv")
with open(csv_path, "w") as f:
    f.write("\n".join(csv_rows) + "\n")
print(f"[INFO] Trajectory data saved to: {csv_path}")