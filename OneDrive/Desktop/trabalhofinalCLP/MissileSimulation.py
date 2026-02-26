#!/usr/bin/env python3
"""
missile_sim_gui.py  —  Python front-end for the Rust ballistic engine.

Architecture:
  ┌─────────────────────────┐       ctypes FFI        ┌──────────────────────────┐
  │  Python  (this file)    │  ──────────────────────► │  Rust shared library     │
  │  • Pygame UI / rendering│  SimParams struct        │  • RK4 integrator        │
  │  • User controls        │  TrajectoryPoint[]       │  • drag + gravity model  │
  │  • Animation loop       │ ◄────────────────────── │  • theoretical_range()   │
  └─────────────────────────┘                          └──────────────────────────┘

How the FFI works:
  1. We load `libmissile_sim.so` with ctypes.CDLL.
  2. We declare Python ctypes Structure subclasses that match the `#[repr(C)]`
     structs in Rust exactly (same field order, same numeric types).
  3. We call `lib.compute_trajectory(...)` as a normal Python function call;
     ctypes marshals the arguments into the C ABI automatically.
  4. The Rust code writes results into a pre-allocated ctypes array that Python
     then reads back.

Controls:
  Mouse  — click to set launch angle (direction of click from origin)
  Slider — adjust launch speed (left panel)
  SPACE  — fire missile
  R      — reset / clear all trajectories
  ESC    — quit
"""

import ctypes
import sys
import os
import math
import pygame
from pygame import gfxdraw

# ──────────────────────────────────────────────────────────────────────────────
# 1.  Load the Rust shared library via ctypes
# ──────────────────────────────────────────────────────────────────────────────

def load_library():
    """
    Try several candidate paths for the compiled Rust .so/.dll.
    Returns a loaded ctypes CDLL instance.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # After `cargo build --release`
        os.path.join(script_dir, "target", "release", "libmissile_sim.so"),
        os.path.join(script_dir, "target", "release", "missile_sim.dll"),
        # After `make build` (copies to project root)
        os.path.join(script_dir, "libmissile_sim.so"),
        os.path.join(script_dir, "missile_sim.dll"),
    ]
    for path in candidates:
        if os.path.exists(path):
            print(f"[INFO] Loading Rust library: {path}")
            return ctypes.CDLL(path)
    print("[ERROR] Could not find compiled Rust library.")
    print("        Run:  make build   or   cargo build --release")
    sys.exit(1)


lib = load_library()

# ──────────────────────────────────────────────────────────────────────────────
# 2.  Declare ctypes structures matching the Rust `#[repr(C)]` structs
#     Field order and types MUST match Rust exactly; any mismatch = UB / crash.
# ──────────────────────────────────────────────────────────────────────────────

class SimParams(ctypes.Structure):
    """Mirror of Rust's SimParams struct."""
    _fields_ = [
        ("angle_deg",   ctypes.c_double),
        ("speed",       ctypes.c_double),
        ("drag_coeff",  ctypes.c_double),
        ("gravity",     ctypes.c_double),
        ("dt",          ctypes.c_double),
    ]

class TrajectoryPoint(ctypes.Structure):
    """Mirror of Rust's TrajectoryPoint struct."""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("t", ctypes.c_double),
    ]

# Tell ctypes about argument and return types so it can do the right marshalling.
MAX_POINTS = 8000

lib.compute_trajectory.argtypes = [
    ctypes.POINTER(SimParams),          # params
    ctypes.POINTER(TrajectoryPoint),    # out_points buffer
    ctypes.c_int,                       # max_points
    ctypes.POINTER(ctypes.c_int),       # out_count
]
lib.compute_trajectory.restype = None   # void

lib.theoretical_range.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.theoretical_range.restype = ctypes.c_double

# ──────────────────────────────────────────────────────────────────────────────
# 3.  Helper: call Rust compute_trajectory and return Python list of (x, y, t)
# ──────────────────────────────────────────────────────────────────────────────

def rust_trajectory(angle_deg: float, speed: float,
                    drag_coeff: float = 2e-5,
                    gravity: float = 9.81,
                    dt: float = 0.5) -> list:
    """
    Allocate a buffer of TrajectoryPoint, call the Rust engine,
    and return a Python list of (x_m, y_m, t_s) tuples.
    """
    params = SimParams(
        angle_deg  = angle_deg,
        speed      = speed,
        drag_coeff = drag_coeff,
        gravity    = gravity,
        dt         = dt,
    )
    # Pre-allocate output buffer; Rust writes into it
    out_buf   = (TrajectoryPoint * MAX_POINTS)()
    out_count = ctypes.c_int(0)

    # ← This is the actual FFI call into compiled Rust code ←
    lib.compute_trajectory(
        ctypes.byref(params),
        out_buf,
        MAX_POINTS,
        ctypes.byref(out_count),
    )

    n = out_count.value
    return [(out_buf[i].x, out_buf[i].y, out_buf[i].t) for i in range(n)]

# ──────────────────────────────────────────────────────────────────────────────
# 4.  Rendering / coordinate transform helpers
# ──────────────────────────────────────────────────────────────────────────────

# Window dimensions
WIN_W, WIN_H = 1280, 720
PANEL_W = 260          # Left control panel width
SIM_W   = WIN_W - PANEL_W   # Width of the simulation viewport

# Physics world extent (meters). We auto-scale per launch to fit trajectory.
WORLD_H = 500_000.0    # 500 km default vertical scale
WORLD_W = 2_000_000.0  # 2 000 km default horizontal scale

# Colour palette  (RGB)
C_BG        = (10,  10,  30)    # Deep-space background
C_PANEL     = (20,  20,  50)    # Side panel
C_GROUND    = (34,  85,  34)    # Ground strip
C_GRID      = (30,  30,  70)    # Grid lines
C_TEXT      = (200, 220, 255)   # Default text
C_ACCENT    = (100, 180, 255)   # Accent / highlight
C_MISSILE   = (255, 80,  60)    # Active missile head
C_TRAIL     = (255, 200, 80)    # Trail colour (fades)
C_IMPACT    = (255, 60,  60)    # Impact marker
C_THEOR     = (80,  80, 160)    # Theoretical range line
C_BUTTON    = (50,  80, 140)
C_BTN_HOV   = (70, 110, 190)


def world_to_screen(x_m: float, y_m: float,
                    x_origin_px: int, y_ground_px: int,
                    scale_x: float, scale_y: float) -> tuple:
    """Convert world coordinates (metres) to pixel screen coordinates."""
    sx = x_origin_px + int(x_m * scale_x)
    sy = y_ground_px - int(y_m * scale_y)   # y flips: up in world = up on screen
    return sx, sy


def screen_to_angle(mx: int, my: int,
                    x_origin_px: int, y_ground_px: int) -> float:
    """Return launch angle (degrees) from origin toward mouse position."""
    dx = mx - x_origin_px
    dy = y_ground_px - my   # flip y
    if dx <= 0:
        return 45.0
    return max(1.0, min(89.0, math.degrees(math.atan2(dy, dx))))


# ──────────────────────────────────────────────────────────────────────────────
# 5.  Simple UI widgets
# ──────────────────────────────────────────────────────────────────────────────

class Slider:
    """Horizontal slider for float values."""

    def __init__(self, x, y, w, h, vmin, vmax, value, label, fmt="{:.0f}"):
        self.rect  = pygame.Rect(x, y, w, h)
        self.vmin  = vmin
        self.vmax  = vmax
        self.value = value
        self.label = label
        self.fmt   = fmt
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            rel = (event.pos[0] - self.rect.x) / self.rect.w
            rel = max(0.0, min(1.0, rel))
            self.value = self.vmin + rel * (self.vmax - self.vmin)

    def draw(self, surf, font):
        # Track
        pygame.draw.rect(surf, (50, 60, 100), self.rect, border_radius=4)
        # Fill
        fill_w = int((self.value - self.vmin) / (self.vmax - self.vmin) * self.rect.w)
        fill_r = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.h)
        pygame.draw.rect(surf, C_ACCENT, fill_r, border_radius=4)
        # Handle
        hx = self.rect.x + fill_w
        pygame.draw.circle(surf, (255, 255, 255), (hx, self.rect.centery), 8)
        # Labels
        lbl = font.render(self.label, True, C_TEXT)
        surf.blit(lbl, (self.rect.x, self.rect.y - 20))
        val_str = self.fmt.format(self.value)
        val_surf = font.render(val_str, True, C_ACCENT)
        surf.blit(val_surf, (self.rect.right - val_surf.get_width(), self.rect.y - 20))


class Button:
    def __init__(self, x, y, w, h, text):
        self.rect  = pygame.Rect(x, y, w, h)
        self.text  = text
        self.hover = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False

    def draw(self, surf, font):
        col = C_BTN_HOV if self.hover else C_BUTTON
        pygame.draw.rect(surf, col, self.rect, border_radius=6)
        pygame.draw.rect(surf, C_ACCENT, self.rect, 1, border_radius=6)
        t = font.render(self.text, True, C_TEXT)
        surf.blit(t, t.get_rect(center=self.rect.center))


# ──────────────────────────────────────────────────────────────────────────────
# 6.  Main simulation / animation state
# ──────────────────────────────────────────────────────────────────────────────

class MissileAnimation:
    """Holds a computed trajectory and animates the missile head along it."""

    def __init__(self, points: list, color):
        self.points  = points   # list of (x_m, y_m, t_s)
        self.color   = color
        self.idx     = 0        # current head index
        self.done    = False
        self.speed_factor = 1   # animation speed multiplier

    def step(self):
        """Advance the animation by one frame."""
        if self.done:
            return
        self.idx = min(self.idx + self.speed_factor, len(self.points) - 1)
        if self.idx >= len(self.points) - 1:
            self.done = True

    def impact_x(self) -> float:
        return self.points[-1][0] if self.points else 0.0

    def max_alt(self) -> float:
        return max(p[1] for p in self.points) if self.points else 0.0

    def flight_time(self) -> float:
        return self.points[-1][2] if self.points else 0.0


# Colour cycle for multiple missiles
MISSILE_COLORS = [
    (255, 80,  60), (80,  255, 120), (80,  180, 255),
    (255, 220, 60), (220, 80,  255), (255, 140, 60),
]


# ──────────────────────────────────────────────────────────────────────────────
# 7.  Main application loop
# ──────────────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    screen  = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("ICBM Ballistic Simulation  |  Rust + Python")
    clock   = pygame.time.Clock()

    font_sm = pygame.font.SysFont("monospace", 13)
    font_md = pygame.font.SysFont("monospace", 15, bold=True)
    font_lg = pygame.font.SysFont("monospace", 18, bold=True)
    font_ti = pygame.font.SysFont("monospace", 11)

    # ── Widgets (positioned inside the left panel) ──
    px = PANEL_W - 240   # left margin inside panel
    sliders = [
        Slider(px + 10, 80,  220, 14, 1000, 12000, 7000,  "Speed (m/s)",   "{:.0f}"),
        Slider(px + 10, 140, 220, 14, 1.0,  89.0,  45.0,  "Angle (°)",     "{:.1f}"),
        Slider(px + 10, 200, 220, 14, 0.0,  8e-5,  2e-5,  "Drag coeff",    "{:.1e}"),
        Slider(px + 10, 260, 220, 14, 1,    50,    10,    "Anim speed",    "{:.0f}"),
    ]
    btn_fire  = Button(px + 10, 310, 100, 32, "FIRE [SPC]")
    btn_reset = Button(px + 120, 310, 100, 32, "RESET [R]")
    btn_info  = Button(px + 10, 360, 210, 28, "Toggle theory line")

    show_theory = True
    missiles: list[MissileAnimation] = []
    color_idx = 0

    # Simulation viewport geometry (will be updated dynamically)
    x_origin_px = PANEL_W + 60   # launch site pixel x
    y_ground_px = WIN_H - 80     # ground level pixel y

    # Auto-scale: recomputed whenever a missile is fired
    scale_x = (SIM_W - 80) / WORLD_W
    scale_y = (WIN_H - 160) / WORLD_H
    current_max_x = WORLD_W

    def update_scale():
        nonlocal scale_x, scale_y, current_max_x
        if not missiles:
            return
        max_x = max(m.impact_x() for m in missiles)
        max_y = max(m.max_alt()  for m in missiles)
        if max_x < 1: max_x = 1
        if max_y < 1: max_y = 1
        current_max_x = max_x
        scale_x = (SIM_W - 100) / max_x
        scale_y = (y_ground_px - 80) / max_y

    def fire():
        nonlocal color_idx
        speed  = sliders[0].value
        angle  = sliders[1].value
        drag   = sliders[2].value
        pts = rust_trajectory(angle, speed, drag_coeff=drag)
        if len(pts) < 2:
            return
        anim = MissileAnimation(pts, MISSILE_COLORS[color_idx % len(MISSILE_COLORS)])
        anim.speed_factor = max(1, int(sliders[3].value))
        missiles.append(anim)
        color_idx += 1
        update_scale()

    # ── Main loop ──
    running = True
    while running:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    fire()
                elif event.key == pygame.K_r:
                    missiles.clear()
                    color_idx = 0
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                # If click is in simulation area, set angle from mouse direction
                if mx > PANEL_W:
                    sliders[1].value = screen_to_angle(mx, my, x_origin_px, y_ground_px)

            # Widget events
            for s in sliders:
                s.handle_event(event)
            if btn_fire.handle_event(event):
                fire()
            if btn_reset.handle_event(event):
                missiles.clear()
                color_idx = 0
            if btn_info.handle_event(event):
                show_theory = not show_theory

        # Advance animations
        for m in missiles:
            m.step()

        # ── Draw ──────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Left panel background
        pygame.draw.rect(screen, C_PANEL, (0, 0, PANEL_W, WIN_H))
        pygame.draw.line(screen, C_ACCENT, (PANEL_W, 0), (PANEL_W, WIN_H), 1)

        # Panel title
        t = font_lg.render("ICBM Sim", True, C_ACCENT)
        screen.blit(t, (px + 10, 20))
        t2 = font_sm.render("Rust engine · Python UI", True, (120, 140, 180))
        screen.blit(t2, (px + 10, 44))

        for s in sliders:
            s.draw(screen, font_sm)
        btn_fire.draw(screen, font_md)
        btn_reset.draw(screen, font_md)
        btn_info.draw(screen, font_sm)

        # Key hints at bottom of panel
        hints = ["Click sim area → set angle", "SPACE → fire", "R → reset"]
        for i, h in enumerate(hints):
            ht = font_ti.render(h, True, (100, 120, 160))
            screen.blit(ht, (px + 10, WIN_H - 90 + i * 18))

        # ── Simulation viewport ────────────────────────────────────────────
        # Grid lines (horizontal = altitude, vertical = range)
        n_vlines = 8
        n_hlines = 6
        for i in range(n_vlines + 1):
            gx = PANEL_W + int(i / n_vlines * (SIM_W - 20)) + 10
            pygame.draw.line(screen, C_GRID, (gx, 60), (gx, y_ground_px), 1)
            # Range label (km)
            if current_max_x > 0:
                km = int(i / n_vlines * current_max_x / 1000)
                lt = font_ti.render(f"{km}km", True, (70, 80, 120))
                screen.blit(lt, (gx - 15, y_ground_px + 6))

        for i in range(n_hlines + 1):
            gy = y_ground_px - int(i / n_hlines * (y_ground_px - 60))
            pygame.draw.line(screen, C_GRID, (PANEL_W + 10, gy), (WIN_W - 10, gy), 1)
            # Altitude label (km)
            if i > 0:
                alt_km = int(i / n_hlines * (y_ground_px - 60) / scale_y / 1000) if scale_y > 0 else 0
                at = font_ti.render(f"{alt_km}km", True, (70, 80, 120))
                screen.blit(at, (PANEL_W + 12, gy - 8))

        # Ground
        pygame.draw.rect(screen, C_GROUND,
                         (PANEL_W, y_ground_px, SIM_W, WIN_H - y_ground_px))
        pygame.draw.line(screen, (60, 160, 60),
                         (PANEL_W, y_ground_px), (WIN_W, y_ground_px), 2)

        # Launch pad marker
        pygame.draw.polygon(screen, (200, 200, 80), [
            (x_origin_px,     y_ground_px),
            (x_origin_px - 8, y_ground_px + 14),
            (x_origin_px + 8, y_ground_px + 14),
        ])

        # Aim arrow (from origin toward mouse in sim area)
        mx, my = pygame.mouse.get_pos()
        if mx > PANEL_W:
            aim_angle = screen_to_angle(mx, my, x_origin_px, y_ground_px)
            ar = math.radians(aim_angle)
            alen = 60
            ae = (x_origin_px + int(alen * math.cos(ar)),
                  y_ground_px  - int(alen * math.sin(ar)))
            pygame.draw.line(screen, (80, 160, 80), (x_origin_px, y_ground_px), ae, 2)

        # Theoretical range line
        if show_theory and missiles:
            speed = sliders[0].value
            angle = sliders[1].value
            trange = lib.theoretical_range(speed, angle, 9.81)
            tx, ty = world_to_screen(trange, 0, x_origin_px, y_ground_px, scale_x, scale_y)
            pygame.draw.line(screen, C_THEOR, (tx, y_ground_px - 20), (tx, y_ground_px + 2), 2)
            tt = font_ti.render(f"Theor. {trange/1000:.0f}km", True, C_THEOR)
            screen.blit(tt, (tx - 40, y_ground_px - 34))

        # Draw all missile trajectories
        for m in missiles:
            pts_px = [
                world_to_screen(x, y, x_origin_px, y_ground_px, scale_x, scale_y)
                for (x, y, _) in m.points[:m.idx + 1]
            ]
            # Trail: faded line
            if len(pts_px) >= 2:
                # Draw trail segments with fading alpha
                step = max(1, len(pts_px) // 120)
                trail_pts = pts_px[::step]
                if len(trail_pts) >= 2:
                    for i in range(1, len(trail_pts)):
                        alpha = int(255 * i / len(trail_pts))
                        col = tuple(int(c * alpha / 255) for c in m.color)
                        pygame.draw.line(screen, col, trail_pts[i-1], trail_pts[i], 2)

            # Missile head
            if not m.done and pts_px:
                hx, hy = pts_px[-1]
                pygame.draw.circle(screen, m.color, (hx, hy), 5)
                # Glow
                glow_surf = pygame.Surface((24, 24), pygame.SRCALPHA)
                pygame.draw.circle(glow_surf, (*m.color, 60), (12, 12), 12)
                screen.blit(glow_surf, (hx - 12, hy - 12))

            # Impact marker
            if m.done:
                ix, iy = world_to_screen(m.impact_x(), 0,
                                         x_origin_px, y_ground_px, scale_x, scale_y)
                pygame.draw.circle(screen, C_IMPACT, (ix, y_ground_px), 6)
                pygame.draw.circle(screen, (255, 200, 200), (ix, y_ground_px), 10, 2)

        # ── Telemetry panel (right side) ──────────────────────────────────────
        if missiles:
            m = missiles[-1]   # show stats for last missile
            idx = min(m.idx, len(m.points) - 1)
            cx, cy, ct = m.points[idx]
            lines = [
                f"── Last Missile ──",
                f"Range:  {cx/1000:8.1f} km",
                f"Alt:    {cy/1000:8.1f} km",
                f"Time:   {ct:8.1f} s",
                f"MaxAlt: {m.max_alt()/1000:8.1f} km",
                f"Impact: {m.impact_x()/1000:8.1f} km",
                f"FlightT:{m.flight_time():8.1f} s",
            ]
            for i, ln in enumerate(lines):
                lt = font_sm.render(ln, True, C_TEXT if i > 0 else C_ACCENT)
                screen.blit(lt, (WIN_W - 200, 70 + i * 18))

        # Title bar
        title = font_md.render(
            "Intercontinental Ballistic Missile Simulation  –  Rust physics engine + Python/Pygame UI",
            True, C_TEXT)
        screen.blit(title, (PANEL_W + 10, 6))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()