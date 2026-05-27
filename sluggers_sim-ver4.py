#!/usr/bin/env python3
"""sluggers_sim-ver4.py — Sluggers of the Lost Goal (pygame)

This version supports JSON robot configs (including hot-swap) and a visual
sensor overlay panel. It can also run a Python controller (including the
ES_Framework adapter shim) by emitting a `sluggers_sim.py`-compatible sensors
dictionary.

Usage:
    python sluggers_sim-ver4.py
    python sluggers_sim-ver4.py --robot bot.json
    python sluggers_sim-ver4.py --seed 118
    python sluggers_sim-ver4.py --controller es_framework_controller.py
    python sluggers_sim-ver4.py --log sim_log.jsonl

In-game:
    WASD     — drive (manual)
    R        — reset
    P        — toggle panel
    Z        — toggle zone overlay
    L        — load a robot JSON
    S        — switch panel to Sensors tab
    Drag & drop a .json file onto the window to hot-swap the robot.
"""

import pygame
import math
import random
import sys
import json
import os
import argparse
import importlib.util
from pathlib import Path
from typing import Any, Callable

import numpy as np

# ── Field constants ──────────────────────────────────────────────────────────
SCALE   = 80
FIELD_W = 8 * SCALE
FIELD_H = 4 * SCALE
FIELD_GAP  = int(6/12*SCALE)
MARGIN_H   = int(6/12*SCALE)
MARGIN_V   = int(6/12*SCALE)
TAPE       = int(2/12*SCALE)

W = MARGIN_H + FIELD_W + FIELD_GAP + FIELD_W + MARGIN_H
H = MARGIN_V + FIELD_H + MARGIN_V

PANEL_H  = 280
SCREEN_W = max(W, H + PANEL_H)
FPS      = 60
SCREEN_H = H + PANEL_H

# ── Hardcoded robot configuration ─────────────────────────────────────────────
_HARDCODED_ROBOT_JSON = """{
  "name": "Default Bot",
  "shape": {
    "type": "rect",
    "rectw": 11.0,
    "recth": 11.0,
    "circler": 5.5,
    "hexbasew": 11.0,
    "hexmidw": 16.6,
    "hextopw": 8.0,
    "hexmidh": 0.1,
    "hexhalfh": 5.5,
    "pentbasew": 11.0,
    "pentshoulderw": 17.8,
    "pentshoulderh": 2.89,
    "penthalfh": 9.36,
    "rotation_deg": 0.0,
    "forward_deg": 90.0
  },
  "motion": {
    "speed": 90.0,
    "turn_speed": 2.5,
    "omni": false
  },
  "wheels": [
    {
      "name": "WL",
      "pos_x": -3.5,
      "pos_y": -3.0,
      "width_in": 0.75,
      "radius_in": 1.5,
      "height_in": 1.5,
      "angle_deg": 0.0,
      "color": "#89b4fa"
    },
    {
      "name": "WR",
      "pos_x": 3.5,
      "pos_y": -3.0,
      "width_in": 0.75,
      "radius_in": 1.5,
      "height_in": 1.5,
      "angle_deg": 0.0,
      "color": "#89b4fa"
    }
  ],
  "sensors": [
    {
      "name": "tapeRight",
      "type": "tape",
      "pos_x": 1.0,
      "pos_y": 4.0,
      "angle_deg": 90.0,
      "color": "#FF6400"
    },
    {
      "name": "tapeLeft",
      "type": "tape",
      "pos_x": -1.0,
      "pos_y": 4.0,
      "angle_deg": 90.0,
      "color": "#FF6400"
    },
    {
      "name": "tapeCenter",
      "type": "tape",
      "pos_x": 0.0,
      "pos_y": 4.61,
      "angle_deg": 90.0,
      "color": "#FF6400"
    },
    {
      "name": "bumpLeft",
      "type": "bump",
      "pos_x": -2.75,
      "pos_y": 6.0,
      "angle_deg": 90.0,
      "color": "#C800C8",
      "shape": "rect",
      "rect_w_in": 5.5,
      "rect_h_in": 0.5,
      "radius_in": 0.6,
      "arc_radius_in": 2.0,
      "arc_start_deg": -35,
      "arc_end_deg": 35,
      "arc_thickness_in": 0.35
    },
    {
      "name": "bumpRight",
      "type": "bump",
      "pos_x": 2.75,
      "pos_y": 6.0,
      "angle_deg": 90.0,
      "color": "#C800C8",
      "shape": "rect",
      "rect_w_in": 5.5,
      "rect_h_in": 0.5,
      "radius_in": 0.6,
      "arc_radius_in": 2.0,
      "arc_start_deg": -35,
      "arc_end_deg": 35,
      "arc_thickness_in": 0.35
    },
    {
      "name": "ir2k",
      "type": "ir",
      "pos_x": 0.0,
      "pos_y": 2.5,
      "angle_deg": 90.0,
      "color": "#FFDC00",
      "fov_deg": 40,
      "detect_freqs": [
        2000
      ],
      "mode": "analog",
      "threshold": 0.15,
      "rangein": 192.0
    },
    {
      "name": "irObsticle",
      "type": "ir",
      "pos_x": 0.0,
      "pos_y": 0.0,
      "angle_deg": 90.0,
      "color": "#b9850b",
      "fov_deg": 40,
      "detect_freqs": [
        2500,
        1500
      ],
      "mode": "analog",
      "threshold": 0.15,
      "rangein": 98.0
    }
  ]
}"""

WHITE=(255,255,255); BLACK=(0,0,0); GRAY=(160,160,160)
RED=(220,50,50); BLUE=(50,100,220); DARK=(40,40,40)
PURPLE=(160,60,200); ORANGE=(240,140,30)
GREEN=(30,180,60); CYAN=(0,200,200); YELLOW=(220,200,0)
PANEL_BG=(235,235,240)
TAB_SENSORS='sensors'; TAB_UNO='uno'; TAB_ZONES='zones'

TAPE_W     = int(2/12*SCALE)
CENTER_Y   = FIELD_H // 2
H_TAPE_START = int(12/12*SCALE)
H_TAPE_END   = int(84/12*SCALE)
ISZ_END    = int(12/12*SCALE)
VERT_X     = [int(26/12*SCALE), int(48/12*SCALE), int(70/12*SCALE)]
VERT_LEN   = int(11/12*SCALE)
OBS_W      = int(3/12*SCALE)
OBS_H      = int(10/12*SCALE)

def ip(v): return v / 12.0 * SCALE


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_controller(path: str | None) -> Callable[[dict[str, Any], float], Any] | None:
    if not path:
        return None
    controller_path = Path(path).expanduser().resolve()
    spec = importlib.util.spec_from_file_location("sluggers_user_controller", controller_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load controller: {controller_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "update"):
        raise RuntimeError(f"{controller_path} must define update(sensors, dt)")
    return getattr(module, "update")


def _robot_local_to_world_px(robot_pos: list[float], robot_angle: float, forward_in: float, left_in: float) -> tuple[float, float]:
    """Convert robot-local (forward,left) inches to world pixel coordinates.

Coordinate convention matches sluggers_sim.py controllers:
  - forward is along heading (theta)
  - left is positive to robot's left
World axes are screen-like: +x right, +y down.
"""
    fpx = ip(forward_in)
    lpx = ip(left_in)
    ca = math.cos(robot_angle)
    sa = math.sin(robot_angle)
    wx = robot_pos[0] + fpx * ca + lpx * sa
    wy = robot_pos[1] + fpx * sa - lpx * ca
    return wx, wy


def _world_px_to_field_in(world_x: float, world_y: float, field_ox: float, field_oy: float) -> tuple[float, float]:
    return ((world_x - field_ox) * 12.0 / SCALE, (world_y - field_oy) * 12.0 / SCALE)


def _field_in_to_world_px(x_in: float, y_in: float, field_ox: float, field_oy: float) -> tuple[float, float]:
    return (field_ox + x_in * SCALE / 12.0, field_oy + y_in * SCALE / 12.0)


def _ray_circle_distance(origin: tuple[float, float], direction: tuple[float, float], center: tuple[float, float], radius: float, max_dist: float) -> float | None:
    # Ray origin O, direction D (unit), circle center C.
    ox, oy = origin
    dx, dy = direction
    cx, cy = center
    fx = ox - cx
    fy = oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0
    hit = None
    if t1 >= 0.0:
        hit = t1
    elif t2 >= 0.0:
        hit = t2
    if hit is None or hit > max_dist:
        return None
    return hit


def build_compat_sensors(
    *,
    time_s: float,
    dt: float,
    robot_pos_px: list[float],
    robot_angle: float,
    field_ox: float,
    field_oy: float,
    main_obs: list,
    enemy_pos_px: list[float] | None,
) -> dict[str, Any]:
    """Build a sluggers_sim.py-compatible sensors dict (in inches).

This enables using controllers written for `sluggers_sim.py`, including
`es_framework_controller.py`.
"""
    # Pose
    x_in, y_in = _world_px_to_field_in(robot_pos_px[0], robot_pos_px[1], field_ox, field_oy)
    pose = {"x": round(x_in, 2), "y": round(y_in, 2), "heading_deg": round(math.degrees(robot_angle), 2)}

    # Tape points (match sluggers_sim.py layout)
    ROBOT_L_IN = 11.0
    ROBOT_W_IN = 10.0
    tape_points = {
        "front_left": _robot_local_to_world_px(robot_pos_px, robot_angle, ROBOT_L_IN / 2.0, ROBOT_W_IN * 0.35),
        "front_center": _robot_local_to_world_px(robot_pos_px, robot_angle, ROBOT_L_IN / 2.0, 0.0),
        "front_right": _robot_local_to_world_px(robot_pos_px, robot_angle, ROBOT_L_IN / 2.0, -ROBOT_W_IN * 0.35),
        "mid_left": _robot_local_to_world_px(robot_pos_px, robot_angle, 0.0, ROBOT_W_IN / 2.0),
        "mid_right": _robot_local_to_world_px(robot_pos_px, robot_angle, 0.0, -ROBOT_W_IN / 2.0),
        "rear_left": _robot_local_to_world_px(robot_pos_px, robot_angle, -ROBOT_L_IN / 2.0, ROBOT_W_IN * 0.35),
        "rear_center": _robot_local_to_world_px(robot_pos_px, robot_angle, -ROBOT_L_IN / 2.0, 0.0),
        "rear_right": _robot_local_to_world_px(robot_pos_px, robot_angle, -ROBOT_L_IN / 2.0, -ROBOT_W_IN * 0.35),
    }
    tape = {name: bool(point_on_tape(pt[0], pt[1], field_ox, field_oy)) for name, pt in tape_points.items()}
    tape.update(
        {
            "front": tape["front_left"] or tape["front_center"] or tape["front_right"],
            "rear": tape["rear_left"] or tape["rear_center"] or tape["rear_right"],
            "left": tape["front_left"] or tape["mid_left"] or tape["rear_left"],
            "right": tape["front_right"] or tape["mid_right"] or tape["rear_right"],
        }
    )

    # Obstacles as field-local inches
    obs_in = []
    for o in main_obs:
        obs_in.append({
            "x": float(o.x) * 12.0 / SCALE,
            "y": float(o.y) * 12.0 / SCALE,
            "hw": float(o.w) * 6.0 / SCALE,
            "hh": float(o.h) * 6.0 / SCALE,
        })

    # Basic collision + bump direction (approx.)
    bump = {"any": False, "front": False, "rear": False, "left": False, "right": False, "with": "none"}
    body_radius_in = max(ROBOT_L_IN, ROBOT_W_IN) / 2.0
    collision_with = "none"
    for idx, o in enumerate(obs_in, start=1):
        dx = o["x"] - x_in
        dy = o["y"] - y_in
        if abs(dx) <= o["hw"] + body_radius_in and abs(dy) <= o["hh"] + body_radius_in:
            bump["any"] = True
            collision_with = f"obstacle_{idx}"
            # Convert to robot-local forward/left (inches)
            ca = math.cos(robot_angle)
            sa = math.sin(robot_angle)
            # world = forward*(ca,sa) + left*(sa,-ca)
            forward = dx * ca + dy * sa
            left = dx * sa - dy * ca
            if abs(forward) >= abs(left):
                bump["front" if forward >= 0 else "rear"] = True
            else:
                bump["left" if left >= 0 else "right"] = True
            break
    bump["with"] = collision_with

    # Ping rays (approximate)
    def ping_at(offset_deg: float, max_dist: float = 120.0) -> dict[str, Any]:
        # Ray origin at front center.
        ox_in = x_in + math.cos(robot_angle) * (ROBOT_L_IN / 2.0)
        oy_in = y_in + math.sin(robot_angle) * (ROBOT_L_IN / 2.0)
        ang = robot_angle + math.radians(offset_deg)
        rdx = math.cos(ang)
        rdy = math.sin(ang)
        best = max_dist
        hit = "none"
        for i, o in enumerate(obs_in, start=1):
            t = ray_aabb(ox_in, oy_in, rdx, rdy, o["x"], o["y"], o["hw"], o["hh"])
            if t is not None and 0.0 < t < best:
                best = t
                hit = f"obstacle_{i}"
        if enemy_pos_px is not None:
            ex_in, ey_in = _world_px_to_field_in(enemy_pos_px[0], enemy_pos_px[1], field_ox, field_oy)
            t = _ray_circle_distance((ox_in, oy_in), (rdx, rdy), (ex_in, ey_in), 5.5, max_dist)
            if t is not None and 0.0 < t < best:
                best = t
                hit = "enemy"
        return {"range_in": round(best, 2), "object": hit}

    ping = {
        "front": ping_at(0.0),
        "front_left": ping_at(25.0),
        "front_right": ping_at(-25.0),
        "left": ping_at(90.0),
        "right": ping_at(-90.0),
    }

    # IR readings (geometric approximation)
    def ir_reading(tx_in: float, ty_in: float) -> dict[str, Any]:
        dx = tx_in - x_in
        dy = ty_in - y_in
        rng = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - robot_angle)
        in_front = abs(math.degrees(bearing)) <= 35.0
        in_rear = abs(math.degrees(wrap_angle(bearing - math.pi))) <= 35.0
        strength = 0.0 if rng < 1e-6 else 1.0 / (rng * rng)
        return {
            "range_in": round(rng, 2),
            "bearing_deg": round(math.degrees(bearing), 2),
            "strength": round(strength, 6),
            "front": in_front,
            "rear": in_rear,
        }

    ir_target = {"range_in": 0.0, "bearing_deg": 0.0, "strength": 0.0, "front": False, "rear": False}
    if enemy_pos_px is not None:
        ex_in, ey_in = _world_px_to_field_in(enemy_pos_px[0], enemy_pos_px[1], field_ox, field_oy)
        ir_target = ir_reading(ex_in, ey_in)

    nearest_obs_ir = {"range_in": 0.0, "bearing_deg": 0.0, "strength": 0.0, "front": False, "rear": False}
    if obs_in:
        nearest = min(obs_in, key=lambda o: math.hypot(o["x"] - x_in, o["y"] - y_in))
        nearest_obs_ir = ir_reading(nearest["x"], nearest["y"])

    # Minimal game state (enough for ES adapter)
    isz_reached = x_in <= 12.0
    # Very rough zones from tape markers: 0..4
    zone_lines_in = (26.0, 48.0, 70.0)
    legal_zone = 0
    if isz_reached:
        legal_zone = 1
    for i, z in enumerate(zone_lines_in, start=1):
        if x_in <= z:
            legal_zone = max(legal_zone, i)
    game = {
        "ammo": 6,
        "score": 0,
        "hits": 0,
        "goal": False,
        "legal_zone": legal_zone,
        "isz_reached": isz_reached,
        "last_shot": "none",
        "disqualified": False,
        "disq_reason": "",
    }

    return {
        "time_s": round(time_s, 3),
        "dt_s": float(dt),
        "pose": pose,
        "tape": tape,
        "bump": bump,
        "ping": ping,
        "ir": {
            "target_2k": ir_target,
            "nearest_obstacle": nearest_obs_ir,
            "obstacles": [nearest_obs_ir],
        },
        "track_wire": {"left": 0.0, "right": 0.0, "nearest_obstacle_x": 0.0},
        "game": game,
    }

# ── Robot config globals (overwritten by load_robot_json) ─────────────────────
ROBOT_NAME  = "Hexagon"
OMNI_WHEELS  = False
ROBOT_WHEELS = []  # [{name,pos_x,pos_y,radius_in,color}]
SHAPE_TYPE  = 'hexagon'
ROBOT_SPEED = 90.0
TURN_SPEED  = 2.5
RECT_W, RECT_H = 11.0, 11.0
CIRCLE_R       = 5.5
HEX_BASE_W, HEX_MID_W, HEX_TOP_W, HEX_MID_H, HEX_HALF_H = 11.0, 16.6, 8.0, 0.1, 5.5
PENT_BASE_W, PENT_SHOULDER_W, PENT_SHOULDER_H, PENT_HALF_H = 11.0, 17.80, 2.89, 9.36
SHAPE_ROT_DEG = 0.0
FORWARD_DEG = 0.0
ROBOT_MASS     = 1.0
ROBOT_SHAPE    = []
ROBOT_INERTIA  = 1.0
SENSOR_DEFS    = []


def _build_shape():
    if SHAPE_TYPE == 'rect':
        hw, hh = RECT_W/2, RECT_H/2
        pts = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
    elif SHAPE_TYPE == 'circle':
        n = 20
        pts = [(CIRCLE_R*math.cos(2*math.pi*i/n), CIRCLE_R*math.sin(2*math.pi*i/n))
               for i in range(n)]
    elif SHAPE_TYPE == 'hexagon':
        bw = HEX_BASE_W / 2.0
        mw = HEX_MID_W / 2.0
        tw = HEX_TOP_W / 2.0
        mh = max(0.0, min(HEX_HALF_H - 1.0, HEX_MID_H))
        bottom_y = 0.0
        mid_y = mh
        top_y = HEX_HALF_H
        y_center = (bottom_y + top_y) / 2.0
        pts = [
            (-bw, bottom_y - y_center),
            ( bw, bottom_y - y_center),
            ( mw, mid_y - y_center),
            ( tw, top_y - y_center),
            (-tw, top_y - y_center),
            (-mw, mid_y - y_center),
        ]
    elif SHAPE_TYPE == 'pentagon':
        bw = PENT_BASE_W / 2.0
        sw = PENT_SHOULDER_W / 2.0
        sh = max(0.0, min(PENT_HALF_H - 1.0, PENT_SHOULDER_H))
        bottom_y = 0.0
        shoulder_y = sh
        top_y = PENT_HALF_H
        y_center = (bottom_y + top_y) / 2.0
        pts = [
            (-bw, bottom_y - y_center),
            ( bw, bottom_y - y_center),
            ( sw, shoulder_y - y_center),
            ( 0.0, top_y - y_center),
            (-sw, shoulder_y - y_center),
        ]
    else:
        hw, hh = RECT_W/2, RECT_H/2
        pts = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
    if abs(SHAPE_ROT_DEG) > 1e-9:
        a = math.radians(SHAPE_ROT_DEG)
        ca, sa = math.cos(a), math.sin(a)
        pts = [(x*ca - y*sa, x*sa + y*ca) for x, y in pts]
    return pts


def _rebuild_robot_globals():
    global ROBOT_SHAPE, ROBOT_INERTIA
    ROBOT_SHAPE = _build_shape()
    if SHAPE_TYPE == 'rect':
        rw,rh = ip(RECT_W),ip(RECT_H)
        ROBOT_INERTIA = ROBOT_MASS*(rw**2+rh**2)/12.0
    elif SHAPE_TYPE == 'circle':
        r = ip(CIRCLE_R)
        ROBOT_INERTIA = ROBOT_MASS*r**2/2.0
    elif SHAPE_TYPE == 'hexagon':
        rw, rh = ip(max(HEX_BASE_W, HEX_MID_W, HEX_TOP_W)), ip(HEX_HALF_H * 2)
        ROBOT_INERTIA = ROBOT_MASS * (rw**2 + rh**2) / 12.0
    elif SHAPE_TYPE == 'pentagon':
        rw, rh = ip(max(PENT_BASE_W, PENT_SHOULDER_W)), ip(PENT_HALF_H * 2)
        ROBOT_INERTIA = ROBOT_MASS * (rw**2 + rh**2) / 12.0
    else:
        rw,rh = ip(RECT_W),ip(RECT_H)
        ROBOT_INERTIA = ROBOT_MASS*(rw**2+rh**2)/12.0


def _hex_str(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i+2],16) for i in (0,2,4))


def _sensor_from_json(s):
    stype = s['type']
    col   = _hex_str(s.get('color','#B4B4B4'))
    pos   = (float(s.get('pos_x',0.0)), float(s.get('pos_y',0.0)))
    ang   = float(s.get('angle_deg',0.0))
    name  = s.get('name', stype)
    params = {}
    if stype == 'bump':
        params = {
            'shape':            s.get('shape', 'rect'),
            'radius_in':        float(s.get('radius_in', 0.6)),
            'rect_w_in':        float(s.get('rect_w_in', 2.0)),
            'rect_h_in':        float(s.get('rect_h_in', 0.5)),
            'arc_radius_in':    float(s.get('arc_radius_in', 2.0)),
            'arc_start_deg':    float(s.get('arc_start_deg', -35)),
            'arc_end_deg':      float(s.get('arc_end_deg', 35)),
            'arc_thickness_in': float(s.get('arc_thickness_in', 0.35)),
        }
    elif stype == 'ping':
        params = {'fov_deg': float(s.get('fov_deg',20)), 'max_range_in': float(s.get('max_range_in',72)), 'n_rays': int(s.get('nrays',7))}
    elif stype == 'trackwire':
        params = {'max_range_in': float(s.get('max_range_in',6)), 'gain': float(s.get('gain',1.0))}
    elif stype == 'ir':
        raw = s.get('detect_freqs',[2000])
        if isinstance(raw, str): raw = [int(x.strip()) for x in raw.split(',') if x.strip()]
        params = {'fov_deg': float(s.get('fov_deg',40)), 'detect_freqs': [int(f) for f in raw],
                  'mode': s.get('mode','analog'), 'threshold': float(s.get('threshold',0.15)),
                  'rangein': float(s.get('rangein',16.0)), 'draw_mode': 'analog', 'draw_scale': 28}
    elif stype == 'imu':
        params = {'gyro_noise': float(s.get('gyro_noise',0.002)),
                  'accel_noise': float(s.get('accel_noise',0.5)),
                  'gyro_bias': float(s.get('gyro_bias',0.001))}
    return {'name':name,'type':stype,'pos':pos,'angle_deg':ang,'params':params,'color':col}


def load_robot_json(path):
    with open(path) as f:
        data = json.load(f)
    _load_robot_data(data)
    print(f"[ver4] Loaded '{ROBOT_NAME}' — shape={SHAPE_TYPE} speed={ROBOT_SPEED} omni={OMNI_WHEELS} sensors={len(SENSOR_DEFS)}")

def _load_robot_data(data):
    """Load robot configuration from a data dictionary (used for hardcoded configs)."""
    global ROBOT_NAME,OMNI_WHEELS,SHAPE_TYPE,ROBOT_SPEED,TURN_SPEED
    global RECT_W,RECT_H,CIRCLE_R
    global HEX_BASE_W,HEX_MID_W,HEX_TOP_W,HEX_MID_H,HEX_HALF_H
    global PENT_BASE_W,PENT_SHOULDER_W,PENT_SHOULDER_H,PENT_HALF_H
    global SHAPE_ROT_DEG,FORWARD_DEG,SENSOR_DEFS,ROBOT_WHEELS
    ROBOT_NAME   = data.get('name','Robot')
    sh = data.get('shape',{})
    SHAPE_TYPE   = sh.get('type','rect')
    RECT_W       = float(sh.get('rectw', sh.get('rect_w', 11.0)))
    RECT_H       = float(sh.get('recth', sh.get('rect_h', 11.0)))
    CIRCLE_R     = float(sh.get('circler', sh.get('circle_r', 5.5)))
    HEX_BASE_W   = float(sh.get('hexbasew', 11.0))
    HEX_MID_W    = float(sh.get('hexmidw', 16.6))
    HEX_TOP_W    = float(sh.get('hextopw', 8.0))
    HEX_MID_H    = float(sh.get('hexmidh', 0.1))
    HEX_HALF_H   = float(sh.get('hexhalfh', 5.5))
    PENT_BASE_W  = float(sh.get('pentbasew', 11.0))
    PENT_SHOULDER_W = float(sh.get('pentshoulderw', 17.80))
    PENT_SHOULDER_H = float(sh.get('pentshoulderh', 2.89))
    PENT_HALF_H  = float(sh.get('penthalfh', 9.36))
    SHAPE_ROT_DEG = float(sh.get('rotation_deg', 0.0))
    FORWARD_DEG = float(sh.get('forward_deg', 0.0))
    mo = data.get('motion',{})
    ROBOT_SPEED  = float(mo.get('speed',90.0))
    TURN_SPEED   = float(mo.get('turn_speed',2.5))
    OMNI_WHEELS  = bool(mo.get('omni',False))
    raw_sensors  = data.get('sensors', None)
    SENSOR_DEFS  = [_sensor_from_json(s) for s in raw_sensors] if raw_sensors else []
    ROBOT_WHEELS = data.get('wheels', [])
    _rebuild_robot_globals()
    print(f"[ver4] Loaded '{ROBOT_NAME}' — shape={SHAPE_TYPE} speed={ROBOT_SPEED} omni={OMNI_WHEELS} sensors={len(SENSOR_DEFS)}")

# ── Sensor and collision helpers ──────────────────────────────────────────────
def world_corners(pos, angle):
    cx,cy = pos; ca,sa = math.cos(angle),math.sin(angle)
    return [(cx+ip(lx)*ca-ip(ly)*sa, cy+ip(lx)*sa+ip(ly)*ca) for lx,ly in ROBOT_SHAPE]

def obs_pts(o):
    ox,oy=o['x'],o['y']; hw,hh=o['w']/2,o['h']/2
    return [(ox-hw,oy-hh),(ox+hw,oy-hh),(ox+hw,oy+hh),(ox-hw,oy+hh)]

# ── SAT ───────────────────────────────────────────────────────────────────────
def _proj(pts,ax):
    d=[p[0]*ax[0]+p[1]*ax[1] for p in pts]; return min(d),max(d)

def sat_query(robot_pts, obs_pts_list):
    best_depth=float('inf'); best_axis=None
    for poly in (robot_pts, obs_pts_list):
        n=len(poly)
        for i in range(n):
            ex=poly[(i+1)%n][0]-poly[i][0]; ey=poly[(i+1)%n][1]-poly[i][1]
            ln=math.hypot(ex,ey)
            if ln<1e-9: continue
            axis=(-ey/ln,ex/ln)
            a0,a1=_proj(robot_pts,axis); b0,b1=_proj(obs_pts_list,axis)
            if a1<b0 or b1<a0: return True,None,0.0,[]
            overlap=min(a1,b1)-max(a0,b0)
            if overlap<best_depth: best_depth=overlap; best_axis=axis
    if best_axis is None: return True,None,0.0,[]
    rcx=sum(p[0] for p in robot_pts)/len(robot_pts); rcy=sum(p[1] for p in robot_pts)/len(robot_pts)
    ocx=sum(p[0] for p in obs_pts_list)/len(obs_pts_list); ocy=sum(p[1] for p in obs_pts_list)/len(obs_pts_list)
    if (rcx-ocx)*best_axis[0]+(rcy-ocy)*best_axis[1]<0: best_axis=(-best_axis[0],-best_axis[1])
    nx,ny=best_axis
    if len(obs_pts_list)==4:
        oxs=[p[0] for p in obs_pts_list]; oys=[p[1] for p in obs_pts_list]
        ox0,ox1=min(oxs),max(oxs); oy0,oy1=min(oys),max(oys)
        contact_pts=[p for p in robot_pts if ox0<=p[0]<=ox1 and oy0<=p[1]<=oy1]
        if not contact_pts: contact_pts=[min(robot_pts,key=lambda p:p[0]*nx+p[1]*ny)]
    else:
        contact_pts=[min(robot_pts,key=lambda p:p[0]*nx+p[1]*ny)]
    return False,best_axis,best_depth,contact_pts

def any_col(pos,angle,obstacles):
    rc=world_corners(pos,angle)
    return any(not sat_query(rc,obs_pts(o))[0] for o in obstacles)

# ── Obstacle ──────────────────────────────────────────────────────────────────
class Obstacle:
    def __init__(self,x,y,w,h,front_angle=0.0,is_enemy=0):
        self.x=x;self.y=y;self.w=w;self.h=h;self.front_angle=front_angle;self.is_enemy=is_enemy;self.ir_freq=None
    def __getitem__(self,k): return getattr(self,k)
    def __setitem__(self,k,v): setattr(self,k,v)
    def get(self,k,d=None): return getattr(self,k,d)

# ── OOB ───────────────────────────────────────────────────────────────────────
def polygon_area(pts):
    n=len(pts);a=0
    for i in range(n):j=(i+1)%n;a+=pts[i][0]*pts[j][1]-pts[j][0]*pts[i][1]
    return abs(a)/2

def clip_hp(poly,nx,ny,d):
    out=[];n=len(poly)
    for i in range(n):
        A=poly[i];B=poly[(i+1)%n];dA=nx*A[0]+ny*A[1]-d;dB=nx*B[0]+ny*B[1]-d
        if dA<=0: out.append(A)
        if (dA<0)!=(dB<0):
            t=dA/(dA-dB);out.append((A[0]+t*(B[0]-A[0]),A[1]+t*(B[1]-A[1])))
    return out

def inside_area(pts,ox,oy):
    poly=list(pts)
    poly=clip_hp(poly,1,0,ox+FIELD_W);poly=clip_hp(poly,-1,0,-ox)
    poly=clip_hp(poly,0,1,oy+FIELD_H);poly=clip_hp(poly,0,-1,-oy)
    return polygon_area(poly) if len(poly)>=3 else 0

def is_oob(pos,angle,fox,foy):
    c=world_corners(pos,angle);t=polygon_area(c)
    return (inside_area(c,fox,foy)/t)<0.5 if t else False

# ── Physics ───────────────────────────────────────────────────────────────────
def step_robot(pos,angle,vx,vy,omega,obstacles,dt):
    nvx,nvy,nom=vx,vy,omega; contacts=[]
    rc=world_corners(pos,angle)
    for o in obstacles:
        sep,normal,depth,contact_pts=sat_query(rc,obs_pts(o))
        if sep or normal is None or not contact_pts: continue
        nx,ny=normal
        for cp in contact_pts:
            rx=cp[0]-pos[0];ry=cp[1]-pos[1];contacts.append((nx,ny,depth,rx,ry))
    for _ in range(4):
        for (nx,ny,depth,rx,ry) in contacts:
            vcx=nvx-nom*ry;vcy=nvy+nom*rx;vn=vcx*nx+vcy*ny
            if vn>=0: continue
            rn=rx*ny-ry*nx;eff_mass=1.0/ROBOT_MASS+rn*rn/ROBOT_INERTIA
            j=max(-vn/eff_mass,0.0);nvx+=j*nx/ROBOT_MASS;nvy+=j*ny/ROBOT_MASS;nom+=j*rn/ROBOT_INERTIA
    for (nx,ny,depth,rx,ry) in contacts:
        vn=nvx*nx+nvy*ny
        if vn<0: nvx-=vn*nx;nvy-=vn*ny
    new_angle=angle+nom*dt; new_pos=[pos[0]+nvx*dt,pos[1]+nvy*dt]
    for _ in range(6):
        rc2=world_corners(new_pos,new_angle); resolved=True
        for o in obstacles:
            sep,normal,depth,cpts=sat_query(rc2,obs_pts(o))
            if sep or normal is None or not cpts: continue
            resolved=False;nx,ny=normal;new_pos[0]+=nx*depth;new_pos[1]+=ny*depth
        if resolved: break
    return new_pos,new_angle,nvx,nvy,nom

# ── Obstacles ─────────────────────────────────────────────────────────────────
def create_obstacles(is_enemy=0):
    obs=[]
    for mid_in in [77,59,37]:
        cx=int(mid_in/12*SCALE);m=int(2/12*SCALE)
        ox=random.uniform(cx-m,cx+m);oy=random.uniform(OBS_H//2+TAPE,FIELD_H-OBS_H//2-TAPE)
        fa=0.0
        o=Obstacle(ox,oy,OBS_W,OBS_H,fa,is_enemy=is_enemy);o.ir_freq=random.choice([1500,2500]);obs.append(o)
    return obs

def field_origin(is_opp=False):
    return (MARGIN_H+(FIELD_W+FIELD_GAP if is_opp else 0),MARGIN_V)

def ray_aabb(rx,ry,rdx,rdy,ox,oy,hw,hh):
    tmin,tmax=-1e9,1e9
    for(ro,rd,lo,hi) in [(rx,rdx,ox-hw,ox+hw),(ry,rdy,oy-hh,oy+hh)]:
        if abs(rd)<1e-9:
            if ro<lo or ro>hi: return None
        else:
            t1=(lo-ro)/rd;t2=(hi-ro)/rd
            if t1>t2: t1,t2=t2,t1
            tmin=max(tmin,t1);tmax=min(tmax,t2)
    if tmin>tmax: return None
    return tmin if tmin>=0 else(tmax if tmax>=0 else None)

def ray_polygon(rx,ry,rdx,rdy,poly):
    best=None;n=len(poly)
    for i in range(n):
        ax,ay=poly[i];bx,by=poly[(i+1)%n];ex,ey=bx-ax,by-ay;denom=rdx*ey-rdy*ex
        if abs(denom)<1e-9: continue
        fx,fy=ax-rx,ay-ry;t=(fx*ey-fy*ex)/denom;u=(fx*rdy-fy*rdx)/denom
        if 0<=u<=1 and t>=0:
            if best is None or t<best: best=t
    return best

def aabb_segment_nearest(ox,oy,hw,hh,px,py):
    cx=max(ox-hw,min(ox+hw,px));cy=max(oy-hh,min(oy+hh,py))
    if cx==px and cy==py:
        return min(px-(ox-hw),(ox+hw)-px,py-(oy-hh),(oy+hh)-py)
    return math.hypot(px-cx,py-cy)

def point_on_tape(px,py,fox,foy):
    fx=px-fox;fy=py-foy;hw=TAPE_W/2
    if fx<TAPE_W or fx>FIELD_W-TAPE_W or fy<TAPE_W or fy>FIELD_H-TAPE_W: return True
    lx_start=H_TAPE_START;lx_end=H_TAPE_END
    if lx_start<=fx<=lx_end and abs(fy-CENTER_Y)<=hw: return True
    for vx2 in VERT_X:
        if abs(fx-vx2)<=hw and abs(fy-CENTER_Y)<=VERT_LEN//2: return True
    return False

def sensor_world_pose(robot_pos,robot_angle,s):
    lx, ly = s['pos']
    ca, sa = math.cos(robot_angle), math.sin(robot_angle)
    wx = robot_pos[0] + ip(lx)*ca - ip(ly)*sa
    wy = robot_pos[1] + ip(lx)*sa + ip(ly)*ca
    wa = robot_angle + math.radians(s.get('angle_deg', 0.0))
    return wx,wy,wa

# ── IMU state ─────────────────────────────────────────────────────────────────
_imu_prev_vx=0.0;_imu_prev_vy=0.0;_imu_yaw=0.0;_imu_omega=0.0;_imu_vx=0.0;_imu_vy=0.0

# ── Sensor samplers ───────────────────────────────────────────────────────────
def sample_tape(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    wx,wy,_=sensor_world_pose(rp,ra,s);on=point_on_tape(wx,wy,fox,foy)
    return {'on_tape':on,'value':1.0 if on else 0.0,'world_pos':(wx,wy)}

def sample_bump(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    wx,wy,wa=sensor_world_pose(rp,ra,s);p=s['params'];pressed=False;hit_what=None
    def chk(px,py,r):
        nonlocal pressed,hit_what
        fx,fy=px-fox,py-foy
        for o in obs:
            if not o.is_enemy:
                hw,hh=o.w/2,o.h/2;dx=max(abs(px-(fox+o.x))-hw,0);dy=max(abs(py-(foy+o.y))-hh,0)
                if math.hypot(dx,dy)<r: pressed=True;hit_what='obstacle';return True
        if ep:
            if math.hypot(px-ep[0],py-ep[1])<r+ip(max(HEX_MID_W,HEX_HALF_H)): pressed=True;hit_what='enemy';return True
        return False
    shape = p.get('shape', 'circle')
    if shape == 'rect':
        rw = ip(p.get('rect_w_in', 2.0)) / 2
        rh = ip(p.get('rect_h_in', 0.5)) / 2
        ca2, sa2 = math.cos(wa), math.sin(wa)
        for u in (-1.0, -0.5, 0.0, 0.5, 1.0):
            chk(wx + rh*ca2 + rw*(-sa2)*u, wy + rh*sa2 + rw*ca2*u, ip(0.25))
            if pressed: break
        return {'pressed': pressed, 'hit': hit_what, 'world_pos': (wx, wy),
                'rect_w_px': ip(p.get('rect_w_in', 2.0)), 'rect_h_px': ip(p.get('rect_h_in', 0.5)),
                'shape': 'rect', 'angle': wa}
    elif shape == 'arc':
        rr = ip(p.get('arc_radius_in', 2.0))
        th = max(1.0, ip(p.get('arc_thickness_in', 0.35)))
        start_deg = float(p.get('arc_start_deg', -35))
        end_deg = float(p.get('arc_end_deg', 35))
        if end_deg < start_deg: start_deg, end_deg = end_deg, start_deg
        span = max(1.0, end_deg - start_deg)
        ro = rr + th / 2
        ri = max(1.0, rr - th / 2)
        steps = max(18, int(span / 3))
        for j in range(steps + 1):
            a = wa + math.radians(start_deg + span * j / steps)
            for rad in (ri, (ri + ro) / 2, ro):
                chk(wx + rad * math.cos(a), wy + rad * math.sin(a), ip(0.18))
                if pressed: break
            if pressed: break
        return {'pressed': pressed, 'hit': hit_what, 'world_pos': (wx, wy),
                'shape': 'arc', 'angle': wa,
                'arc_radius_px': rr, 'arc_thickness_px': th,
                'arc_start_deg': start_deg, 'arc_end_deg': end_deg}
    else:
        r = ip(p.get('radius_in', 0.5)); chk(wx, wy, r)
        return {'pressed': pressed, 'hit': hit_what, 'world_pos': (wx, wy), 'radius_px': r, 'shape': 'circle'}

def sample_ping(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    wx,wy,wa=sensor_world_pose(rp,ra,s);p=s['params']
    fov=math.radians(p.get('fov_deg',20));maxr=ip(p.get('max_range_in',60));nrays=p.get('nrays',5)
    best_dist=maxr;best_type=None;ray_results=[]
    angles=[wa + fov*(i/(nrays-1)-0.5) for i in range(nrays)] if nrays>1 else [wa]
    for ra2 in angles:
        rdx,rdy=math.cos(ra2),math.sin(ra2)
        hit_d,hit_type=maxr,None
        localx=wx-fox; localy=wy-foy
        for o in obs:
            if o.get('is_enemy', o.get('isenemy', 0)) == 0:
                t=ray_aabb(localx,localy,rdx,rdy,o.x,o.y,o.w/2,o.h/2)
                if t is not None and 0<t<hit_d:
                    hit_d,hit_type=t,'main_obs'
        if oobs is not None and ofox is not None and ofoy is not None:
            opp_localx=wx-ofox; opp_localy=wy-ofoy
            for o in oobs:
                if o.get('is_enemy', o.get('isenemy', 1)) == 1:
                    t=ray_aabb(opp_localx,opp_localy,rdx,rdy,FIELD_W-o.x,o.y,o.w/2,o.h/2)
                    if t is not None and 0<t<hit_d:
                        hit_d,hit_type=t,'opp_obs'
        if ep:
            t=ray_polygon(wx,wy,rdx,rdy,world_corners(ep,ea))
            if t is not None and 0<t<hit_d:
                hit_d,hit_type=t,'enemy'
        ray_results.append((ra2,rdx,rdy,hit_d,hit_type))
        if hit_d<best_dist:
            best_dist=hit_d;best_type=hit_type
    return {
        'distance_in':best_dist/SCALE*12.0,
        'hit':best_type,
        'ray_results':ray_results,
        'origin':(wx,wy),
        'debug':{
            'opp_obs_count':len(oobs) if oobs else 0,
            'enemy_pos':ep,
            'oppfox':ofox,
            'oppfoy':ofoy,
        }
    }

def sample_lidar(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    wx,wy,wa=sensor_world_pose(rp,ra,s);p=s['params']
    maxr=ip(p.get('max_range_in',60))
    rdx,rdy=math.cos(wa),math.sin(wa)
    best_dist=maxr;best_type=None
    localx=wx-fox; localy=wy-foy
    for o in obs:
        if o.get('is_enemy', o.get('isenemy', 0)) == 0:
            t=ray_aabb(localx,localy,rdx,rdy,o.x,o.y,o.w/2,o.h/2)
            if t is not None and 0<t<best_dist:
                best_dist,best_type=t,'main_obs'
    if oobs is not None and ofox is not None and ofoy is not None:
        opp_localx=wx-ofox; opp_localy=wy-ofoy
        for o in oobs:
            if o.get('is_enemy', o.get('isenemy', 1)) == 1:
                t=ray_aabb(opp_localx,opp_localy,rdx,rdy,FIELD_W-o.x,o.y,o.w/2,o.h/2)
                if t is not None and 0<t<best_dist:
                    best_dist,best_type=t,'opp_obs'
    if ep:
        t=ray_polygon(wx,wy,rdx,rdy,world_corners(ep,ea))
        if t is not None and 0<t<best_dist:
            best_dist,best_type=t,'enemy'
    return {
        'distance_m': best_dist / SCALE * 12.0 * 0.0254,
        'hit': best_type,
        'origin': (wx,wy),
        'ray_results': [(wa, rdx, rdy, best_dist, best_type)],
    }

def sample_trackwire(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    wx,wy,_=sensor_world_pose(rp,ra,s);p=s['params'];maxr=ip(p.get('max_range_in',6));gain=p.get('gain',1.0)
    best_d=maxr
    for o in obs:
        if not o.is_enemy:
            d=aabb_segment_nearest(fox+o.x,foy+o.y,o.w/2,o.h/2,wx,wy)
            if d<best_d: best_d=d
    return {'strength':gain*max(0.0,1.0-best_d/maxr),'nearest_dist_in':best_d/SCALE*12.0,'world_pos':(wx,wy)}

EMITTER_FOV_RAD = math.radians(30)  # half-angle of each IR emitter's beam

def _obs_world_pos(o, fox, foy, ofox, ofoy):
    """World position of an obstacle emitter."""
    if o.is_enemy and ofox is not None:
        return ofox + (FIELD_W - o.x), ofoy + o.y
    return fox + o.x, foy + o.y

def _ray_occluded(wx, wy, tx, ty, target_dist, blocking_obs, fox, foy, ofox, ofoy):
    """True if any obstacle blocks line-of-sight from (wx,wy) to (tx,ty)."""
    rdx, rdy = tx - wx, ty - wy
    for o in blocking_obs:
        ocx, ocy = _obs_world_pos(o, fox, foy, ofox, ofoy)
        t = ray_aabb(wx, wy, rdx, rdy, ocx, ocy, o.w / 2, o.h / 2)
        if t is not None and 0.01 < t < target_dist - 1.0:
            return True
    return False

def sample_ir(s, rp, ra, fox, foy, obs, ep, ea, pvx, pvy, dt, oobs=None, ofox=None, ofoy=None):
    wx, wy, wa = sensor_world_pose(rp, ra, s)
    p = s['params']
    detect_freqs = list(p.get('detect_freqs', [1500, 2000, 2500]))
    mode = p.get('mode', 'analog')
    threshold = p.get('threshold', 0.1)
    sensor_fov = math.radians(p.get('fov_deg', 40)) / 2
    rangein = max(0.001, float(p.get('rangein', 16.0)))
    analog = {f: 0.0 for f in detect_freqs}
    all_obs = obs + (oobs or [])

    def try_receive(ecx, ecy, emitter_angle, freq, blockers):
        """Check emitter→sensor path and accumulate signal if valid."""
        # Vector from emitter to sensor
        dx, dy = wx - ecx, wy - ecy
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return
        ux, uy = dx / d, dy / d

        # 1. Is the sensor within the emitter's beam? (emitter-side FOV)
        emit_dot = ux * math.cos(emitter_angle) + uy * math.sin(emitter_angle)
        if emit_dot < math.cos(EMITTER_FOV_RAD):
            return

        # 2. Is the emitter within the sensor's FOV? (sensor-side FOV)
        # Direction from sensor to emitter is opposite: (-ux, -uy)
        receive_dot = max(-1.0, min(1.0, (-ux) * math.cos(wa) + (-uy) * math.sin(wa)))
        if math.acos(receive_dot) > sensor_fov:
            return

        d_in = d / SCALE * 12.0
        if d_in > rangein:
            return

        # 3. Occlusion: nothing between sensor and emitter
        if _ray_occluded(wx, wy, ecx, ecy, d, blockers, fox, foy, ofox, ofoy):
            return

        strength = max(0.0, emit_dot) * (1.0 - d_in / rangein)
        analog[freq] = max(analog.get(freq, 0.0), strength)

    # ── Enemy robot emitter (2 kHz) ──────────────────────────────────────────
    if 2000 in analog and ep:
        enemy_emit_angle = ea + math.radians(FORWARD_DEG)
        try_receive(ep[0], ep[1], enemy_emit_angle, 2000, all_obs)

    # ── Obstacle emitters ────────────────────────────────────────────────────
    for o in all_obs:
        freq = o.get('ir_freq', 1500)
        if freq not in analog:
            continue
        ecx, ecy = _obs_world_pos(o, fox, foy, ofox, ofoy)
        emitter_angle = (math.pi - o.get('front_angle', 0.0)) if o.is_enemy else o.get('front_angle', 0.0)
        blockers = [b for b in all_obs if b is not o]
        try_receive(ecx, ecy, emitter_angle, freq, blockers)

    if mode == 'digital':
        return {'mode': 'digital', 'detect_freqs': detect_freqs,
                'digital': {f: 1 if analog.get(f, 0) >= threshold else 0 for f in detect_freqs},
                'analog': analog, 'world_pos': (wx, wy), 'world_angle': wa}
    return {'mode': 'analog', 'detect_freqs': detect_freqs,
            'analog': analog, 'world_pos': (wx, wy), 'world_angle': wa}

def sample_imu(s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs=None,ofox=None,ofoy=None):
    global _imu_prev_vx,_imu_prev_vy,_imu_yaw;p=s['params']
    gn=p.get('gyro_noise',0.002);an=p.get('accel_noise',0.5);gb=p.get('gyro_bias',0.001)
    gyro_z=_imu_omega+gb+random.gauss(0,gn);_imu_yaw+=gyro_z*dt
    ca,sa=math.cos(ra),math.sin(ra)
    dvx=(_imu_vx-_imu_prev_vx)/(dt if dt>0 else 0.001);dvy=(_imu_vy-_imu_prev_vy)/(dt if dt>0 else 0.001)
    ax=dvx*ca+dvy*sa+random.gauss(0,an);ay=-dvx*sa+dvy*ca+random.gauss(0,an)
    _imu_prev_vx=_imu_vx;_imu_prev_vy=_imu_vy
    return {'yaw_true_rad':ra,'yaw_integrated_rad':_imu_yaw,'gyro_z_rad_s':gyro_z,'accel_x_body':ax,'accel_y_body':ay}

_SAMPLERS={'tape':sample_tape,'bump':sample_bump,'ping':sample_ping,'lidar':sample_lidar,
           'trackwire':sample_trackwire,'ir':sample_ir,'imu':sample_imu}

def update_sensors(sensor_defs,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs,ofox,ofoy):
    return {s['name']:_SAMPLERS[s['type']](s,rp,ra,fox,foy,obs,ep,ea,pvx,pvy,dt,oobs,ofox,ofoy)
            for s in sensor_defs if s['type'] in _SAMPLERS}

# ── Drawing ───────────────────────────────────────────────────────────────────
def draw_dotted_line(surf,color,start,end,dash=6,gap=5,width=2):
    x1,y1=start;x2,y2=end;dx,dy=x2-x1,y2-y1;dist=math.hypot(dx,dy)
    if dist<1e-6: return
    ux,uy=dx/dist,dy/dist;t=0.0
    while t<dist:
        a=(x1+ux*t,y1+uy*t);t2=min(dist,t+dash);b=(x1+ux*t2,y1+uy*t2)
        pygame.draw.line(surf,color,(int(a[0]),int(a[1])),(int(b[0]),int(b[1])),width);t+=dash+gap

def draw_hash_zone(surf,ox,oy,x0,x1,color,is_opp=False):
    if is_opp: x0,x1=FIELD_W-x1,FIELD_W-x0
    zw=int(x1-x0)
    if zw<=0: return
    zs=pygame.Surface((zw,FIELD_H),pygame.SRCALPHA);zs.fill((*color,28))
    for i in range(-FIELD_H//10,zw//10+FIELD_H//10+2):
        pygame.draw.line(zs,(*color,90),(i*10,0),(i*10+FIELD_H,FIELD_H),2)
    surf.blit(zs,(ox+int(x0),oy))

def draw_field_unit(surf,ox,oy,obs,is_opp=False,show_zones=False):
    pygame.draw.rect(surf,WHITE,(ox,oy,FIELD_W,FIELD_H))
    if show_zones:
        draw_hash_zone(surf,ox,oy,H_TAPE_END,FIELD_W-TAPE,(200,100,50),is_opp)
        draw_hash_zone(surf,ox,oy,TAPE,ISZ_END,(50,180,80),is_opp)
        draw_hash_zone(surf,ox,oy,ISZ_END,VERT_X[0],(50,100,220),is_opp)
        draw_hash_zone(surf,ox,oy,VERT_X[0],VERT_X[1],(150,50,220),is_opp)
        draw_hash_zone(surf,ox,oy,VERT_X[1],VERT_X[2],(220,180,50),is_opp)
        draw_hash_zone(surf,ox,oy,VERT_X[2],H_TAPE_END,(200,80,150),is_opp)
    pygame.draw.rect(surf,BLACK,(ox,oy,FIELD_W,FIELD_H),TAPE)
    lx=FIELD_W-H_TAPE_END if is_opp else H_TAPE_START
    pygame.draw.rect(surf,BLACK,(ox+lx,oy+CENTER_Y-TAPE_W//2,H_TAPE_END-H_TAPE_START,TAPE_W))
    vt=oy+CENTER_Y-VERT_LEN//2
    for vx2 in VERT_X:
        fx=(FIELD_W-vx2) if is_opp else vx2
        pygame.draw.rect(surf,BLACK,(ox+fx-TAPE_W//2,vt,TAPE_W,int(VERT_LEN)))
    for o in obs:
        rx=o.x if not is_opp else FIELD_W-o.x
        pygame.draw.rect(surf,BLUE,(ox+int(rx-o.w/2),oy+int(o.y-o.h/2),int(o.w),int(o.h)))
        fa=o.get('front_angle',0.0)
        if is_opp: fa=math.pi-fa
        freq=o.get('ir_freq',1500);ecol={2000:(255,0,0),1500:(0,120,255),2500:(180,0,255)}.get(freq,(255,255,255))
        ecx=ox+int(rx);ecy=oy+int(o['y'])
        draw_dotted_line(surf,ecol,(ecx,ecy),(int(ecx+120*math.cos(fa)),int(ecy+120*math.sin(fa))))

def draw_fields(surf,mo,oo,show_zones):
    sw,sh=surf.get_size();pygame.draw.rect(surf,GRAY,(0,0,sw,sh))
    draw_field_unit(surf,*field_origin(False),mo,False,show_zones)
    draw_field_unit(surf,*field_origin(True),oo,True,show_zones)

def draw_robot(surf,pos,angle,collision,oob):
    corners=world_corners(pos,angle);col=ORANGE if oob else((255,80,80) if collision else RED)
    pygame.draw.polygon(surf,col,[(int(x),int(y)) for x,y in corners])
    cx,cy=pos
    draw_angle = angle + math.radians(FORWARD_DEG)
    ca,sa=math.cos(draw_angle),math.sin(draw_angle)
    if ROBOT_SHAPE:
        fd = ip(max(math.hypot(x, y) for x, y in ROBOT_SHAPE))
    else:
        fd = ip(5.5)
    pygame.draw.line(surf,BLACK,(int(cx),int(cy)),(int(cx+fd*ca),int(cy+fd*sa)),3)


def draw_wheels(surf, robot_pos, robot_angle):
    """Draw robot wheels from JSON mount points in the physical robot frame."""
    cx, cy = robot_pos
    cra, sra = math.cos(robot_angle), math.sin(robot_angle)
    for wh in ROBOT_WHEELS:
        lx = ip(float(wh.get('pos_x', 0)))
        ly = ip(float(wh.get('pos_y', 0)))
        wx = cx + lx*cra - ly*sra
        wy = cy + lx*sra + ly*cra
        wheel_thickness = float(wh.get('width_in', 0.75))
        wheel_diameter = float(wh.get('height_in', wh.get('radius_in', 1.5) * 2.0))
        wheel_len = max(4.0, float(ip(wheel_diameter)))
        wheel_wid = max(2.0, float(ip(wheel_thickness)))
        raw = str(wh.get('color', '#89b4fa')).lstrip('#')
        col = tuple(int(raw[i:i+2], 16) for i in (0,2,4)) if len(raw) == 6 else (137, 180, 250)
        wang = robot_angle + math.radians(float(wh.get('angle_deg', 0.0)) + 90.0)
        fwd = (math.cos(wang), math.sin(wang))
        lat = (-math.sin(wang), math.cos(wang))
        hl = wheel_len / 2.0
        hw = wheel_wid / 2.0
        pts = [
            (wx - hl*fwd[0] - hw*lat[0], wy - hl*fwd[1] - hw*lat[1]),
            (wx + hl*fwd[0] - hw*lat[0], wy + hl*fwd[1] - hw*lat[1]),
            (wx + hl*fwd[0] + hw*lat[0], wy + hl*fwd[1] + hw*lat[1]),
            (wx - hl*fwd[0] + hw*lat[0], wy - hl*fwd[1] + hw*lat[1]),
        ]
        ptsi = [(int(x), int(y)) for x,y in pts]
        pygame.draw.polygon(surf, (30,30,46), ptsi, 0)
        pygame.draw.polygon(surf, col, ptsi, 2)
        pygame.draw.line(surf, (255,255,255),
                         (int(wx - hw*lat[0]), int(wy - hw*lat[1])),
                         (int(wx + hw*lat[0]), int(wy + hw*lat[1])), 2)
        pygame.draw.circle(surf, col, (int(wx), int(wy)), 3)


def draw_sensors_overlay(surf,sensor_defs,sim_sensors,robot_pos,robot_angle):
    for s in sensor_defs:
        reading=sim_sensors.get(s['name'],{});col=s.get('color',(180,180,180));stype=s['type']
        wx,wy,wa=sensor_world_pose(robot_pos,robot_angle,s)
        if stype=='tape':
            pygame.draw.circle(surf,col,(int(wx),int(wy)),5)
            if reading.get('on_tape'): pygame.draw.circle(surf,WHITE,(int(wx),int(wy)),2)
            pygame.draw.circle(surf,BLACK,(int(wx),int(wy)),5,1)
        elif stype=='bump':
            shape = reading.get('shape', 'circle')
            active_col = WHITE if reading.get('pressed') else col
            if shape=='rect':
                rw2=reading.get('rect_w_px',ip(2.0))/2; rh2=reading.get('rect_h_px',ip(0.5))/2
                bang=reading.get('angle',wa); ca2,sa2=math.cos(bang),math.sin(bang)
                corners=[(rh2,-rw2),(rh2,rw2),(-rh2,rw2),(-rh2,-rw2)]
                pts2=[(int(wx+lx*ca2-ly*sa2),int(wy+lx*sa2+ly*ca2)) for lx,ly in corners]
                pygame.draw.polygon(surf, active_col, pts2, 0)
                pygame.draw.polygon(surf, WHITE if reading.get('pressed') else BLACK, pts2, 2)
            elif shape=='arc':
                rr = reading.get('arc_radius_px', ip(2.0))
                th = max(1.0, reading.get('arc_thickness_px', ip(0.35)))
                start_deg = reading.get('arc_start_deg', -35)
                end_deg = reading.get('arc_end_deg', 35)
                if end_deg < start_deg: start_deg, end_deg = end_deg, start_deg
                span = max(1.0, end_deg - start_deg)
                ro = rr + th/2
                ri = max(1.0, rr - th/2)
                bang = reading.get('angle', wa)
                steps = max(18, int(span / 3))
                outer_pts=[]; inner_pts=[]
                for j in range(steps+1):
                    a = bang + math.radians(start_deg + span*j/steps)
                    outer_pts.append((int(wx + ro*math.cos(a)), int(wy + ro*math.sin(a))))
                for j in range(steps, -1, -1):
                    a = bang + math.radians(start_deg + span*j/steps)
                    inner_pts.append((int(wx + ri*math.cos(a)), int(wy + ri*math.sin(a))))
                pts = outer_pts + inner_pts
                if len(pts) >= 3:
                    pygame.draw.polygon(surf, active_col, pts, 0)
                    pygame.draw.polygon(surf, WHITE if reading.get('pressed') else BLACK, pts, 2)
            else:
                r=int(reading.get('radius_px',6))
                pygame.draw.circle(surf,active_col,(int(wx),int(wy)),max(4,r),0)
                pygame.draw.circle(surf,WHITE if reading.get('pressed') else BLACK,(int(wx),int(wy)),max(4,r),2)
        elif stype=='ping':
            for(ra2,rdx,rdy,hit_d,hit_type) in reading.get('ray_results',[]):
                ex=int(wx+rdx*hit_d);ey=int(wy+rdy*hit_d)
                pygame.draw.line(surf,col,(int(wx),int(wy)),(ex,ey),1)
                if hit_type: pygame.draw.circle(surf,YELLOW,(ex,ey),5)
            pygame.draw.circle(surf,col,(int(wx),int(wy)),4)
        elif stype=='lidar':
            beam_col=(255,0,0)
            for(ra2,rdx,rdy,hit_d,hit_type) in reading.get('ray_results',[]):
                ex=int(wx+rdx*hit_d);ey=int(wy+rdy*hit_d)
                pygame.draw.line(surf,beam_col,(int(wx),int(wy)),(ex,ey),2)
                if hit_type: pygame.draw.circle(surf,beam_col,(ex,ey),5)
            pygame.draw.circle(surf,beam_col,(int(wx),int(wy)),4)
        elif stype=='trackwire':
            st=reading.get('strength',0.0);r=5+int(st*8)
            pygame.draw.circle(surf,col,(int(wx),int(wy)),r,2);pygame.draw.circle(surf,col,(int(wx),int(wy)),3)
        elif stype=='ir':
            mode=reading.get('mode','analog');amps=reading.get('analog',{})
            params=s['params'];draw_scale=ip(params.get('draw_scale',28));fov_half=math.radians(params.get('fov_deg',40))/2
            total_amp=sum(amps.values());beam_len=max(ip(6),min(draw_scale,draw_scale*total_amp)) if mode!='digital' else(draw_scale if any(reading.get('digital',{}).values()) else ip(6))
            lp=(int(wx+beam_len*math.cos(wa-fov_half)),int(wy+beam_len*math.sin(wa-fov_half)))
            rp2=(int(wx+beam_len*math.cos(wa+fov_half)),int(wy+beam_len*math.sin(wa+fov_half)))
            mp=(int(wx+beam_len*math.cos(wa)),int(wy+beam_len*math.sin(wa)))
            pygame.draw.circle(surf,col,(int(wx),int(wy)),5)
            pygame.draw.line(surf,col,(int(wx),int(wy)),lp,1);pygame.draw.line(surf,col,(int(wx),int(wy)),rp2,1)
            pygame.draw.line(surf,col,(int(wx),int(wy)),mp,2)
            if total_amp>0.01: pygame.draw.circle(surf,WHITE,(int(wx),int(wy)),2)
        elif stype=='imu':
            ya=reading.get('yaw_integrated_rad',robot_angle)
            pygame.draw.line(surf,col,(int(wx),int(wy)),(int(wx+ip(3)*math.cos(ya)),int(wy+ip(3)*math.sin(ya))),2)
            pygame.draw.circle(surf,col,(int(wx),int(wy)),4)

def draw_sensor_panel(surf,sensor_defs,sim_sensors,px,start_y):
    y=start_y;f15=pygame.font.SysFont(None,15);line_h=16;x0=px+8;name_x=x0+18;val_x=px+170;row_w=380
    surf.blit(pygame.font.SysFont(None,16).render('Sensors',True,DARK),(x0,y));y+=18
    for sdef in sensor_defs:
        reading=sim_sensors.get(sdef['name'],{});col=sdef.get('color',BLACK);stype=sdef['type']
        row_rect=pygame.Rect(x0-2,y-1,row_w,line_h)
        pygame.draw.rect(surf,(248,248,250),row_rect);pygame.draw.rect(surf,(225,225,230),row_rect,1)
        pygame.draw.rect(surf,col,(x0+1,y+3,10,10));surf.blit(f15.render(sdef['name'],True,BLACK),(name_x,y+1))
        if stype=='tape': vt='ON TAPE' if reading.get('on_tape') else 'off';vc=col if reading.get('on_tape') else GRAY
        elif stype=='bump': hit=reading.get('pressed');tgt=reading.get('hit') or '';vt=(f'HIT {tgt}').strip() if hit else '---';vc=col if hit else GRAY
        elif stype=='ping': vt=f"{reading.get('distance_in',0):.1f} in [{reading.get('hit') or 'none'}]";vc=col
        elif stype=='lidar': vt=f"{reading.get('distance_m',0):.2f} m [{reading.get('hit') or 'none'}]";vc=col
        elif stype=='trackwire': vt=f"{reading.get('strength',0):.2f} ({reading.get('nearest_dist_in',0):.1f} in)";vc=col
        elif stype=='ir':
            mode=reading.get('mode','analog')
            if mode=='digital': vt=' '.join(f'{k}:{v}' for k,v in sorted(reading.get('digital',{}).items()))
            else: vt=' '.join(f'{k}:{v:.2f}' for k,v in sorted(reading.get('analog',{}).items()))
            vc=col
        elif stype=='imu': vt=f"yaw {math.degrees(reading.get('yaw_true_rad',0)):.1f}deg w{reading.get('gyro_z_rad_s',0):.3f}";vc=col
        else: vt='';vc=DARK
        surf.blit(f15.render(vt,True,BLACK),(val_x,y+1));y+=line_h
    return y

def draw_sensor_minimap(surf,sensor_defs,sensor_values,x0,y0):
    scale=10
    pcx=x0+100
    pcy=y0+90
    box=pygame.Rect(x0,y0,210,180)
    pygame.draw.rect(surf,(248,248,250),box)
    pygame.draw.rect(surf,GRAY,box,1)
    surf.blit(pygame.font.SysFont(None,16).render('Robot sensor map',True,DARK),(x0+8,y0+6))
    pts=[(int(pcx+sx*scale),int(pcy-sy*scale)) for sx,sy in ROBOT_SHAPE]
    if len(pts)>=3:
        pygame.draw.polygon(surf,(255,220,220),pts)
        pygame.draw.polygon(surf,RED,pts,2)
    pygame.draw.line(surf,BLACK,(pcx,pcy),(int(pcx+5*scale),pcy),2)
    for sdef in sensor_defs:
        lx,ly=sdef['pos']
        sx2=int(pcx+lx*scale)
        sy2=int(pcy-ly*scale)
        basecol=sdef.get('color',(180,180,180))
        if isinstance(basecol,str):
            try:
                basecol=pygame.Color(basecol)
                basecol=(basecol.r,basecol.g,basecol.b)
            except:
                basecol=(180,180,180)
        rd=sensor_values.get(sdef['name'],{})
        st=sdef['type']
        active=False
        if st=='tape': active=rd.get('on_tape',False)
        elif st=='bump': active=rd.get('pressed',False)
        elif st=='ping': active=rd.get('hit') is not None
        elif st=='lidar': active=rd.get('hit') is not None
        elif st=='trackwire': active=rd.get('strength',0)>0.05
        elif st=='ir': active=sum(rd.get('analog',{}).values())>0.05 or any(rd.get('digital',{}).values())
        elif st=='imu': active=True
        drawcol=tuple(min(255,c+40) for c in basecol) if active else basecol

        if st=='ping':
            params=sdef.get('params',{})
            fov=math.radians(params.get('fov_deg',20))
            nrays=max(1,int(params.get('nrays',7)))
            ang0=math.radians(sdef.get('angle_deg',0.0))
            if nrays>1:
                for i in range(nrays):
                    a=ang0 + fov*(i/(nrays-1)-0.5)
                    ex=int(sx2 + 26*math.cos(a))
                    ey=int(sy2 - 26*math.sin(a))
                    pygame.draw.line(surf,drawcol,(sx2,sy2),(ex,ey),1)
                    pygame.draw.circle(surf,drawcol,(ex,ey),2)
            else:
                ex=int(sx2 + 26*math.cos(ang0))
                ey=int(sy2 - 26*math.sin(ang0))
                pygame.draw.line(surf,drawcol,(sx2,sy2),(ex,ey),1)
                pygame.draw.circle(surf,drawcol,(ex,ey),2)
        elif st=='lidar':
            ang0=math.radians(sdef.get('angle_deg',0.0))
            lidar_col=(255,0,0)
            ex=int(sx2 + 26*math.cos(ang0))
            ey=int(sy2 - 26*math.sin(ang0))
            pygame.draw.line(surf,lidar_col,(sx2,sy2),(ex,ey),2)
            pygame.draw.circle(surf,lidar_col,(ex,ey),2)

        pygame.draw.circle(surf,drawcol,(sx2,sy2),5)
        pygame.draw.circle(surf,BLACK,(sx2,sy2),5,1)

def draw_panel(surf,active_tab,show_zones,sensor_defs=None,sim_sensors=None,robot_name=""):
    px=0;py=H;pygame.draw.rect(surf,PANEL_BG,(px,py,SCREEN_W,PANEL_H))
    pygame.draw.line(surf,GRAY,(0,py),(SCREEN_W,py),2)
    tab_h=28;f16=pygame.font.SysFont(None,18)
    tabs=[(TAB_SENSORS,'Sensors',pygame.Rect(px+8,py+6,100,tab_h)),
          (TAB_UNO,'UNO Pins',pygame.Rect(px+112,py+6,100,tab_h)),
          (TAB_ZONES,'Zones',pygame.Rect(px+216,py+6,100,tab_h))]
    for key,label,rect in tabs:
        active=(key==active_tab)
        pygame.draw.rect(surf,WHITE if active else(220,220,226),rect)
        pygame.draw.rect(surf,DARK if active else GRAY,rect,2 if active else 1)
        txt=f16.render(label,True,BLACK);surf.blit(txt,(rect.x+(rect.w-txt.get_width())//2,rect.y+6))
    surf.blit(pygame.font.SysFont(None,18).render(f"Robot: {robot_name}",True,DARK),(px+330,py+12))
    y=py+42
    if active_tab==TAB_SENSORS and sensor_defs and sim_sensors:
        draw_sensor_panel(surf,sensor_defs,sim_sensors,px,y)
        draw_sensor_minimap(surf,sensor_defs,sim_sensors,surf.get_width()-220,py+36)
    elif active_tab==TAB_ZONES:
        f18=pygame.font.SysFont(None,20);f15=pygame.font.SysFont(None,15)
        surf.blit(f18.render('Zones',True,DARK),(px+8,y));y+=28
        for label2,col in [("Start (84-96in)",(200,100,50)),("ISZ (0-12in)",(50,180,80)),
                            ("Zone 1 (12-26in)",(50,100,220)),("Zone 2 (26-48in)",(150,50,220)),
                            ("Zone 3 (48-70in)",(220,180,50)),("Zone 4 (70-84in)",(200,80,150))]:
            pygame.draw.rect(surf,col,(px+8,y+2,12,12));surf.blit(f15.render(label2,True,BLACK),(px+28,y));y+=20

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global _imu_omega,_imu_vx,_imu_vy,_imu_yaw

    parser=argparse.ArgumentParser(description="Sluggers of the Lost Goal – ver4 (pygame)")
    parser.add_argument('--robot','-r',metavar='JSON',default=None,help="Robot JSON config to load")
    parser.add_argument('--seed',type=int,default=None,help="Seed for repeatable obstacles/spawns")
    parser.add_argument('--controller',default=None,help="Python controller file exporting update(sensors, dt)")
    parser.add_argument('--controller-format',choices=['compat','native'],default='compat',help="compat=sluggers_sim.py sensors (works with ES adapter)")
    parser.add_argument('--log',default=None,help="Write JSONL log of sensors/controls")
    parser.add_argument('--fps',type=int,default=FPS,help="Render FPS cap (physics uses fixed step)")
    parser.add_argument('--sim-hz',type=float,default=60.0,help="Physics tick rate")
    args=parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        try:
            np.random.seed(args.seed)
        except Exception:
            pass

    controller_update = None
    if args.controller:
        controller_update = load_controller(args.controller)

    loaded_path=None
    if args.robot:
        # Allow override with command-line argument
        try: load_robot_json(args.robot); loaded_path=args.robot
        except Exception as e: print(f"[ver4] WARNING: {e}\nFalling back to hardcoded robot.")
    
    if not loaded_path:
        # Load hardcoded robot configuration
        global SENSOR_DEFS,ROBOT_SHAPE,ROBOT_INERTIA,ROBOT_NAME,OMNI_WHEELS,ROBOT_SPEED,TURN_SPEED,FORWARD_DEG
        try:
            robot_data=json.loads(_HARDCODED_ROBOT_JSON)
            _load_robot_data(robot_data)
            loaded_path="[hardcoded]"
        except Exception as e:
            print(f"[ver4] FATAL: Failed to parse hardcoded robot: {e}")
            sys.exit(1)

    pygame.init()
    show_panel=True;active_tab=TAB_SENSORS
    screen=pygame.display.set_mode((SCREEN_W,SCREEN_H),pygame.RESIZABLE)
    pygame.display.set_caption(f"Sluggers v4 - {ROBOT_NAME}")
    clock=pygame.time.Clock();font=pygame.font.SysFont(None,21);small=pygame.font.SysFont(None,17)

    ox,oy=field_origin(False)
    oox,ooy=field_origin(True)

    # Fixed-step simulation (decoupled from render FPS)
    sim_dt = 1.0 / max(1.0, float(args.sim_hz))
    elapsed_s = 0.0
    accumulator = 0.0

    # World state
    pos=[0.0,0.0]
    angle=0.0
    vx=vy=omega=0.0
    main_obs=[]
    opp_obs=[]
    enemy_pos=[0.0,0.0]
    enemy_angle=0.0
    sim_sensors={}
    prev_vx=prev_vy=0.0
    show_zones=False

    def reset_world():
        nonlocal elapsed_s, accumulator
        nonlocal pos, angle, vx, vy, omega, main_obs, opp_obs, enemy_pos, enemy_angle
        elapsed_s = 0.0
        accumulator = 0.0
        start_x=float(ox+H_TAPE_END+(FIELD_W-TAPE-H_TAPE_END)/2)
        start_y=float(oy+FIELD_H/2.0)
        pos=[start_x,start_y]
        angle=random.uniform(-math.pi, math.pi)
        vx=vy=omega=0.0
        main_obs=create_obstacles(0)
        opp_obs=create_obstacles(1)
        for o in main_obs:
            o['ir_freq']=random.choice([1500,2500])
        for o in opp_obs:
            o['ir_freq']=random.choice([1500,2500])
        # Spawn the enemy in *your field coordinates*; draw it mirrored on the opponent field.
        ex = random.uniform(ox + ip(8), ox + FIELD_W - ip(8))
        ey = random.uniform(oy + ip(8), oy + FIELD_H - ip(8))
        ea = random.uniform(-math.pi, math.pi)
        enemy_pos=[ex, ey]
        enemy_angle=ea
        return pos, angle, vx, vy, omega, main_obs, opp_obs, enemy_pos, enemy_angle

    reset_world()

    log_f = None
    if args.log:
        log_path = Path(args.log).expanduser().resolve()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_f = log_path.open('w', encoding='utf-8')

    def reload_dialog():
        try:
            import tkinter as tk;from tkinter import filedialog
            root=tk.Tk();root.withdraw()
            path=filedialog.askopenfilename(filetypes=[("Robot JSON","*.json"),("All","*.*")],title="Load Robot JSON")
            root.destroy()
            if path:
                load_robot_json(path)
                pygame.display.set_caption(f"Sluggers v4 - {ROBOT_NAME}")
        except Exception as e:
            print(f"[ver4] WARNING: load dialog failed: {e}")

    def try_load_drop(path: str) -> None:
        if not path:
            return
        p = Path(path)
        if p.suffix.lower() != '.json':
            return
        try:
            load_robot_json(str(p))
            pygame.display.set_caption(f"Sluggers v4 - {ROBOT_NAME}")
        except Exception as e:
            print(f"[ver4] WARNING: failed to load {p}: {e}")

    def parse_controls(raw: Any) -> dict[str, float]:
        """Return omni-style controls dict: vx, vy, omega (robot frame, inches/s and rad/s)."""
        if raw is None:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        if isinstance(raw, dict):
            if any(k in raw for k in ('vx','vy','omega')):
                return {
                    "vx": float(raw.get('vx', 0.0)),
                    "vy": float(raw.get('vy', 0.0)),
                    "omega": float(raw.get('omega', 0.0)),
                }
            if 'left' in raw and 'right' in raw:
                left = float(raw.get('left', 0.0))
                right = float(raw.get('right', 0.0))
                # Convert tank wheel speeds (in/s) -> forward/omega (robot frame)
                WHEEL_BASE_IN = 8.5
                v = (left + right) / 2.0
                om = (right - left) / WHEEL_BASE_IN
                return {"vx": v, "vy": 0.0, "omega": om}
            if all(k in raw for k in ("front_left","front_right","rear_left","rear_right")):
                fl=float(raw["front_left"]); fr=float(raw["front_right"]); rl=float(raw["rear_left"]); rr=float(raw["rear_right"])
                vx=(fl+fr+rl+rr)/4.0
                vy=(-fl+fr+rl-rr)/4.0
                omega=(-fl+fr-rl+rr)/(2.0*(11.0+10.0))
                return {"vx": vx, "vy": vy, "omega": omega}
        if isinstance(raw, (tuple, list)) and len(raw) >= 2:
            return {"vx": float(raw[0]), "vy": float(raw[1]), "omega": float(raw[2]) if len(raw) > 2 else 0.0}
        raise RuntimeError("Controller must return dict/tuple/list or None")

    def manual_controls() -> dict[str, float]:
        keys = pygame.key.get_pressed()
        base = float(ROBOT_SPEED)
        turn = float(TURN_SPEED)
        # Hold shift for a bit more oomph.
        if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
            base *= 1.6
            turn *= 1.6
        forward = 0.0
        left = 0.0
        om = 0.0
        if keys[pygame.K_w]:
            forward += base
        if keys[pygame.K_s]:
            forward -= base
        if OMNI_WHEELS:
            if keys[pygame.K_a]:
                left += base
            if keys[pygame.K_d]:
                left -= base
            if keys[pygame.K_q]:
                om -= turn
            if keys[pygame.K_e]:
                om += turn
        else:
            if keys[pygame.K_a]:
                om -= turn
            if keys[pygame.K_d]:
                om += turn
        return {"vx": forward, "vy": left, "omega": om}

    def draw_enemy_on_opp_field() -> None:
        # Draw enemy pose mirrored on the opponent field.
        lx = enemy_pos[0] - ox
        ly = enemy_pos[1] - oy
        ex = oox + (FIELD_W - lx)
        ey = ooy + ly
        ea = math.pi - enemy_angle
        corners = world_corners((ex, ey), ea)
        pygame.draw.polygon(screen, GREEN, [(int(x), int(y)) for x, y in corners])
        pygame.draw.polygon(screen, BLACK, [(int(x), int(y)) for x, y in corners], 2)

    running = True
    while running:
        frame_dt = clock.tick(int(args.fps)) / 1000.0
        frame_dt = min(frame_dt, 0.08)
        accumulator += frame_dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reset_world()
                elif event.key == pygame.K_p:
                    show_panel = not show_panel
                elif event.key == pygame.K_z:
                    show_zones = not show_zones
                elif event.key == pygame.K_l:
                    reload_dialog()
                elif event.key == pygame.K_s:
                    active_tab = TAB_SENSORS
                elif event.key == pygame.K_1:
                    active_tab = TAB_SENSORS
                elif event.key == pygame.K_2:
                    active_tab = TAB_UNO
                elif event.key == pygame.K_3:
                    active_tab = TAB_ZONES
            elif hasattr(pygame, 'DROPFILE') and event.type == pygame.DROPFILE:
                try_load_drop(getattr(event, 'file', ''))
            elif event.type == pygame.MOUSEBUTTONDOWN and show_panel:
                mx, my = event.pos
                # Tab click handling
                py = H
                tab_h = 28
                tabs = [
                    (TAB_SENSORS, pygame.Rect(8, py + 6, 100, tab_h)),
                    (TAB_UNO, pygame.Rect(112, py + 6, 100, tab_h)),
                    (TAB_ZONES, pygame.Rect(216, py + 6, 100, tab_h)),
                ]
                for key, rect in tabs:
                    if rect.collidepoint(mx, my):
                        active_tab = key
                        break

        # Fixed-step physics/controller update
        while accumulator >= sim_dt:
            # Controller input
            if controller_update is not None:
                if args.controller_format == 'compat':
                    ctrl_sensors = build_compat_sensors(
                        time_s=elapsed_s,
                        dt=sim_dt,
                        robot_pos_px=pos,
                        robot_angle=angle,
                        field_ox=ox,
                        field_oy=oy,
                        main_obs=main_obs,
                        enemy_pos_px=enemy_pos,
                    )
                else:
                    # Native ver4 sensors (per-config sensor array)
                    ctrl_sensors = sim_sensors
                try:
                    controls = parse_controls(controller_update(ctrl_sensors, sim_dt))
                except Exception as e:
                    print(f"[ver4] Controller error: {e}")
                    controls = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
            else:
                controls = manual_controls()

            # Clamp controller output
            controls['vx'] = clamp(float(controls.get('vx', 0.0)), -200.0, 200.0)
            controls['vy'] = clamp(float(controls.get('vy', 0.0)), -200.0, 200.0)
            controls['omega'] = clamp(float(controls.get('omega', 0.0)), -8.0, 8.0)

            # Convert robot-frame inches/s to world pixel/s
            ca = math.cos(angle)
            sa = math.sin(angle)
            vx_world_in = controls['vx'] * ca + controls['vy'] * sa
            vy_world_in = controls['vx'] * sa - controls['vy'] * ca
            vx = vx_world_in * SCALE / 12.0
            vy = vy_world_in * SCALE / 12.0
            omega = controls['omega']

            # Physics obstacles must be in world pixels
            phys_obs = [
                {'x': ox + o.x, 'y': oy + o.y, 'w': o.w, 'h': o.h}
                for o in main_obs
            ]
            pos, angle, vx, vy, omega = step_robot(pos, angle, vx, vy, omega, phys_obs, sim_dt)

            # Update IMU state variables used by sample_imu
            _imu_omega = omega
            _imu_vx = vx
            _imu_vy = vy

            # Update sensors for drawing/debug (uses local-field obstacle coords)
            sim_sensors = update_sensors(SENSOR_DEFS, pos, angle, ox, oy, main_obs, enemy_pos, enemy_angle, prev_vx, prev_vy, sim_dt, opp_obs, oox, ooy)
            prev_vx, prev_vy = vx, vy

            # Optional log
            if log_f is not None:
                try:
                    row = {
                        'time_s': elapsed_s,
                        'dt_s': sim_dt,
                        'pose_px': {'x': pos[0], 'y': pos[1], 'theta': angle},
                        'controls': controls,
                        'sensors_compat': build_compat_sensors(
                            time_s=elapsed_s,
                            dt=sim_dt,
                            robot_pos_px=pos,
                            robot_angle=angle,
                            field_ox=ox,
                            field_oy=oy,
                            main_obs=main_obs,
                            enemy_pos_px=enemy_pos,
                        ),
                    }
                    log_f.write(json.dumps(row) + "\n")
                except Exception:
                    pass

            elapsed_s += sim_dt
            accumulator -= sim_dt

        # Draw
        draw_fields(screen, main_obs, opp_obs, show_zones)
        collision = any_col(pos, angle, [{'x': ox + o.x, 'y': oy + o.y, 'w': o.w, 'h': o.h} for o in main_obs])
        oob = is_oob(pos, angle, ox, oy)
        draw_robot(screen, pos, angle, collision, oob)
        draw_wheels(screen, pos, angle)
        draw_enemy_on_opp_field()
        draw_sensors_overlay(screen, SENSOR_DEFS, sim_sensors, pos, angle)

        if show_panel:
            draw_panel(screen, active_tab, show_zones, SENSOR_DEFS, sim_sensors, ROBOT_NAME)

        # HUD
        hud = f"seed={args.seed}  ctrl={'on' if controller_update else 'manual'}  fps={clock.get_fps():.1f}  oob={int(oob)}  col={int(collision)}"
        screen.blit(small.render(hud, True, BLACK), (8, 8))
        pygame.display.flip()

    if log_f is not None:
        try:
            log_f.close()
        except Exception:
            pass
    pygame.quit()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())