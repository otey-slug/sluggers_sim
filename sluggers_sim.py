"""
Sluggers of the Lost Goal simulator.

Run:
    python sluggers_sim.py
    python sluggers_sim.py --seed 118 --controller my_controller.py

Controller API:
    Put this in a separate .py file and pass it with --controller:

        def update(sensors, dt):
            return {"left": 8.0, "right": 8.0, "shoot": False}

Coordinates are inches. The robot starts on its own 48 in x 96 in field.
This simulator is intentionally approximate, but it models the project rules
that matter for state control testing: tape, bumps, obstacle track wire,
IR beacon direction/range, legal shooting zones, ammo, and scoring.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
import sys
import time
import tkinter as tk
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable


FIELD_L = 96.0
FIELD_W = 48.0
TAPE_W = 2.0
ROBOT_L = 11.0
ROBOT_W = 10.0
WHEEL_BASE = 8.5
MAX_WHEEL_SPEED = 18.0
OBSTACLE_W = 10.0
OBSTACLE_D = 3.0
BOUNDARY_H = 36.0
BOUNDARY_SIDE_GAP = 6.0
BOUNDARY_LEFT = BOUNDARY_SIDE_GAP
BOUNDARY_RIGHT = FIELD_L - BOUNDARY_SIDE_GAP
BOUNDARY_TOP = (FIELD_W - BOUNDARY_H) / 2.0
BOUNDARY_BOTTOM = BOUNDARY_TOP + BOUNDARY_H
OUTER_LEFT = BOUNDARY_LEFT - TAPE_W / 2.0
OUTER_RIGHT = BOUNDARY_RIGHT + TAPE_W / 2.0
OUTER_TOP = BOUNDARY_TOP - TAPE_W / 2.0
OUTER_BOTTOM = BOUNDARY_BOTTOM + TAPE_W / 2.0
ISZ_WIDTH = 12.0
CENTER_TAPE_END_GAP_FROM_SPAWN = 12.0
CENTER_TAPE_END_X = BOUNDARY_RIGHT - CENTER_TAPE_END_GAP_FROM_SPAWN
OFFSHOOT_SPACING = 14.0
OFFSHOOT_LEN = 14.0
OFFSHOOT_XS = tuple(CENTER_TAPE_END_X - OFFSHOOT_SPACING * i for i in range(1, 4))
OBSTACLE_OFFSET_FROM_OFFSHOOT = 6.0
OBSTACLE_LINES = tuple(x + OBSTACLE_OFFSET_FROM_OFFSHOOT for x in OFFSHOOT_XS)
ZONE_LINES = OFFSHOOT_XS
ISZ_X = BOUNDARY_LEFT + ISZ_WIDTH / 2.0
START_X = BOUNDARY_RIGHT - 6.5
START_Y = FIELD_W / 2.0
TARGET_BEACON_FREQ = "2k"
SIM_DT = 1.0 / 40.0


@dataclass
class Pose:
    x: float
    y: float
    theta: float


@dataclass
class Obstacle:
    x: float
    y: float
    freq: str

    @property
    def rect(self) -> tuple[float, float, float, float]:
        return (
            self.x - OBSTACLE_D / 2.0,
            self.y - OBSTACLE_W / 2.0,
            self.x + OBSTACLE_D / 2.0,
            self.y + OBSTACLE_W / 2.0,
        )


@dataclass
class Target:
    x: float
    y: float
    theta: float


@dataclass
class Controls:
    left: float = 0.0
    right: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    shoot: bool = False
    drive_mode: str = "tank"


@dataclass
class GameState:
    robot: Pose
    target: Target
    obstacles: list[Obstacle]
    ammo: int = 6
    score: int = 0
    hits: int = 0
    goal: bool = False
    disqualified: bool = False
    disq_reason: str = ""
    elapsed: float = 0.0
    legal_zone: int = 0
    isz_reached: bool = False
    active_pause_obstacle: int | None = None
    pause_time: float = 0.0
    collision_time: float = 0.0
    shot_cooldown: float = 0.0
    last_shot: str = "none"
    shots: list[dict[str, Any]] = field(default_factory=list)
    log: list[dict[str, Any]] = field(default_factory=list)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rotate(x: float, y: float, theta: float) -> tuple[float, float]:
    c = math.cos(theta)
    s = math.sin(theta)
    return x * c - y * s, x * s + y * c


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rect_contains(rect: tuple[float, float, float, float], x: float, y: float) -> bool:
    x1, y1, x2, y2 = rect
    return x1 <= x <= x2 and y1 <= y <= y2


def expanded_rect(rect: tuple[float, float, float, float], amount: float) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = rect
    return x1 - amount, y1 - amount, x2 + amount, y2 + amount


def polygon_area(points: list[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    area = 0.0
    for i, (x1, y1) in enumerate(points):
        x2, y2 = points[(i + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def clip_polygon_to_rect(
    points: list[tuple[float, float]],
    rect: tuple[float, float, float, float],
) -> list[tuple[float, float]]:
    def clip_edge(poly: list[tuple[float, float]], inside: Callable[[tuple[float, float]], bool], intersect: Callable[[tuple[float, float], tuple[float, float]], tuple[float, float]]) -> list[tuple[float, float]]:
        if not poly:
            return []
        out = []
        prev = poly[-1]
        prev_inside = inside(prev)
        for cur in poly:
            cur_inside = inside(cur)
            if cur_inside:
                if not prev_inside:
                    out.append(intersect(prev, cur))
                out.append(cur)
            elif prev_inside:
                out.append(intersect(prev, cur))
            prev = cur
            prev_inside = cur_inside
        return out

    x_min, y_min, x_max, y_max = rect
    clipped = points
    clipped = clip_edge(clipped, lambda p: p[0] >= x_min, lambda a, b: (x_min, a[1] + (b[1] - a[1]) * (x_min - a[0]) / (b[0] - a[0] or 1e-9)))
    clipped = clip_edge(clipped, lambda p: p[0] <= x_max, lambda a, b: (x_max, a[1] + (b[1] - a[1]) * (x_max - a[0]) / (b[0] - a[0] or 1e-9)))
    clipped = clip_edge(clipped, lambda p: p[1] >= y_min, lambda a, b: (a[0] + (b[0] - a[0]) * (y_min - a[1]) / (b[1] - a[1] or 1e-9), y_min))
    clipped = clip_edge(clipped, lambda p: p[1] <= y_max, lambda a, b: (a[0] + (b[0] - a[0]) * (y_max - a[1]) / (b[1] - a[1] or 1e-9), y_max))
    return clipped


def ray_rect_distance(
    origin: tuple[float, float],
    angle: float,
    rect: tuple[float, float, float, float],
    max_dist: float,
) -> float | None:
    ox, oy = origin
    dx = math.cos(angle)
    dy = math.sin(angle)
    x1, y1, x2, y2 = rect
    tmin = -float("inf")
    tmax = float("inf")

    if abs(dx) < 1e-9:
        if ox < x1 or ox > x2:
            return None
    else:
        tx1 = (x1 - ox) / dx
        tx2 = (x2 - ox) / dx
        tmin = max(tmin, min(tx1, tx2))
        tmax = min(tmax, max(tx1, tx2))

    if abs(dy) < 1e-9:
        if oy < y1 or oy > y2:
            return None
    else:
        ty1 = (y1 - oy) / dy
        ty2 = (y2 - oy) / dy
        tmin = max(tmin, min(ty1, ty2))
        tmax = min(tmax, max(ty1, ty2))

    if tmax < 0 or tmin > tmax:
        return None
    hit = tmin if tmin >= 0 else tmax
    if 0 <= hit <= max_dist:
        return hit
    return None


def load_controller(path: str | None) -> Callable[[dict[str, Any], float], Any] | None:
    if not path:
        return None
    controller_path = Path(path)
    spec = importlib.util.spec_from_file_location("sluggers_user_controller", controller_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load controller: {controller_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "update"):
        raise RuntimeError(f"{controller_path} must define update(sensors, dt)")
    return module.update


class SluggersSim:
    def __init__(self, seed: int | None = None, controller_path: str | None = None, log_path: str | None = None):
        self.seed = seed
        self.rng = random.Random(seed)
        self.controller = load_controller(controller_path)
        self.log_path = Path(log_path) if log_path else None
        self.use_demo_controller = self.controller is None
        self.manual_left = 0.0
        self.manual_right = 0.0
        self.manual_shoot = False
        self.paused = False
        self.scale = 5.0
        self.margin = 24.0
        self.panel_w = 360.0
        self.last_controls = Controls()
        self.reset()

    def reset(self) -> None:
        robot = Pose(
            START_X + self.rng.uniform(-2.0, 2.0),
            START_Y + self.rng.uniform(-4.0, 4.0),
            self.rng.uniform(-math.pi, math.pi),
        )
        target = Target(
            START_X + self.rng.uniform(-3.0, 3.0),
            START_Y + self.rng.uniform(-5.0, 5.0),
            self.rng.uniform(-math.pi, math.pi),
        )
        obstacles = []
        for x in OBSTACLE_LINES:
            obstacles.append(
                Obstacle(
                    x=x,
                    y=self.rng.uniform(BOUNDARY_TOP + OBSTACLE_W / 2.0, BOUNDARY_BOTTOM - OBSTACLE_W / 2.0),
                    freq=self.rng.choice(("1.5k", "2.5k")),
                )
            )
        self.state = GameState(robot=robot, target=target, obstacles=obstacles)
        self.last_sensors: dict[str, Any] = {}
        self.last_controls = Controls()

    def tape_rects(self) -> list[tuple[float, float, float, float]]:
        rects = [
            (BOUNDARY_LEFT, BOUNDARY_TOP - TAPE_W / 2.0, BOUNDARY_RIGHT, BOUNDARY_TOP + TAPE_W / 2.0),
            (BOUNDARY_LEFT, BOUNDARY_BOTTOM - TAPE_W / 2.0, BOUNDARY_RIGHT, BOUNDARY_BOTTOM + TAPE_W / 2.0),
            (BOUNDARY_LEFT - TAPE_W / 2.0, BOUNDARY_TOP, BOUNDARY_LEFT + TAPE_W / 2.0, BOUNDARY_BOTTOM),
            (BOUNDARY_RIGHT - TAPE_W / 2.0, BOUNDARY_TOP, BOUNDARY_RIGHT + TAPE_W / 2.0, BOUNDARY_BOTTOM),
            (BOUNDARY_LEFT + ISZ_WIDTH - TAPE_W / 2.0, BOUNDARY_TOP, BOUNDARY_LEFT + ISZ_WIDTH + TAPE_W / 2.0, BOUNDARY_BOTTOM),
            (BOUNDARY_LEFT + ISZ_WIDTH, START_Y - TAPE_W / 2.0, CENTER_TAPE_END_X, START_Y + TAPE_W / 2.0),
        ]
        for x in OFFSHOOT_XS:
            rects.append(
                (
                    x - TAPE_W / 2.0,
                    START_Y - OFFSHOOT_LEN / 2.0,
                    x + TAPE_W / 2.0,
                    START_Y + OFFSHOOT_LEN / 2.0,
                )
            )
        return rects

    def robot_corners(self) -> list[tuple[float, float]]:
        p = self.state.robot
        corners = []
        for lx, ly in ((ROBOT_L / 2, ROBOT_W / 2), (ROBOT_L / 2, -ROBOT_W / 2), (-ROBOT_L / 2, -ROBOT_W / 2), (-ROBOT_L / 2, ROBOT_W / 2)):
            rx, ry = rotate(lx, ly, p.theta)
            corners.append((p.x + rx, p.y + ry))
        return corners

    def sensor_point(self, local_x: float, local_y: float) -> tuple[float, float]:
        p = self.state.robot
        rx, ry = rotate(local_x, local_y, p.theta)
        return p.x + rx, p.y + ry

    def on_tape(self, point: tuple[float, float]) -> bool:
        return any(rect_contains(rect, point[0], point[1]) for rect in self.tape_rects())

    def ping(self, angle_offset: float, max_dist: float = 120.0) -> tuple[float, str]:
        p = self.state.robot
        origin = self.sensor_point(ROBOT_L / 2.0, 0.0)
        angle = p.theta + angle_offset
        candidates: list[tuple[float, str]] = []

        # Distance sensors ignore flat black tape. Only raised objects stop rays.
        for i, obstacle in enumerate(self.state.obstacles, start=1):
            d = ray_rect_distance(origin, angle, obstacle.rect, max_dist)
            if d is not None:
                candidates.append((d, f"obstacle_{i}"))
        target_rect = expanded_rect((self.state.target.x - 5.5, self.state.target.y - 5.5, self.state.target.x + 5.5, self.state.target.y + 5.5), 0.0)
        d = ray_rect_distance(origin, angle, target_rect, max_dist)
        if d is not None:
            candidates.append((d, "enemy"))

        if not candidates:
            return max_dist, "none"
        return min(candidates, key=lambda item: item[0])

    def ir_reading(self, source: tuple[float, float], freq: str, fov_deg: float = 70.0) -> dict[str, Any]:
        p = self.state.robot
        dx = source[0] - p.x
        dy = source[1] - p.y
        rng = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - p.theta)
        in_front = abs(math.degrees(bearing)) <= fov_deg / 2.0
        in_rear = abs(math.degrees(wrap_angle(bearing - math.pi))) <= fov_deg / 2.0
        strength = 0.0 if rng < 1e-6 else 1.0 / (rng * rng)
        return {
            "freq": freq,
            "range_in": round(rng, 2),
            "bearing_deg": round(math.degrees(bearing), 2),
            "front": in_front,
            "rear": in_rear,
            "strength": round(strength, 6),
        }

    def collision_info(self) -> tuple[bool, str]:
        p = self.state.robot
        body_radius = max(ROBOT_L, ROBOT_W) / 2.0
        for i, obstacle in enumerate(self.state.obstacles, start=1):
            if rect_contains(expanded_rect(obstacle.rect, body_radius), p.x, p.y):
                return True, f"obstacle_{i}"
        return False, "none"

    def bump_readings(self, collision: bool, collision_with: str) -> dict[str, Any]:
        readings = {"any": collision, "front": False, "rear": False, "left": False, "right": False, "with": collision_with}
        if not collision or not collision_with.startswith("obstacle_"):
            return readings

        obstacle_idx = int(collision_with.split("_", 1)[1]) - 1
        if obstacle_idx < 0 or obstacle_idx >= len(self.state.obstacles):
            return readings

        p = self.state.robot
        obstacle = self.state.obstacles[obstacle_idx]
        dx = obstacle.x - p.x
        dy = obstacle.y - p.y
        # Convert obstacle direction into robot-local coordinates.
        local_x, local_y = rotate(dx, dy, -p.theta)
        if abs(local_x) >= abs(local_y):
            readings["front" if local_x >= 0 else "rear"] = True
        else:
            readings["left" if local_y >= 0 else "right"] = True
        return readings

    def out_of_bounds(self) -> bool:
        corners = self.robot_corners()
        inside_poly = clip_polygon_to_rect(corners, (OUTER_LEFT, OUTER_TOP, OUTER_RIGHT, OUTER_BOTTOM))
        return polygon_area(inside_poly) < polygon_area(corners) / 2.0

    def build_sensors(self) -> dict[str, Any]:
        p = self.state.robot
        tape_points = {
            "front_left": self.sensor_point(ROBOT_L / 2.0, ROBOT_W * 0.35),
            "front_center": self.sensor_point(ROBOT_L / 2.0, 0.0),
            "front_right": self.sensor_point(ROBOT_L / 2.0, -ROBOT_W * 0.35),
            "mid_left": self.sensor_point(0.0, ROBOT_W / 2.0),
            "mid_right": self.sensor_point(0.0, -ROBOT_W / 2.0),
            "rear_left": self.sensor_point(-ROBOT_L / 2.0, ROBOT_W * 0.35),
            "rear_center": self.sensor_point(-ROBOT_L / 2.0, 0.0),
            "rear_right": self.sensor_point(-ROBOT_L / 2.0, -ROBOT_W * 0.35),
        }
        tape = {name: self.on_tape(point) for name, point in tape_points.items()}
        tape.update(
            {
                "front": tape["front_left"] or tape["front_center"] or tape["front_right"],
                "rear": tape["rear_left"] or tape["rear_center"] or tape["rear_right"],
                "left": tape["front_left"] or tape["mid_left"] or tape["rear_left"],
                "right": tape["front_right"] or tape["mid_right"] or tape["rear_right"],
            }
        )
        collision, collision_with = self.collision_info()
        bump = self.bump_readings(collision, collision_with)
        target_ir = self.ir_reading((self.state.target.x, self.state.target.y), TARGET_BEACON_FREQ)
        obstacle_irs = [self.ir_reading((o.x, o.y), o.freq) for o in self.state.obstacles]
        obstacle_irs.sort(key=lambda item: item["range_in"])

        pings = {
            "front": self.ping(0.0),
            "front_left": self.ping(math.radians(25.0)),
            "front_right": self.ping(math.radians(-25.0)),
            "left": self.ping(math.radians(90.0)),
            "right": self.ping(math.radians(-90.0)),
        }
        nearest_wire = min(self.state.obstacles, key=lambda o: abs(p.x - o.x))
        track_wire_left = 1.0 / max(0.25, distance(tape_points["mid_left"], (nearest_wire.x, nearest_wire.y)))
        track_wire_right = 1.0 / max(0.25, distance(tape_points["mid_right"], (nearest_wire.x, nearest_wire.y)))

        sensors = {
            "time_s": round(self.state.elapsed, 3),
            "pose": {"x": round(p.x, 2), "y": round(p.y, 2), "heading_deg": round(math.degrees(p.theta), 2)},
            "tape": tape,
            "bump": bump,
            "track_wire": {
                "left": round(track_wire_left, 4),
                "right": round(track_wire_right, 4),
                "nearest_obstacle_x": nearest_wire.x,
            },
            "ir": {
                "target_2k": target_ir,
                "nearest_obstacle": obstacle_irs[0],
                "obstacles": obstacle_irs,
            },
            "ping": {name: {"range_in": round(value[0], 2), "object": value[1]} for name, value in pings.items()},
            "game": {
                "ammo": self.state.ammo,
                "score": self.state.score,
                "hits": self.state.hits,
                "goal": self.state.goal,
                "legal_zone": self.state.legal_zone,
                "isz_reached": self.state.isz_reached,
                "last_shot": self.state.last_shot,
                "disqualified": self.state.disqualified,
                "disq_reason": self.state.disq_reason,
            },
        }
        self.last_sensors = sensors
        return sensors

    def parse_controls(self, raw: Any) -> Controls:
        if raw is None:
            return Controls()
        if isinstance(raw, Controls):
            controls = raw
        elif isinstance(raw, dict):
            if any(key in raw for key in ("vx", "vy", "omega")):
                controls = Controls(
                    vx=float(raw.get("vx", 0.0)),
                    vy=float(raw.get("vy", 0.0)),
                    omega=float(raw.get("omega", 0.0)),
                    shoot=bool(raw.get("shoot", False)),
                    drive_mode="omni",
                )
            elif all(key in raw for key in ("front_left", "front_right", "rear_left", "rear_right")):
                fl = float(raw["front_left"])
                fr = float(raw["front_right"])
                rl = float(raw["rear_left"])
                rr = float(raw["rear_right"])
                controls = Controls(
                    vx=(fl + fr + rl + rr) / 4.0,
                    vy=(-fl + fr + rl - rr) / 4.0,
                    omega=(-fl + fr - rl + rr) / (2.0 * (ROBOT_L + ROBOT_W)),
                    shoot=bool(raw.get("shoot", False)),
                    drive_mode="omni",
                )
            else:
                controls = Controls(
                    left=float(raw.get("left", raw.get("left_speed", 0.0))),
                    right=float(raw.get("right", raw.get("right_speed", 0.0))),
                    shoot=bool(raw.get("shoot", False)),
                )
        elif isinstance(raw, (tuple, list)):
            controls = Controls(float(raw[0]), float(raw[1]), bool(raw[2]) if len(raw) > 2 else False)
        else:
            raise RuntimeError("Controller must return dict, tuple/list, Controls, or None")
        controls.left = clamp(controls.left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        controls.right = clamp(controls.right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        controls.vx = clamp(controls.vx, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        controls.vy = clamp(controls.vy, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        controls.omega = clamp(controls.omega, -4.0, 4.0)
        return controls

    def demo_controller(self, sensors: dict[str, Any], _dt: float) -> Controls:
        if sensors["game"]["disqualified"] or sensors["game"]["goal"]:
            return Controls()
        if sensors["bump"]["any"]:
            return Controls(left=-7.0, right=7.0)
        if not sensors["game"]["isz_reached"]:
            heading = wrap_angle(math.radians(sensors["pose"]["heading_deg"]) - math.pi)
            if abs(math.degrees(heading)) > 12.0:
                turn = clamp(heading / math.radians(90.0), -1.0, 1.0)
                return Controls(left=7.0 * turn, right=-7.0 * turn)
            return Controls(left=8.0, right=8.0)
        bearing = sensors["ir"]["target_2k"]["bearing_deg"]
        target_seen = sensors["ir"]["target_2k"]["front"]
        legal = sensors["game"]["isz_reached"]
        if legal and target_seen and abs(bearing) < 4.0 and sensors["game"]["ammo"] > 0:
            return Controls(shoot=True)
        if target_seen:
            turn = clamp(bearing / 45.0, -1.0, 1.0)
            return Controls(left=8.0 - 5.0 * turn, right=8.0 + 5.0 * turn)
        return Controls(left=6.0, right=10.0)

    def update_zones(self, controls: Controls, dt: float) -> None:
        if controls.drive_mode == "omni":
            speed = abs(controls.vx) + abs(controls.vy) + abs(controls.omega)
        else:
            speed = abs(controls.left) + abs(controls.right)
        p = self.state.robot
        if not self.state.isz_reached and p.x <= BOUNDARY_LEFT + ISZ_WIDTH - ROBOT_L / 2.0:
            self.state.isz_reached = True
            self.state.legal_zone = max(self.state.legal_zone, 1)
            self.state.score += 10

        candidate = None
        for idx, obstacle in enumerate(self.state.obstacles, start=1):
            fully_behind = p.x < obstacle.x - ROBOT_L / 2.0 and abs(p.y - obstacle.y) <= OBSTACLE_W
            if fully_behind and idx >= self.state.legal_zone and speed < 0.5:
                candidate = idx
                break

        if candidate is None:
            if self.state.active_pause_obstacle is not None and 1.0 <= self.state.pause_time <= 5.0:
                self.state.legal_zone = max(self.state.legal_zone, self.state.active_pause_obstacle + 1)
            self.state.active_pause_obstacle = None
            self.state.pause_time = 0.0
            return

        if self.state.active_pause_obstacle != candidate:
            self.state.active_pause_obstacle = candidate
            self.state.pause_time = 0.0
        self.state.pause_time += dt

    def is_shot_legal(self) -> bool:
        return self.state.isz_reached and not self.state.disqualified

    def line_of_sight_blocked(self) -> bool:
        p = self.state.robot
        angle = math.atan2(self.state.target.y - p.y, self.state.target.x - p.x)
        dist_to_target = distance((p.x, p.y), (self.state.target.x, self.state.target.y))
        for obstacle in self.state.obstacles:
            d = ray_rect_distance((p.x, p.y), angle, obstacle.rect, dist_to_target)
            if d is not None:
                return True
        return False

    def shoot(self) -> None:
        s = self.state
        if s.shot_cooldown > 0.0 or s.ammo <= 0 or s.disqualified:
            return
        s.shot_cooldown = 0.6
        s.ammo -= 1
        p = s.robot
        start_x = p.x + math.cos(p.theta) * ROBOT_L / 2.0
        start_y = p.y + math.sin(p.theta) * ROBOT_L / 2.0
        end_x = s.target.x
        end_y = s.target.y
        outcome = "invalid"
        if not self.is_shot_legal():
            s.last_shot = "invalid: not in legal shooting zone"
        elif self.line_of_sight_blocked():
            s.last_shot = "miss: obstacle blocked line of sight"
            outcome = "blocked"
        else:
            target_angle = math.atan2(s.target.y - p.y, s.target.x - p.x)
            err = abs(math.degrees(wrap_angle(target_angle - p.theta)))
            rng = distance((p.x, p.y), (s.target.x, s.target.y))
            goal_window = max(2.0, 140.0 / max(rng, 1.0))
            body_window = max(6.0, 360.0 / max(rng, 1.0))
            if err <= goal_window:
                s.goal = True
                s.score += 100
                s.last_shot = f"GOAL: angle error {err:.1f} deg"
                outcome = "goal"
            elif err <= body_window:
                s.hits += 1
                if s.hits == 1:
                    s.score += 20
                elif s.hits == 2:
                    s.score += 40
                else:
                    s.score += 10
                s.last_shot = f"hit body: angle error {err:.1f} deg"
                outcome = "hit"
            else:
                s.last_shot = f"miss: angle error {err:.1f} deg"
                outcome = "miss"

        s.shots.append(
            {
                "start_time": s.elapsed,
                "duration": 0.7,
                "start": (start_x, start_y),
                "end": (end_x, end_y),
                "outcome": outcome,
            }
        )
        s.shots = s.shots[-8:]

    def disqualify(self, reason: str) -> None:
        if not self.state.disqualified:
            self.state.disqualified = True
            self.state.disq_reason = reason

    def step(self, dt: float) -> None:
        sensors = self.build_sensors()
        if self.paused:
            return

        if self.state.disqualified or self.state.goal or self.state.elapsed >= 120.0:
            self.state.elapsed += dt
            return

        try:
            if self.use_demo_controller:
                controls = self.demo_controller(sensors, dt)
            elif self.controller is not None:
                controls = self.parse_controls(self.controller(sensors, dt))
            else:
                controls = Controls(self.manual_left, self.manual_right, self.manual_shoot)
        except Exception as exc:
            self.disqualify(f"controller error: {exc}")
            return

        if not self.use_demo_controller and self.controller is None:
            controls = Controls(self.manual_left, self.manual_right, self.manual_shoot)

        if self.controller is None and not self.use_demo_controller:
            controls.shoot = self.manual_shoot

        controls.left = clamp(controls.left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        controls.right = clamp(controls.right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        self.last_controls = controls
        self.update_zones(controls, dt)

        p = self.state.robot
        if controls.drive_mode == "omni":
            vx_world, vy_world = rotate(controls.vx, controls.vy, p.theta)
            p.theta = wrap_angle(p.theta + controls.omega * dt)
            p.x += vx_world * dt
            p.y += vy_world * dt
        else:
            v = (controls.left + controls.right) / 2.0
            omega = (controls.right - controls.left) / WHEEL_BASE
            p.theta = wrap_angle(p.theta + omega * dt)
            p.x += v * math.cos(p.theta) * dt
            p.y += v * math.sin(p.theta) * dt

        collision, collision_with = self.collision_info()
        if collision:
            self.state.collision_time += dt
            if self.state.collision_time > 1.0:
                self.disqualify(f"contact with {collision_with} for more than 1 second")
        else:
            self.state.collision_time = 0.0

        if self.out_of_bounds():
            self.disqualify("more than half of robot left the field")

        if controls.shoot:
            self.shoot()
        self.manual_shoot = False
        self.state.shot_cooldown = max(0.0, self.state.shot_cooldown - dt)
        self.state.elapsed += dt

        if self.log_path:
            self.state.log.append({"sensors": sensors, "controls": asdict(controls)})

    def save_log(self) -> None:
        if not self.log_path:
            return
        with self.log_path.open("w", encoding="utf-8") as f:
            for row in self.state.log:
                f.write(json.dumps(row) + "\n")


class SluggersApp:
    def __init__(self, sim: SluggersSim):
        self.sim = sim
        self.root = tk.Tk()
        self.root.title("Sluggers of the Lost Goal Simulator")
        w = int(sim.margin * 3 + FIELD_L * sim.scale * 2 + sim.panel_w)
        h = int(sim.margin * 3 + FIELD_W * sim.scale + 260)
        self.canvas = tk.Canvas(self.root, width=w, height=h, bg="#b0b0b0")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.last_tick = time.perf_counter()
        self.panel_tab = "Sensors"
        self.tab_bounds: list[tuple[str, float, float, float, float]] = []
        self.button_bounds: list[tuple[str, float, float, float, float]] = []
        self.bind_keys()

    def bind_keys(self) -> None:
        self.root.bind("<KeyPress-Up>", lambda _e: self.set_manual(10.0, 10.0))
        self.root.bind("<KeyRelease-Up>", lambda _e: self.set_manual(0.0, 0.0))
        self.root.bind("<KeyPress-Down>", lambda _e: self.set_manual(-8.0, -8.0))
        self.root.bind("<KeyRelease-Down>", lambda _e: self.set_manual(0.0, 0.0))
        self.root.bind("<KeyPress-Left>", lambda _e: self.set_manual(-7.0, 7.0))
        self.root.bind("<KeyRelease-Left>", lambda _e: self.set_manual(0.0, 0.0))
        self.root.bind("<KeyPress-Right>", lambda _e: self.set_manual(7.0, -7.0))
        self.root.bind("<KeyRelease-Right>", lambda _e: self.set_manual(0.0, 0.0))
        self.root.bind("<space>", lambda _e: self.manual_shoot())
        self.root.bind("r", lambda _e: self.sim.reset())
        self.root.bind("p", lambda _e: setattr(self.sim, "paused", not self.sim.paused))
        self.root.bind("d", lambda _e: setattr(self.sim, "use_demo_controller", not self.sim.use_demo_controller))
        self.root.bind("1", lambda _e: setattr(self, "panel_tab", "Sensors"))
        self.root.bind("2", lambda _e: setattr(self, "panel_tab", "UNO Pins"))
        self.root.bind("3", lambda _e: setattr(self, "panel_tab", "Zones"))
        self.root.bind("<Button-1>", self.on_click)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def on_click(self, event: tk.Event) -> None:
        for action, x1, y1, x2, y2 in self.button_bounds:
            if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                if action == "reset":
                    self.sim.reset()
                break
        for label, x1, y1, x2, y2 in self.tab_bounds:
            if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                self.panel_tab = label
                break

    def close(self) -> None:
        self.sim.save_log()
        self.root.destroy()

    def set_manual(self, left: float, right: float) -> None:
        self.sim.use_demo_controller = False
        self.sim.manual_left = left
        self.sim.manual_right = right

    def manual_shoot(self) -> None:
        self.sim.use_demo_controller = False
        self.sim.manual_shoot = True

    def field_point(self, field_origin: tuple[float, float], x: float, y: float, rotate180: bool = False) -> tuple[float, float]:
        if rotate180:
            x = FIELD_L - x
            y = FIELD_W - y
        return field_origin[0] + x * self.sim.scale, field_origin[1] + y * self.sim.scale

    def fx(self, field_origin: tuple[float, float], x: float) -> float:
        return self.field_point(field_origin, x, 0.0)[0]

    def fy(self, field_origin: tuple[float, float], y: float) -> float:
        return self.field_point(field_origin, 0.0, y)[1]

    def draw_rect(self, origin: tuple[float, float], rect: tuple[float, float, float, float], rotate180: bool = False, **kwargs: Any) -> None:
        x1, y1, x2, y2 = rect
        px1, py1 = self.field_point(origin, x1, y1, rotate180)
        px2, py2 = self.field_point(origin, x2, y2, rotate180)
        self.canvas.create_rectangle(min(px1, px2), min(py1, py2), max(px1, px2), max(py1, py2), **kwargs)

    def draw_zone_overlays(self, origin: tuple[float, float], rotate180: bool = False) -> None:
        self.draw_rect(origin, (BOUNDARY_LEFT, 0.0, BOUNDARY_LEFT + ISZ_WIDTH, FIELD_W), rotate180=rotate180, fill="#eef2ff", outline="")
        self.draw_rect(
            origin,
            (START_X - 5.5, START_Y - 5.5, START_X + 5.5, START_Y + 5.5),
            rotate180=rotate180,
            outline="#4caf50",
            width=2,
            dash=(4, 3),
        )

    def draw_field(self, origin: tuple[float, float], title: str, show_robot: bool, rotate180: bool = False) -> None:
        self.draw_rect(origin, (0.0, 0.0, FIELD_L, FIELD_W), rotate180=rotate180, fill="white", outline="black", width=3)
        self.draw_zone_overlays(origin, rotate180)
        for rect in self.sim.tape_rects():
            self.draw_rect(origin, rect, rotate180=rotate180, fill="black", outline="")
        for obstacle in self.sim.state.obstacles:
            self.draw_rect(origin, obstacle.rect, rotate180=rotate180, fill="#4169e1", outline="#4169e1")
            tx, ty = self.field_point(origin, obstacle.x, obstacle.y - 7.0, rotate180)
            self.canvas.create_text(tx, ty, text=obstacle.freq, fill="#4169e1", font=("Arial", 8))
        title_x, title_y = self.field_point(origin, FIELD_L / 2.0, -3.0, False)
        self.canvas.create_text(title_x, title_y, text=title, font=("Arial", 11, "bold"))
        isz_x, isz_y = self.field_point(origin, BOUNDARY_LEFT + ISZ_WIDTH / 2.0, BOUNDARY_TOP - 2.0, rotate180)
        start_x, start_y = self.field_point(origin, START_X, START_Y + 8.0, rotate180)
        self.canvas.create_text(isz_x, isz_y, text="ISZ", font=("Arial", 8, "bold"), fill="#1d4ed8")
        self.canvas.create_text(start_x, start_y, text="START", font=("Arial", 8, "bold"), fill="#15803d")
        if show_robot:
            self.draw_robot(origin)
            self.draw_sensor_rays(origin)
        else:
            self.draw_target(origin, rotate180=rotate180)

    def draw_robot(self, origin: tuple[float, float]) -> None:
        pts = []
        for x, y in self.sim.robot_corners():
            pts.extend([self.fx(origin, x), self.fy(origin, y)])
        self.canvas.create_polygon(*pts, fill="#d62f45", outline="#8b0014", width=2)
        nose = self.sim.sensor_point(ROBOT_L / 2.0, 0.0)
        center = (self.sim.state.robot.x, self.sim.state.robot.y)
        self.canvas.create_line(self.fx(origin, center[0]), self.fy(origin, center[1]), self.fx(origin, nose[0]), self.fy(origin, nose[1]), width=2)
        markers = (
            (ROBOT_L / 2.0, ROBOT_W * 0.35, "orange"),
            (ROBOT_L / 2.0, 0.0, "yellow"),
            (ROBOT_L / 2.0, -ROBOT_W * 0.35, "orange"),
            (0.0, ROBOT_W / 2.0, "purple"),
            (0.0, -ROBOT_W / 2.0, "purple"),
            (-ROBOT_L / 2.0, ROBOT_W * 0.35, "orange"),
            (-ROBOT_L / 2.0, 0.0, "yellow"),
            (-ROBOT_L / 2.0, -ROBOT_W * 0.35, "orange"),
        )
        for lx, ly, color in markers:
            sx, sy = self.sim.sensor_point(lx, ly)
            r = 0.75 * self.sim.scale
            self.canvas.create_oval(self.fx(origin, sx) - r, self.fy(origin, sy) - r, self.fx(origin, sx) + r, self.fy(origin, sy) + r, fill=color, outline="black")

    def draw_target(self, origin: tuple[float, float], rotate180: bool = False) -> None:
        t = self.sim.state.target
        r = 5.5 * self.sim.scale
        cx, cy = self.field_point(origin, t.x, t.y, rotate180)
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#32cd32", outline="#228b22", width=2)
        hx, hy = self.field_point(origin, t.x + math.cos(t.theta) * 5.0, t.y + math.sin(t.theta) * 5.0, rotate180)
        self.canvas.create_line(cx, cy, hx, hy, width=2)
        tx, ty = self.field_point(origin, t.x, t.y - 9.0, rotate180)
        self.canvas.create_text(tx, ty, text="enemy 2 kHz", fill="#228b22", font=("Arial", 8))

    def draw_shots(self, robot_origin: tuple[float, float], target_origin: tuple[float, float]) -> None:
        colors = {
            "goal": "#22c55e",
            "hit": "#facc15",
            "blocked": "#f97316",
            "invalid": "#94a3b8",
            "miss": "#ef4444",
        }
        now = self.sim.state.elapsed
        for shot in self.sim.state.shots:
            age = now - shot["start_time"]
            if age < 0.0 or age > shot["duration"] + 0.45:
                continue
            progress = clamp(age / shot["duration"], 0.0, 1.0)
            sx, sy = shot["start"]
            ex, ey = shot["end"]
            bx = sx + (ex - sx) * progress
            by = sy + (ey - sy) * progress
            color = colors.get(shot["outcome"], "#ef4444")
            start_px = self.field_point(robot_origin, sx, sy)
            end_px = self.field_point(target_origin, ex, ey, rotate180=True)
            ball_px = (
                start_px[0] + (end_px[0] - start_px[0]) * progress,
                start_px[1] + (end_px[1] - start_px[1]) * progress,
            )
            self.canvas.create_line(start_px[0], start_px[1], ball_px[0], ball_px[1], fill=color, width=2, dash=(5, 4))
            r = 1.4 * self.sim.scale
            self.canvas.create_oval(ball_px[0] - r, ball_px[1] - r, ball_px[0] + r, ball_px[1] + r, fill="white", outline=color, width=3)
            if progress >= 1.0:
                self.canvas.create_text(end_px[0], end_px[1] - 10, text=shot["outcome"].upper(), fill=color, font=("Arial", 9, "bold"))

    def draw_sensor_rays(self, origin: tuple[float, float]) -> None:
        p = self.sim.state.robot
        target = self.sim.state.target
        blocked = self.sim.line_of_sight_blocked()
        color = "#cc8800" if blocked else "#40c4ff"
        self.canvas.create_line(self.fx(origin, p.x), self.fy(origin, p.y), self.fx(origin, target.x), self.fy(origin, target.y), fill=color, dash=(3, 3))
        rays = (
            (0.0, "#2dd4bf", "F"),
            (math.radians(25.0), "#99f6e4", "FL"),
            (math.radians(-25.0), "#99f6e4", "FR"),
            (math.radians(90.0), "#a7f3d0", "L"),
            (math.radians(-90.0), "#a7f3d0", "R"),
        )
        for offset, ray_color, label in rays:
            d, _obj = self.sim.ping(offset, 70.0)
            angle = p.theta + offset
            x2 = p.x + math.cos(angle) * d
            y2 = p.y + math.sin(angle) * d
            end_x = self.fx(origin, x2)
            end_y = self.fy(origin, y2)
            self.canvas.create_line(self.fx(origin, p.x), self.fy(origin, p.y), end_x, end_y, fill=ray_color)
            self.canvas.create_text(end_x, end_y, text=f"{label} {d:.0f}", anchor="s", font=("Arial", 7), fill="#0f766e")

    def draw_tabs(self, x: float, y: float) -> None:
        tabs = ("Sensors", "UNO Pins", "Zones")
        self.tab_bounds = []
        for i, label in enumerate(tabs):
            x1 = x + i * 112
            x2 = x1 + 106
            fill = "#ffffff" if self.panel_tab == label else "#d8d8df"
            self.canvas.create_rectangle(x1, y, x2, y + 26, fill=fill, outline="#555555")
            self.canvas.create_text(x1 + 53, y + 13, text=label, font=("Arial", 9, "bold"), fill="#111111")
            self.tab_bounds.append((label, x1, y, x2, y + 26))

    def draw_status_bar(self, x: float, y: float, width: float, sensors: dict[str, Any]) -> None:
        self.button_bounds = []
        game = sensors["game"]
        if game["disqualified"]:
            fill = "#7f1d1d"
            text = f"DISQUALIFIED: {game['disq_reason']}"
        elif game["goal"]:
            fill = "#166534"
            text = "GOAL SCORED"
        elif self.sim.paused:
            fill = "#854d0e"
            text = "PAUSED"
        else:
            fill = "#1f2937"
            text = "RUNNING"
        self.canvas.create_rectangle(x, y, x + width, y + 32, fill=fill, outline="")
        mode = "demo/controller" if self.sim.use_demo_controller else "manual"
        summary = (
            f"{text}   mode={mode}   t={sensors['time_s']:.1f}s   "
            f"score={game['score']}   ammo={game['ammo']}   zone={game['legal_zone']}   "
            f"last shot={game['last_shot']}"
        )
        self.canvas.create_text(x + 10, y + 16, text=summary, anchor="w", font=("Consolas", 10, "bold"), fill="white")
        btn_w = 78
        btn_x1 = x + width - btn_w - 8
        btn_y1 = y + 5
        btn_x2 = btn_x1 + btn_w
        btn_y2 = y + 27
        self.canvas.create_rectangle(btn_x1, btn_y1, btn_x2, btn_y2, fill="#f8fafc", outline="#111827", width=1)
        self.canvas.create_text((btn_x1 + btn_x2) / 2, (btn_y1 + btn_y2) / 2, text="Reset", font=("Arial", 9, "bold"), fill="#111827")
        self.button_bounds.append(("reset", btn_x1, btn_y1, btn_x2, btn_y2))

    def draw_rows(self, x: float, y: float, rows: list[tuple[str, str, str | None]], col_w: tuple[int, int] = (170, 330)) -> None:
        row_h = 19
        for i, (name, value, color) in enumerate(rows):
            yy = y + i * row_h
            bg = "#f8fafc" if i % 2 == 0 else "#eef2f7"
            self.canvas.create_rectangle(x, yy, x + col_w[0] + col_w[1], yy + row_h, fill=bg, outline="#d0d7de")
            if color:
                self.canvas.create_rectangle(x + 5, yy + 5, x + 13, yy + 13, fill=color, outline="#555555")
            self.canvas.create_text(x + 18, yy + row_h / 2, text=name, anchor="w", font=("Consolas", 9), fill="#111111")
            self.canvas.create_text(x + col_w[0], yy + row_h / 2, text=value, anchor="w", font=("Consolas", 9), fill="#111111")

    def draw_sensor_panel(self, x: float, y: float) -> None:
        sensors = self.sim.last_sensors or self.sim.build_sensors()
        panel_w = FIELD_L * self.sim.scale * 2 + self.sim.margin + 170
        self.canvas.create_rectangle(x - 8, y - 8, x + panel_w, y + 232, fill="#e5e7eb", outline="#9ca3af")
        self.draw_tabs(x, y)
        self.draw_status_bar(x + 350, y, panel_w - 360, sensors)

        content_y = y + 38
        if self.panel_tab == "Sensors":
            rows = [
                ("tape_front_L/C/R", f"{int(sensors['tape']['front_left'])}/{int(sensors['tape']['front_center'])}/{int(sensors['tape']['front_right'])}", "#f59e0b"),
                ("tape_mid_L/R", f"{int(sensors['tape']['mid_left'])}/{int(sensors['tape']['mid_right'])}", "#fbbf24"),
                ("tape_rear_L/C/R", f"{int(sensors['tape']['rear_left'])}/{int(sensors['tape']['rear_center'])}/{int(sensors['tape']['rear_right'])}", "#f59e0b"),
                ("tape_alias_F/L/R/B", f"{int(sensors['tape']['front'])}/{int(sensors['tape']['left'])}/{int(sensors['tape']['right'])}/{int(sensors['tape']['rear'])}", "#fbbf24"),
                ("bump_F/L/R/B", f"{int(sensors['bump']['front'])}/{int(sensors['bump']['left'])}/{int(sensors['bump']['right'])}/{int(sensors['bump']['rear'])}", "#a855f7"),
                ("bump_any", f"{sensors['bump']['any']} [{sensors['bump']['with']}]", "#7c3aed"),
                ("ping_front", f"{sensors['ping']['front']['range_in']} in [{sensors['ping']['front']['object']}]", "#38bdf8"),
                ("track_wire_L", f"{sensors['track_wire']['left']} ({sensors['track_wire']['nearest_obstacle_x']:.1f} in)", "#22c55e"),
                ("track_wire_R", f"{sensors['track_wire']['right']} ({sensors['track_wire']['nearest_obstacle_x']:.1f} in)", "#22c55e"),
                ("ir_front_2k", f"{sensors['ir']['target_2k']['range_in']} in bearing={sensors['ir']['target_2k']['bearing_deg']} front={sensors['ir']['target_2k']['front']}", "#f97316"),
                ("ir_obstacle", f"{sensors['ir']['nearest_obstacle']['freq']} {sensors['ir']['nearest_obstacle']['range_in']} in bearing={sensors['ir']['nearest_obstacle']['bearing_deg']}", "#fb923c"),
                ("imu", f"yaw {sensors['pose']['heading_deg']} deg", "#6366f1"),
            ]
            self.draw_rows(x, content_y, rows)
        elif self.panel_tab == "UNO Pins":
            controls = self.sim.last_controls
            rows = [
                ("DRIVE_MODE", controls.drive_mode, "#64748b"),
                ("OMNI_VX", f"{controls.vx:.2f} in/s", "#dc2626"),
                ("OMNI_VY", f"{controls.vy:.2f} in/s", "#2563eb"),
                ("OMNI_OMEGA", f"{controls.omega:.2f} rad/s", "#9333ea"),
                ("TANK_LEFT/RIGHT", f"{controls.left:.2f} / {controls.right:.2f}", "#64748b"),
                ("LAUNCH_SERVO", "FIRE" if controls.shoot else "idle", "#16a34a"),
                ("REMOTE_ENABLE", "on", "#22c55e"),
                ("STATE_MODE", "demo/controller" if self.sim.use_demo_controller else "manual", "#64748b"),
                ("KEYS", "arrows drive | space shoot | r reset | p pause | d mode | 1-3 tabs", None),
            ]
            self.draw_rows(x, content_y, rows)
        else:
            game = sensors["game"]
            rows = [
                ("ISZ", f"reached={game['isz_reached']}  x <= {BOUNDARY_LEFT + ISZ_WIDTH - ROBOT_L / 2.0:.1f}", "#2563eb"),
                ("legal_zone", str(game["legal_zone"]), "#2563eb"),
                ("score", str(game["score"]), "#16a34a"),
                ("hits", str(game["hits"]), "#16a34a"),
                ("goal", str(game["goal"]), "#16a34a"),
                ("collision_timer", f"{self.sim.state.collision_time:.2f} / 1.00 s", "#ef4444"),
                ("pause_timer", f"{self.sim.state.pause_time:.2f} s", "#f97316"),
                ("shot_cooldown", f"{self.sim.state.shot_cooldown:.2f} s", "#8b5cf6"),
                ("robot_pose", f"x={sensors['pose']['x']} y={sensors['pose']['y']} yaw={sensors['pose']['heading_deg']}", "#64748b"),
                ("field", f"96x48, outside edge x={OUTER_LEFT:g}..{OUTER_RIGHT:g}, y={OUTER_TOP:g}..{OUTER_BOTTOM:g}", None),
                ("robot_size", f"{ROBOT_L:g} x {ROBOT_W:g} in", None),
                ("offshoots", ", ".join(f"{x:.0f}" for x in OFFSHOOT_XS), None),
                ("obstacles", ", ".join(f"{x:.0f}" for x in OBSTACLE_LINES), None),
            ]
            self.draw_rows(x, content_y, rows)

    def draw_sensor_map(self, x: float, y: float) -> None:
        self.canvas.create_rectangle(x, y, x + 170, y + 150, fill="#eeeeee", outline="#999999")
        self.canvas.create_text(x + 8, y + 8, text="Robot sensor map", anchor="nw", font=("Arial", 9, "bold"))
        cx = x + 85
        cy = y + 82
        r = 36
        poly = []
        for i in range(6):
            a = math.radians(60 * i + 30)
            poly.extend([cx + math.cos(a) * r, cy + math.sin(a) * r])
        self.canvas.create_polygon(*poly, fill="#f4b6c2", outline="#b00020", width=2)
        points = [
            (cx + 42, cy - 16, "orange"),
            (cx + 46, cy, "yellow"),
            (cx + 42, cy + 16, "orange"),
            (cx, cy - 38, "purple"),
            (cx, cy + 38, "purple"),
            (cx - 42, cy - 16, "orange"),
            (cx - 46, cy, "yellow"),
            (cx - 42, cy + 16, "orange"),
            (cx + 52, cy, "red"),
            (cx - 52, cy, "red"),
            (cx, cy - 46, "red"),
            (cx, cy + 46, "red"),
        ]
        for px, py, color in points:
            self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill=color, outline="black")
        self.canvas.create_line(cx, cy, cx + 42, cy, width=2)

    def redraw(self) -> None:
        self.canvas.delete("all")
        left_origin = (self.sim.margin, self.sim.margin)
        right_origin = (self.sim.margin * 2 + FIELD_L * self.sim.scale, self.sim.margin)
        panel_y = self.sim.margin * 2 + FIELD_W * self.sim.scale
        self.draw_field(left_origin, "Your robot field", show_robot=True)
        self.draw_field(right_origin, "Opponent/target field", show_robot=False, rotate180=True)
        self.draw_shots(left_origin, right_origin)
        self.draw_sensor_panel(self.sim.margin, panel_y)
        self.draw_sensor_map(self.sim.margin * 2 + FIELD_L * self.sim.scale * 2, panel_y + 58)

    def tick(self) -> None:
        now = time.perf_counter()
        dt = min(0.08, now - self.last_tick)
        self.last_tick = now
        accumulator = dt
        while accumulator > 0.0:
            step = min(SIM_DT, accumulator)
            self.sim.step(step)
            accumulator -= step
        self.redraw()
        self.root.after(int(SIM_DT * 1000), self.tick)

    def run(self) -> None:
        self.tick()
        self.root.mainloop()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sluggers of the Lost Goal simulator")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for repeatable fields")
    parser.add_argument("--controller", default=None, help="Path to a Python controller file with update(sensors, dt)")
    parser.add_argument("--log", default=None, help="Write JSONL sensor/control log when the window closes")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    sim = SluggersSim(seed=args.seed, controller_path=args.controller, log_path=args.log)
    app = SluggersApp(sim)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
