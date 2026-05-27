"""
Game controller template for sluggers_sim.py.

Run:
    python sluggers_sim.py --controller sluggers_controller_template.py

This is intentionally written like a robot state machine. Replace the sensor
checks and motor constants with the logic you plan to run on the Uno.
"""

import math


state = "FACE_ISZ"
state_time = 0.0
shots_requested = 0
avoid_dir = 1


DRIVE_SPEED = 8.5
STRAFE_SPEED = 6.0
TURN_RATE = 1.4
BACKUP_SPEED = -7.0
MAX_SHOTS = 6
SLOW_RANGE_IN = 18.0
STOP_RANGE_IN = 6.5
SIDE_CLEARANCE_IN = 9.0
TAPE_GUARD_TURN = 0.7
BYPASS_SPEED = 6.5
OBSTACLE_PLAN_RANGE_IN = 32.0
OBSTACLE_LATERAL_CLEARANCE_IN = 13.0


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def heading_error_deg(target_deg, current_deg):
    return ((target_deg - current_deg + 540) % 360) - 180


def set_state(next_state):
    global state, state_time
    if state != next_state:
        state = next_state
        state_time = 0.0


def drive(vx, vy=0.0, omega=0.0, shoot=False):
    # 4-omni holonomic command in robot coordinates:
    # vx = forward, vy = left strafe, omega = CCW yaw rate.
    return {"vx": vx, "vy": vy, "omega": omega, "shoot": shoot}


def obstacle_relative_pos(obstacle):
    if "rel_x_in" in obstacle and "rel_y_in" in obstacle:
        return obstacle["rel_x_in"], obstacle["rel_y_in"]

    # Older simulator logs only have polar obstacle IR. Bearing is already in
    # robot coordinates, so convert it into forward/left components.
    rng = obstacle.get("range_in", 120.0)
    bearing = math.radians(obstacle.get("bearing_deg", 0.0))
    return rng * math.cos(bearing), rng * math.sin(bearing)


def nearest_planned_obstacle(obstacles):
    candidates = []
    for obstacle in obstacles:
        rel_x, rel_y = obstacle_relative_pos(obstacle)
        if 0.0 < rel_x < OBSTACLE_PLAN_RANGE_IN and abs(rel_y) < OBSTACLE_LATERAL_CLEARANCE_IN:
            candidates.append((rel_x, abs(rel_y), obstacle))
    if not candidates:
        return None
    return min(candidates, key=lambda item: (item[0], item[1]))[2]


def obstacle_plan_weight(obstacle):
    if obstacle is None:
        return 0.0
    rel_x, rel_y = obstacle_relative_pos(obstacle)
    forward_weight = clamp((OBSTACLE_PLAN_RANGE_IN - rel_x) / OBSTACLE_PLAN_RANGE_IN, 0.0, 1.0)
    lateral_weight = clamp((OBSTACLE_LATERAL_CLEARANCE_IN - abs(rel_y)) / OBSTACLE_LATERAL_CLEARANCE_IN, 0.0, 1.0)
    return forward_weight * lateral_weight


def choose_avoid_dir(ping, obstacle=None):
    # Return +1 to go left around the object, -1 to go right.
    if obstacle is not None:
        _rel_x, rel_y = obstacle_relative_pos(obstacle)
        if abs(rel_y) > 1.0:
            return -1 if rel_y > 0.0 else 1

    left_room = min(ping["front_left"]["range_in"], ping["left"]["range_in"])
    right_room = min(ping["front_right"]["range_in"], ping["right"]["range_in"])
    return 1 if left_room >= right_room else -1


def obstacle_avoidance_turn(ping, obstacle=None):
    """Positive turn steers left, negative turn steers right."""
    front = ping["front"]["range_in"]
    front_left = ping["front_left"]["range_in"]
    front_right = ping["front_right"]["range_in"]
    left = ping["left"]["range_in"]
    right = ping["right"]["range_in"]

    turn = 0.0
    planned_weight = obstacle_plan_weight(obstacle)
    if planned_weight > 0.0:
        turn += choose_avoid_dir(ping, obstacle) * planned_weight

    # If something is directly ahead, choose the side with more room.
    if front < SLOW_RANGE_IN:
        turn += 1.0 if front_left > front_right else -1.0
        turn *= (SLOW_RANGE_IN - front) / SLOW_RANGE_IN

    # Bias away from whichever front diagonal is more blocked.
    if front_left < SLOW_RANGE_IN:
        turn -= (SLOW_RANGE_IN - front_left) / SLOW_RANGE_IN
    if front_right < SLOW_RANGE_IN:
        turn += (SLOW_RANGE_IN - front_right) / SLOW_RANGE_IN

    # Keep side clearance so the robot does not scrape obstacles.
    if left < SIDE_CLEARANCE_IN:
        turn -= 0.7 * (SIDE_CLEARANCE_IN - left) / SIDE_CLEARANCE_IN
    if right < SIDE_CLEARANCE_IN:
        turn += 0.7 * (SIDE_CLEARANCE_IN - right) / SIDE_CLEARANCE_IN

    return clamp(turn, -1.0, 1.0)


def tape_guard_turn(tape):
    # Tape is flat, so distance sensors ignore it. Use tape sensors to stay
    # inside the field and to ride along black guide lines.
    turn = 0.0
    if tape["left"]:
        turn -= TAPE_GUARD_TURN
    if tape["right"]:
        turn += TAPE_GUARD_TURN
    return clamp(turn, -1.0, 1.0)


def tape_guard_strafe(tape):
    # Omni wheels can correct side tape contact by strafing away from it without
    # changing heading.
    strafe = 0.0
    if tape["left"]:
        strafe -= STRAFE_SPEED * 0.7
    if tape["right"]:
        strafe += STRAFE_SPEED * 0.7
    return strafe


def drive_to_isz_with_ping(sensors):
    global avoid_dir

    pose = sensors["pose"]
    ping = sensors["ping"]
    tape = sensors["tape"]
    front = ping["front"]
    planned_obstacle = nearest_planned_obstacle(sensors["ir"].get("obstacles", []))
    planned_weight = obstacle_plan_weight(planned_obstacle)

    heading_turn = clamp(heading_error_deg(180, pose["heading_deg"]) / 90, -1, 1)
    avoid_turn = obstacle_avoidance_turn(ping, planned_obstacle)
    tape_turn = tape_guard_turn(tape)

    # Obstacle avoidance gets priority near objects; otherwise hold course to ISZ.
    nearest_front = min(
        ping["front"]["range_in"],
        ping["front_left"]["range_in"],
        ping["front_right"]["range_in"],
    )
    if planned_obstacle is not None:
        rel_x, _rel_y = obstacle_relative_pos(planned_obstacle)
        nearest_front = min(nearest_front, max(0.0, rel_x))
    avoid_weight = max(clamp((SLOW_RANGE_IN - nearest_front) / SLOW_RANGE_IN, 0.0, 1.0), planned_weight)
    base_turn = heading_turn * 0.65 + tape_turn
    turn = clamp(base_turn * (1.0 - avoid_weight) + avoid_turn * avoid_weight, -1.0, 1.0)

    speed = DRIVE_SPEED
    if nearest_front < SLOW_RANGE_IN:
        speed = clamp(DRIVE_SPEED * nearest_front / SLOW_RANGE_IN, 3.0, DRIVE_SPEED)

    if planned_obstacle is not None:
        avoid_dir = choose_avoid_dir(ping, planned_obstacle)

    if front["range_in"] < STOP_RANGE_IN and front["object"].startswith("obstacle"):
        avoid_dir = choose_avoid_dir(ping, planned_obstacle)
        set_state("BACK_UP")
        return drive(BACKUP_SPEED)

    strafe = clamp(avoid_turn * STRAFE_SPEED + tape_guard_strafe(tape), -STRAFE_SPEED, STRAFE_SPEED)
    return drive(speed, strafe, TURN_RATE * turn)


def update(sensors, dt):
    global state_time, shots_requested, avoid_dir
    state_time += dt

    game = sensors["game"]
    pose = sensors["pose"]
    tape = sensors["tape"]
    bump = sensors["bump"]
    ping = sensors["ping"]
    target_ir = sensors["ir"]["target_2k"]

    if game["disqualified"] or game["goal"] or game["ammo"] <= 0:
        return drive(0, 0, 0)

    if bump["any"] and state not in ("BACK_UP", "TURN_AWAY", "BYPASS_OBSTACLE"):
        avoid_dir = choose_avoid_dir(ping)
        set_state("BACK_UP")

    if state == "BACK_UP":
        if state_time < 0.35:
            return drive(BACKUP_SPEED)
        set_state("TURN_AWAY")

    if state == "TURN_AWAY":
        if state_time < 0.55:
            return drive(0, 0, TURN_RATE * avoid_dir)
        set_state("BYPASS_OBSTACLE" if not game["isz_reached"] else "SEARCH_BEACON")

    if state == "BYPASS_OBSTACLE":
        # Commit to an arc around the obstacle before reacquiring the ISZ heading.
        # This prevents backing up, turning, then immediately driving into the
        # same obstacle again.
        if ping["front"]["range_in"] < STOP_RANGE_IN:
            set_state("BACK_UP")
            return drive(BACKUP_SPEED)
        if state_time < 1.1 or ping["front"]["range_in"] < SLOW_RANGE_IN:
            return drive(BYPASS_SPEED * 0.7, STRAFE_SPEED * avoid_dir, TURN_RATE * 0.25 * avoid_dir)
        set_state("FACE_ISZ" if not game["isz_reached"] else "SEARCH_BEACON")

    if not game["isz_reached"]:
        if state not in ("FACE_ISZ", "DRIVE_TO_ISZ", "BYPASS_OBSTACLE"):
            set_state("FACE_ISZ")

        if state == "FACE_ISZ":
            # Spawn is on the right, ISZ is on the left, so face 180 degrees.
            error = heading_error_deg(180, pose["heading_deg"])
            if abs(error) < 10:
                set_state("DRIVE_TO_ISZ")
            turn = clamp(error / 90, -1, 1)
            return drive(0, 0, TURN_RATE * turn)

        if state == "DRIVE_TO_ISZ":
            return drive_to_isz_with_ping(sensors)

    if state in ("FACE_ISZ", "DRIVE_TO_ISZ"):
        set_state("SEARCH_BEACON")

    if state == "SEARCH_BEACON":
        if target_ir["front"]:
            set_state("AIM")
        return drive(0, 0, 0.8)

    if state == "AIM":
        bearing = target_ir["bearing_deg"]
        if target_ir["front"] and abs(bearing) < 3.0:
            set_state("FIRE")
            return drive(0, 0, 0)
        turn = clamp(bearing / 45, -1, 1)
        return drive(0, 0, TURN_RATE * turn)

    if state == "FIRE":
        if state_time < 0.15:
            return drive(0, 0, 0)
        shots_requested += 1
        set_state("SHOT_SETTLE")
        return drive(0, 0, 0, shoot=True)

    if state == "SHOT_SETTLE":
        if state_time < 0.7:
            return drive(0, 0, 0)
        if shots_requested >= MAX_SHOTS:
            set_state("DONE")
        else:
            set_state("AIM")

    return drive(0, 0, 0)
