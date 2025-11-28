import math
import os

from services.coppelia import Pose2D
from utils.geometry import normalize_angle

def calc_heading_score(
    robot_pose: Pose2D,
    final_pose: tuple[float, float],
    goal: tuple[float, float] | None,
) -> float:
    if goal is None:
        return 0.0

    dx_goal = goal[0] - final_pose[0]
    dy_goal = goal[1] - final_pose[1]
    remaining_dist = math.sqrt(dx_goal**2 + dy_goal**2)

    if remaining_dist < 0.05:
        return 1.0

    target_angle = math.atan2(dy_goal, dx_goal)

    dx_traj = final_pose[0] - robot_pose.x
    dy_traj = final_pose[1] - robot_pose.y
    if dx_traj == 0 and dy_traj == 0:
        return 0.0

    traj_angle = math.atan2(dy_traj, dx_traj)

    angle_diff = abs(normalize_angle(target_angle - traj_angle))
    alignment = 1.0 - (angle_diff / math.pi)

    initial_dx = goal[0] - robot_pose.x
    initial_dy = goal[1] - robot_pose.y
    initial_dist = math.sqrt(initial_dx**2 + initial_dy**2)
    if initial_dist < 1e-3:
        progress = 0.0
    else:
        progress = max(0.0, (initial_dist - remaining_dist) / initial_dist)

    return 0.6 * alignment + 0.4 * progress

def calc_clearance_score(
    traj: list[tuple[float, float]],
    obstacles: list[tuple[float, float]],
    other_robots: list[Pose2D] | None,
    robot_radius: float,
) -> float:
    if not obstacles and not other_robots:
        return 1.0

    min_dist = float("inf")

    for point in traj:
        for obs in obstacles:
            dist = math.sqrt((point[0] - obs[0]) ** 2 + (point[1] - obs[1]) ** 2)
            min_dist = min(min_dist, dist)

        if other_robots:
            for robot in other_robots:
                dist = math.sqrt((point[0] - robot.x) ** 2 + (point[1] - robot.y) ** 2)
                min_dist = min(min_dist, dist)

    safety_margin_multiplier = float(os.getenv("SAFETY_MARGIN_MULTIPLIER", "2.0"))
    if min_dist < safety_margin_multiplier * robot_radius:
        return 0.0

    safe_distance = float(os.getenv("DWA_SAFE_DISTANCE", "0.5"))
    max_clearance = float(os.getenv("DWA_MAX_CLEARANCE", "2.0"))

    if min_dist < safe_distance:
        return 0.3 * ((min_dist - safety_margin_multiplier * robot_radius) / (safe_distance - safety_margin_multiplier * robot_radius))
    else:
        return 0.3 + 0.7 * min(1.0, (min_dist - safe_distance) / (max_clearance - safe_distance))

def calc_velocity_score(
    v: float,
    w: float,
    max_speed: float,
    max_angular_speed: float,
    current_v: float,
    current_w: float,
) -> float:
    if max_speed <= 0 or max_angular_speed <= 0:
        return 0.0

    forward_score = max(0.0, v) / max_speed
    turn_penalty = 1.0 - min(1.0, abs(w) / max_angular_speed)

    dv = abs(v - current_v)
    dw = abs(w - current_w)
    accel_norm = max_speed
    angular_norm = max_angular_speed
    smoothness = 1.0 - min(1.0, (dv / accel_norm) * 0.5 + (dw / angular_norm) * 0.5)

    return 0.5 * forward_score + 0.25 * turn_penalty + 0.25 * smoothness
