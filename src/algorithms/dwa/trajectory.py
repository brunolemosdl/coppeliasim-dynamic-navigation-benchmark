import math
import os

from services.coppelia import Pose2D

def predict_trajectory(
    pose: Pose2D, v: float, w: float, predict_time: float = 1.5, dt: float = 0.1
) -> list[tuple[float, float]]:
    traj = []
    time = 0.0
    x, y, theta = pose.x, pose.y, pose.theta

    while time <= predict_time:
        traj.append((x, y))
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        time += dt

    return traj

def check_collision(
    traj: list[tuple[float, float]],
    obstacles: list[tuple[float, float]],
    other_robots: list,
    robot_radius: float,
) -> bool:
    if not obstacles and not other_robots:
        return False

    if len(traj) < 2:
        return False

    collision_threshold = float(os.getenv("COLLISION_THRESHOLD", "0.15"))
    collision_distance = robot_radius + collision_threshold

    for i, point in enumerate(traj):
        if obstacles:
            for obs in obstacles:
                dist = math.sqrt((point[0] - obs[0]) ** 2 + (point[1] - obs[1]) ** 2)

                if dist < collision_distance:
                    return True

        if other_robots:
            for robot in other_robots:
                dist = math.sqrt((point[0] - robot.x) ** 2 + (point[1] - robot.y) ** 2)
                if dist < collision_distance:
                    return True

    return False

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
