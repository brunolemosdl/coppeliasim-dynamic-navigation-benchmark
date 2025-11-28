import math
import os
from collections import deque
from typing import Dict, Tuple

from services.coppelia import Pose2D
from utils.geometry import laser_to_obstacles, normalize_angle

from ..common import BasePlanner, PlannerConfig
from .geometry import compute_orca_line, linear_program2d

class ORCAPlanner(BasePlanner):
    def __init__(self, config: PlannerConfig | None = None):
        super().__init__(config)
        self.max_speed = config.max_linear_vel if config else 0.3
        self.max_angular_speed = config.max_angular_vel if config else 1.0
        self.robot_radius = config.robot_radius if config else 0.175
        self.time_horizon = float(os.getenv("ORCA_TIME_HORIZON", "3.0"))
        self.time_horizon_static = float(os.getenv("ORCA_TIME_HORIZON_STATIC", "2.5"))
        self.time_step = float(os.getenv("ORCA_TIME_STEP", os.getenv("TIME_STEP", "0.1")))
        self.neighbor_dist = float(os.getenv("ORCA_NEIGHBOR_DIST", "3.0"))

        velocity_history_size = int(os.getenv("ORCA_VELOCITY_HISTORY_SIZE", "5"))
        self.obstacle_history: Dict[Tuple[float, float], deque] = {}
        self.robot_velocity_history: deque = deque(maxlen=velocity_history_size)
        self.last_robot_pose: Pose2D | None = None
        self.last_time: float | None = None

        self.other_robot_history: Dict[int, deque] = {}

    def compute_velocity(
        self,
        robot_pose: Pose2D,
        laser_scan: list[tuple[float, float]],
        other_robots: list[Pose2D] | None = None,
    ) -> tuple[float, float]:
        import time as time_module

        target = self.current_waypoint
        if target is None:
            target = self.goal
        if target is None:
            return 0.0, 0.0

        current_time = time_module.time()
        if self.last_robot_pose is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 1e-6 and dt < 1.0:
                vx = (robot_pose.x - self.last_robot_pose.x) / dt
                vy = (robot_pose.y - self.last_robot_pose.y) / dt
                self.robot_velocity_history.append((vx, vy))
        self.last_robot_pose = robot_pose
        self.last_time = current_time

        preferred_vel = self._calculate_preferred_velocity(robot_pose, target)

        obstacles = laser_to_obstacles(robot_pose, laser_scan)

        orca_vel = self._compute_orca_velocity(robot_pose, preferred_vel, obstacles, other_robots)

        linear_vel = min(self.max_speed, math.sqrt(orca_vel[0] ** 2 + orca_vel[1] ** 2))

        desired_heading = math.atan2(orca_vel[1], orca_vel[0])
        heading_error = normalize_angle(desired_heading - robot_pose.theta)
        orca_heading_gain = float(os.getenv("ORCA_HEADING_GAIN", "2.0"))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, orca_heading_gain * heading_error))

        return linear_vel, angular_vel

    def _calculate_preferred_velocity(
        self, robot_pose: Pose2D, target: tuple[float, float] | None = None
    ) -> tuple[float, float]:
        if target is None:
            target = self.goal
        if target is None:
            return (0.0, 0.0)

        dx = target[0] - robot_pose.x
        dy = target[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            return (0.0, 0.0)

        vx = (dx / distance) * self.max_speed
        vy = (dy / distance) * self.max_speed

        return (vx, vy)

    def _compute_orca_velocity(
        self,
        robot_pose: Pose2D,
        preferred_vel: tuple[float, float],
        obstacles: list[tuple[float, float]],
        other_robots: list[Pose2D] | None = None,
    ) -> tuple[float, float]:
        orca_lines: list[tuple[tuple[float, float], float]] = []

        robot_vel = self._get_robot_velocity()

        self._update_obstacle_positions(obstacles)

        max_relevant_dist = self.max_speed * self.time_horizon + self.robot_radius + 1.0

        for obs in obstacles:
            dist = math.sqrt(
                (robot_pose.x - obs[0]) ** 2 + (robot_pose.y - obs[1]) ** 2
            )
            if dist > max_relevant_dist:
                continue

            obs_vel = (0.0, 0.0)

            collision_threshold = float(os.getenv("COLLISION_THRESHOLD", "0.15"))
            orca_line = compute_orca_line(
                (robot_pose.x, robot_pose.y),
                (obs[0], obs[1]),
                robot_vel,
                obs_vel,
                self.robot_radius,
                collision_threshold,
                self.time_horizon_static,
                reciprocal=False,
            )
            if orca_line:
                orca_lines.append(orca_line)

        if other_robots:
            import time as time_module
            current_time = time_module.time()

            robot_history_size = int(os.getenv("ORCA_ROBOT_HISTORY_SIZE", "10"))

            for idx, other_robot in enumerate(other_robots):
                dist = math.sqrt(
                    (robot_pose.x - other_robot.x) ** 2 + (robot_pose.y - other_robot.y) ** 2
                )
                if dist < self.neighbor_dist:
                    if idx not in self.other_robot_history:
                        self.other_robot_history[idx] = deque(maxlen=robot_history_size)
                    self.other_robot_history[idx].append((other_robot, current_time))

                    other_vel = self._estimate_other_robot_velocity(idx)

                    orca_line = compute_orca_line(
                        (robot_pose.x, robot_pose.y),
                        (other_robot.x, other_robot.y),
                        robot_vel,
                        other_vel,
                        self.robot_radius,
                        self.robot_radius,
                        self.time_horizon,
                        reciprocal=True,
                    )
                    if orca_line:
                        orca_lines.append(orca_line)

        optimal_vel = linear_program2d(
            orca_lines,
            self.max_speed,
            preferred_vel,
            direction_opt=False,
        )

        if optimal_vel is None:
            speed = math.sqrt(preferred_vel[0] ** 2 + preferred_vel[1] ** 2)
            if speed > self.max_speed:
                return (
                    (preferred_vel[0] / speed) * self.max_speed,
                    (preferred_vel[1] / speed) * self.max_speed,
                )
            return preferred_vel

        return optimal_vel

    def _get_robot_velocity(self) -> tuple[float, float]:
        if not self.robot_velocity_history:
            return (0.0, 0.0)

        vx_sum = sum(v[0] for v in self.robot_velocity_history)
        vy_sum = sum(v[1] for v in self.robot_velocity_history)
        n = len(self.robot_velocity_history)

        return (vx_sum / n, vy_sum / n)

    def _update_obstacle_positions(self, obstacles: list[tuple[float, float]]) -> None:
        import time as time_module

        current_time = time_module.time()
        obstacle_match_threshold = float(os.getenv("ORCA_OBSTACLE_MATCH_THRESHOLD", "0.5"))
        obstacle_history_size = int(os.getenv("ORCA_OBSTACLE_HISTORY_SIZE", "10"))

        matched_keys = set()
        for obs_pos in obstacles:
            min_dist = float('inf')
            closest_key = None

            for key in self.obstacle_history.keys():
                dist = math.sqrt((obs_pos[0] - key[0])**2 + (obs_pos[1] - key[1])**2)
                if dist < min_dist and dist < obstacle_match_threshold:
                    min_dist = dist
                    closest_key = key

            if closest_key is not None:
                self.obstacle_history[closest_key].append((obs_pos, current_time))
                matched_keys.add(closest_key)
            else:
                new_history = deque(maxlen=obstacle_history_size)
                new_history.append((obs_pos, current_time))
                self.obstacle_history[obs_pos] = new_history

        keys_to_remove = [key for key in self.obstacle_history.keys() if key not in matched_keys]
        for key in keys_to_remove:
            del self.obstacle_history[key]

    def _get_obstacle_velocity(
        self, robot_pose: Pose2D, obstacle: tuple[float, float]
    ) -> tuple[float, float]:
        min_dist = float('inf')
        closest_key = None
        obstacle_match_threshold = float(os.getenv("ORCA_OBSTACLE_MATCH_THRESHOLD", "0.5"))

        for key in self.obstacle_history.keys():
            dist = math.sqrt((obstacle[0] - key[0])**2 + (obstacle[1] - key[1])**2)
            if dist < min_dist and dist < obstacle_match_threshold:
                min_dist = dist
                closest_key = key

        if closest_key is None or closest_key not in self.obstacle_history:
            return (0.0, 0.0)

        history = self.obstacle_history[closest_key]
        if len(history) < 2:
            return (0.0, 0.0)

        (pos1, t1) = history[-2]
        (pos2, t2) = history[-1]

        dt = t2 - t1
        if dt < 1e-6:
            return (0.0, 0.0)

        vx = (pos2[0] - pos1[0]) / dt
        vy = (pos2[1] - pos1[1]) / dt

        speed = math.sqrt(vx**2 + vy**2)
        if speed < 0.05:
            return (0.0, 0.0)

        return (vx, vy)

    def _estimate_other_robot_velocity(self, robot_idx: int) -> tuple[float, float]:
        if robot_idx not in self.other_robot_history:
            return (0.0, 0.0)

        history = self.other_robot_history[robot_idx]
        if len(history) < 2:
            return (0.0, 0.0)

        (pose1, t1) = history[-2]
        (pose2, t2) = history[-1]

        dt = t2 - t1
        if dt < 1e-6:
            return (0.0, 0.0)

        vx = (pose2.x - pose1.x) / dt
        vy = (pose2.y - pose1.y) / dt

        return (vx, vy)
