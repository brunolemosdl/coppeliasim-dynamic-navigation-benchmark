import math
import os

import numpy as np

from services.coppelia import Pose2D
from utils.geometry import laser_to_obstacles
from utils.logger import get_logger

from ..common import BasePlanner, PlannerConfig
from .scoring import calc_clearance_score, calc_heading_score, calc_velocity_score
from .trajectory import check_collision, predict_trajectory

logger = get_logger("algorithms.dwa")

class DWAPlanner(BasePlanner):
    def __init__(self, config: PlannerConfig | None = None):
        super().__init__(config)
        self.max_speed = config.max_linear_vel if config else 0.3
        self.max_angular_speed = config.max_angular_vel if config else 1.0
        self.max_accel = config.max_linear_acc if config else 0.5
        self.max_angular_accel = config.max_angular_acc if config else 1.0
        self.robot_radius = config.robot_radius if config else 0.175
        self.dt = float(os.getenv("DWA_TIME_STEP", os.getenv("TIME_STEP", "0.1")))
        self.predict_time = float(os.getenv("DWA_PREDICT_TIME", os.getenv("PREDICT_TIME", "2.0")))
        self.v_resolution = float(os.getenv("DWA_V_RESOLUTION", "0.02"))
        self.w_resolution = float(os.getenv("DWA_W_RESOLUTION", "0.05"))
        self.alpha = float(os.getenv("DWA_ALPHA", "0.4"))
        self.beta = float(os.getenv("DWA_BETA", "0.5"))
        self.gamma = float(os.getenv("DWA_GAMMA", "0.1"))
        self.current_v = 0.0
        self.current_w = 0.0

    def compute_velocity(
        self,
        robot_pose: Pose2D,
        laser_scan: list[tuple[float, float]],
        other_robots: list[Pose2D] | None = None,
    ) -> tuple[float, float]:
        target = self.current_waypoint
        if target is None:
            target = self.goal
        if target is None:
            return 0.0, 0.0

        obstacles = laser_to_obstacles(robot_pose, laser_scan)

        v_min, v_max, w_min, w_max = self._calc_dynamic_window(
            robot_pose, self.current_v, self.current_w
        )

        dw_min_v = float(os.getenv("DWA_DYNAMIC_WINDOW_MIN_V", "0.01"))
        dw_min_w = float(os.getenv("DWA_DYNAMIC_WINDOW_MIN_W", "0.1"))

        if v_max < dw_min_v or v_max - v_min < 0.05:
            v_min = 0.0
            v_max = self.max_speed
        if abs(w_max - w_min) < dw_min_w:
            w_min = -self.max_angular_speed
            w_max = self.max_angular_speed

        best_v, best_w = self._search_best_velocity(
            robot_pose, obstacles, v_min, v_max, w_min, w_max, other_robots, target
        )

        self.current_v = best_v
        self.current_w = best_w

        return best_v, best_w

    def _calc_dynamic_window(
        self, pose: Pose2D, v: float, w: float
    ) -> tuple[float, float, float, float]:
        vs = [0.0, self.max_speed, -self.max_angular_speed, self.max_angular_speed]

        vd = [
            v - self.max_accel * self.dt,
            v + self.max_accel * self.dt,
            w - self.max_angular_accel * self.dt,
            w + self.max_angular_accel * self.dt,
        ]

        v_min = max(vs[0], vd[0])
        v_max = min(vs[1], vd[1])
        w_min = max(vs[2], vd[2])
        w_max = min(vs[3], vd[3])

        return v_min, v_max, w_min, w_max

    def _search_best_velocity(
        self,
        pose: Pose2D,
        obstacles: list[tuple[float, float]],
        v_min: float,
        v_max: float,
        w_min: float,
        w_max: float,
        other_robots: list[Pose2D] | None = None,
        target: tuple[float, float] | None = None,
    ) -> tuple[float, float]:
        if target is None:
            target = self.goal
        if target is None:
            return 0.0, 0.0

        best_score = float("-inf")
        best_v = 0.0
        best_w = 0.0

        best_escape_score = float("-inf")
        best_escape_v = 0.0
        best_escape_w = 0.0
        best_escape_min_dist = 0.0

        v_range = np.arange(v_min, v_max + self.v_resolution, self.v_resolution)
        w_range = np.arange(w_min, w_max + self.w_resolution, self.w_resolution)

        if len(v_range) == 0 or len(w_range) == 0:
            return 0.0, 0.0

        for v in v_range:
            for w in w_range:
                traj = predict_trajectory(pose, v, w, self.predict_time, self.dt)

                has_collision = check_collision(traj, obstacles, other_robots or [], self.robot_radius)

                if has_collision:
                    min_dist = self._get_min_obstacle_distance(traj, obstacles, other_robots or [])
                    heading_score = calc_heading_score(pose, traj[-1], target)
                    escape_score = min_dist + 0.3 * heading_score

                    if escape_score > best_escape_score:
                        best_escape_score = escape_score
                        best_escape_min_dist = min_dist
                        best_escape_v = v
                        best_escape_w = w
                    continue

                heading_score = calc_heading_score(pose, traj[-1], target)
                clearance_score = calc_clearance_score(
                    traj, obstacles, other_robots, self.robot_radius
                )
                velocity_score = calc_velocity_score(
                    v,
                    w,
                    self.max_speed,
                    self.max_angular_speed,
                    self.current_v,
                    self.current_w,
                )

                score = (
                    self.alpha * heading_score
                    + self.beta * clearance_score
                    + self.gamma * velocity_score
                )

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w

        if best_score == float("-inf"):
            if best_escape_min_dist > 0.0:
                logger.warning(
                    f"DWA: No collision-free trajectories, using safest escape. "
                    f"min_dist={best_escape_min_dist:.3f}m, obstacles={len(obstacles)}, "
                    f"window=[v:{v_min:.2f}-{v_max:.2f}, w:{w_min:.2f}-{w_max:.2f}]"
                )
                return best_escape_v, best_escape_w
            else:
                logger.warning(
                    f"DWA: No valid velocities found! obstacles={len(obstacles)}, "
                    f"window=[v:{v_min:.2f}-{v_max:.2f}, w:{w_min:.2f}-{w_max:.2f}]"
                )
                best_v = 0.0
                best_w = 0.0

        return best_v, best_w

    def _get_min_obstacle_distance(
        self,
        traj: list[tuple[float, float]],
        obstacles: list[tuple[float, float]],
        other_robots: list,
    ) -> float:
        min_dist = float("inf")

        for point in traj:
            for obs in obstacles:
                dist = math.sqrt((point[0] - obs[0]) ** 2 + (point[1] - obs[1]) ** 2)
                min_dist = min(min_dist, dist)

            for robot in other_robots:
                dist = math.sqrt((point[0] - robot.x) ** 2 + (point[1] - robot.y) ** 2)
                min_dist = min(min_dist, dist)

        return min_dist

    def reset(self) -> None:
        super().reset()
        self.current_v = 0.0
        self.current_w = 0.0
