import math
import os

from services.coppelia import Pose2D
from utils.geometry import laser_to_obstacles, normalize_angle

from ..common import BasePlanner, PlannerConfig
from .optimization import optimize_band

class TEBPlanner(BasePlanner):
    def __init__(self, config: PlannerConfig | None = None):
        super().__init__(config)
        self.max_speed = config.max_linear_vel if config else 0.3
        self.max_angular_speed = config.max_angular_vel if config else 1.0
        self.robot_radius = config.robot_radius if config else 0.175
        self.horizon = int(os.getenv("TEB_HORIZON", "12"))
        self.dt = float(os.getenv("TEB_TIME_STEP", os.getenv("TIME_STEP", "0.1")))
        self.optimization_iterations = int(os.getenv("TEB_OPTIMIZATION_ITERATIONS", "10"))

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

        if not self.current_path:
            self._initialize_band(robot_pose, target)
        else:
            self._update_band(robot_pose, laser_scan, other_robots, target)

        if len(self.current_path) < 2:
            if target:
                dx = target[0] - robot_pose.x
                dy = target[1] - robot_pose.y
                distance = math.sqrt(dx**2 + dy**2)
                if distance < 0.1:
                    return 0.0, 0.0
                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - robot_pose.theta)
                linear_vel = min(self.max_speed * 0.3, distance * 0.5)
                angular_vel = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, 1.5 * heading_error),
                )
                return linear_vel, angular_vel
            return 0.0, 0.0

        next_waypoint = self.current_path[0]
        dx = next_waypoint[0] - robot_pose.x
        dy = next_waypoint[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        obstacles = laser_to_obstacles(robot_pose, laser_scan)
        min_obstacle_dist = float("inf")
        for obs in obstacles:
            dist_to_obs = math.sqrt((robot_pose.x - obs[0]) ** 2 + (robot_pose.y - obs[1]) ** 2)
            min_obstacle_dist = min(min_obstacle_dist, dist_to_obs)

        speed_factor = 1.0
        collision_threshold = float(os.getenv("COLLISION_THRESHOLD", "0.15"))
        critical_dist = self.robot_radius + collision_threshold
        teb_min_speed_factor = float(os.getenv("TEB_MIN_SPEED_FACTOR", "0.2"))
        if min_obstacle_dist < critical_dist:
            speed_factor = max(teb_min_speed_factor, (min_obstacle_dist - self.robot_radius) / collision_threshold)

        waypoint_threshold = float(os.getenv("TEB_WAYPOINT_THRESHOLD", os.getenv("WAYPOINT_THRESHOLD", "0.05")))
        while distance < waypoint_threshold and len(self.current_path) > 1:
            self.current_path.pop(0)
            next_waypoint = self.current_path[0]
            dx = next_waypoint[0] - robot_pose.x
            dy = next_waypoint[1] - robot_pose.y
            distance = math.sqrt(dx**2 + dy**2)

        if distance < waypoint_threshold:
            if target:
                dx = target[0] - robot_pose.x
                dy = target[1] - robot_pose.y
                distance = math.sqrt(dx**2 + dy**2)
                if distance < waypoint_threshold:
                    return 0.0, 0.0
                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - robot_pose.theta)
                linear_vel = min(self.max_speed, distance)
                angular_vel = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, 2.5 * heading_error / self.dt),
                )
                return linear_vel, angular_vel
            return 0.0, 0.0

        target_angle = math.atan2(dy, dx)
        heading_error = normalize_angle(target_angle - robot_pose.theta)

        linear_vel = self.max_speed

        teb_heading_threshold_high = float(os.getenv("TEB_HEADING_THRESHOLD_HIGH", "0.8"))
        teb_heading_threshold_low = float(os.getenv("TEB_HEADING_THRESHOLD_LOW", "0.5"))
        teb_heading_speed_factor_high = float(os.getenv("TEB_HEADING_SPEED_FACTOR_HIGH", "0.5"))
        teb_heading_speed_factor_low = float(os.getenv("TEB_HEADING_SPEED_FACTOR_LOW", "0.8"))

        if abs(heading_error) > teb_heading_threshold_high:
            linear_vel *= teb_heading_speed_factor_high
        elif abs(heading_error) > teb_heading_threshold_low:
            linear_vel *= teb_heading_speed_factor_low

        linear_vel *= speed_factor

        angular_vel = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, 2.5 * heading_error / self.dt),
        )

        return linear_vel, angular_vel

    def _initialize_band(
        self, robot_pose: Pose2D, target: tuple[float, float] | None = None
    ) -> None:
        if target is None:
            target = self.goal
        if target is None:
            return

        self.current_path = []
        dx = target[0] - robot_pose.x
        dy = target[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            self.current_path.append((target[0], target[1]))
            return

        num_points = max(1, min(self.horizon - 1, int(distance / 0.5)))

        for i in range(1, num_points + 1):
            t = i / float(num_points + 1)
            x = robot_pose.x + t * dx
            y = robot_pose.y + t * dy
            self.current_path.append((x, y))

        self.current_path.append((target[0], target[1]))

    def _update_band(
        self,
        robot_pose: Pose2D,
        laser_scan: list[tuple[float, float]],
        other_robots: list[Pose2D] | None = None,
        target: tuple[float, float] | None = None,
    ) -> None:
        if target is None:
            target = self.goal
        if not self.current_path or target is None:
            return

        obstacles = laser_to_obstacles(robot_pose, laser_scan)

        for _ in range(self.optimization_iterations):
            optimize_band(
                self.current_path,
                robot_pose,
                target,
                obstacles,
                other_robots,
                self.robot_radius,
            )
