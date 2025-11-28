import math
import os

from services.coppelia import Pose2D
from utils.geometry import normalize_angle

from ..common import BasePlanner, PlannerConfig


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, max_integral: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = float(os.getenv("TIME_STEP", "0.1"))

    def update(self, error: float, dt: float | None = None) -> float:
        """Compute PID output."""
        if dt is None:
            dt = self.dt

        if dt <= 0.0:
            dt = self.dt


        p_term = self.kp * error

        self.integral += error * dt
        if self.max_integral > 0:
            self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative

        self.prev_error = error

        return p_term + i_term + d_term

    def reset(self) -> None:
        """Reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0


class PIDPlanner(BasePlanner):
    """PID-based local planner for robot navigation."""

    def __init__(self, config: PlannerConfig | None = None):
        super().__init__(config)
        self.max_linear_vel = config.max_linear_vel if config else 0.3
        self.max_angular_vel = config.max_angular_vel if config else 1.0


        self.linear_kp = float(os.getenv("PID_LINEAR_KP", "1.5"))
        self.linear_ki = float(os.getenv("PID_LINEAR_KI", "0.0"))
        self.linear_kd = float(os.getenv("PID_LINEAR_KD", "0.1"))
        self.angular_kp = float(os.getenv("PID_ANGULAR_KP", "2.0"))
        self.angular_ki = float(os.getenv("PID_ANGULAR_KI", "0.0"))
        self.angular_kd = float(os.getenv("PID_ANGULAR_KD", "0.2"))

        self.linear_pid = PIDController(self.linear_kp, self.linear_ki, self.linear_kd)
        self.angular_pid = PIDController(self.angular_kp, self.angular_ki, self.angular_kd)
        self.dt = float(os.getenv("TIME_STEP", "0.1"))

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

        dx = target[0] - robot_pose.x
        dy = target[1] - robot_pose.y
        distance_error = math.sqrt(dx**2 + dy**2)

        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - robot_pose.theta)

        linear_error = distance_error
        linear_vel_raw = self.linear_pid.update(linear_error, self.dt)
        linear_vel = max(0.0, min(self.max_linear_vel, linear_vel_raw))

        angular_vel_raw = self.angular_pid.update(heading_error, self.dt)
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel_raw))

        if distance_error < self.config.goal_tolerance * 2:
            linear_vel *= 0.5
            if abs(heading_error) > 0.1:
                angular_vel *= 1.5

        return linear_vel, angular_vel

    def reset(self) -> None:
        """Reset planner state."""
        super().reset()
        self.linear_pid.reset()
        self.angular_pid.reset()

