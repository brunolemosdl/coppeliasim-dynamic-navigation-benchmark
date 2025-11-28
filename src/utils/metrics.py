import math
import os
import time
from dataclasses import dataclass, field

from services.coppelia import Pose2D

@dataclass
class Metrics:
    success: bool = False
    time_to_goal: float | None = None
    path_length: float = 0.0
    num_collisions: int = 0
    num_deadlocks: int = 0
    min_wall_distance: float = float("inf")
    smoothness_index: float = 0.0
    min_robot_distance: float = float("inf")
    min_intersection_distance: float = float("inf")
    num_near_misses: int = 0
    near_miss_threshold: float = float(os.getenv("NEAR_MISS_THRESHOLD", "0.25"))
    trajectory: list[tuple[float, float]] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    start_time: float | None = None
    goal_reached_time: float | None = None

    def start(self) -> None:
        self.start_time = time.time()
        self.trajectory = []
        self.timestamps = []

    def update_position(self, pose: Pose2D) -> None:
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time

        self.trajectory.append((pose.x, pose.y))
        self.timestamps.append(current_time)

        if len(self.trajectory) > 1:
            prev_x, prev_y = self.trajectory[-2]
            dx = pose.x - prev_x
            dy = pose.y - prev_y
            self.path_length += math.sqrt(dx**2 + dy**2)

    def record_collision(self) -> None:
        self.num_collisions += 1

    def record_deadlock(self) -> None:
        self.num_deadlocks += 1

    def record_goal_reached(self) -> None:
        if not self.success:
            self.success = True
            if self.start_time is not None:
                self.goal_reached_time = time.time()
                self.time_to_goal = self.goal_reached_time - self.start_time

    def update_wall_distance(self, distance: float) -> None:
        if distance < self.min_wall_distance:
            self.min_wall_distance = distance

    def update_robot_distance(self, distance: float) -> None:
        if distance < self.min_robot_distance:
            self.min_robot_distance = distance

    def update_intersection_distance(self, distance: float) -> None:
        if distance < self.min_intersection_distance:
            self.min_intersection_distance = distance

    def check_near_miss(self, distance: float) -> None:
        if distance < self.near_miss_threshold:
            self.num_near_misses += 1

    def calculate_smoothness(self) -> float:
        if len(self.trajectory) < 3:
            return 0.0

        total_curvature = 0.0
        for i in range(1, len(self.trajectory) - 1):
            x1, y1 = self.trajectory[i - 1]
            x2, y2 = self.trajectory[i]
            x3, y3 = self.trajectory[i + 1]

            v1 = (x2 - x1, y2 - y1)
            v2 = (x3 - x2, y3 - y2)

            len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
            len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

            if len_v1 > 0 and len_v2 > 0:
                dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (len_v1 * len_v2)
                dot = max(-1.0, min(1.0, dot))
                angle = math.acos(dot)
                total_curvature += angle

        self.smoothness_index = total_curvature / (len(self.trajectory) - 2)
        return self.smoothness_index

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "time_to_goal": self.time_to_goal,
            "path_length": self.path_length,
            "num_collisions": self.num_collisions,
            "num_deadlocks": self.num_deadlocks,
            "min_wall_distance": self.min_wall_distance
            if self.min_wall_distance != float("inf")
            else None,
            "smoothness_index": self.smoothness_index,
            "min_robot_distance": self.min_robot_distance
            if self.min_robot_distance != float("inf")
            else None,
            "min_intersection_distance": self.min_intersection_distance
            if self.min_intersection_distance != float("inf")
            else None,
            "num_near_misses": self.num_near_misses,
        }

class CollisionDetector:
    def __init__(self, robot_radius: float = 0.175, collision_threshold: float | None = None):
        self.robot_radius = robot_radius
        self.collision_threshold = collision_threshold or float(os.getenv("COLLISION_THRESHOLD", "0.15"))
        self._in_collision = False

    def check_collision(self, laser_scan: list[tuple[float, float]]) -> bool:
        if not laser_scan:
            self._in_collision = False
            return False

        is_colliding = False
        for point in laser_scan:
            distance = math.sqrt(point[0] ** 2 + point[1] ** 2)
            if distance < self.robot_radius + self.collision_threshold:
                is_colliding = True
                break

        if is_colliding and not self._in_collision:
            self._in_collision = True
            return True
        elif not is_colliding:
            self._in_collision = False

        return False

    def get_min_distance(self, laser_scan: list[tuple[float, float]]) -> float:
        if not laser_scan:
            return float("inf")

        min_dist = float("inf")
        for point in laser_scan:
            distance = math.sqrt(point[0] ** 2 + point[1] ** 2)
            min_dist = min(min_dist, distance)

        return min_dist

class DeadlockDetector:
    def __init__(self, time_threshold: float = 5.0, distance_threshold: float = 0.1):
        self.time_threshold = time_threshold
        self.distance_threshold = distance_threshold
        self.last_position: tuple[float, float] | None = None
        self.last_time: float | None = None

    def check_deadlock(self, pose: Pose2D) -> bool:
        current_time = time.time()
        current_pos = (pose.x, pose.y)

        if self.last_position is None:
            self.last_position = current_pos
            self.last_time = current_time
            return False

        distance = math.sqrt(
            (current_pos[0] - self.last_position[0]) ** 2
            + (current_pos[1] - self.last_position[1]) ** 2
        )

        if distance < self.distance_threshold:
            if self.last_time is not None:
                elapsed = current_time - self.last_time
                if elapsed > self.time_threshold:
                    return True
        else:
            self.last_position = current_pos
            self.last_time = current_time

        return False
