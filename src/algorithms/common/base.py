from abc import ABC, abstractmethod

from services.coppelia import Pose2D

from .config import PlannerConfig

class BasePlanner(ABC):
    def __init__(self, config: PlannerConfig | None = None):
        self.config = config or PlannerConfig()
        self.current_path: list[tuple[float, float]] = []
        self.goal: tuple[float, float] | None = None
        self.current_waypoint: tuple[float, float] | None = None
        self.last_valid_waypoint: tuple[float, float] | None = None

    def set_goal(self, goal: tuple[float, float]) -> None:
        self.goal = goal

    def set_waypoint(self, waypoint: tuple[float, float] | None) -> None:
        if waypoint is not None:
            self.current_waypoint = waypoint
            self.last_valid_waypoint = waypoint
        else:
            self.current_waypoint = None

    def get_goal(self) -> tuple[float, float] | None:
        return self.goal

    def get_waypoint(self) -> tuple[float, float] | None:
        return self.current_waypoint

    def get_path(self) -> list[tuple[float, float]]:
        return self.current_path.copy()

    @abstractmethod
    def compute_velocity(
        self,
        robot_pose: Pose2D,
        laser_scan: list[tuple[float, float]],
        other_robots: list[Pose2D] | None = None,
    ) -> tuple[float, float]:
        pass

    def is_goal_reached(self, robot_pose: Pose2D) -> bool:
        if self.goal is None:
            return False

        dx = self.goal[0] - robot_pose.x
        dy = self.goal[1] - robot_pose.y
        distance = (dx**2 + dy**2) ** 0.5

        return distance < self.config.goal_tolerance

    def reset(self) -> None:
        self.current_path = []
        self.goal = None
        self.current_waypoint = None
        self.last_valid_waypoint = None
