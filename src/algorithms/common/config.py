import os
from dataclasses import dataclass

@dataclass
class PlannerConfig:
    max_linear_vel: float = 0.3
    max_angular_vel: float = 1.0
    min_linear_vel: float = 0.0
    min_angular_vel: float = -1.0
    max_linear_acc: float = float(os.getenv("MAX_LINEAR_ACC", "0.6"))
    max_angular_acc: float = float(os.getenv("MAX_ANGULAR_ACC", "1.5"))
    robot_radius: float = 0.175
    goal_tolerance: float = 0.2
    lookahead_distance: float = float(os.getenv("LOOKAHEAD_DISTANCE", "1.0"))
