import os
from dataclasses import dataclass

@dataclass
class DifferentialRobotConfig:
    base_path: str = os.getenv("ROBOT_BASE_PATH", "/Kobuki")
    base_ref_path: str = os.getenv("ROBOT_BASE_REF_PATH", "/Kobuki/turtlebot_base_ref")
    left_motor_path: str = os.getenv("ROBOT_LEFT_MOTOR_PATH", "/Kobuki/kobuki_leftMotor")
    right_motor_path: str = os.getenv("ROBOT_RIGHT_MOTOR_PATH", "/Kobuki/kobuki_rightMotor")

    lidar_path: str = os.getenv("ROBOT_LIDAR_PATH", "/Kobuki/fastHokuyo")

    wheel_radius: float = float(os.getenv("ROBOT_WHEEL_RADIUS", "0.035"))
    wheel_separation: float = float(os.getenv("ROBOT_WHEEL_SEPARATION", "0.23"))
    max_wheel_speed: float = float(os.getenv("ROBOT_MAX_WHEEL_SPEED", "12.0"))
    robot_radius: float = float(os.getenv("ROBOT_RADIUS", "0.175"))

    base_ref_handle: int | None = None
    left_motor_handle: int | None = None
    right_motor_handle: int | None = None
    lidar_handle: int | None = None

    def resolve_handles(self, session) -> None:
        simulation = session.simulation

        self.base_ref_handle = int(simulation.getObject(self.base_ref_path))
        self.left_motor_handle = int(simulation.getObject(self.left_motor_path))
        self.right_motor_handle = int(simulation.getObject(self.right_motor_path))

        try:
            self.lidar_handle = int(simulation.getObject(self.lidar_path))
        except Exception:
            self.lidar_handle = None
