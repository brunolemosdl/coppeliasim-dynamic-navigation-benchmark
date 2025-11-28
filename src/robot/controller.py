import math
import os

from services.coppelia import CoppeliaSimSession, Pose2D
from services.hokuyo import HokuyoSensorSim

from .config import DifferentialRobotConfig

class DifferentialRobotController:
    def __init__(
        self,
        session: CoppeliaSimSession,
        config: DifferentialRobotConfig | None = None,
    ) -> None:
        self.session = session
        self.simulation = self.session.simulation
        self.config = config or DifferentialRobotConfig()
        self.config.resolve_handles(self.session)

        self._hokuyo: HokuyoSensorSim | None = None
        try:
            if self.simulation.getObject(self.config.lidar_path) != -1:
                self._hokuyo = HokuyoSensorSim(self.simulation, self.config.lidar_path)
                self._hokuyo.is_range_data = False
        except Exception:
            self._hokuyo = None

    def get_pose(self) -> Pose2D:
        if self.config.base_ref_handle is None:
            raise RuntimeError("Base ref handle not resolved.")
        return self.session.get_pose2d_world(self.config.base_ref_handle)

    def read_lidar(self) -> list[tuple[float, float]] | None:
        if self.simulation is None:
            return None

        try:
            range_signal = self.simulation.getBufferProperty(
                self.simulation.handle_scene, "signal.hokuyo_range_data"
            )
            angle_signal = self.simulation.getBufferProperty(
                self.simulation.handle_scene, "signal.hokuyo_angle_data"
            )

            if range_signal and angle_signal:
                ranges = self.simulation.unpackFloatTable(range_signal)
                angles = self.simulation.unpackFloatTable(angle_signal)

                if ranges and angles and len(ranges) == len(angles):
                    sensor_points: list[tuple[float, float]] = []
                    max_range = float(os.getenv("LASER_MAX_RANGE", "5.0"))
                    saturation_factor = float(os.getenv("LASER_SATURATION_FACTOR", "0.95"))
                    saturation_threshold = max_range * saturation_factor

                    for dist, angle in zip(ranges, angles, strict=True):
                        if dist > 0 and dist < saturation_threshold:
                            x = dist * math.cos(angle)
                            y = dist * math.sin(angle)
                            sensor_points.append((float(x), float(y)))

                    if sensor_points:
                        return sensor_points
        except Exception:
            pass

        try:
            if self._hokuyo is not None:
                sensor_points_3d = self._hokuyo.get_sensor_data()
                if sensor_points_3d and len(sensor_points_3d) > 0:
                    sensor_points = []
                    max_range = float(os.getenv("LASER_MAX_RANGE", "5.0"))
                    saturation_factor = float(os.getenv("LASER_SATURATION_FACTOR", "0.95"))
                    saturation_threshold = max_range * saturation_factor

                    for point in sensor_points_3d:
                        if len(point) >= 2:
                            x = float(point[0])
                            y = float(point[1])
                            if math.isfinite(x) and math.isfinite(y):
                                dist = math.sqrt(x*x + y*y)
                                if 0 < dist < saturation_threshold:
                                    sensor_points.append((x, y))

                    if sensor_points:
                        return sensor_points
        except Exception:
            pass

        return None

    def set_wheel_speeds(self, left: float, right: float) -> None:
        if self.config.left_motor_handle is None or self.config.right_motor_handle is None:
            raise RuntimeError("Motor handles not resolved.")

        max_wheel_speed = abs(self.config.max_wheel_speed)
        left_velocity = max(-max_wheel_speed, min(max_wheel_speed, float(left)))
        right_velocity = max(-max_wheel_speed, min(max_wheel_speed, float(right)))

        self.simulation.setJointTargetVelocity(self.config.left_motor_handle, left_velocity)
        self.simulation.setJointTargetVelocity(self.config.right_motor_handle, right_velocity)

    def set_velocity(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        wheel_radius = self.config.wheel_radius
        wheel_separation = self.config.wheel_separation

        left_wheel_velocity = (
            linear_velocity - (angular_velocity * wheel_separation) / 2.0
        ) / wheel_radius
        right_wheel_velocity = (
            linear_velocity + (angular_velocity * wheel_separation) / 2.0
        ) / wheel_radius

        self.set_wheel_speeds(left_wheel_velocity, right_wheel_velocity)
        return left_wheel_velocity, right_wheel_velocity

    def stop(self) -> None:
        try:
            self.set_wheel_speeds(0.0, 0.0)
        except Exception:
            pass
