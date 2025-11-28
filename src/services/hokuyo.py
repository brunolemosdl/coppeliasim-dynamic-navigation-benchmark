import math
from typing import Any

class HokuyoSensorSim:
    _vision_sensor_name_template = "{}/sensor{}"
    _angle_min = -120.0 * math.pi / 180.0
    _angle_max = 120.0 * math.pi / 180.0
    _angle_increment = (240.0 / 684.0) * math.pi / 180.0

    def __init__(self, sim: Any, base_name: str, is_range_data: bool = False) -> None:
        if "fastHokuyo" not in base_name:
            raise ValueError("invalid Hokuyo base name")

        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        self._base_obj = self._sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError("base object not found")

        self._vision_sensors_obj = self._init_vision_sensors()
        if any(vs == -1 for vs in self._vision_sensors_obj):
            raise ValueError("vision sensors not found")

    @property
    def is_range_data(self) -> bool:
        return self._is_range_data

    @is_range_data.setter
    def is_range_data(self, is_range_data: bool) -> None:
        self._is_range_data = bool(is_range_data)

    def get_sensor_data(self) -> list[list[float]]:
        angle = self._angle_min
        data: list[list[float]] = []

        for vs in self._vision_sensors_obj:
            angle = self._read_vision_sensor(vs, angle, data)

        return data

    def _init_vision_sensors(self) -> list[int]:
        return [
            self._sim.getObject(self._vision_sensor_name_template.format(self._base_name, 1)),
            self._sim.getObject(self._vision_sensor_name_template.format(self._base_name, 2)),
        ]

    def _get_relative_matrix(self, sensor_obj: int) -> list[float]:
        sensor_m = self._sim.getObjectMatrix(sensor_obj)
        base_m = self._sim.getObjectMatrix(self._base_obj)
        base_m = self._sim.getMatrixInverse(base_m)
        return self._sim.multiplyMatrices(base_m, sensor_m)

    def _read_vision_sensor(
        self,
        vision_sensor_obj: int,
        angle: float,
        data: list[list[float]],
    ) -> float:
        try:
            _, _, u = self._sim.readVisionSensor(vision_sensor_obj)
        except Exception:
            return angle

        if not u:
            return angle

        rel_ref_m = self._get_relative_matrix(vision_sensor_obj)
        pts_y = int(u[1])
        pts_x = int(u[0])

        for j in range(pts_y):
            for k in range(pts_x):
                w = 2 + 4 * (j * pts_x + k)
                v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                angle += self._angle_increment

                if self._is_range_data:
                    data.append([angle, v[3]])
                else:
                    p = self._sim.multiplyVector(rel_ref_m, v)
                    data.append([p[0], p[1], p[2]])

        return angle
