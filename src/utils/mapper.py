import os

import cv2
import numpy as np

from services.coppelia import CoppeliaSimSession
from utils.logger import get_logger

logger = get_logger("utils.mapper")

class Mapper:
    def __init__(self, session: CoppeliaSimSession, sensor_path: str = "/Sensor"):
        self.session = session
        self.simulation = session.simulation
        self.sensor_path = sensor_path
        self.sensor_handle: int | None = None
        self.map_image: np.ndarray | None = None
        self.map_resolution: float = 0.0
        self.map_origin: tuple[float, float] = (0.0, 0.0)

    def initialize(self) -> bool:
        try:
            self.sensor_handle = self.simulation.getObject(self.sensor_path)
            if self.sensor_handle == -1:
                logger.warning(f"Sensor not found: {self.sensor_path}")
                return False
            return True
        except Exception as e:
            logger.warning(f"Error initializing sensor {self.sensor_path}: {e}")
            return False

    def extract_map(self) -> np.ndarray | None:
        if self.sensor_handle is None or self.sensor_handle == -1:
            logger.warning("Invalid sensor handle when extracting map")
            return None

        try:
            self.session.step()

            try:
                image_buffer, resolution = self.simulation.getVisionSensorImg(self.sensor_handle)
                width, height = resolution

                if width <= 0 or height <= 0:
                    logger.warning(f"Invalid image dimensions: {width}x{height}")
                    return None

                image_array = np.frombuffer(image_buffer, dtype=np.uint8).reshape(height, width, 3)

                image_gray = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
                image_gray = cv2.flip(image_gray, 1)

                _, binary_image = cv2.threshold(image_gray, 128, 255, cv2.THRESH_BINARY)

                self.map_image = binary_image

                self._calculate_map_parameters(width, height)

                return binary_image

            except AttributeError:
                sensor_result = self.simulation.readVisionSensor(self.sensor_handle)
                if len(sensor_result) < 2:
                    return None
                result = sensor_result[0]
                if result == 0 or result == -1:
                    return None

                packet1 = sensor_result[1] if len(sensor_result) > 1 else None
                packet2 = sensor_result[2] if len(sensor_result) > 2 else None

                image_data = (
                    packet2
                    if packet2 is not None
                    else (packet1 if packet1 is not None and len(packet1) > 1 else None)
                )
                if image_data is None or len(image_data) == 0:
                    return None

                image_array = np.array(image_data, dtype=np.uint8)

                if packet1 is not None and len(packet1) >= 2:
                    if isinstance(packet1[0], (list, tuple)) and len(packet1[0]) >= 2:
                        width = int(packet1[0][0])
                        height = int(packet1[0][1])
                    else:
                        width = int(packet1[0])
                        height = int(packet1[1])
                else:
                    if len(image_array.shape) == 2:
                        height, width = image_array.shape
                    elif len(image_array.shape) == 3:
                        height, width, _ = image_array.shape
                    else:
                        size = int(np.sqrt(len(image_array)))
                        width = height = size if size * size == len(image_array) else 0

                if width <= 0 or height <= 0:
                    return None

                if len(image_array.shape) == 1:
                    if len(image_array) == width * height * 3:
                        image_array = image_array.reshape((height, width, 3))
                    elif len(image_array) == width * height:
                        image_array = image_array.reshape((height, width))

                if len(image_array.shape) == 3:
                    image_gray = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
                else:
                    image_gray = image_array

                map_threshold = int(os.getenv("MAP_OBSTACLE_THRESHOLD", "128"))
                _, binary_image = cv2.threshold(image_gray, map_threshold, 255, cv2.THRESH_BINARY)
                self.map_image = binary_image
                self._calculate_map_parameters(width, height)
                return binary_image

        except Exception as e:
            logger.warning(f"Error processing sensor data: {e}")
            return None

    def _calculate_map_parameters(self, width: int, height: int) -> None:
        try:
            ortho_size = float(os.getenv("SENSOR_ORTHO_SIZE", "12.0"))

            self.map_resolution = ortho_size / max(width, height)

            sensor_position = self.simulation.getObjectPosition(
                self.sensor_handle, self.simulation.handle_world
            )
            sensor_x = float(sensor_position[0])
            sensor_y = float(sensor_position[1])

            map_width_m = width * self.map_resolution
            map_height_m = height * self.map_resolution

            self.map_origin = (
                sensor_x - map_width_m / 2.0,
                sensor_y - map_height_m / 2.0,
            )
        except Exception as e:
            logger.warning(f"Could not get map parameters: {e}")
            self.map_resolution = 1.0
            self.map_origin = (0.0, 0.0)

    def get_map_origin(self) -> tuple[float, float]:
        return self.map_origin

    def process_map(self, robot_radius: float) -> np.ndarray | None:
        if self.map_image is None:
            return None

        map_threshold = int(os.getenv("MAP_OBSTACLE_THRESHOLD", "128"))
        obstacles = (self.map_image <= map_threshold).astype(np.uint8) * 255

        dilation_radius_pixels = round((3 * robot_radius) / self.map_resolution)

        if dilation_radius_pixels > 0:
            kernel_size = 2 * dilation_radius_pixels + 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )
            dilated_obstacles = cv2.dilate(obstacles, kernel, iterations=1)
        else:
            dilated_obstacles = obstacles

        free_space = 255 - dilated_obstacles
        return free_space

    def _get_resolution(self, resolution: float | None) -> float:
        if resolution is None:
            resolution = self.map_resolution
        return resolution if resolution > 0 else 0.05

    def _invert_y(self, map_y: int) -> int:
        if self.map_image is not None:
            height = self.map_image.shape[0]
            return height - 1 - map_y
        return map_y

    def world_to_map(self, x: float, y: float, resolution: float | None = None) -> tuple[int, int]:
        resolution = self._get_resolution(resolution)
        origin_x, origin_y = self.get_map_origin()

        map_x = int((x - origin_x) / resolution)
        map_y_raw = int((y - origin_y) / resolution)
        map_y = self._invert_y(map_y_raw)

        return (map_x, map_y)

    def map_to_world(
        self, map_x: int, map_y: int, resolution: float | None = None
    ) -> tuple[float, float]:
        resolution = self._get_resolution(resolution)
        origin_x, origin_y = self.get_map_origin()

        map_y_raw = self._invert_y(map_y)
        world_x = origin_x + map_x * resolution
        world_y = origin_y + map_y_raw * resolution

        return (world_x, world_y)

    def save_map(self, filepath: str) -> bool:
        if self.map_image is None:
            logger.warning("No map image to save")
            return False

        try:
            directory = os.path.dirname(filepath)
            if directory:
                os.makedirs(directory, exist_ok=True)
            cv2.imwrite(filepath, self.map_image)
            logger.info(f"Map saved to {filepath}")
            return True
        except Exception as e:
            logger.warning(f"Error saving map to {filepath}: {e}")
            return False
