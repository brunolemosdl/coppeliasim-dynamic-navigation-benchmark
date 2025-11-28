import heapq
import math
import os

import numpy as np

from services.coppelia import Pose2D
from utils.logger import get_logger
from utils.mapper import Mapper

logger = get_logger("algorithms.route")

class RoutePlanner:
    def __init__(self, mapper: Mapper, robot_radius: float = 0.175):
        self.mapper = mapper
        self.robot_radius = robot_radius
        self.obstacle_map: np.ndarray | None = None
        self.global_path: list[tuple[float, float]] = []
        self.current_waypoint_index: int = 0
        self.last_start: tuple[float, float] | None = None
        self.last_goal: tuple[float, float] | None = None

    def initialize(self) -> bool:
        if not self.mapper.initialize():
            logger.warning("Failed to initialize mapper")
            return False

        raw_map = self.mapper.extract_map()
        if raw_map is None:
            logger.warning("Failed to extract map from sensor")
            return False

        self.obstacle_map = self.mapper.process_map(self.robot_radius)

        if self.obstacle_map is None:
            logger.warning("Failed to process map")
            return False

        return True

    def plan_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]] | None:
        self.last_start = start
        self.last_goal = goal

        if self.obstacle_map is None:
            return None

        height, width = self.obstacle_map.shape

        start_map = self.mapper.world_to_map(start[0], start[1])
        goal_map = self.mapper.world_to_map(goal[0], goal[1])

        start_x, start_y = int(start_map[0]), int(start_map[1])
        goal_x, goal_y = int(goal_map[0]), int(goal_map[1])

        start_x = max(0, min(width - 1, start_x))
        start_y = max(0, min(height - 1, start_y))
        goal_x = max(0, min(width - 1, goal_x))
        goal_y = max(0, min(height - 1, goal_y))

        map_threshold = int(os.getenv("MAP_OBSTACLE_THRESHOLD", "128"))
        if self.obstacle_map[start_y, start_x] < map_threshold:
            logger.warning("Start position is in obstacle area")
            return None
        if self.obstacle_map[goal_y, goal_x] < map_threshold:
            logger.warning("Goal position is in obstacle area")
            return None

        path_map = self._astar((start_x, start_y), (goal_x, goal_y), width, height)

        if not path_map or len(path_map) < 2:
            return None

        world_path = []
        for map_x, map_y in path_map:
            world_x, world_y = self.mapper.map_to_world(map_x, map_y)
            world_path.append((world_x, world_y))

        self.global_path = world_path
        self.current_waypoint_index = 0

        return world_path

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _get_neighbors(
        self, node: tuple[int, int], width: int, height: int
    ) -> list[tuple[int, int]]:
        x, y = node
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    map_threshold = int(os.getenv("MAP_OBSTACLE_THRESHOLD", "128"))
                    if self.obstacle_map[ny, nx] >= map_threshold:
                        neighbors.append((nx, ny))
        return neighbors

    def _astar(
        self, start: tuple[int, int], goal: tuple[int, int], width: int, height: int
    ) -> list[tuple[int, int]] | None:
        open_set = [(0, start)]
        came_from: dict = {}
        g_score: dict = {start: 0}
        f_score: dict = {start: self._heuristic(start, goal)}
        visited: set[tuple[int, int]] = set()

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self._get_neighbors(current, width, height):
                if neighbor in visited:
                    continue

                tentative_g = g_score[current] + self._heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def get_next_waypoint(
        self, robot_pose: Pose2D, lookahead_distance: float = 1.0
    ) -> tuple[float, float] | None:
        if not self.global_path:
            return None

        waypoint_threshold = float(
            os.getenv("ROUTE_WAYPOINT_THRESHOLD", os.getenv("WAYPOINT_THRESHOLD", "0.05"))
        )

        while self.current_waypoint_index < len(self.global_path):
            wp = self.global_path[self.current_waypoint_index]
            dx = wp[0] - robot_pose.x
            dy = wp[1] - robot_pose.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < waypoint_threshold:
                self.current_waypoint_index += 1
            else:
                break

        if self.current_waypoint_index >= len(self.global_path):
            return None

        best_wp = self.global_path[self.current_waypoint_index]

        i = self.current_waypoint_index + 1
        while i < len(self.global_path):
            wp = self.global_path[i]
            dx = wp[0] - robot_pose.x
            dy = wp[1] - robot_pose.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > lookahead_distance:
                break

            best_wp = wp
            self.current_waypoint_index = i
            i += 1

        return best_wp

    def get_global_path(self) -> list[tuple[float, float]]:
        return self.global_path.copy()

    def get_last_path_points(
        self,
    ) -> tuple[tuple[float, float] | None, tuple[float, float] | None]:
        return (self.last_start, self.last_goal)

    def reset(self) -> None:
        self.global_path = []
        self.current_waypoint_index = 0
        self.last_start = None
        self.last_goal = None
