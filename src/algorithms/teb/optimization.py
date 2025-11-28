import math
import os

from services.coppelia import Pose2D

def optimize_band(
    current_path: list[tuple[float, float]],
    robot_pose: Pose2D,
    goal: tuple[float, float] | None,
    obstacles: list[tuple[float, float]],
    other_robots: list[Pose2D] | None,
    robot_radius: float,
) -> None:
    if len(current_path) < 2:
        return

    teb_opt_iterations = int(os.getenv("TEB_OPT_ITERATIONS", "5"))
    base_learning_rate = float(os.getenv("TEB_OPT_BASE_LEARNING_RATE", "0.08"))

    for iteration in range(teb_opt_iterations):
        learning_rate = base_learning_rate * (1.0 - 0.5 * iteration / float(teb_opt_iterations))
        for i in range(1, len(current_path) - 1):
            x, y = current_path[i]

            goal_gradient_x = 0.0
            goal_gradient_y = 0.0
            if goal:
                goal_weight = float(os.getenv("TEB_OPT_GOAL_WEIGHT", "0.5"))
                dx_goal = goal[0] - x
                dy_goal = goal[1] - y
                dist_goal = math.sqrt(dx_goal**2 + dy_goal**2)
                if dist_goal > 0.01:
                    goal_gradient_x = goal_weight * (dx_goal / dist_goal)
                    goal_gradient_y = goal_weight * (dy_goal / dist_goal)

            obs_gradient_x = 0.0
            obs_gradient_y = 0.0
            teb_opt_safety_dist = float(os.getenv("TEB_OPT_SAFETY_DISTANCE", "0.25"))
            teb_opt_obstacle_range = float(os.getenv("TEB_OPT_OBSTACLE_RANGE", "1.5"))
            teb_opt_obstacle_weight = float(os.getenv("TEB_OPT_OBSTACLE_WEIGHT", "0.7"))

            for obs in obstacles:
                dx_obs = x - obs[0]
                dy_obs = y - obs[1]
                dist_obs = math.sqrt(dx_obs**2 + dy_obs**2)
                safety_dist = robot_radius + teb_opt_safety_dist

                if dist_obs < safety_dist + teb_opt_obstacle_range:
                    if dist_obs < 0.01:
                        dist_obs = 0.01
                    repulsion_weight = teb_opt_obstacle_weight
                    repulsion = repulsion_weight / max(0.05, (dist_obs - safety_dist + 0.15))
                    obs_gradient_x += repulsion * (dx_obs / dist_obs)
                    obs_gradient_y += repulsion * (dy_obs / dist_obs)

            if other_robots:
                teb_opt_robot_safety = float(os.getenv("TEB_OPT_ROBOT_SAFETY", "0.3"))
                teb_opt_robot_range = float(os.getenv("TEB_OPT_ROBOT_RANGE", "2.0"))
                teb_opt_robot_weight = float(os.getenv("TEB_OPT_ROBOT_WEIGHT", "0.8"))

                for robot in other_robots:
                    dx_robot = x - robot.x
                    dy_robot = y - robot.y
                    dist_robot = math.sqrt(dx_robot**2 + dy_robot**2)
                    safety_dist = 2 * robot_radius + teb_opt_robot_safety

                    if dist_robot < safety_dist + teb_opt_robot_range:
                        if dist_robot < 0.01:
                            dist_robot = 0.01
                        repulsion_weight = teb_opt_robot_weight
                        repulsion = repulsion_weight / max(0.05, (dist_robot - safety_dist + 0.15))
                        obs_gradient_x += repulsion * (dx_robot / dist_robot)
                        obs_gradient_y += repulsion * (dy_robot / dist_robot)

            smoothness_weight = float(os.getenv("TEB_OPT_SMOOTHNESS_WEIGHT", "0.25"))
            if i > 0 and i < len(current_path) - 1:
                prev_x, prev_y = current_path[i - 1]
                next_x, next_y = current_path[i + 1]
                smooth_gradient_x = smoothness_weight * (prev_x + next_x - 2 * x)
                smooth_gradient_y = smoothness_weight * (prev_y + next_y - 2 * y)
            else:
                smooth_gradient_x = 0.0
                smooth_gradient_y = 0.0

            x += learning_rate * (goal_gradient_x + obs_gradient_x + smooth_gradient_x)
            y += learning_rate * (goal_gradient_y + obs_gradient_y + smooth_gradient_y)

            current_path[i] = (x, y)
