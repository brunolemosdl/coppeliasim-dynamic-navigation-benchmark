import math

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def laser_to_obstacles(
    robot_pose, laser_scan: list[tuple[float, float]], min_distance: float = 0.05
) -> list[tuple[float, float]]:
    if not laser_scan:
        return []

    obstacles = []
    cos_theta = math.cos(robot_pose.theta)
    sin_theta = math.sin(robot_pose.theta)

    filtered_points = []
    for point in laser_scan:
        x_robot, y_robot = point
        dist = math.sqrt(x_robot**2 + y_robot**2)

        if dist < 0.1:
            continue

        too_close = False
        for fx, fy in filtered_points:
            dx = x_robot - fx
            dy = y_robot - fy
            if math.sqrt(dx*dx + dy*dy) < min_distance:
                too_close = True
                break

        if not too_close:
            filtered_points.append((x_robot, y_robot))

    for point in filtered_points:
        x_robot, y_robot = point
        x_world = robot_pose.x + x_robot * cos_theta - y_robot * sin_theta
        y_world = robot_pose.y + x_robot * sin_theta + y_robot * cos_theta
        obstacles.append((x_world, y_world))

    return obstacles

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll_term_1 = +2.0 * (w * x + y * z)
    roll_term_2 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(roll_term_1, roll_term_2)

    pitch_term = +2.0 * (w * y - z * x)
    pitch_term = +1.0 if pitch_term > +1.0 else pitch_term
    pitch_term = -1.0 if pitch_term < -1.0 else pitch_term
    pitch = math.asin(pitch_term)

    yaw_term_1 = +2.0 * (w * z + x * y)
    yaw_term_2 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(yaw_term_1, yaw_term_2)

    return roll, pitch, yaw
