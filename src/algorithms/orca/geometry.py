import math

def compute_orca_line(
    pos_a: tuple[float, float],
    pos_b: tuple[float, float],
    vel_a: tuple[float, float],
    vel_b: tuple[float, float],
    radius_a: float,
    radius_b: float,
    time_horizon: float = 2.0,
    reciprocal: bool = False,
) -> tuple[tuple[float, float], float] | None:
    relative_pos = (pos_b[0] - pos_a[0], pos_b[1] - pos_a[1])
    relative_vel = (vel_b[0] - vel_a[0], vel_b[1] - vel_a[1])
    dist_sq = relative_pos[0] ** 2 + relative_pos[1] ** 2
    combined_radius = radius_a + radius_b
    combined_radius_sq = combined_radius**2

    if dist_sq < combined_radius_sq:
        dist = math.sqrt(dist_sq) if dist_sq > 1e-6 else 1e-6
        normal = (relative_pos[0] / dist, relative_pos[1] / dist)
        return (normal, -0.5)

    w = (
        relative_vel[0] - relative_pos[0] / time_horizon,
        relative_vel[1] - relative_pos[1] / time_horizon,
    )

    w_length_sq = w[0] ** 2 + w[1] ** 2
    dist = math.sqrt(dist_sq)
    radius_over_tau = combined_radius / time_horizon

    if dist < 1e-6:
        return None

    if w_length_sq < 1e-6:
        n = (
            relative_pos[0] / dist,
            relative_pos[1] / dist,
        )
        u_length = radius_over_tau
    else:
        w_length = math.sqrt(w_length_sq)
        dot = (relative_pos[0] * w[0] + relative_pos[1] * w[1]) / (dist * w_length)

        u = (
            (dot * radius_over_tau / dist - w_length) * (w[0] / w_length) if w_length > 1e-6 else 0.0,
            (dot * radius_over_tau / dist - w_length) * (w[1] / w_length) if w_length > 1e-6 else 0.0,
        )

        u_length = math.sqrt(u[0] ** 2 + u[1] ** 2)
        if u_length < 1e-6:
            n = (
                relative_pos[0] / dist,
                relative_pos[1] / dist,
            )
            u_length = radius_over_tau
        else:
            n = (u[0] / u_length, u[1] / u_length)

    if reciprocal:
        offset = 0.5 * u_length
    else:
        offset = u_length

    return (n, offset)

def linear_program2d(
    lines: list[tuple[tuple[float, float], float]],
    radius: float,
    opt_velocity: tuple[float, float],
    direction_opt: bool = False,
) -> tuple[float, float] | None:
    if not lines:
        speed = math.sqrt(opt_velocity[0] ** 2 + opt_velocity[1] ** 2)
        if speed <= radius:
            return opt_velocity
        return (
            (opt_velocity[0] / speed) * radius,
            (opt_velocity[1] / speed) * radius,
        )

    result = list(opt_velocity)

    max_iterations = 10
    for iteration in range(max_iterations):
        changed = False

        for normal, offset in lines:
            dot = result[0] * normal[0] + result[1] * normal[1]
            if dot > offset + 1e-6:
                result[0] = result[0] + (offset - dot) * normal[0]
                result[1] = result[1] + (offset - dot) * normal[1]
                changed = True

        speed = math.sqrt(result[0] ** 2 + result[1] ** 2)
        if speed > radius + 1e-6:
            if speed < 1e-6:
                return None
            result[0] = (result[0] / speed) * radius
            result[1] = (result[1] / speed) * radius
            changed = True

        all_satisfied = True
        for normal, offset in lines:
            dot = result[0] * normal[0] + result[1] * normal[1]
            if dot > offset + 1e-6:
                all_satisfied = False
                break

        if all_satisfied and speed <= radius + 1e-6:
            break

        if not changed:
            break

    speed = math.sqrt(result[0] ** 2 + result[1] ** 2)
    if speed > radius + 1e-6:
        result[0] = (result[0] / speed) * radius
        result[1] = (result[1] / speed) * radius

    return tuple(result)

def linear_program1(
    velocity: list[float],
    line_dir: tuple[float, float],
    line_offset: float,
) -> list[float] | None:
    dot = velocity[0] * line_dir[0] + velocity[1] * line_dir[1]

    if dot <= line_offset + 1e-6:
        return velocity

    new_vel = [
        velocity[0] + (line_offset - dot) * line_dir[0],
        velocity[1] + (line_offset - dot) * line_dir[1],
    ]

    return new_vel
