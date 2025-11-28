import argparse
import atexit
import os
import signal
import sys
import time

from algorithms import PlannerConfig, RoutePlanner
from robot.config import DifferentialRobotConfig
from robot.controller import DifferentialRobotController
from utils.logger import setup_logging
from utils.mapper import Mapper
from utils.metrics import CollisionDetector, DeadlockDetector, Metrics
from utils.paths import (
    get_organized_output_dir,
    get_scenario_number,
    resolve_scene_path,
    validate_scene_file,
)
from utils.planners import create_planner
from utils.results import save_results
from utils.session import (
    check_connection,
    get_coppelia_session,
    get_goal_position,
    get_other_robots,
    load_scene,
    reset_coppelia_session,
)
from visualization.plotter import NavigationPlotter

def run_experiment(args, logger, organized_output_dir, timestamp):
    logger.info(
        f"Starting navigation experiment - Algorithm: {args.planner.upper()}, Scene: {args.scene}"
    )

    if not check_connection(args.host, args.port):
        logger.error(f"Connection unavailable at {args.host}:{args.port}")
        sys.exit(1)

    scene_file = resolve_scene_path(args.scene_dir, args.scene)
    load_scene(str(scene_file))
    logger.info(f"Scene loaded: {scene_file}")

    session = get_coppelia_session(args.host, args.port)
    session.start()
    session.wait_until_running()
    logger.info("Simulation started")

    try:
        goal_pos = get_goal_position(session, args.goal_name)
        if goal_pos is None:
            logger.error("Could not find goal in scene. Make sure there's a 'Goal' object.")
            sys.exit(1)

        robot_wheel_radius = float(os.getenv("ROBOT_WHEEL_RADIUS", "0.035"))
        max_wheel_speed_rads = args.max_speed / robot_wheel_radius
        config = DifferentialRobotConfig(
            wheel_radius=robot_wheel_radius,
            max_wheel_speed=max_wheel_speed_rads,
            robot_radius=args.robot_radius,
        )
        controller = DifferentialRobotController(session, config)

        planner_config = PlannerConfig(
            max_linear_vel=args.max_speed,
            max_angular_vel=args.max_ang_speed,
            robot_radius=args.robot_radius,
            goal_tolerance=args.goal_tolerance,
        )

        planner = create_planner(args.planner, planner_config)
        planner.set_goal(goal_pos)
        logger.info(
            f"Planner {args.planner.upper()} configured. Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})"
        )

        mapper = Mapper(session, sensor_path=os.getenv("SENSOR_PATH", "/Sensor"))
        global_planner = RoutePlanner(mapper, robot_radius=args.robot_radius)

        session.step()
        sim_init_sleep = float(os.getenv("SIM_INIT_SLEEP", "0.1"))
        time.sleep(sim_init_sleep)

        robot_pose = controller.get_pose()
        start_point = (robot_pose.x, robot_pose.y)
        goal_point = goal_pos

        if global_planner.initialize():
            if mapper.map_image is not None:
                map_save_path = organized_output_dir / "extracted_map.png"
                mapper.save_map(str(map_save_path))

            global_path = global_planner.plan_path(start_point, goal_point)
            if not global_path:
                logger.warning("Could not plan global path, using direct goal")
                global_planner = None
        else:
            logger.warning("Could not initialize global planner, using direct goal")
            global_planner = None

        metrics = Metrics()
        metrics.start()
        collision_detector = CollisionDetector(robot_radius=args.robot_radius)
        deadlock_detector = DeadlockDetector(
            time_threshold=args.deadlock_time_threshold,
            distance_threshold=args.deadlock_distance_threshold,
        )

        plotter = NavigationPlotter(
            output_dir=str(organized_output_dir),
            plot_interval=args.plot_interval,
            save_trajectory=bool(args.save_trajectory),
            visualize=args.visualize,
            goal_pos=goal_pos,
        )

        if args.visualize:
            if mapper.map_image is not None:
                map_origin = mapper.get_map_origin()
                plotter.set_map(mapper.map_image, map_origin, map_resolution=mapper.map_resolution)

                if global_planner:
                    last_start, last_goal = global_planner.get_last_path_points()
                    if last_start is not None and last_goal is not None:
                        plotter.set_path_points(last_start, last_goal)
                    else:
                        plotter.set_path_points(start_point, goal_point)
                else:
                    plotter.set_path_points(start_point, goal_point)

            if global_planner is not None:
                global_path = global_planner.get_global_path()
                if global_path:
                    plotter.set_global_path(global_path)
                    logger.info(f"Global path configured ({len(global_path)} waypoints)")

        start_time = time.time()
        step_count = 0
        last_position = None
        stuck_counter = 0

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time > args.sim_time:
                logger.warning(f"Simulation time limit reached ({args.sim_time}s)")
                break

            robot_pose = controller.get_pose()
            metrics.update_position(robot_pose)

            if global_planner is not None:
                lookahead_multiplier = float(os.getenv("LOOKAHEAD_MULTIPLIER", "2.0"))
                next_waypoint = global_planner.get_next_waypoint(
                    robot_pose, lookahead_distance=args.goal_tolerance * lookahead_multiplier
                )
                planner.set_waypoint(next_waypoint if next_waypoint else None)

            if planner.is_goal_reached(robot_pose):
                metrics.record_goal_reached()
                logger.info("Goal reached")
                logger.info(f"Time: {metrics.time_to_goal:.2f}s")
                logger.info(f"Path length: {metrics.path_length:.2f}m")
                logger.info(f"Collisions: {metrics.num_collisions}")
                logger.info(f"Deadlocks: {metrics.num_deadlocks}")
                break

            raw_laser_points = controller.read_lidar()
            laser_scan_robot_frame = []

            if raw_laser_points:
                saturation_factor = float(os.getenv("LASER_SATURATION_FACTOR", "0.95"))
                saturation_threshold = args.laser_max_range * saturation_factor

                for point_x, point_y in raw_laser_points:
                    distance = (point_x**2 + point_y**2) ** 0.5
                    if 0 < distance <= saturation_threshold:
                        laser_scan_robot_frame.append((point_x, point_y))

            if collision_detector.check_collision(laser_scan_robot_frame):
                metrics.record_collision()
                if metrics.num_collisions == 1 or metrics.num_collisions % 5 == 0:
                    logger.warning(f"Collision detected (total: {metrics.num_collisions})")

            other_robots = get_other_robots(session)

            if laser_scan_robot_frame:
                min_dist = collision_detector.get_min_distance(laser_scan_robot_frame)
                metrics.update_wall_distance(min_dist)

                if other_robots:
                    for other_robot in other_robots:
                        dist = (
                            (robot_pose.x - other_robot.x) ** 2
                            + (robot_pose.y - other_robot.y) ** 2
                        ) ** 0.5
                        metrics.update_robot_distance(dist)

                metrics.update_intersection_distance(min_dist)
                if min_dist < metrics.near_miss_threshold:
                    metrics.check_near_miss(min_dist)

            if deadlock_detector.check_deadlock(robot_pose):
                metrics.record_deadlock()
                if metrics.num_deadlocks == 1 or metrics.num_deadlocks % 3 == 0:
                    logger.warning(f"Deadlock detected (total: {metrics.num_deadlocks})")

            linear_vel, angular_vel = planner.compute_velocity(
                robot_pose,
                laser_scan_robot_frame,
                other_robots,
            )

            planned_path = planner.get_path()
            if global_planner is not None:
                global_path = global_planner.get_global_path()
                if global_path:
                    planned_path = global_path

            plotter.update(
                robot_pose,
                laser_scan_robot_frame,
                planned_path,
                metrics,
            )

            controller.set_velocity(linear_vel, angular_vel)

            if last_position:
                stuck_distance_threshold = float(os.getenv("STUCK_DISTANCE_THRESHOLD", "0.05"))
                dist_moved = (
                    (robot_pose.x - last_position[0]) ** 2 + (robot_pose.y - last_position[1]) ** 2
                ) ** 0.5
                if dist_moved < stuck_distance_threshold:
                    stuck_counter += 1
                    stuck_counter_threshold = int(os.getenv("STUCK_COUNTER_THRESHOLD", "100"))
                    if stuck_counter > stuck_counter_threshold:
                        logger.warning("Robot stuck detected, ending experiment")
                        logger.warning(f"Final position: ({robot_pose.x:.2f}, {robot_pose.y:.2f})")
                        break
                else:
                    stuck_counter = 0

            last_position = (robot_pose.x, robot_pose.y)

            session.step()
            sim_step_sleep = float(os.getenv("SIM_STEP_SLEEP", "0.05"))
            time.sleep(sim_step_sleep)

            step_count += 1

            log_interval = int(os.getenv("LOG_INTERVAL", "500"))
            if args.verbose and step_count % log_interval == 0:
                logger.debug(
                    f"Step {step_count}: pos=({robot_pose.x:.2f}, {robot_pose.y:.2f}), "
                    f"vel=({linear_vel:.2f}, {angular_vel:.2f}), "
                    f"collisions={metrics.num_collisions}, deadlocks={metrics.num_deadlocks}, "
                    f"time={elapsed_time:.1f}s"
                )

        controller.stop()
        session.stop()
        session.wait_until_stopped()
        logger.info("Simulation finished")

        metrics.calculate_smoothness()

        scenario = get_scenario_number(args.scene)
        save_results(metrics, plotter, args, logger, scenario, organized_output_dir, timestamp)

    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        controller.stop()
        session.stop()
        session.wait_until_stopped()
        raise
    except Exception as e:
        logger.error(f"Error during experiment: {e}", exc_info=True)
        try:
            controller.stop()
            session.stop()
            session.wait_until_stopped()
        except Exception:
            pass
        raise

def main():
    parser = argparse.ArgumentParser(description="Local Planner Benchmark: PID, DWA, TEB, ORCA")

    parser.add_argument(
        "--scene",
        required=True,
        help="Scene name (scene_1, scene_2, scene_3)",
    )
    parser.add_argument(
        "--planner",
        required=True,
        choices=["pid", "dwa", "teb", "orca"],
        help="Planner type",
    )
    parser.add_argument("--scene-dir", default="./scenes", help="Scene directory")
    parser.add_argument("--host", default="localhost", help="CoppeliaSim host")
    parser.add_argument("--port", type=int, default=23000, help="CoppeliaSim port")
    parser.add_argument("--goal-name", default=os.getenv("GOAL_NAME", "/Goal"), help="Goal object name in scene")

    parser.add_argument("--sim-time", type=float, default=300.0, help="Simulation time (seconds)")
    parser.add_argument(
        "--laser-max-range",
        type=float,
        default=5.0,
        help="Laser maximum range (m)",
    )

    parser.add_argument("--max-speed", type=float, default=0.30, help="Max linear speed (m/s)")
    parser.add_argument(
        "--max-ang-speed",
        type=float,
        default=1.00,
        help="Max angular speed (rad/s)",
    )
    parser.add_argument("--robot-radius", type=float, default=0.175, help="Robot radius (m)")
    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.2,
        help="Goal tolerance (m)",
    )

    parser.add_argument(
        "--deadlock-time-threshold",
        type=float,
        default=5.0,
        help="Time threshold for deadlock detection (s)",
    )
    parser.add_argument(
        "--deadlock-distance-threshold",
        type=float,
        default=0.1,
        help="Distance threshold for deadlock detection (m)",
    )

    parser.add_argument("--output-dir", default="./results", help="Output directory")
    parser.add_argument("--plot-interval", type=int, default=5, help="Plot update interval")
    parser.add_argument(
        "--save-trajectory",
        type=int,
        default=1,
        help="Save trajectory plot (0/1)",
    )
    parser.add_argument("--save-logs", type=int, default=1, help="Save logs (0/1)")

    parser.add_argument("--visualize", action="store_true", help="Show plots interactively")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose logging")

    args = parser.parse_args()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    organized_output_dir = get_organized_output_dir(
        args.output_dir, args.scene, args.planner, timestamp
    )

    logger = setup_logging(
        verbose=args.verbose,
        log_to_file=bool(args.save_logs),
        log_dir=str(organized_output_dir),
        timestamp=timestamp,
    )

    if not validate_scene_file(args.scene_dir, args.scene):
        scene_path = resolve_scene_path(args.scene_dir, args.scene)
        logger.error(f"Scene not found: {scene_path}")
        sys.exit(1)

    get_coppelia_session(args.host, args.port)

    def _cleanup():
        try:
            reset_coppelia_session()
        except Exception:
            pass

    atexit.register(_cleanup)

    def _signal_handler(signum=None, frame=None):
        try:
            _cleanup()
        finally:
            sys.exit(1)

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, _signal_handler)
        except Exception:
            pass

    try:
        run_experiment(args, logger, organized_output_dir, timestamp)
    except Exception as e:
        logger.error(f"Experiment failed: {e}", exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()
