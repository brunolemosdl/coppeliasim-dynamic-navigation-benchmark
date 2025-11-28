import json
from pathlib import Path

from utils.metrics import Metrics
from visualization.plotter import NavigationPlotter

def save_results(
    metrics: Metrics,
    plotter: NavigationPlotter,
    args,
    logger,
    scenario: int,
    organized_output_dir: Path,
    timestamp: str,
):
    if args.save_trajectory:
        plotter.save_trajectory_plot("trajectory.png")

    results = {
        "scene": args.scene,
        "planner": args.planner,
        "scenario": scenario,
        "timestamp": timestamp,
        "metrics": metrics.to_dict(),
        "parameters": {
            "max_speed": args.max_speed,
            "max_ang_speed": args.max_ang_speed,
            "robot_radius": args.robot_radius,
            "goal_tolerance": args.goal_tolerance,
            "sim_time": args.sim_time,
            "laser_max_range": args.laser_max_range,
        },
    }

    results_file = organized_output_dir / "results.json"
    with open(results_file, "w") as f:
        json.dump(results, f, indent=2)

    logger.info(f"Results saved to: {results_file}")
    time_str = f"{metrics.time_to_goal:.2f}s" if metrics.time_to_goal else "N/A"
    logger.info(
        f"Experiment summary - Success: {'Yes' if metrics.success else 'No'}, "
        f"Time: {time_str}, Path length: {metrics.path_length:.2f}m, "
        f"Collisions: {metrics.num_collisions}, Deadlocks: {metrics.num_deadlocks}"
    )

    if args.verbose:
        if scenario == 1:
            logger.debug(f"Min wall distance: {metrics.min_wall_distance:.2f}m")
            logger.debug(f"Smoothness: {metrics.smoothness_index:.3f}")
        elif scenario == 2:
            logger.debug(f"Min robot distance: {metrics.min_robot_distance:.2f}m")
        elif scenario == 3:
            logger.debug(f"Min intersection distance: {metrics.min_intersection_distance:.2f}m")
            logger.debug(f"Near misses: {metrics.num_near_misses}")

    plotter.close()
