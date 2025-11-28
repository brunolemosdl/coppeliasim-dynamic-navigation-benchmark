from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from services.coppelia import Pose2D
from utils.metrics import Metrics

class NavigationPlotter:
    def __init__(
        self,
        output_dir: str,
        plot_interval: int = 5,
        save_trajectory: bool = True,
        visualize: bool = False,
        goal_pos: tuple[float, float] | None = None,
    ):
        self.output_dir = Path(output_dir)
        self.plot_interval = plot_interval
        self.save_trajectory = save_trajectory
        self.visualize = visualize
        self.goal_pos = goal_pos

        self.trajectory: list[tuple[float, float]] = []
        self.laser_readings: list[tuple[float, float]] = []
        self.map_image: np.ndarray | None = None
        self.map_origin: tuple[float, float] = (0.0, 0.0)
        self.map_resolution: float = 0.05
        self.global_path: list[tuple[float, float]] = []
        self.path_start: tuple[float, float] | None = None
        self.path_goal: tuple[float, float] | None = None

        self.fig = None
        self.ax = None
        self.ax_map = None
        self.step_count = 0

        if self.visualize:
            self._setup_plot()

    def _setup_plot(self) -> None:
        if not self.visualize:
            return

        plt.ion()
        self.fig, (self.ax, self.ax_map) = plt.subplots(1, 2, figsize=(20, 10))

        self.ax.set_title("Navigation Experiment - Real-time")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.3)
        self.ax.axis("equal")

        self.ax_map.set_title("Global Path Planning")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.axis("equal")

        plt.tight_layout()

    def set_map(
        self,
        map_image: np.ndarray | None,
        map_origin: tuple[float, float],
        map_resolution: float = 0.05,
    ) -> None:
        self.map_image = map_image
        self.map_origin = map_origin
        self.map_resolution = map_resolution

    def set_global_path(self, global_path: list[tuple[float, float]]) -> None:
        self.global_path = global_path

    def set_path_points(
        self, start: tuple[float, float] | None, goal: tuple[float, float] | None
    ) -> None:
        self.path_start = start
        self.path_goal = goal

    def update(
        self,
        robot_pose: Pose2D,
        laser_scan: list[tuple[float, float]] | None = None,
        planned_path: list[tuple[float, float]] | None = None,
        metrics: Metrics | None = None,
    ) -> None:
        self.step_count += 1

        self.trajectory.append((robot_pose.x, robot_pose.y))

        if laser_scan:
            cos_theta = np.cos(robot_pose.theta)
            sin_theta = np.sin(robot_pose.theta)
            for point in laser_scan:
                x_robot, y_robot = point
                x_world = robot_pose.x + x_robot * cos_theta - y_robot * sin_theta
                y_world = robot_pose.y + x_robot * sin_theta + y_robot * cos_theta
                self.laser_readings.append((x_world, y_world))

        if self.visualize and self.step_count % self.plot_interval == 0:
            self._update_plot(robot_pose, planned_path, metrics)

    def _update_plot(
        self,
        robot_pose: Pose2D,
        planned_path: list[tuple[float, float]] | None = None,
        metrics: Metrics | None = None,
    ) -> None:
        if not self.visualize or self.fig is None:
            return

        self.ax.clear()
        if self.ax_map:
            self.ax_map.clear()

        if self.map_image is not None and self.ax_map:
            height, width = self.map_image.shape
            extent = [
                self.map_origin[0],
                self.map_origin[0] + width * self.map_resolution,
                self.map_origin[1],
                self.map_origin[1] + height * self.map_resolution,
            ]
            self.ax_map.imshow(
                self.map_image,
                extent=extent,
                cmap="gray",
                alpha=0.7,
                origin="upper",
                interpolation="nearest",
                zorder=0,
            )

        if self.global_path and len(self.global_path) > 0:
            global_x = [p[0] for p in self.global_path]
            global_y = [p[1] for p in self.global_path]

            if self.ax_map:
                self.ax_map.plot(
                    global_x,
                    global_y,
                    "m-",
                    linewidth=3,
                    alpha=0.8,
                    label="Global Path",
                    zorder=1,
                )
                self.ax_map.scatter(
                    [global_x[0]],
                    [global_y[0]],
                    c="green",
                    s=200,
                    marker="o",
                    label="Path Start",
                    zorder=10,
                    edgecolors="black",
                    linewidths=2,
                )
                if self.goal_pos:
                    self.ax_map.scatter(
                        [self.goal_pos[0]],
                        [self.goal_pos[1]],
                        c="red",
                        s=300,
                        marker="*",
                        label="Goal",
                        zorder=10,
                        edgecolors="black",
                        linewidths=2,
                    )

        if self.trajectory:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            self.ax.plot(traj_x, traj_y, "b-", linewidth=2, alpha=0.7, label="Trajectory")

            self.ax.scatter(
                [traj_x[0]],
                [traj_y[0]],
                c="green",
                s=200,
                marker="o",
                label="Start",
                zorder=10,
                edgecolors="black",
                linewidths=2,
            )

        if self.goal_pos:
            self.ax.scatter(
                [self.goal_pos[0]],
                [self.goal_pos[1]],
                c="red",
                s=300,
                marker="*",
                label="Goal",
                zorder=10,
                edgecolors="black",
                linewidths=2,
            )

        self.ax.scatter(
            [robot_pose.x],
            [robot_pose.y],
            c="blue",
            s=150,
            marker="s",
            label="Robot",
            zorder=9,
            edgecolors="black",
            linewidths=2,
        )

        arrow_length = 0.3
        dx = arrow_length * np.cos(robot_pose.theta)
        dy = arrow_length * np.sin(robot_pose.theta)
        self.ax.arrow(
            robot_pose.x,
            robot_pose.y,
            dx,
            dy,
            head_width=0.1,
            head_length=0.1,
            fc="blue",
            ec="blue",
            zorder=9,
        )

        if planned_path and len(planned_path) > 0:
            path_x = [p[0] for p in planned_path]
            path_y = [p[1] for p in planned_path]
            self.ax.plot(path_x, path_y, "g--", linewidth=2, alpha=0.8, label="Planned Path")

        if self.laser_readings:
            recent_readings = self.laser_readings[-500:]
            laser_x = [p[0] for p in recent_readings]
            laser_y = [p[1] for p in recent_readings]
            self.ax.scatter(
                laser_x,
                laser_y,
                c="orange",
                s=1,
                alpha=0.3,
                label="Laser",
            )

        self.ax.set_title(f"Navigation Experiment (Step {self.step_count})")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.3)
        self.ax.axis("equal")
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(handles, labels, loc="upper right")

        if self.ax_map:
            self.ax_map.set_title("Global Path Planning")
            self.ax_map.set_xlabel("X (m)")
            self.ax_map.set_ylabel("Y (m)")
            self.ax_map.grid(True, alpha=0.3)
            self.ax_map.axis("equal")
            handles_map, labels_map = self.ax_map.get_legend_handles_labels()
            if handles_map:
                self.ax_map.legend(handles_map, labels_map, loc="upper right")

        if metrics:
            metrics_text = (
                f"Step: {self.step_count}\n"
                f"Collisions: {metrics.num_collisions}\n"
                f"Deadlocks: {metrics.num_deadlocks}\n"
                f"Path Length: {metrics.path_length:.2f}m\n"
            )

            if metrics.time_to_goal:
                metrics_text += f"Time to Goal: {metrics.time_to_goal:.2f}s\n"
            else:
                metrics_text += "Time to Goal: N/A\n"

            if metrics.min_wall_distance != float("inf"):
                metrics_text += f"Min Wall Dist: {metrics.min_wall_distance:.2f}m\n"

            if metrics.min_robot_distance != float("inf"):
                metrics_text += f"Min Robot Dist: {metrics.min_robot_distance:.2f}m\n"

            if metrics.num_near_misses > 0:
                metrics_text += f"Near Misses: {metrics.num_near_misses}\n"

            self.ax.text(
                0.02,
                0.98,
                metrics_text,
                transform=self.ax.transAxes,
                fontsize=10,
                verticalalignment="top",
                bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.8},
            )

        plt.draw()
        plt.pause(0.01)

    def save_trajectory_plot(self, filename: str = "trajectory.png") -> Path | None:
        if not self.trajectory or not self.save_trajectory:
            return None

        self.output_dir.mkdir(parents=True, exist_ok=True)
        output_path = self.output_dir / filename

        fig, ax = plt.subplots(figsize=(12, 10))

        if self.trajectory:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            ax.plot(traj_x, traj_y, "b-", linewidth=2, alpha=0.7, label="Trajectory")

            ax.scatter(
                [traj_x[0]],
                [traj_y[0]],
                c="green",
                s=200,
                marker="o",
                label="Start",
                zorder=10,
                edgecolors="black",
                linewidths=2,
            )

        if self.goal_pos:
            ax.scatter(
                [self.goal_pos[0]],
                [self.goal_pos[1]],
                c="red",
                s=300,
                marker="*",
                label="Goal",
                zorder=10,
                edgecolors="black",
                linewidths=2,
            )

        if self.trajectory:
            ax.scatter(
                [traj_x[-1]],
                [traj_y[-1]],
                c="blue",
                s=200,
                marker="s",
                label="End",
                zorder=10,
                edgecolors="black",
                linewidths=2,
            )

        ax.set_title("Robot Trajectory")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")
        ax.legend()

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        return output_path

    def close(self) -> None:
        if self.visualize and self.fig is not None:
            plt.close(self.fig)
            plt.ioff()
