import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.table import Table

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 11,
    'axes.labelsize': 11,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 14,
    'text.usetex': False,
    'axes.linewidth': 0.9,
    'grid.linewidth': 0.5,
    'lines.linewidth': 1.5,
    'patch.linewidth': 0.9,
    'xtick.major.width': 0.9,
    'ytick.major.width': 0.9,
    'axes.grid': True,
    'grid.alpha': 0.25,
    'grid.linestyle': '--',
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
})

PLANNER_STYLES = {
    'dwa': {'color': '#1f77b4', 'linestyle': '-', 'marker': 'o', 'markerfacecolor': '#1f77b4', 'markeredgecolor': '#1f77b4'},
    'teb': {'color': '#ff7f0e', 'linestyle': '--', 'marker': 's', 'markerfacecolor': '#ff7f0e', 'markeredgecolor': '#ff7f0e'},
    'orca': {'color': '#2ca02c', 'linestyle': '-.', 'marker': '^', 'markerfacecolor': '#2ca02c', 'markeredgecolor': '#2ca02c'},
    'pid': {'color': '#d62728', 'linestyle': ':', 'marker': 'D', 'markerfacecolor': '#d62728', 'markeredgecolor': '#d62728'},
}

PLANNER_LABELS = {
    'dwa': 'DWA',
    'teb': 'TEB',
    'orca': 'ORCA',
    'pid': 'PID',
}

SCENE_LABELS = {
    'scene_1': 'Scene 1: Static L-Shaped Corridor',
    'scene_2': 'Scene 2: Head-On Encounter in Straight Corridor',
    'scene_3': 'Scene 3: Cross-Shaped Corridor',
}


def load_results(results_dir: Path) -> List[Dict]:
    results = []
    for json_file in results_dir.rglob('results.json'):
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
                results.append(data)
        except Exception as e:
            print(f"Warning: Could not load {json_file}: {e}")
    return results


def aggregate_data(results: List[Dict]) -> Dict:
    aggregated = defaultdict(lambda: defaultdict(list))

    for result in results:
        scene = result.get('scene', 'unknown')
        planner = result.get('planner', 'unknown')
        metrics = result.get('metrics', {})

        run_data = {
            'success': metrics.get('success', False),
            'time_to_goal': metrics.get('time_to_goal'),
            'path_length': metrics.get('path_length', 0),
            'num_collisions': metrics.get('num_collisions', 0),
            'num_deadlocks': metrics.get('num_deadlocks', 0),
            'min_wall_distance': metrics.get('min_wall_distance'),
            'smoothness_index': metrics.get('smoothness_index', 0),
            'min_robot_distance': metrics.get('min_robot_distance'),
            'min_intersection_distance': metrics.get('min_intersection_distance'),
            'num_near_misses': metrics.get('num_near_misses', 0),
        }
        aggregated[scene][planner].append(run_data)

    return aggregated


def calculate_statistics(data: List[Dict], metric: str) -> Tuple[float, float]:
    values = [
        d[metric]
        for d in data
        if d.get('success')
        and d.get(metric) is not None
        and d.get(metric) != float('inf')
        and not (isinstance(d.get(metric), float) and np.isnan(d.get(metric)))
    ]
    if not values:
        return 0.0, 0.0
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return mean, std


def autolabel(ax, rects, fmt='{:.1f}', offset=3):
    for rect in rects:
        height = rect.get_height()
        ax.annotate(
            fmt.format(height),
            xy=(rect.get_x() + rect.get_width() / 2, height),
            xytext=(0, offset),
            textcoords="offset points",
            ha='center',
            va='bottom',
            fontsize=9,
        )


def plot_scene1_results(aggregated: Dict, output_dir: Path):
    scene = 'scene_1'
    if scene not in aggregated:
        print(f"Warning: No data for {scene}")
        return

    planners = ['dwa', 'teb', 'orca']
    scene_data = aggregated[scene]

    successes = []
    mean_times = []
    std_times = []
    mean_paths = []
    mean_wall_dist = []
    smoothness = []

    for planner in planners:
        if planner not in scene_data:
            successes.append(0)
            mean_times.append(0)
            std_times.append(0)
            mean_paths.append(0)
            mean_wall_dist.append(0)
            smoothness.append(0)
            continue

        data = scene_data[planner]
        success_count = sum(1 for d in data if d.get('success', False))
        successes.append(success_count)

        successful_runs = [d for d in data if d.get('success', False)]
        if successful_runs:
            mean_time, std_time = calculate_statistics(successful_runs, 'time_to_goal')
            mean_path, _ = calculate_statistics(successful_runs, 'path_length')
            mean_wall, _ = calculate_statistics(successful_runs, 'min_wall_distance')
            mean_smooth, _ = calculate_statistics(successful_runs, 'smoothness_index')

            mean_times.append(mean_time)
            std_times.append(std_time)
            mean_paths.append(mean_path)
            mean_wall_dist.append(mean_wall if mean_wall != float('inf') else 0)
            smoothness.append(mean_smooth)
        else:
            mean_times.append(0)
            std_times.append(0)
            mean_paths.append(0)
            mean_wall_dist.append(0)
            smoothness.append(0)

    x = np.arange(len(planners))
    width = 0.6

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    ax1, ax2, ax3, ax4 = axes.flatten()

    success_rate = [s / 20 * 100 for s in successes]
    bars1 = ax1.bar(x, success_rate, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax1.set_title('Success rate')
    ax1.set_ylabel('Success [%]')
    ax1.set_xticks(x)
    ax1.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax1.set_ylim(0, max(success_rate + [100]) * 1.1)
    autolabel(ax1, bars1, fmt='{:.0f}')

    bars2 = ax2.bar(x, mean_times, width, yerr=std_times, capsize=4,
                    color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax2.set_title('Time to goal (successful runs)')
    ax2.set_ylabel('Time [s]')
    ax2.set_xticks(x)
    ax2.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax2.set_ylim(0, max(mean_times + [0]) * 1.2 if any(mean_times) else 1)
    autolabel(ax2, bars2, fmt='{:.1f}')

    bars3 = ax3.bar(x, mean_paths, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax3.set_title('Path length (successful runs)')
    ax3.set_ylabel('Path length [m]')
    ax3.set_xticks(x)
    ax3.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax3.set_ylim(0, max(mean_paths + [0]) * 1.2 if any(mean_paths) else 1)
    autolabel(ax3, bars3, fmt='{:.1f}')

    bars4 = ax4.bar(x, mean_wall_dist, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax4.set_title('Min wall distance (successful runs)')
    ax4.set_ylabel('Distance [m]')
    ax4.set_xticks(x)
    ax4.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax4.set_ylim(0, max(mean_wall_dist + [0]) * 1.2 if any(mean_wall_dist) else 1)
    autolabel(ax4, bars4, fmt='{:.2f}')

    for i, p in enumerate(planners):
        if smoothness[i] != 0:
            ax4.scatter(x[i], mean_wall_dist[i], s=0)

    fig.suptitle(SCENE_LABELS[scene], fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_dir / 'scene1_results.png')
    plt.close(fig)


def plot_scene2_results(aggregated: Dict, output_dir: Path):
    scene = 'scene_2'
    if scene not in aggregated:
        print(f"Warning: No data for {scene}")
        return

    planners = ['dwa', 'teb', 'orca']
    scene_data = aggregated[scene]

    successes = []
    collisions = []
    deadlocks = []
    mean_times = []
    std_times = []
    mean_robot_dist = []

    for planner in planners:
        if planner not in scene_data:
            successes.append(0)
            collisions.append(0)
            deadlocks.append(0)
            mean_times.append(0)
            std_times.append(0)
            mean_robot_dist.append(0)
            continue

        data = scene_data[planner]
        success_count = sum(1 for d in data if d.get('success', False))
        collision_count = sum(d.get('num_collisions', 0) for d in data)
        deadlock_count = sum(1 for d in data if not d.get('success', False) and d.get('num_deadlocks', 0) > 0)

        successes.append(success_count)
        collisions.append(collision_count)
        deadlocks.append(deadlock_count)

        successful_runs = [d for d in data if d.get('success', False)]
        if successful_runs:
            mean_time, std_time = calculate_statistics(successful_runs, 'time_to_goal')
            mean_robot, _ = calculate_statistics(successful_runs, 'min_robot_distance')

            mean_times.append(mean_time)
            std_times.append(std_time)
            mean_robot_dist.append(mean_robot if mean_robot != float('inf') else 0)
        else:
            mean_times.append(0)
            std_times.append(0)
            mean_robot_dist.append(0)

    x = np.arange(len(planners))
    width = 0.6

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    ax1, ax2, ax3, ax4 = axes.flatten()

    success_rate = [s / 20 * 100 for s in successes]
    bars1 = ax1.bar(x, success_rate, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax1.set_title('Success rate')
    ax1.set_ylabel('Success [%]')
    ax1.set_xticks(x)
    ax1.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax1.set_ylim(0, max(success_rate + [100]) * 1.1)
    autolabel(ax1, bars1, fmt='{:.0f}')

    bars2 = ax2.bar(x, collisions, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax2.set_title('Collisions (all runs)')
    ax2.set_ylabel('Collisions / 20')
    ax2.set_xticks(x)
    ax2.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax2.set_ylim(0, max(collisions + [0]) * 1.2 if any(collisions) else 1)
    autolabel(ax2, bars2, fmt='{:.0f}')

    bars3 = ax3.bar(x, deadlocks, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax3.set_title('Deadlocks (all runs)')
    ax3.set_ylabel('Deadlocks / 20')
    ax3.set_xticks(x)
    ax3.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax3.set_ylim(0, max(deadlocks + [0]) * 1.2 if any(deadlocks) else 1)
    autolabel(ax3, bars3, fmt='{:.0f}')

    bars4 = ax4.bar(x, mean_times, width, yerr=std_times, capsize=4,
                    color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax4.set_title('Time and clearance (successful runs)')
    ax4.set_ylabel('Time to goal [s]')
    ax4.set_xticks(x)
    ax4.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax4.set_ylim(0, max(mean_times + [0]) * 1.2 if any(mean_times) else 1)
    autolabel(ax4, bars4, fmt='{:.1f}')

    ax4b = ax4.twinx()
    ax4b.plot(
        x,
        mean_robot_dist,
        linestyle='--',
        marker='o',
        linewidth=1.5,
        color='#111111',
        label='Min robot distance [m]',
    )
    ax4b.set_ylabel('Min robot distance [m]')
    ax4b.set_ylim(0, max(mean_robot_dist + [0]) * 1.3 if any(mean_robot_dist) else 1)
    ax4b.legend(loc='upper right', fontsize=9)

    fig.suptitle(SCENE_LABELS[scene], fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_dir / 'scene2_results.png')
    plt.close(fig)


def plot_scene3_results(aggregated: Dict, output_dir: Path):
    scene = 'scene_3'
    if scene not in aggregated:
        print(f"Warning: No data for {scene}")
        return

    planners = ['dwa', 'teb', 'orca']
    scene_data = aggregated[scene]

    successes = []
    collisions = []
    deadlocks = []
    mean_times = []
    mean_intersection_dist = []
    near_misses = []

    for planner in planners:
        if planner not in scene_data:
            successes.append(0)
            collisions.append(0)
            deadlocks.append(0)
            mean_times.append(0)
            mean_intersection_dist.append(0)
            near_misses.append(0)
            continue

        data = scene_data[planner]
        success_count = sum(1 for d in data if d.get('success', False))
        collision_count = sum(1 for d in data if d.get('num_collisions', 0) > 0)
        deadlock_count = sum(1 for d in data if d.get('num_deadlocks', 0) > 0)

        successes.append(success_count)
        collisions.append(collision_count)
        deadlocks.append(deadlock_count)

        successful_runs = [d for d in data if d.get('success', False)]
        if successful_runs:
            mean_time, _ = calculate_statistics(successful_runs, 'time_to_goal')
            mean_intersection, _ = calculate_statistics(successful_runs, 'min_intersection_distance')
            total_near_misses = sum(d.get('num_near_misses', 0) for d in successful_runs)

            mean_times.append(mean_time)
            mean_intersection_dist.append(mean_intersection if mean_intersection and mean_intersection != float('inf') else 0)
            near_misses.append(total_near_misses)
        else:
            mean_times.append(0)
            mean_intersection_dist.append(0)
            near_misses.append(0)

    x = np.arange(len(planners))
    width = 0.6

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    ax1, ax2, ax3, ax4 = axes.flatten()

    success_rate = [s / 20 * 100 for s in successes]
    bars1 = ax1.bar(x, success_rate, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax1.set_title('Success rate')
    ax1.set_ylabel('Success [%]')
    ax1.set_xticks(x)
    ax1.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax1.set_ylim(0, max(success_rate + [100]) * 1.1)
    autolabel(ax1, bars1, fmt='{:.0f}')

    bars2 = ax2.bar(x, collisions, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax2.set_title('Collisions (all runs)')
    ax2.set_ylabel('Collisions / 20')
    ax2.set_xticks(x)
    ax2.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax2.set_ylim(0, max(collisions + [0]) * 1.2 if any(collisions) else 1)
    autolabel(ax2, bars2, fmt='{:.0f}')

    bars3 = ax3.bar(x, deadlocks, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax3.set_title('Deadlocks (all runs)')
    ax3.set_ylabel('Deadlocks / 20')
    ax3.set_xticks(x)
    ax3.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax3.set_ylim(0, max(deadlocks + [0]) * 1.2 if any(deadlocks) else 1)
    autolabel(ax3, bars3, fmt='{:.0f}')

    bars4 = ax4.bar(x, mean_times, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax4.set_title('Intersection safety (successful runs)')
    ax4.set_ylabel('Time to goal [s]')
    ax4.set_xticks(x)
    ax4.set_xticklabels([PLANNER_LABELS[p] for p in planners])
    ax4.set_ylim(0, max(mean_times + [0]) * 1.2 if any(mean_times) else 1)
    autolabel(ax4, bars4, fmt='{:.1f}')

    ax4b = ax4.twinx()
    ax4b.plot(
        x,
        mean_intersection_dist,
        linestyle='--',
        marker='o',
        linewidth=1.5,
        color='#111111',
        label='Min distance at intersection [m]',
    )
    ax4b.set_ylabel('Min distance at intersection [m]')
    ax4b.set_ylim(0, max(mean_intersection_dist + [0]) * 1.3 if any(mean_intersection_dist) else 1)

    for i, p in enumerate(planners):
        if near_misses[i] > 0:
            ax4b.annotate(
                f'{near_misses[i]} near misses',
                xy=(x[i], mean_intersection_dist[i]),
                xytext=(0, 8),
                textcoords='offset points',
                ha='center',
                va='bottom',
                fontsize=8,
            )

    ax4b.legend(loc='upper right', fontsize=9)

    fig.suptitle(SCENE_LABELS[scene], fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_dir / 'scene3_results.png')
    plt.close(fig)


def plot_overall_comparison(aggregated: Dict, output_dir: Path):
    planners = ['dwa', 'teb', 'orca']

    total_runs = defaultdict(int)
    total_successes = defaultdict(int)
    all_times = defaultdict(list)
    all_paths = defaultdict(list)
    all_clearances = defaultdict(list)

    for scene in ['scene_1', 'scene_2', 'scene_3']:
        if scene not in aggregated:
            continue
        scene_data = aggregated[scene]

        for planner in planners:
            if planner not in scene_data:
                continue
            data = scene_data[planner]
            total_runs[planner] += len(data)
            success_count = sum(1 for d in data if d.get('success', False))
            total_successes[planner] += success_count

            successful_runs = [d for d in data if d.get('success', False)]
            for run in successful_runs:
                if run.get('time_to_goal'):
                    all_times[planner].append(run['time_to_goal'])
                if run.get('path_length'):
                    all_paths[planner].append(run['path_length'])

                if scene == 'scene_1':
                    clearance = run.get('min_wall_distance')
                elif scene == 'scene_2':
                    clearance = run.get('min_robot_distance')
                else:
                    clearance = run.get('min_intersection_distance')

                if clearance and clearance != float('inf'):
                    all_clearances[planner].append(clearance)

    success_rates = []
    mean_times = []
    mean_paths = []
    mean_clearances = []
    labels = []

    for planner in planners:
        runs = total_runs[planner]
        successes = total_successes[planner]
        success_rate = (successes / runs * 100) if runs > 0 else 0

        mean_time = statistics.mean(all_times[planner]) if all_times[planner] else 0
        mean_path = statistics.mean(all_paths[planner]) if all_paths[planner] else 0
        mean_clearance = statistics.mean(all_clearances[planner]) if all_clearances[planner] else 0

        labels.append(PLANNER_LABELS[planner])
        success_rates.append(success_rate)
        mean_times.append(mean_time)
        mean_paths.append(mean_path)
        mean_clearances.append(mean_clearance)

    x = np.arange(len(planners))
    width = 0.6

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    ax1, ax2, ax3 = axes

    bars1 = ax1.bar(x, success_rates, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax1.set_title('Overall success rate')
    ax1.set_ylabel('Success [%]')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.set_ylim(0, max(success_rates + [100]) * 1.1)
    autolabel(ax1, bars1, fmt='{:.0f}')

    bars2 = ax2.bar(x, mean_times, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax2.set_title('Mean time to goal (successful runs)')
    ax2.set_ylabel('Time [s]')
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels)
    ax2.set_ylim(0, max(mean_times + [0]) * 1.2 if any(mean_times) else 1)
    autolabel(ax2, bars2, fmt='{:.1f}')

    bars3 = ax3.bar(x, mean_clearances, width, color=[PLANNER_STYLES[p]['color'] for p in planners])
    ax3.set_title('Mean min clearance (successful runs)')
    ax3.set_ylabel('Clearance [m]')
    ax3.set_xticks(x)
    ax3.set_xticklabels(labels)
    ax3.set_ylim(0, max(mean_clearances + [0]) * 1.2 if any(mean_clearances) else 1)
    autolabel(ax3, bars3, fmt='{:.2f}')

    fig.suptitle('Overall Quantitative Comparison', fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(output_dir / 'overall_comparison.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Generate plots from benchmark results')
    parser.add_argument('--results-dir', type=str, default='results',
                        help='Directory containing results (default: results)')
    parser.add_argument('--output-dir', type=str, default='plots',
                        help='Output directory for plots (default: plots)')

    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not results_dir.exists():
        print(f"Error: Results directory not found: {results_dir}")
        return

    print(f"Loading results from {results_dir}...")
    results = load_results(results_dir)
    print(f"Loaded {len(results)} result files")

    if not results:
        print("No results found. Please run benchmarks first.")
        return

    print("Aggregating data...")
    aggregated = aggregate_data(results)

    print("Generating plots...")
    plot_scene1_results(aggregated, output_dir)
    plot_scene2_results(aggregated, output_dir)
    plot_scene3_results(aggregated, output_dir)
    plot_overall_comparison(aggregated, output_dir)

    print(f"Plots saved to {output_dir}")


if __name__ == '__main__':
    main()
