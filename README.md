<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->

<a id="readme-top"></a>

<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <h3 align="center">CoppeliaSim Dynamic Navigation Benchmark</h3>

  <p align="center">
    A comprehensive benchmark framework for evaluating local navigation algorithms in dynamic environments using CoppeliaSim
    <br />
    <a href="https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#how-it-works">How It Works</a></li>
    <li><a href="#project-structure">Project Structure</a></li>
    <li><a href="#algorithms">Algorithms</a></li>
    <li><a href="#metrics">Metrics</a></li>
    <li><a href="#troubleshooting">Troubleshooting</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

This project provides a comprehensive benchmarking framework for evaluating local navigation algorithms in dynamic environments using CoppeliaSim. It implements and compares four popular local navigation algorithms:

- **DWA (Dynamic Window Approach)**: A velocity-based local planner that searches for collision-free velocities in a dynamic window
- **TEB (Timed Elastic Band)**: An optimization-based planner that deforms a trajectory to avoid obstacles
- **ORCA (Optimal Reciprocal Collision Avoidance)**: A velocity-based method for multi-agent collision avoidance
- **PID Controller**: A simple proportional-integral-derivative controller for comparison

The framework includes three challenging test scenarios with static and dynamic obstacles, enabling systematic performance evaluation across different navigation challenges.

### Key Features

- 🎯 **Multiple Algorithms**: Compare DWA, TEB, ORCA, and PID planners
- 📊 **Comprehensive Metrics**: Track success rate, path length, time to goal, collisions, deadlocks, and more
- 🎨 **Visualization**: Real-time plotting and trajectory visualization
- 🔄 **Automated Benchmarking**: Run multiple iterations across all algorithms and scenes
- 📈 **Statistical Analysis**: Generate comparative plots and performance reports
- 🛠️ **Easy Configuration**: Flexible parameters via Makefile or command-line arguments

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![Python][Python.org]][Python-url]
* [![NumPy][NumPy.org]][NumPy-url]
* [![Matplotlib][Matplotlib.org]][Matplotlib-url]
* [![OpenCV][OpenCV.org]][OpenCV-url]
* [![CoppeliaSim][CoppeliaSim.org]][CoppeliaSim-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

This section will guide you through setting up the project on your local machine.

### Prerequisites

Before you begin, ensure you have the following installed:

1. **Python 3.8+**
   ```sh
   python3 --version
   ```

2. **CoppeliaSim** (Edu or Pro version)
   - Download from [CoppeliaSim website](https://www.coppeliarobotics.com/downloads)
   - Ensure CoppeliaSim is installed and accessible

3. **Make** (optional, for using Makefile commands)
   - Linux/Mac: Usually pre-installed
   - Windows: Install via WSL, MinGW, or use Git Bash

### Installation

1. **Clone the repository**
   ```sh
   git clone https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.git
   cd coppeliasim-dynamic-navigation-benchmark
   ```

2. **Install Python dependencies**
   ```sh
   make setup
   ```
   Or manually:
   ```sh
   pip install -r requirements.txt
   ```

3. **Verify dependencies**
   ```sh
   make check-deps
   ```

4. **Start CoppeliaSim**
   - Launch CoppeliaSim
   - Ensure the remote API server is running (default port: 23000)
   - You can test the connection with:
     ```sh
     make test-connection
     ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE -->
## Usage

### Quick Start

Run a single experiment:
```sh
make run SCENE=scene_1 PLANNER=dwa
```

### Basic Commands

**Run a single simulation:**
```sh
make run SCENE=scene_1 PLANNER=dwa VISUALIZE=1
```

**Run with custom parameters:**
```sh
make run SCENE=scene_2 PLANNER=teb SIM_TIME=300 MAX_SPEED=0.30
```

**Run full benchmark (20 iterations per algorithm per scene):**
```sh
make benchmark
```

**Generate plots from results:**
```sh
make plots
```

### Available Scenes

- `scene_1`: Static L-Shaped Corridor
- `scene_2`: Head-On Encounter in Straight Corridor
- `scene_3`: Cross-Shaped Corridor

### Available Planners

- `dwa`: Dynamic Window Approach
- `teb`: Timed Elastic Band
- `orca`: Optimal Reciprocal Collision Avoidance
- `pid`: PID Controller

### Configuration Parameters

Key parameters you can customize:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SCENE` | `scene_1` | Scene to run (scene_1, scene_2, scene_3) |
| `PLANNER` | `dwa` | Planner algorithm (dwa, teb, orca, pid) |
| `SIM_TIME` | `300` | Maximum simulation time in seconds |
| `MAX_SPEED` | `0.30` | Maximum linear speed (m/s) |
| `MAX_ANG_SPEED` | `1.00` | Maximum angular speed (rad/s) |
| `ROBOT_RADIUS` | `0.175` | Robot radius in meters |
| `GOAL_TOLERANCE` | `0.2` | Goal tolerance in meters |
| `HOST` | `localhost` | CoppeliaSim host address |
| `PORT` | `23000` | CoppeliaSim port |
| `VISUALIZE` | `0` | Enable real-time visualization (0/1) |
| `VERBOSE` | `0` | Enable verbose logging (0/1) |

### Advanced Usage

**Using Python directly:**
```sh
python src/main.py \
  --scene scene_1 \
  --planner dwa \
  --sim-time 300 \
  --max-speed 0.30 \
  --visualize
```

**Custom output directory:**
```sh
make run SCENE=scene_1 PLANNER=dwa RESULTS_PATH=./my_results
```

**Custom scene directory:**
```sh
make run SCENE=scene_1 PLANNER=dwa SCENE_PATH=./my_scenes
```

### Makefile Commands

The project includes a comprehensive Makefile for easy operation:

| Command | Description |
|---------|-------------|
| `make help` | Show help message with all available options |
| `make setup` | Install Python dependencies |
| `make check-deps` | Verify all dependencies are installed |
| `make test-connection` | Test connection to CoppeliaSim |
| `make run` | Run a single simulation (requires SCENE and PLANNER) |
| `make benchmark` | Run full benchmark (20 iterations per algorithm per scene) |
| `make plots` | Generate comparative plots from results |
| `make status` | Show current configuration and latest results |
| `make clean` | Remove results and temporary files |

**Example:**
```sh
make help              # See all options
make setup             # Install dependencies
make test-connection   # Verify CoppeliaSim connection
make run SCENE=scene_1 PLANNER=dwa VISUALIZE=1
make plots             # Generate plots after running experiments
```

### Viewing Results

Results are saved in `results/` directory, organized by scene and planner:
```
results/
├── scene_1/
│   ├── dwa/
│   │   └── YYYYMMDD_HHMMSS/
│   │       ├── results.json
│   │       ├── trajectory.png
│   │       └── logs/
│   ├── teb/
│   └── orca/
└── ...
```

Generate comparative plots:
```sh
make plots
```

Plots will be saved in the `plots/` directory.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- HOW IT WORKS -->
## How It Works

The benchmark framework follows this workflow:

### 1. **Scene Loading**
- Loads a CoppeliaSim scene file (`.ttt`) containing the environment
- Extracts map information using vision sensors
- Identifies the goal position and initial robot pose

### 2. **Global Path Planning**
- Uses A* algorithm to compute a global path from start to goal
- Extracts waypoints along the path for local planner guidance
- Falls back to direct goal navigation if path planning fails

### 3. **Local Navigation Loop**
The main navigation loop runs at each simulation step:

1. **Sensor Reading**: Reads LiDAR data to detect obstacles
2. **State Update**: Updates robot pose and metrics
3. **Collision Detection**: Checks for collisions and near misses
4. **Deadlock Detection**: Monitors for stuck situations
5. **Velocity Computation**: The selected planner computes desired velocities
6. **Robot Control**: Applies velocities to the robot
7. **Visualization**: Updates plots if visualization is enabled
8. **Metrics Collection**: Records performance metrics

### 4. **Termination Conditions**
The simulation stops when:
- Goal is reached (success)
- Maximum simulation time is exceeded
- Robot gets stuck (deadlock detection)
- User interruption (Ctrl+C)

### 5. **Results Collection**
After each run:
- Metrics are calculated (path length, time, collisions, etc.)
- Trajectory plots are generated
- Results are saved in JSON format
- Logs are recorded (if enabled)

### 6. **Benchmark Analysis**
The `generate_plots.py` script:
- Aggregates results from multiple runs
- Computes statistics (mean, std, median)
- Generates comparative visualizations
- Creates performance tables

### Architecture

```
┌─────────────────┐
│  CoppeliaSim    │
│   (Simulator)   │
└────────┬────────┘
         │
         │ (Remote API)
         │
┌────────▼─────────────────────────────┐
│         Main Controller              │
│  ┌─────────────────────────────────┐ │
│  │  Robot Controller               │ │
│  │  - Pose tracking                │ │
│  │  - Velocity control             │ │
│  │  - LiDAR reading                │ │
│  └─────────────────────────────────┘ │
│  ┌─────────────────────────────────┐ │
│  │  Planner (DWA/TEB/ORCA/PID)     │ │
│  │  - Velocity computation         │ │
│  │  - Obstacle avoidance           │ │
│  └─────────────────────────────────┘ │
│  ┌─────────────────────────────────┐ │
│  │  Metrics Collector              │ │
│  │  - Success tracking             │ │
│  │  - Collision detection          │ │
│  │  - Deadlock detection           │ │
│  └─────────────────────────────────┘ │
│  ┌─────────────────────────────────┐ │
│  │  Visualizer                     │ │
│  │  - Real-time plots              │ │
│  │  - Trajectory visualization     │ │
│  └─────────────────────────────────┘ │
└──────────────────────────────────────┘
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- PROJECT STRUCTURE -->
## Project Structure

```
coppeliasim-dynamic-navigation-benchmark/
├── src/
│   ├── algorithms/         # Navigation algorithms
│   │   ├── dwa/            # Dynamic Window Approach
│   │   ├── teb/            # Timed Elastic Band
│   │   ├── orca/           # Optimal Reciprocal Collision Avoidance
│   │   ├── pid/            # PID Controller
│   │   ├── route/          # Global path planning
│   │   └── common/         # Common interfaces and configs
│   ├── robot/              # Robot controller and configuration
│   ├── services/           # CoppeliaSim and sensor services
│   ├── utils/              # Utilities (metrics, logging, etc.)
│   ├── visualization/      # Plotting and visualization
│   └── main.py             # Main entry point
├── scenes/                 # CoppeliaSim scene files (.ttt)
├── scripts/                # Analysis scripts
│   └── generate_plots.py   # Plot generation script
├── results/                # Experiment results (generated)
├── plots/                  # Generated plots (generated)
├── docs/                   # Documentation
├── makefile                # Build automation
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ALGORITHMS -->
## Algorithms

### DWA (Dynamic Window Approach)

The Dynamic Window Approach is a velocity-based local planner that:
- Searches for collision-free velocities in a dynamically feasible window
- Evaluates trajectories based on heading, clearance, and velocity
- Handles dynamic obstacles through real-time sensor feedback

**Key Features:**
- Fast real-time performance
- Good for static and slow-moving obstacles
- Configurable prediction horizon and resolution

### TEB (Timed Elastic Band)

The Timed Elastic Band planner:
- Optimizes a trajectory by deforming an elastic band
- Considers temporal constraints and obstacle avoidance
- Performs iterative optimization to find smooth paths

**Key Features:**
- Produces smooth, time-optimal trajectories
- Good for complex obstacle configurations
- Computationally more intensive

### ORCA (Optimal Reciprocal Collision Avoidance)

ORCA is designed for multi-agent scenarios:
- Uses velocity obstacles for collision avoidance
- Assumes reciprocal responsibility between agents
- Computes optimal collision-free velocities

**Key Features:**
- Excellent for multi-robot scenarios
- Guarantees collision avoidance under assumptions
- Handles dynamic obstacles and other robots

### PID Controller

A simple proportional-integral-derivative controller:
- Basic reactive navigation
- Serves as a baseline for comparison
- Direct heading and distance control

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- METRICS -->
## Metrics

The framework tracks comprehensive performance metrics:

### Success Metrics
- **Success Rate**: Percentage of successful goal-reaching runs
- **Time to Goal**: Time taken to reach the goal
- **Path Length**: Total distance traveled

### Safety Metrics
- **Collisions**: Number of collisions detected
- **Near Misses**: Close encounters with obstacles
- **Minimum Distances**: Minimum distance to walls, robots, and intersections

### Quality Metrics
- **Deadlocks**: Number of deadlock situations detected
- **Smoothness Index**: Trajectory smoothness measure
- **Wall Distance**: Average and minimum distances to walls

All metrics are saved in JSON format and can be analyzed using the plotting scripts.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- TROUBLESHOOTING -->
## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to CoppeliaSim
```sh
make test-connection
```

**Solutions**:
- Ensure CoppeliaSim is running
- Check that the remote API server is enabled (default port: 23000)
- Verify HOST and PORT settings match CoppeliaSim configuration
- On Windows with WSL, use `localhost` or the Windows host IP

### Scene Not Found

**Problem**: Scene file not found error

**Solutions**:
- Verify scene files exist in `scenes/` directory
- Check scene name spelling (scene_1, scene_2, scene_3)
- Use `SCENE_PATH` to specify custom scene directory

### Import Errors

**Problem**: Python import errors

**Solutions**:
```sh
# Reinstall dependencies
make setup

# Or manually
pip install -r requirements.txt --upgrade
```

### Performance Issues

**Problem**: Simulation runs slowly

**Solutions**:
- Disable visualization: Remove `VISUALIZE=1` flag
- Reduce `SIM_TIME` for quick tests
- Check system resources (CoppeliaSim can be resource-intensive)

### Results Not Generated

**Problem**: No results or plots generated

**Solutions**:
- Check that simulations completed successfully
- Verify `results/` directory exists and has data
- Run `make plots` after collecting results
- Check logs in result directories for errors

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Bruno Lemos

Project Link: [https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark](https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark)

If you have questions or suggestions, please open an issue on GitHub.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.svg?style=for-the-badge
[contributors-url]: https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.svg?style=for-the-badge
[forks-url]: https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/network/members
[stars-shield]: https://img.shields.io/github/stars/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.svg?style=for-the-badge
[stars-url]: https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/stargazers
[issues-shield]: https://img.shields.io/github/issues/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.svg?style=for-the-badge
[issues-url]: https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/issues
[license-shield]: https://img.shields.io/github/license/brunolemosdl/coppeliasim-dynamic-navigation-benchmark.svg?style=for-the-badge
[license-url]: https://github.com/brunolemosdl/coppeliasim-dynamic-navigation-benchmark/blob/master/LICENSE

[Python.org]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[Python-url]: https://www.python.org/
[NumPy.org]: https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white
[NumPy-url]: https://numpy.org/
[Matplotlib.org]: https://img.shields.io/badge/Matplotlib-11557C?style=for-the-badge&logo=matplotlib&logoColor=white
[Matplotlib-url]: https://matplotlib.org/
[OpenCV.org]: https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white
[OpenCV-url]: https://opencv.org/
[CoppeliaSim.org]: https://img.shields.io/badge/CoppeliaSim-FF6B00?style=for-the-badge&logo=coppeliasim&logoColor=white
[CoppeliaSim-url]: https://www.coppeliarobotics.com/

