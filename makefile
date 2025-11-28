# ─── Environment variable loading ───
ifneq (,$(wildcard .env))
	include .env
	export
endif
ifneq (,$(wildcard .env.local))
	include .env.local
	export
endif

# ─── General settings ───
PYTHON        := python3
SRC_DIR       := src
RESULTS_DIR   := results
SCENE_DIR     := scenes

HOST          ?= localhost
PORT          ?= 23000
RESULTS_PATH  ?= $(RESULTS_DIR)
SCENE_PATH    ?= $(SCENE_DIR)

# ─── Message prefixes ───
INFO    := printf "[INFO]  %s\n"
WARN    := printf "[WARN]  %s\n"
ERROR   := printf "[ERROR] %s\n"

# ─── Colors ───
CYAN    := \033[0;36m
GREEN   := \033[0;32m
RED     := \033[0;31m
NC      := \033[0m

# ─── Flags ───
EXTRA_FLAGS   :=

ifeq ($(VISUALIZE),1)
	EXTRA_FLAGS += --visualize
endif
ifeq ($(VERBOSE),1)
	EXTRA_FLAGS += --verbose
endif
ifneq ($(RESULTS_PATH),$(RESULTS_DIR))
	EXTRA_FLAGS += --output-dir $(RESULTS_PATH)
endif
ifneq ($(SCENE_PATH),$(SCENE_DIR))
	EXTRA_FLAGS += --scene-dir $(SCENE_PATH)
endif

# ─── Default params ───
SIM_TIME           ?= 300
LASER_MAX_RANGE    ?= 5.0

MAX_SPEED     ?= 0.30
MAX_ANG_SPEED ?= 1.00
ROBOT_RADIUS  ?= 0.175
GOAL_TOLERANCE ?= 0.2

PLANNER       ?= dwa
PLOT_INTERVAL ?= 5
SAVE_TRAJECTORY ?= 1
SAVE_LOGS     ?= 1

DEADLOCK_TIME_THRESHOLD     ?= 5.0
DEADLOCK_DISTANCE_THRESHOLD ?= 0.1

SENSOR_ORTHO_SIZE ?= 12.0

# ─── Phony ───
.PHONY: help run benchmark plots setup clean check-deps test-connection status

# ─── Help ───
help:
	@printf "Usage:\n"
	@printf "  make run SCENE=scene_1 PLANNER=dwa [PARAMS] [VISUALIZE=1] [VERBOSE=1]\n\n"
	@printf "Core paths (defaults in brackets):\n"
	@printf "  SCENE         [%s]\n" "$(SCENE)"
	@printf "  SCENE_PATH    [%s]\n" "$(SCENE_PATH)"
	@printf "  RESULTS_PATH  [%s]\n\n" "$(RESULTS_PATH)"
	@printf "Connection:\n"
	@printf "  HOST=%s PORT=%s\n\n" "$(HOST)" "$(PORT)"
	@printf "Key parameters:\n"
	@printf "  PLANNER       [%s] (dwa, teb, orca)\n" "$(PLANNER)"
	@printf "  SIM_TIME      [%s]\n" "$(SIM_TIME)"
	@printf "  MAX_SPEED     [%s]\n" "$(MAX_SPEED)"
	@printf "  MAX_ANG_SPEED [%s]\n" "$(MAX_ANG_SPEED)"
	@printf "  ROBOT_RADIUS  [%s]\n\n" "$(ROBOT_RADIUS)"
	@printf "Examples:\n"
	@printf "  make run SCENE=scene_1 PLANNER=dwa VISUALIZE=1\n"
	@printf "  make run SCENE=scene_2 PLANNER=teb SIM_TIME=300\n"
	@printf "  make run SCENE=scene_3 PLANNER=orca\n"
	@printf "  make benchmark  # Run 20x each algorithm in each scene (unlimited time)\n"
	@printf "  make plots      # Generate plots from results\n"

# ─── Validation ───
validate:
	@if [ ! -d "$(SCENE_PATH)" ]; then \
		$(ERROR) "scene path not found: '$(SCENE_PATH)'"; exit 1; \
	fi
	@mkdir -p "$(RESULTS_PATH)"

# ─── Scene selection ───
SCENE ?= scene_1

# ─── Execution ───
run: validate
	@$(INFO) "starting simulation"
	@$(INFO) "scene=$(SCENE) planner=$(PLANNER) scene_dir=$(SCENE_PATH) output=$(RESULTS_PATH) host=$(HOST):$(PORT)"
	@$(PYTHON) $(SRC_DIR)/main.py \
		--scene $(SCENE) \
		--planner $(PLANNER) \
		--scene-dir $(SCENE_PATH) \
		--host $(HOST) \
		--port $(PORT) \
		--sim-time $(SIM_TIME) \
		--laser-max-range $(LASER_MAX_RANGE) \
		--max-speed $(MAX_SPEED) \
		--max-ang-speed $(MAX_ANG_SPEED) \
		--robot-radius $(ROBOT_RADIUS) \
		--goal-tolerance $(GOAL_TOLERANCE) \
		--deadlock-time-threshold $(DEADLOCK_TIME_THRESHOLD) \
		--deadlock-distance-threshold $(DEADLOCK_DISTANCE_THRESHOLD) \
		--plot-interval $(PLOT_INTERVAL) \
		--save-trajectory $(SAVE_TRAJECTORY) \
		--save-logs $(SAVE_LOGS) \
		--output-dir $(RESULTS_PATH) \
		$(EXTRA_FLAGS)

# ─── Benchmark: Run 20x each algorithm in each scene ───
BENCHMARK_ITERATIONS ?= 20
BENCHMARK_SCENES := scene_1 scene_2 scene_3
BENCHMARK_PLANNERS := dwa teb orca
BENCHMARK_SIM_TIME := 999999

benchmark: validate
	@$(INFO) "Starting benchmark: $(BENCHMARK_ITERATIONS) iterations per algorithm per scene"
	@$(INFO) "Scenes: $(BENCHMARK_SCENES)"
	@$(INFO) "Algorithms: $(BENCHMARK_PLANNERS)"
	@$(INFO) "Simulation time: unlimited ($(BENCHMARK_SIM_TIME)s)"
	@total=0; \
	total_runs=$$((3 * 3 * $(BENCHMARK_ITERATIONS))); \
	for scene in $(BENCHMARK_SCENES); do \
		for planner in $(BENCHMARK_PLANNERS); do \
			$(INFO) "─────────────────────────────────────────"; \
			$(INFO) "Scene: $$scene | Algorithm: $$planner"; \
			$(INFO) "─────────────────────────────────────────"; \
			for iter in $$(seq 1 $(BENCHMARK_ITERATIONS)); do \
				total=$$((total + 1)); \
				$(INFO) "Iteration $$iter/$(BENCHMARK_ITERATIONS) (Total: $$total/$$total_runs)"; \
				$(PYTHON) $(SRC_DIR)/main.py \
					--scene $$scene \
					--planner $$planner \
					--scene-dir $(SCENE_PATH) \
					--host $(HOST) \
					--port $(PORT) \
					--sim-time $(BENCHMARK_SIM_TIME) \
					--laser-max-range $(LASER_MAX_RANGE) \
					--max-speed $(MAX_SPEED) \
					--max-ang-speed $(MAX_ANG_SPEED) \
					--robot-radius $(ROBOT_RADIUS) \
					--goal-tolerance $(GOAL_TOLERANCE) \
					--deadlock-time-threshold $(DEADLOCK_TIME_THRESHOLD) \
					--deadlock-distance-threshold $(DEADLOCK_DISTANCE_THRESHOLD) \
					--plot-interval $(PLOT_INTERVAL) \
					--save-trajectory $(SAVE_TRAJECTORY) \
					--save-logs $(SAVE_LOGS) \
					--output-dir $(RESULTS_PATH) \
					$(EXTRA_FLAGS) || { \
						$(ERROR) "Failed: $$scene/$$planner iteration $$iter"; \
						exit 1; \
					}; \
				$(INFO) "Completed: $$scene/$$planner iteration $$iter"; \
			done; \
			$(INFO) "Finished all iterations for $$scene/$$planner"; \
		done; \
	done; \
	$(INFO) "════════════════════════════════════════════"; \
	$(INFO) "Benchmark complete! Total runs: $$total"; \
	$(INFO) "════════════════════════════════════════════"

# ─── Generate plots ───
PLOTS_DIR := plots
PLOTS_RESULTS_DIR ?= $(RESULTS_DIR)

plots:
	@$(INFO) "Generating plots from results"
	@mkdir -p $(PLOTS_DIR)
	@$(PYTHON) scripts/generate_plots.py \
		--results-dir $(PLOTS_RESULTS_DIR) \
		--output-dir $(PLOTS_DIR)
	@$(INFO) "Plots saved to $(PLOTS_DIR)"

# ─── Env & diagnostics ───
setup:
	@$(INFO) "creating directories"
	@mkdir -p $(RESULTS_DIR)
	@$(INFO) "installing requirements"
	@$(PYTHON) -m pip install -r requirements.txt
	@$(INFO) "setup complete"

check-deps:
	@$(INFO) "checking python and libs"
	@$(PYTHON) -c "import sys; print('python', sys.version.split()[0])"
	@$(PYTHON) -c "import numpy; print('numpy', numpy.__version__)" 2>/dev/null || printf "[WARN]  numpy missing\n"
	@$(PYTHON) -c "import colorlog; print('colorlog', colorlog.__version__)" 2>/dev/null || printf "[WARN]  colorlog missing\n"
	@$(PYTHON) -c "import matplotlib; print('matplotlib', matplotlib.__version__)" 2>/dev/null || printf "[WARN]  matplotlib missing\n"
	@$(PYTHON) -c "import cv2; print('opencv', cv2.__version__)" 2>/dev/null || printf "[WARN]  opencv missing\n"
	@$(PYTHON) -c "import coppeliasim_zmqremoteapi_client; print('coppeliasim_zmqremoteapi_client', coppeliasim_zmqremoteapi_client.__version__)" 2>/dev/null || printf "[WARN]  coppeliasim_zmqremoteapi_client missing\n"
	@$(INFO) "dependency check done"

test-connection:
	@$(INFO) "testing connection to $(HOST):$(PORT)"
	@$(PYTHON) -c "import socket; socket.create_connection(('$(HOST)', $(PORT)), timeout=2); print('[INFO]  connection successful')" 2>/dev/null || printf "[ERROR] connection failed: could not connect to CoppeliaSim at $(HOST):$(PORT)\n"

status:
	@printf "Configuration:\n"
	@printf "  host = %s\n" "$(HOST):$(PORT)"
	@printf "  scene_dir = %s\n" "$(SCENE_PATH)"
	@printf "  results_path(tool) = %s\n" "$(RESULTS_PATH)"
	@printf "  results_dir(local) = %s\n\n" "$(RESULTS_DIR)"
	@printf "Latest results:\n"
	@if [ -d "$(RESULTS_DIR)" ]; then \
		find $(RESULTS_DIR) -name "*.json" -type f -exec ls -lt {} + 2>/dev/null | head -5 || printf "  none\n"; \
	else \
		printf "  directory '%s' not found\n" "$(RESULTS_DIR)"; \
	fi

clean:
	@$(INFO) "cleaning artifacts"
	@rm -rf $(RESULTS_DIR)
	@find . -type f -name "*.pyc" -delete
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@$(INFO) "done"
