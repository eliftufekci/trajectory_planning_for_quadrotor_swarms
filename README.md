# Trajectory Planning for Quadrotor Swarms

This repository contains a C++ implementation of a trajectory planning pipeline for quadrotor swarms in 3D environments with obstacles.

The project is based on the paper **"Trajectory Planning for Quadrotor Swarms"** by Wolfgang Honig, James A. Preiss, T. K. Satish Kumar, Gaurav S. Sukhatme, and Nora Ayanian. It combines discrete multi-agent planning with continuous trajectory refinement to produce collision-free quadrotor trajectories.

## Overview

The planner follows a multi-stage pipeline:

1. Load a 3D environment from a YAML file.
2. Build a roadmap in the free space.
3. Connect robot start and goal positions to the roadmap.
4. Assign goals for labeled or unlabeled scenarios.
5. Solve a discrete multi-agent path planning problem with ECBS.
6. Annotate robot-robot and robot-obstacle conflicts.
7. Refine the discrete paths into continuous Bezier trajectories.
8. Export CSV files for visualization and simulation.

The executable writes roadmap, path, hyperplane, and control point data to CSV files. The repository includes both a Plotly-based HTML visualizer and a lightweight pygame viewer for watching the robots move along the optimized trajectories.

<img width="425" alt="RoadMap Generation and Conflict Annotation" src="https://github.com/user-attachments/assets/784f13f0-ac4b-4634-bb4b-977dbadccdbe" />
<br>
RoadMap Generation and Conflict Annotation

<br><br>

<img width="425" alt="Discrete Planning" src="https://github.com/user-attachments/assets/7ddfe7f0-3d9e-42d7-a66e-6f99b997db85" />
<br>
Discrete Planning

<br><br>

<img width="425" alt="Continuous Refinement" src="https://github.com/user-attachments/assets/d5a5f770-cead-4cd2-9b91-20bb87c0d3ce" />
<br>
Continuous Refinement

<br><br>

<img width="425" alt="Labelled Planner" src="https://github.com/user-attachments/assets/42151ab7-ee22-4933-90ba-22cb4ab5e715" />
<br>
Labelled Planner

<br><br>

<img width="425" alt="Unlabelled Planner" src="https://github.com/user-attachments/assets/201f78b1-24a7-4896-bcca-2b743327319e" />
<br>
Unlabelled Planner

<br><br>

[Sample Simulation](https://github.com/user-attachments/assets/799e5696-5fdb-4e90-a20f-c70a15eb7168)


## Features

- 3D YAML environment loading
- SPARS-based roadmap generation with OMPL
- Collision checking with FCL and OctoMap
- ECBS-based multi-agent discrete planning using `libMultiRobotPlanning`
- Labeled and unlabeled goal assignment
- Swept and FCL-based conflict annotation
- Continuous Bezier trajectory optimization
- Safe separating hyperplanes
- OSQP / osqp-eigen based quadratic programming
- CSV export for visualization and downstream simulation
- Lightweight pygame trajectory viewer using `control_points.csv`
- Optional Crazyswarm2 trajectory conversion helper

## Repository Layout

```text
.
├── CMakeLists.txt
├── README.md
├── config/                  # Example YAML environment files
├── include/                 # C++ headers
├── src/                     # C++ source files
├── libs/                    # Local third-party dependencies
├── simulation/              # Crazyswarm2 conversion helper
├── pygame_quadrotor_viewer.py # Lightweight pygame trajectory viewer
├── visualize_interactive.py # Plotly visualization script
└── setup.sh                 # Local helper script; review before using
```

## Dependencies

The project is intended for Linux, especially Ubuntu-based systems.

Required system dependencies:

- CMake
- C++17 compiler
- Eigen3
- Boost
- yaml-cpp
- OctoMap
- FCL
- OMPL
- CDD
- OSQP

Project-local dependencies:

- `osqp-eigen`
- `libMultiRobotPlanning`

Python dependencies for visualization and trajectory conversion:

- `numpy`
- `scipy`
- `pandas`
- `plotly`
- `pygame`

## Clone the Repository

```bash
git clone <https://github.com/eliftufekci/trajectory_planning_for_quadrotor_swarms.git>
cd trajectory_planning_for_quadrotor_swarms
```


## Install System Dependencies

On Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libeigen3-dev libboost-all-dev libyaml-cpp-dev \
  liboctomap-dev libfcl-dev libompl-dev libcdd-dev
```

## Install OSQP

If your package manager provides a compatible OSQP package, you can use it. Otherwise, build OSQP from source:

```bash
cd /tmp
git clone --recursive https://github.com/osqp/osqp.git
cd osqp
mkdir -p build
cd build
cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

## Install Local Third-Party Libraries

From the repository root:

```bash
mkdir -p libs
git clone --depth 1 https://github.com/robotology/osqp-eigen.git libs/osqp-eigen
git clone --depth 1 https://github.com/whoenig/libMultiRobotPlanning.git libs/libMultiRobotPlanning
```

Optional cleanup for `libMultiRobotPlanning`:

```bash
rm -rf libs/libMultiRobotPlanning/{benchmark,doc,example,test,.github}
rm -f libs/libMultiRobotPlanning/CMakeLists.txt \
      libs/libMultiRobotPlanning/.clang-format \
      libs/libMultiRobotPlanning/.gitignore
```

## Install Python Packages

```bash
python3 -m pip install numpy scipy pandas plotly pygame
```

## Build

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j"$(nproc)"
```

The executable is created at:

```text
build/env_test
```

## Run

From the `build/` directory:

```bash
./env_test ../config/environment.yaml
```

If no YAML path is provided, the executable uses:

```text
../config/environment.yaml
```

You can run another environment file in `config/` the same way:

```bash
./env_test ../config/environment11.yaml
```

## Output Files

The executable writes output files to the current working directory. If you run it from `build/`, the files are created in `build/`.

Common outputs:

- `environment_data.csv`
- `octree_voxels.csv`
- `vertices.csv`
- `edges.csv`
- `paths.csv`
- `hyperplanes_iter_*.csv`
- `control_points_iter_*.csv`
- `control_points.csv`

## Visualization and Lightweight Simulation

### Plotly HTML Viewer

After running the planner, execute the visualizer from the repository root:

```bash
python3 visualize_interactive.py
```

The script reads the generated CSV files from `build/` and writes an interactive HTML visualization, typically:

```text
solution_with_planes.html
```

### Pygame Trajectory Viewer

If you only need to watch the robots move and do not want to set up Crazyswarm, use the pygame viewer:

```bash
python3 pygame_quadrotor_viewer.py
```

By default it reads:

```text
build/environment_data.csv
build/control_points.csv
```

You can also pass explicit paths:

```bash
python3 pygame_quadrotor_viewer.py \
  --env build/environment_data.csv \
  --control-points build/control_points.csv
```

Controls:

- `Space`: pause/resume
- `R`: restart playback
- `+` / `-`: change playback speed
- Arrow keys: rotate the camera
- Mouse wheel: zoom
- Left mouse drag: pan the view

This viewer is not a physics simulator. It projects the 3D trajectory into a pygame window and animates the quadrotor positions from the optimized Bezier control points.

## Crazyswarm2 Trajectory Conversion

The `simulation/` folder includes a helper script for converting optimized Bezier control points into trajectory CSV files.

Example:

```bash
python3 simulation/cp_to_crazyswarm2_format.py \
  --input build/control_points.csv \
  --duration 42.0 \
  --out_dir simulation/trajectories
```

Adjust `--duration` to match the total trajectory duration you want to execute.

## Environment YAML Format

Environment files define world bounds, obstacles, agents, goals, and whether the instance is labeled.

Example:

```yaml
labeled: true

world_bounds:
  min: [0.0, 0.0, 0.0]
  max: [9.0, 5.5, 2.2]

obstacles:
  - min: [3.5, 1.5, 0.0]
    max: [5.5, 4.0, 2.0]

agents:
  - id: 0
    start: [0.5, 0.5, 0.5]

goals:
  - [8.5, 4.5, 1.8]
```

When `labeled: true`, goals are assigned in order. When `labeled: false`, the planner performs a threshold-based goal assignment step.

## Notes

- `setup.sh` is a local convenience script and contains machine-specific absolute paths. Review and edit it before running.
- The build expects `libs/osqp-eigen` and `libs/libMultiRobotPlanning` to exist.
- Running the executable from `build/` keeps the default config path and visualizer paths aligned.
- This repository is an implementation study of the paper and is not an official release from the paper authors.

## Reference

Wolfgang Honig, James A. Preiss, T. K. Satish Kumar, Gaurav S. Sukhatme, and Nora Ayanian, **"Trajectory Planning for Quadrotor Swarms."**
