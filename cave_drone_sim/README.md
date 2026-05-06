# CaveDroneSim: Autonomous Drone Exploration of Procedurally-Generated Cave Environments

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=ros)
![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-green?logo=python)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

CaveDroneSim is a full-stack autonomous drone exploration platform for GPS-denied underground cave environments. It combines procedurally-generated 3D cave worlds (Gazebo Harmonic), LiDAR-inertial SLAM (FAST-LIO2 front-end, RTAB-Map back-end), and a frontier-based exploration planner with RRT* path planning and artificial potential field (APF) obstacle avoidance — all orchestrated by a 7-state finite-state machine mission manager. The project was developed as the simulation and evaluation platform for a master's thesis on autonomous drone navigation in unstructured, GPS-denied environments, directly motivated by the DARPA Subterranean Challenge problem domain.

---

## Architecture

The system is structured as a five-stage pipeline:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    CaveDroneSim Pipeline                                  │
│                                                                          │
│  Stage 1          Stage 2         Stage 3         Stage 4       Stage 5  │
│  ─────────────    ───────────     ────────────    ──────────    ──────── │
│  Cave             Drone           SLAM            Exploration   Mission  │
│  Generation  ──►  Bringup    ──►  Pipeline   ──►  Planner  ──►  Manager │
│                                                                          │
│  cave_generator   drone_bringup   slam_pipeline   exploration_planner   │
│  ─────────────    ───────────     ────────────    ──────────────────── │
│  Graph topology   PX4 SITL        FAST-LIO2       Frontier-Based       │
│  SDF + MC mesh    Ouster LiDAR    RTAB-Map         Exploration          │
│  Gazebo export    IMU + Camera    OctoMap          RRT* Planner         │
│  Obstacle place   TF broadcast    Degeneracy det.  APF Local Planner    │
└──────────────────────────────────────────────────────────────────────────┘
```

**ROS 2 Node graph (abbreviated):**
```
cave_generator_node ──/cave/model_sdf──► [Gazebo]
                                              │
                              /lidar/points ◄─┘
                              /imu/data     ◄─┘
                                    │
              ┌─────────────────────▼──────────────────────────┐
              │  slam_pipeline                                   │
              │  simple_lidar_odom ──► /slam/odometry           │
              │  octomap_builder   ──► /octomap_full_color      │
              │  degeneracy_detector──► /slam/degeneracy_score  │
              └──────────────┬─────────────────────────────────┘
                             │ /slam/odometry  /octomap_full_color
              ┌──────────────▼──────────────────────────────────┐
              │  exploration_planner                             │
              │  frontier_explorer ──► /exploration/frontiers   │
              │  path_planner      ──► /exploration/path        │
              │  local_planner     ──► /drone/cmd_vel           │
              │  mission_manager   ──► /mission/state           │
              └─────────────────────────────────────────────────┘
```

## Screenshot

> _Simulation screenshot — add `docs/images/cave_demo.png` after first successful run._

```
[Screenshot placeholder: RViz2 showing drone trajectory (white), OctoMap
 occupancy (grey voxels), and frontiers (green spheres) inside a procedurally
 generated cave. Run `ros2 launch drone_bringup drone_sim.launch.py` and
 record via RViz2 → File → Save Screenshot.]
```

---

## Quick Start

```bash
# 1. Clone and build
git clone https://github.com/your-org/cave_drone_sim.git && cd cave_drone_sim
colcon build --symlink-install && source install/setup.bash

# 2. Generate a cave and launch the full demo
ros2 launch cave_drone_sim full_demo.launch.py seed:=42

# 3. Watch in RViz2 (opens automatically) or trigger exploration manually
ros2 topic pub /mission/cmd std_msgs/msg/String "data: 'START'" -1
```

---

## Full Installation

See [docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md) for a step-by-step guide on a fresh Ubuntu 22.04 machine, including Docker alternatives.

### Prerequisites

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 22.04 LTS | Primary supported OS |
| ROS 2 | Humble Hawksbill | `ros-humble-desktop` |
| Gazebo | Harmonic (gz-sim 8.x) | `ros-humble-ros-gz` |
| Python | 3.10+ | System Python |
| NVIDIA GPU | CUDA-capable, ≥ 8 GB VRAM | Required for `gpu_lidar` |

### Dependencies

```bash
# ROS 2 + Gazebo bridge
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge \
                 ros-humble-rtabmap-ros ros-humble-octomap-ros \
                 ros-humble-nav2-msgs ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs ros-humble-visualization-msgs

# Python packages
pip install numpy scipy scikit-learn open3d noise trimesh

# FAST-LIO2 (community ROS 2 port)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git
cd ~/ros2_ws && colcon build --packages-select fast_lio
```

### Build

```bash
cd /path/to/cave_drone_sim
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Full Autonomous Exploration Demo

```bash
# Generate cave, launch Gazebo, start SLAM and exploration
ros2 launch cave_drone_sim full_demo.launch.py seed:=42 slam_backend:=simple
```

Key launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `seed` | `42` | Cave random seed (integer) |
| `slam_backend` | `simple` | `simple`, `fastlio`, or `rtabmap` |
| `num_nodes` | `20` | Cave complexity (graph nodes) |
| `rviz` | `true` | Open RViz2 |

### SLAM Quality Test (no exploration)

```bash
# Launch cave + drone + SLAM only; fly manually to test odometry
ros2 launch slam_pipeline slam_simple.launch.py
# In another terminal, teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/drone/cmd_vel
```

### Manual Teleoperation

```bash
ros2 launch drone_bringup drone_only.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/drone/cmd_vel
```

### Custom Cave Generation

```bash
# Generate and export only (no Gazebo)
ros2 launch cave_generator generate_cave.launch.py \
    seed:=99 num_nodes:=30 bounds_x:=120.0 bounds_z:=30.0 \
    noise_amplitude:=0.4 stalactite_density:=0.5

# Or use the standalone Python script
python3 scripts/generate_cave.py --seed 99 --nodes 30 --output /tmp/my_cave
```

---

## Package Descriptions

### `cave_generator`

Procedural 3D cave generation and Gazebo world export. Uses a four-stage hybrid pipeline:

1. **Graph topology** — Randomized Prim's MST over random 3D nodes, with additional loop edges for SLAM evaluation diversity.
2. **SDF field** — Smooth-union SDF along graph edges, displaced by 3D Simplex noise (fractional Brownian motion).
3. **Marching cubes** — Mesh extraction from the 3D scalar field at configurable voxel resolution.
4. **Gazebo export** — Writes `model.sdf`, `cave_visual.dae`, `cave_collision.stl`, and a complete world `.sdf` file.

Key nodes: `cave_generator_node`  
Key scripts: `scripts/generate_cave.py`

### `drone_bringup`

Drone URDF, sensor models, PX4 SITL integration, and Gazebo simulation launch. Provides:

- Custom quadrotor URDF with Ouster OS0-32 LiDAR, IMU, and depth camera mounts.
- `DroneController` node: PID velocity controller bridging ROS 2 `cmd_vel` to Gazebo `gz-transport` actuator commands.
- `TFBroadcaster` node: publishes `map → odom → base_link → lidar_link` TF chain.
- Gazebo launch files for standalone drone testing and full simulation.

Key nodes: `drone_controller`, `tf_broadcaster`

### `slam_pipeline`

LiDAR odometry, 3D occupancy mapping, and degeneracy detection. Three swappable SLAM backends via launch file argument:

- **Simple ICP** (`slam_simple.launch.py`) — Keyframe-based ICP with sliding-window local map. Self-contained, no external dependencies. Best for prototyping.
- **FAST-LIO2** (`slam_fastlio.launch.py`) — Tight LiDAR-IMU fusion via iterated EKF and ikd-Tree. Recommended for evaluation.
- **RTAB-Map** (`slam_rtabmap.launch.py`) — Full SLAM with appearance-based loop closure. Best for long traversals.

All backends publish `/slam/odometry` and `/slam/cloud_map` on the same topic names for downstream compatibility.

Key nodes: `simple_lidar_odom`, `octomap_builder`, `degeneracy_detector`, `map_saver`

### `exploration_planner`

Frontier-based exploration, RRT* path planning, APF local planner, and mission management.

- **`frontier_explorer`** — Extracts 3D frontier clusters from OctoMap, scores each by utility function (information gain, distance, heading alignment), and publishes ranked frontier goals.
- **`path_planner`** — RRT* with rewiring over the OctoMap collision field; falls back to grid A* on timeout.
- **`local_planner`** — Artificial potential field controller running at 20 Hz. Attractive force toward next waypoint, repulsive force from OctoMap obstacles.
- **`mission_manager`** — 7-state FSM: `IDLE → TAKEOFF → EXPLORE → NAVIGATE → RETURN_HOME → LAND → EMERGENCY`.

Key nodes: `frontier_explorer`, `path_planner`, `local_planner`, `mission_manager`, `collision_checker`

---

## Configuration Reference

### Cave Generation (`config/default_cave.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_nodes` | `20` | Graph nodes (chambers/junctions). More nodes → more complex cave |
| `bounds_x`, `bounds_y`, `bounds_z` | `80, 80, 25` | Bounding volume in metres |
| `num_extra_loops` | `3` | Loop edges added beyond MST. Controls loop-closure difficulty |
| `resolution` | `0.3` | SDF voxel size in metres. Smaller → finer mesh, higher compute |
| `noise_amplitude` | `0.25` | Wall roughness as fraction of tunnel radius |
| `noise_frequency` | `0.08` | Spatial frequency of Simplex noise |
| `stalactite_density` | `0.3` | Coverage density [0–1] |
| `rubble_density` | `0.2` | Floor rubble coverage [0–1] |
| `seed` | `42` | Random seed for reproducibility |

### SLAM (`config/slam_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `icp_max_distance` | `1.0 m` | ICP correspondence threshold; increase for fast motion |
| `keyframe_distance` | `0.5 m` | Translation to trigger new keyframe |
| `local_map_size` | `50` | Sliding-window keyframes |
| `resolution` (OctoMap) | `0.2 m` | Occupancy map voxel size |
| `max_range` | `30.0 m` | Maximum LiDAR range used for mapping |
| `eigenvalue_threshold` | `0.01` | Degeneracy detection threshold (λ_min/λ_max) |

### Exploration (`config/exploration_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `utility_weight_info` | `1.0` | Weight for information gain in frontier scoring |
| `utility_weight_distance` | `0.5` | Weight for distance penalty |
| `utility_weight_heading` | `0.3` | Weight for heading alignment bonus |
| `step_size` (RRT*) | `0.5 m` | Tree extension step |
| `rewiring_radius` | `2.0 m` | RRT* neighbourhood radius |
| `k_attractive` (APF) | `1.0` | Attractive gain |
| `k_repulsive` (APF) | `0.5` | Repulsive gain |
| `obstacle_influence_distance` | `2.0 m` | APF repulsion activation radius |
| `battery_return_threshold` | `30.0 %` | Battery level triggering return-to-home |

---

## Project Structure

```
cave_drone_sim/
├── README.md                          # This file
├── launch/
│   └── full_demo.launch.py            # Top-level demo launch
├── docs/
│   ├── ARCHITECTURE.md                # ROS 2 node/topic/TF documentation
│   ├── CAVE_GENERATION.md             # Procedural generation deep-dive
│   ├── SLAM_GUIDE.md                  # SLAM backends, tuning, evaluation
│   ├── EXPLORATION_GUIDE.md           # Exploration and navigation guide
│   ├── SETUP_GUIDE.md                 # Installation on Ubuntu 22.04
│   ├── THESIS_NOTES.md                # Research context and literature
│   └── API_REFERENCE.md               # Complete topic/service/parameter ref
└── src/
    ├── cave_generator/                # Stage 1: Procedural cave generation
    │   ├── cave_generator/
    │   │   ├── cave_graph.py          # Graph topology (Prim's MST + loops)
    │   │   ├── cave_mesh.py           # SDF computation + marching cubes
    │   │   ├── obstacle_placer.py     # Stalactite + rubble placement
    │   │   ├── gazebo_exporter.py     # SDF/DAE/STL export
    │   │   ├── generator_node.py      # ROS 2 node wrapper
    │   │   └── visualizer.py          # Open3D debug visualization
    │   ├── config/default_cave.yaml
    │   ├── launch/generate_cave.launch.py
    │   └── worlds/
    ├── drone_bringup/                 # Stage 2: Drone + Gazebo
    │   ├── drone_bringup/
    │   │   ├── drone_controller.py    # PID velocity controller
    │   │   └── tf_broadcaster.py      # TF chain publisher
    │   ├── config/
    │   │   ├── drone_params.yaml      # PID gains, mass, limits
    │   │   └── gz_bridge.yaml         # Gazebo ↔ ROS 2 topic bridge
    │   ├── models/cave_drone/         # Drone URDF + mesh assets
    │   └── launch/
    ├── slam_pipeline/                 # Stage 3: SLAM
    │   ├── slam_pipeline/
    │   │   ├── simple_lidar_odom.py   # ICP odometry node
    │   │   ├── octomap_builder.py     # 3D occupancy builder
    │   │   ├── degeneracy_detector.py # Eigenvalue health monitor
    │   │   └── map_saver.py           # PCD/PLY persistence
    │   ├── config/
    │   │   ├── slam_params.yaml
    │   │   ├── fastlio_cave.yaml       # FAST-LIO2 cave config
    │   │   └── rtabmap_cave.yaml       # RTAB-Map cave config
    │   └── launch/
    ├── exploration_planner/           # Stages 4–5: Exploration + Mission
    │   ├── exploration_planner/
    │   │   ├── frontier_explorer.py   # Frontier detection + scoring
    │   │   ├── path_planner.py        # RRT* + A* fallback
    │   │   ├── local_planner.py       # APF obstacle avoidance
    │   │   ├── mission_manager.py     # 7-state FSM
    │   │   └── collision_checker.py   # KD-tree obstacle queries
    │   ├── config/exploration_params.yaml
    │   └── launch/
    └── cave_drone_interfaces/         # Custom ROS 2 message/service types
        └── msg/
```

---

## Citation

If you use this software in your research, please cite the associated thesis:

```bibtex
@mastersthesis{cave_drone_sim_2026,
  author    = {[Author Name]},
  title     = {Autonomous Drone Exploration of Procedurally-Generated Cave
               Environments Using LiDAR SLAM and Frontier-Based Planning},
  school    = {[University Name]},
  year      = {2026},
  type      = {Master's Thesis},
  note      = {Software available at https://github.com/your-org/cave_drone_sim}
}
```

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgments

This project builds directly on the infrastructure and research produced by:

- **DARPA Subterranean (SubT) Challenge** — the primary motivating problem domain and source of open-source cave world assets ([github.com/osrf/subt](https://github.com/osrf/subt)).
- **PX4 Autopilot** — flight controller firmware and Gazebo SITL simulation ([px4.io](https://px4.io)).
- **FAST-LIO2** — Xu et al., HKU-MARS Lab. IEEE Transactions on Robotics, 2022 ([github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)).
- **RTAB-Map** — Labbé & Michaud, IntRoLab, Université de Sherbrooke. Journal of Field Robotics, 2019 ([github.com/introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros)).
- **ROS 2 Community** — Open Robotics / Intrinsic, and all contributors to the ROS 2 Humble ecosystem.
- **LTU-RAI** — Luleå University of Technology, for the open-source `gazebo_cave_world` ([github.com/LTU-RAI/gazebo_cave_world](https://github.com/LTU-RAI/gazebo_cave_world)).
