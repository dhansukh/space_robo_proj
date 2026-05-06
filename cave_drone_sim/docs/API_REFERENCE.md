# API Reference

**Document type:** Complete ROS 2 interface reference  
**Audience:** Developers, integrators, thesis documentation  
**Cross-references:** [ARCHITECTURE.md](ARCHITECTURE.md) | [SLAM_GUIDE.md](SLAM_GUIDE.md) | [EXPLORATION_GUIDE.md](EXPLORATION_GUIDE.md)

---

## Table of Contents

1. [Topics](#1-topics)
2. [Services](#2-services)
3. [Parameters](#3-parameters)
4. [Launch File Arguments](#4-launch-file-arguments)
5. [CLI Script Usage](#5-cli-script-usage)
6. [Custom Message Types](#6-custom-message-types)

---

## 1. Topics

### 1.1 Sensor Topics (Published by Gazebo via gz_bridge)

---

**`/lidar/points`**

| Field | Value |
|-------|-------|
| Type | `sensor_msgs/msg/PointCloud2` |
| Publisher | Gazebo gz_bridge (`drone_bringup`) |
| Subscribers | `simple_lidar_odom`, `degeneracy_detector`, `local_planner`, `octomap_builder` |
| QoS | Reliability: `BestEffort`, Durability: `Volatile`, Depth: 5 |
| Rate | 10 Hz |
| Frame | `lidar_link` |

Simulated Ouster OS0-32 LiDAR point cloud. 32 scan lines, 360° horizontal FOV, 90° vertical FOV, 0.1–20 m range. Gaussian noise model: σ = 0.02 m range noise, 0.01° angular noise.

Fields: `x`, `y`, `z` (float32), `intensity` (float32), `ring` (uint16), `timestamp` (float64).

---

**`/imu/data`**

| Field | Value |
|-------|-------|
| Type | `sensor_msgs/msg/Imu` |
| Publisher | Gazebo gz_bridge (`drone_bringup`) |
| Subscribers | `simple_lidar_odom` (for de-skewing) |
| QoS | Reliability: `BestEffort`, Durability: `Volatile`, Depth: 10 |
| Rate | 200 Hz |
| Frame | `imu_link` |

Simulated 6-axis IMU. Accelerometer range: ±16 g. Gyroscope range: ±2000 °/s. Gaussian noise: acc σ = 0.01 m/s², gyro σ = 0.001 rad/s.

---

**`/drone/ground_truth`**

| Field | Value |
|-------|-------|
| Type | `nav_msgs/msg/Odometry` |
| Publisher | Gazebo gz_bridge (`drone_bringup`) |
| Subscribers | Evaluation scripts only (not used in SLAM pipeline) |
| QoS | Reliability: `Reliable`, Durability: `Volatile`, Depth: 5 |
| Rate | 50 Hz |
| Frame | `world` → `base_link` |

Noiseless ground truth pose from Gazebo physics engine. Used exclusively for SLAM evaluation (ATE/RPE via `evo`). Do not use in the live autonomy pipeline — this would invalidate the experiment.

---

### 1.2 Drone Control Topics

---

**`/drone/cmd_vel`**

| Field | Value |
|-------|-------|
| Type | `geometry_msgs/msg/Twist` |
| Publisher | `local_planner` (20 Hz) |
| Subscriber | `drone_controller` |
| QoS | Reliability: `Reliable`, Durability: `Volatile`, Depth: 1 |
| Rate | 20 Hz (planning) / up to 50 Hz (manual teleop) |

Velocity command in the `odom` frame. `linear.x/y/z` in m/s; `angular.z` in rad/s (yaw rate). The drone controller clamps all values to the configured limits (`max_linear_velocity=2.0 m/s`, `max_angular_velocity=1.0 rad/s`).

To command the drone manually:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/drone/cmd_vel
```

---

**`/drone/battery_state`**

| Field | Value |
|-------|-------|
| Type | `sensor_msgs/msg/BatteryState` |
| Publisher | `drone_controller` |
| Subscriber | `mission_manager` |
| QoS | Reliability: `Reliable`, Depth: 1 |
| Rate | 1 Hz |

Simulated battery state. `percentage` field decreases at `battery_drain_rate` (default 0.1%/s). `voltage` is fixed at 22.2 V (6S LiPo nominal).

---

### 1.3 SLAM Topics

---

**`/slam/odometry`**

| Field | Value |
|-------|-------|
| Type | `nav_msgs/msg/Odometry` |
| Publisher | `simple_lidar_odom` / FAST-LIO2 / RTAB-Map (depending on backend) |
| Subscribers | `frontier_explorer`, `path_planner`, `local_planner`, `mission_manager` |
| QoS | Reliability: `Reliable`, Durability: `Volatile`, Depth: 10 |
| Rate | 10 Hz |
| Frame | `map` → `base_link` |

Primary SLAM pose estimate. All SLAM backends publish to this exact topic with identical QoS, enabling backend swapping without downstream configuration changes.

`pose.covariance` contains the 6×6 pose covariance matrix. During SLAM degeneracy, covariance values in degenerate directions grow significantly.

---

**`/slam/cloud_map`**

| Field | Value |
|-------|-------|
| Type | `sensor_msgs/msg/PointCloud2` |
| Publisher | `simple_lidar_odom` / FAST-LIO2 |
| Subscribers | `octomap_builder`, `map_saver`, RViz2 |
| QoS | Reliability: `BestEffort`, Durability: `Volatile`, Depth: 1 |
| Rate | 1 Hz |
| Frame | `map` |

Accumulated registered point cloud (all keyframes merged in the map frame). This is the 3D map representation for visualization. For OctoMap integration, `octomap_builder` subscribes to this topic.

---

**`/slam/degeneracy_score`**

| Field | Value |
|-------|-------|
| Type | `std_msgs/msg/Float32` |
| Publisher | `degeneracy_detector` |
| Subscribers | `mission_manager` |
| QoS | Reliability: `Reliable`, Depth: 10 |
| Rate | 10 Hz |

Eigenvalue ratio score: `λ_min / λ_max` of the point cloud covariance matrix. Range `[0.0, 1.0]`. Values < 0.01 indicate degenerate geometry. Smoothed over a sliding window of 5 scans.

---

**`/slam/degeneracy_warning`**

| Field | Value |
|-------|-------|
| Type | `std_msgs/msg/Bool` |
| Publisher | `degeneracy_detector` |
| Subscribers | `local_planner`, `mission_manager` |
| QoS | Reliability: `Reliable`, Durability: `TransientLocal` (latched), Depth: 1 |
| Rate | Event-driven (fires when threshold crossed) |

Published `True` when degeneracy score drops below `eigenvalue_threshold` (default 0.01) for more than `window_size` consecutive scans. Published `False` when recovered. Latched so late-joining subscribers receive the current state immediately.

---

### 1.4 Mapping Topics

---

**`/octomap_full_color`**

| Field | Value |
|-------|-------|
| Type | `octomap_msgs/msg/Octomap` |
| Publisher | `octomap_builder` |
| Subscribers | `frontier_explorer`, `path_planner`, `collision_checker` |
| QoS | Reliability: `Reliable`, Durability: `Volatile`, Depth: 1 |
| Rate | 1 Hz |
| Frame | `map` |

3D probabilistic occupancy map (OctoMap). Resolution: 0.2 m (default). Contains full 3D occupancy information required for frontier detection, collision checking, and path planning.

**Note:** This is a binary serialized OctoMap tree — it cannot be visualized directly in RViz2 without the `octomap_rviz_plugins` package (`ros-humble-octomap-rviz-plugins`).

---

**`/octomap_2d`**

| Field | Value |
|-------|-------|
| Type | `nav_msgs/msg/OccupancyGrid` |
| Publisher | `octomap_builder` |
| Subscribers | RViz2, Nav2 (if 2D navigation is used) |
| QoS | Reliability: `Reliable`, Depth: 1 |
| Rate | 1 Hz |
| Frame | `map` |

2D occupancy grid: Z-slice projection of the OctoMap between `z_slice_min` (-0.5 m) and `z_slice_max` (2.5 m). Any OctoMap voxel marked occupied within this Z range is projected to its (x,y) grid cell.

---

### 1.5 Exploration Topics

---

**`/exploration/frontiers`**

| Field | Value |
|-------|-------|
| Type | `visualization_msgs/msg/MarkerArray` |
| Publisher | `frontier_explorer` |
| Subscribers | RViz2 |
| QoS | Reliability: `BestEffort`, Depth: 1 |
| Rate | 2 Hz |

Visualization of all active frontier clusters. Each marker is a sphere at the cluster centroid, colored by utility score (green = high, red = low).

---

**`/exploration/best_frontier`**

| Field | Value |
|-------|-------|
| Type | `geometry_msgs/msg/PoseStamped` |
| Publisher | `frontier_explorer` |
| Subscribers | `path_planner`, `mission_manager` |
| QoS | Reliability: `Reliable`, Depth: 1 |
| Rate | 2 Hz |

Currently selected exploration goal (highest utility frontier centroid). Orientation points from drone's current position toward the frontier centroid.

---

**`/exploration/path`**

| Field | Value |
|-------|-------|
| Type | `nav_msgs/msg/Path` |
| Publisher | `path_planner` |
| Subscribers | `local_planner` |
| QoS | Reliability: `Reliable`, Depth: 1 |
| Rate | On-demand (new goal or replan request) |
| Frame | `map` |

Collision-free 3D path from current pose to the current frontier goal. Path is B-spline-smoothed and uniformly sampled at `step_size / 2` intervals. All `PoseStamped` elements in the path have valid orientations (pointing toward the next waypoint).

---

### 1.6 Mission Topics

---

**`/mission/state`**

| Field | Value |
|-------|-------|
| Type | `std_msgs/msg/String` |
| Publisher | `mission_manager` |
| Subscribers | Logging, RViz2, operator scripts |
| QoS | Reliability: `Reliable`, Durability: `TransientLocal` (latched), Depth: 1 |
| Rate | 5 Hz |

Current mission state string. One of: `IDLE`, `TAKEOFF`, `EXPLORE`, `NAVIGATE`, `RETURN_HOME`, `LAND`, `EMERGENCY`.

---

**`/mission/cmd`**

| Field | Value |
|-------|-------|
| Type | `std_msgs/msg/String` |
| Publisher | Operator / automation scripts |
| Subscriber | `mission_manager` |
| QoS | Reliability: `Reliable`, Depth: 1 |
| Rate | On-demand |

Operator command topic. Valid values: `START`, `RETURN`, `LAND`, `EMERGENCY`, `RESET`, `PAUSE`, `RESUME`.

---

## 2. Services

### `/save_map`

| Field | Value |
|-------|-------|
| Type | `std_srvs/srv/Trigger` |
| Server | `map_saver` |
| Clients | Operator CLI |

Saves the current OctoMap and registered point cloud to the `output_dir` configured in `slam_params.yaml`. The saved files are timestamped: `slam_map_<timestamp>.pcd` and `octomap_<timestamp>.bt`.

```bash
ros2 service call /save_map std_srvs/srv/Trigger '{}'
```

---

### `/plan_path`

| Field | Value |
|-------|-------|
| Type | `cave_drone_interfaces/srv/PlanPath` |
| Server | `path_planner` |
| Clients | `mission_manager` |

Plan a collision-free 3D path from start to goal using RRT*.

**Request:**
```
geometry_msgs/PoseStamped start    # Start pose (use slam/odometry for current pose)
geometry_msgs/PoseStamped goal     # Goal pose
float32 timeout_sec                # Maximum planning time (default 10.0)
```

**Response:**
```
nav_msgs/Path path                 # Planned path (empty if planning failed)
bool success                       # True if path found
string message                     # Human-readable status
```

```bash
# Example: plan from origin to (10, 5, 2)
ros2 service call /plan_path cave_drone_interfaces/srv/PlanPath \
    "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0, y: 0, z: 2}}},
      goal: {header: {frame_id: 'map'}, pose: {position: {x: 10, y: 5, z: 2}}},
      timeout_sec: 10.0}"
```

---

### `/collision_check`

| Field | Value |
|-------|-------|
| Type | `cave_drone_interfaces/srv/CollisionCheck` |
| Server | `collision_checker` |
| Clients | `path_planner`, `local_planner` |

Query whether a 3D point is collision-free given a safety margin.

**Request:**
```
geometry_msgs/Point point          # 3D point to check (in map frame)
float32 safety_margin              # Inflation radius in metres
```

**Response:**
```
bool collision_free                # True if point is free
float32 nearest_obstacle_distance  # Distance to nearest occupied voxel
```

---

### `/generate_cave`

| Field | Value |
|-------|-------|
| Type | `cave_drone_interfaces/srv/GenerateCave` |
| Server | `cave_generator_node` |
| Clients | Launch files, automation scripts |

Generate a new cave with specified parameters. Blocks until generation is complete.

**Request:**
```
int32 seed
int32 num_nodes
float32 bounds_x
float32 bounds_y
float32 bounds_z
float32 resolution
float32 noise_amplitude
float32 noise_frequency
float32 stalactite_density
float32 rubble_density
bool export_gazebo
```

**Response:**
```
bool success
string output_dir
string world_file_path
string message
```

---

## 3. Parameters

### 3.1 `cave_generator_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_nodes` | `int` | `20` | Graph node count |
| `bounds_x` | `float` | `80.0` | X bounding box (m) |
| `bounds_y` | `float` | `80.0` | Y bounding box (m) |
| `bounds_z` | `float` | `25.0` | Z bounding box (m) |
| `num_extra_loops` | `int` | `3` | Extra loop edges beyond MST |
| `resolution` | `float` | `0.3` | SDF voxel size (m) |
| `noise_amplitude` | `float` | `0.25` | Wall noise amplitude (fraction of radius) |
| `noise_frequency` | `float` | `0.08` | Noise spatial frequency (1/m) |
| `stalactite_density` | `float` | `0.3` | Stalactite coverage [0–1] |
| `rubble_density` | `float` | `0.2` | Rubble coverage [0–1] |
| `output_dir` | `string` | `"/tmp/cave_generator_output"` | Output directory |
| `export_gazebo` | `bool` | `true` | Write Gazebo model files |
| `seed` | `int` | `42` | Random seed |

### 3.2 `drone_controller`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_velocity` | `float` | `2.0` | Maximum velocity (m/s) |
| `max_angular_velocity` | `float` | `1.0` | Maximum yaw rate (rad/s) |
| `mass` | `float` | `1.5` | Drone mass (kg) |
| `hover_thrust` | `float` | `14.715` | Hover force (N) = mass × g |
| `pid_linear.kp` | `float` | `5.0` | Linear velocity P gain |
| `pid_linear.ki` | `float` | `0.1` | Linear velocity I gain |
| `pid_linear.kd` | `float` | `2.0` | Linear velocity D gain |
| `pid_angular.kp` | `float` | `3.0` | Yaw rate P gain |
| `pid_angular.ki` | `float` | `0.05` | Yaw rate I gain |
| `pid_angular.kd` | `float` | `1.0` | Yaw rate D gain |

### 3.3 `simple_lidar_odom`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voxel_size` | `float` | `0.1` | Downsampling voxel size (m) |
| `icp_max_distance` | `float` | `1.0` | ICP correspondence distance (m) |
| `icp_max_iter` | `int` | `30` | ICP iterations per scan |
| `keyframe_distance` | `float` | `0.5` | Keyframe translation threshold (m) |
| `keyframe_rotation` | `float` | `0.26` | Keyframe rotation threshold (rad) |
| `local_map_size` | `int` | `50` | Sliding-window keyframe count |
| `map_frame` | `string` | `"map"` | Map TF frame |
| `odom_frame` | `string` | `"odom"` | Odometry TF frame |
| `base_frame` | `string` | `"base_link"` | Robot body frame |

### 3.4 `octomap_builder`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | `float` | `0.2` | OctoMap voxel size (m) |
| `max_range` | `float` | `30.0` | Max ray casting range (m) |
| `sensor_model_hit` | `float` | `0.7` | P(occupied | hit) |
| `sensor_model_miss` | `float` | `0.4` | P(free | miss) |
| `map_frame` | `string` | `"map"` | Map TF frame |
| `publish_rate_hz` | `float` | `1.0` | OctoMap publish rate |
| `z_slice_min` | `float` | `-0.5` | 2D projection Z minimum (m) |
| `z_slice_max` | `float` | `2.5` | 2D projection Z maximum (m) |

### 3.5 `degeneracy_detector`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `eigenvalue_threshold` | `float` | `0.01` | λ_min/λ_max threshold for warning |
| `window_size` | `int` | `5` | Score smoothing window (scans) |
| `voxel_size` | `float` | `0.3` | Downsampling for covariance (m) |
| `min_points` | `int` | `20` | Minimum points for valid score |

### 3.6 `frontier_explorer`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voxel_resolution` | `float` | `0.3` | Must match SLAM voxel size (m) |
| `frontier_min_size` | `int` | `5` | Minimum frontier cluster voxels |
| `utility_weight_info` | `float` | `1.0` | Information gain weight (w1) |
| `utility_weight_distance` | `float` | `0.5` | Distance penalty weight (w2) |
| `utility_weight_heading` | `float` | `0.3` | Heading alignment weight (w3) |
| `max_exploration_range` | `float` | `50.0` | Maximum frontier range (m) |
| `dbscan_eps` | `float` | `1.0` | DBSCAN neighbourhood radius (m) |
| `dbscan_min_samples` | `int` | `3` | DBSCAN minimum cluster size |
| `raytrace_samples` | `int` | `50` | Rays for info-gain estimation |
| `publish_rate` | `float` | `2.0` | Frontier evaluation rate (Hz) |

### 3.7 `path_planner`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `step_size` | `float` | `0.5` | RRT* extension step (m) |
| `max_iterations` | `int` | `5000` | Maximum RRT* iterations |
| `goal_bias` | `float` | `0.1` | Fraction of samples toward goal |
| `rewiring_radius` | `float` | `2.0` | RRT* rewiring radius (m) |
| `safety_margin` | `float` | `0.4` | Obstacle inflation (m) |
| `goal_tolerance` | `float` | `0.3` | Goal reach distance (m) |
| `timeout_sec` | `float` | `10.0` | Planning timeout (s) |
| `astar_grid_resolution` | `float` | `0.5` | A* fallback voxel size (m) |
| `smooth_shortcut_passes` | `int` | `3` | Path shortcutting iterations |
| `smooth_spline_k` | `int` | `3` | B-spline degree |

### 3.8 `local_planner`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_speed` | `float` | `1.5` | Maximum velocity (m/s) |
| `max_angular_speed` | `float` | `1.0` | Maximum yaw rate (rad/s) |
| `obstacle_influence_distance` | `float` | `2.0` | APF repulsion radius d0 (m) |
| `emergency_stop_distance` | `float` | `0.3` | Hard stop distance (m) |
| `k_attractive` | `float` | `1.0` | APF attractive gain |
| `k_repulsive` | `float` | `0.5` | APF repulsive gain |
| `path_deviation_threshold` | `float` | `2.0` | Replan trigger distance (m) |
| `lookahead_distance` | `float` | `1.5` | Forward collision check (m) |
| `waypoint_reach_tolerance` | `float` | `0.4` | Waypoint advance distance (m) |
| `control_rate` | `float` | `20.0` | Control loop rate (Hz) |

### 3.9 `mission_manager`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration_altitude` | `float` | `2.0` | Hover height (m above origin) |
| `takeoff_speed` | `float` | `0.5` | Ascent speed (m/s) |
| `landing_speed` | `float` | `0.3` | Descent speed (m/s) |
| `battery_drain_rate` | `float` | `0.001` | Fraction per second |
| `battery_return_threshold` | `float` | `30.0` | RTH trigger (%) |
| `battery_land_threshold` | `float` | `10.0` | Force-land trigger (%) |
| `home_position` | `float[]` | `[0.0, 0.0, 0.0]` | Home x, y, z (m) |
| `goal_reach_tolerance` | `float` | `0.5` | Mission goal distance (m) |
| `slam_degeneracy_threshold` | `float` | `0.1` | Emergency trigger score |
| `home_reach_tolerance` | `float` | `0.5` | Home distance (m) |
| `state_publish_rate` | `float` | `5.0` | State publication rate (Hz) |

---

## 4. Launch File Arguments

### `full_demo.launch.py`

The top-level launch file for the complete autonomous exploration demo.

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `seed` | `int` | `42` | Cave random seed |
| `num_nodes` | `int` | `20` | Cave graph node count |
| `slam_backend` | `string` | `simple` | `simple`, `fastlio`, or `rtabmap` |
| `rviz` | `bool` | `true` | Launch RViz2 |
| `use_sim_time` | `bool` | `true` | Use Gazebo simulation clock |
| `bounds_x` | `float` | `80.0` | Cave X dimension (m) |
| `bounds_y` | `float` | `80.0` | Cave Y dimension (m) |
| `bounds_z` | `float` | `25.0` | Cave Z dimension (m) |
| `noise_amplitude` | `float` | `0.25` | Cave wall roughness |

Example:
```bash
ros2 launch cave_drone_sim full_demo.launch.py \
    seed:=99 num_nodes:=30 slam_backend:=rtabmap rviz:=true
```

### `generate_cave.launch.py`

Cave generation only (no Gazebo, no SLAM).

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `seed` | `int` | `42` | Random seed |
| `num_nodes` | `int` | `20` | Graph nodes |
| `resolution` | `float` | `0.3` | Voxel size (m) |
| `output_dir` | `string` | `"/tmp/cave_generator_output"` | Output path |
| `export_gazebo` | `bool` | `true` | Write Gazebo files |

### `drone_sim.launch.py`

Drone + Gazebo only (requires pre-generated cave).

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `world_file` | `string` | (required) | Path to cave world `.sdf` |
| `rviz` | `bool` | `false` | Launch RViz2 |

### `drone_only.launch.py`

Drone + empty world for SLAM debugging.

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `rviz` | `bool` | `true` | Launch RViz2 |

### `slam_simple.launch.py` / `slam_fastlio.launch.py` / `slam_rtabmap.launch.py`

SLAM only (requires running Gazebo simulation).

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `params_file` | `string` | (package default) | Path to params YAML |
| `use_sim_time` | `bool` | `true` | Use simulation clock |

### `explore.launch.py`

Exploration planner only (requires SLAM to be running).

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `params_file` | `string` | (package default) | Exploration params YAML |

---

## 5. CLI Script Usage

### `scripts/generate_cave.py`

Standalone cave generation script (no ROS 2 required).

```bash
python3 scripts/generate_cave.py \
    --seed 42 \
    --nodes 20 \
    --bounds 80 80 25 \
    --resolution 0.3 \
    --noise-amplitude 0.25 \
    --noise-frequency 0.08 \
    --stalactites 0.3 \
    --rubble 0.2 \
    --output /tmp/my_cave \
    --visualize       # Open Open3D window showing mesh
```

**Output:** Writes `model.sdf`, `cave_visual.dae`, `cave_collision.stl` to `--output` directory.

### `scripts/run_slam_experiment.py`

Automated SLAM evaluation across multiple seeds and backends.

```bash
python3 scripts/run_slam_experiment.py \
    --seeds 0 42 99 7 123 \
    --backends simple fastlio rtabmap \
    --repetitions 3 \
    --cave-nodes 20 \
    --output-dir results/slam_experiment/ \
    --timeout 600    # Max seconds per run
```

### `scripts/analyze_results.py`

Compute ATE/RPE statistics and generate plots from experiment results.

```bash
python3 scripts/analyze_results.py \
    --results-dir results/slam_experiment/ \
    --output-dir results/figures/ \
    --format pdf      # or png, svg
```

**Output:** `ate_table.csv`, `rpe_table.csv`, `ate_boxplot.pdf`, `trajectory_plots/`.

### `scripts/bag_to_tum.py`

Convert ROS 2 bag to TUM format for `evo` evaluation.

```bash
python3 scripts/bag_to_tum.py \
    --bag <bag_directory> \
    --est-topic /slam/odometry \
    --gt-topic /drone/ground_truth \
    --max-diff 0.05 \       # Max timestamp difference (s)
    --output-dir results/run_01/
```

**Output:** `estimated.txt`, `ground_truth.txt` in TUM format (`timestamp tx ty tz qx qy qz qw`).

---

## 6. Custom Message Types

All custom message and service types are defined in the `cave_drone_interfaces` package.

### `cave_drone_interfaces/msg/Frontier.msg`

```
std_msgs/Header header
geometry_msgs/Point centroid        # Frontier cluster centroid (map frame)
float32 utility_score               # Computed utility U(f)
float32 information_gain            # Estimated information gain (voxels)
float32 distance                    # Distance from drone (m)
int32 cluster_size                  # Number of frontier voxels in cluster
geometry_msgs/Point[] voxel_centers # Individual frontier voxel positions
```

### `cave_drone_interfaces/msg/MissionStatus.msg`

```
std_msgs/Header header
string state                        # Current FSM state string
float32 battery_percentage          # Current battery level [0-100]
float32 exploration_coverage        # Estimated explored volume [0-1]
int32 active_frontiers              # Number of active frontier clusters
float32 slam_degeneracy_score       # Current eigenvalue ratio score
bool slam_degenerate                # True if degeneracy warning active
```

### `cave_drone_interfaces/srv/PlanPath.srv`

```
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
float32 timeout_sec
---
nav_msgs/Path path
bool success
string message
float32 path_length_m
```

### `cave_drone_interfaces/srv/CollisionCheck.srv`

```
geometry_msgs/Point point
float32 safety_margin
---
bool collision_free
float32 nearest_obstacle_distance
```

### `cave_drone_interfaces/srv/GenerateCave.srv`

```
int32 seed
int32 num_nodes
float32 bounds_x
float32 bounds_y
float32 bounds_z
float32 resolution
float32 noise_amplitude
float32 noise_frequency
float32 stalactite_density
float32 rubble_density
bool export_gazebo
string output_dir
---
bool success
string output_dir
string world_file_path
float32 generation_time_sec
int32 mesh_vertices
int32 mesh_faces
string message
```
