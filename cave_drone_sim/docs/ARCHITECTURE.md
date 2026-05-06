# CaveDroneSim System Architecture

**Document type:** Technical architecture reference  
**Audience:** Thesis readers, collaborators, contributors  
**Cross-references:** [README.md](../README.md) | [API_REFERENCE.md](API_REFERENCE.md) | [SLAM_GUIDE.md](SLAM_GUIDE.md) | [EXPLORATION_GUIDE.md](EXPLORATION_GUIDE.md)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [ROS 2 Node Graph](#2-ros-2-node-graph)
3. [Complete Topic and Service Map](#3-complete-topic-and-service-map)
4. [TF Tree](#4-tf-tree)
5. [Data Flow Diagrams](#5-data-flow-diagrams)
6. [Mission Manager State Machine](#6-mission-manager-state-machine)
7. [Timing Diagram](#7-timing-diagram)
8. [Design Decisions and Trade-offs](#8-design-decisions-and-trade-offs)

---

## 1. System Overview

CaveDroneSim implements a five-stage autonomous exploration pipeline running entirely within ROS 2 Humble on a simulated Gazebo Harmonic environment. Each stage maps to one or more ROS 2 packages and a set of nodes.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CaveDroneSim System Architecture                     │
│                            (ROS 2 Humble, Gazebo Harmonic)                   │
│                                                                              │
│  ┌──────────────┐    ┌───────────────┐    ┌───────────────┐                 │
│  │   Stage 1    │    │    Stage 2    │    │    Stage 3    │                 │
│  │cave_generator│    │drone_bringup  │    │slam_pipeline  │                 │
│  │              │    │               │    │               │                 │
│  │ cave_graph   │    │ drone_ctrl    │    │ lidar_odom    │                 │
│  │ cave_mesh    │───►│ tf_broadcaster│───►│ octomap_bld   │                 │
│  │ obs_placer   │    │               │    │ degeneracy    │                 │
│  │ gz_exporter  │    │  [Gazebo Sim] │    │ map_saver     │                 │
│  └──────────────┘    └───────┬───────┘    └──────┬────────┘                 │
│                              │                   │                          │
│                              │ /lidar/points      │ /slam/odometry           │
│                              │ /imu/data          │ /octomap_full_color      │
│                              │ /drone/pose        │ /slam/degeneracy_score   │
│                              │                   │                          │
│              ┌───────────────┴───────────────────┴────────────┐            │
│              │                  Stage 4 + 5                    │            │
│              │             exploration_planner                  │            │
│              │                                                  │            │
│              │  frontier_explorer ──► /exploration/frontiers   │            │
│              │  path_planner      ──► /exploration/path        │            │
│              │  local_planner     ──► /drone/cmd_vel           │            │
│              │  mission_manager   ──► /mission/state           │            │
│              │  collision_checker ──► (internal service)       │            │
│              └──────────────────────────────────────────────────┘            │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Package Responsibilities

| Package | Stage | Primary Responsibility | Key External Interfaces |
|---------|-------|------------------------|------------------------|
| `cave_generator` | 1 | Procedural cave mesh + Gazebo world file | Writes `.sdf`/`.dae`/`.stl` to disk |
| `drone_bringup` | 2 | Drone physics, sensors, control bridge | Gazebo `gz-transport`, `/drone/cmd_vel` |
| `slam_pipeline` | 3 | LiDAR odometry, OctoMap, health monitoring | `/slam/odometry`, `/octomap_full_color` |
| `exploration_planner` | 4–5 | Frontier exploration, path planning, mission FSM | `/mission/state`, `/drone/cmd_vel` |

---

## 2. ROS 2 Node Graph

### Node List

| Node | Package | Type | Rate |
|------|---------|------|------|
| `cave_generator_node` | `cave_generator` | Python | One-shot (on request) |
| `drone_controller` | `drone_bringup` | Python | 50 Hz |
| `tf_broadcaster` | `drone_bringup` | Python | 50 Hz |
| `simple_lidar_odom` | `slam_pipeline` | Python | 10 Hz (scan-driven) |
| `octomap_builder` | `slam_pipeline` | Python | 1 Hz (publish) / scan-driven (integrate) |
| `degeneracy_detector` | `slam_pipeline` | Python | 10 Hz |
| `map_saver` | `slam_pipeline` | Python | On-demand / periodic |
| `frontier_explorer` | `exploration_planner` | Python | 2 Hz |
| `path_planner` | `exploration_planner` | Python | On-demand (goal-driven) |
| `local_planner` | `exploration_planner` | Python | 20 Hz |
| `mission_manager` | `exploration_planner` | Python | 5 Hz |
| `collision_checker` | `exploration_planner` | Python | Internal (service server) |

### Node Connection Diagram

```
  [Gazebo Harmonic]
    │  /lidar/points (PointCloud2, 10 Hz)
    │  /imu/data (Imu, 200 Hz)
    │  /drone/ground_truth (Odometry, 50 Hz)
    │
    ├──────────────────────────────────────────────────────┐
    │                                                      │
    ▼                                                      ▼
  simple_lidar_odom                              drone_controller
    │  (pub) /slam/odometry (10 Hz)                │ (sub) /drone/cmd_vel
    │  (pub) /slam/cloud_map (keyframes)            │ (pub) /gz/cmd_vel (gz-transport)
    │                                              │
    ├──► octomap_builder                          tf_broadcaster
    │      │  (pub) /octomap_full_color (1 Hz)      │ (pub) /tf (map→odom→base_link)
    │      │  (pub) /octomap_2d (1 Hz)              │ (pub) /tf_static (base→lidar)
    │      │
    ├──► degeneracy_detector
    │      │  (pub) /slam/degeneracy_score (10 Hz)
    │      │  (pub) /slam/degeneracy_warning (Bool, latched)
    │
    ├──► map_saver
    │      │  (srv) /save_map
    │
    ▼
  frontier_explorer
    │  (sub) /octomap_full_color
    │  (sub) /slam/odometry
    │  (pub) /exploration/frontiers (MarkerArray, 2 Hz)
    │  (pub) /exploration/best_frontier (PoseStamped, 2 Hz)
    │
    ▼
  path_planner
    │  (sub) /exploration/best_frontier
    │  (sub) /octomap_full_color
    │  (sub) /slam/odometry
    │  (pub) /exploration/path (Path, on-demand)
    │  (srv) /plan_path (PlanPath.srv)
    │
    ▼
  local_planner
    │  (sub) /exploration/path
    │  (sub) /lidar/points (for real-time APF)
    │  (sub) /slam/odometry
    │  (sub) /slam/degeneracy_warning
    │  (pub) /drone/cmd_vel (Twist, 20 Hz)
    │  (pub) /local_planner/force_vectors (MarkerArray, debug)
    │
    ▼
  mission_manager
    │  (sub) /slam/degeneracy_warning
    │  (sub) /slam/odometry
    │  (sub) /drone/battery_state (BatteryState, 1 Hz)
    │  (sub) /mission/cmd (String)
    │  (pub) /mission/state (String, 5 Hz)
    │  (srv) /collision_check (CollisionCheck.srv) ──► collision_checker
```

---

## 3. Complete Topic and Service Map

### Topics

| Topic Name | Message Type | Publisher | Subscriber(s) | QoS | Rate |
|-----------|--------------|-----------|---------------|-----|------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | Gazebo gz_bridge | `simple_lidar_odom`, `degeneracy_detector`, `local_planner` | Sensor (BestEffort, depth 5) | 10 Hz |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo gz_bridge | `simple_lidar_odom` | Sensor (BestEffort, depth 10) | 200 Hz |
| `/drone/ground_truth` | `nav_msgs/Odometry` | Gazebo gz_bridge | (evaluation only) | Reliable, depth 5 | 50 Hz |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | `local_planner` | `drone_controller` | Reliable, depth 1 | 20 Hz |
| `/drone/battery_state` | `sensor_msgs/BatteryState` | `drone_controller` | `mission_manager` | Reliable, depth 1 | 1 Hz |
| `/slam/odometry` | `nav_msgs/Odometry` | `simple_lidar_odom` | `frontier_explorer`, `path_planner`, `local_planner`, `mission_manager` | Reliable, depth 10 | 10 Hz |
| `/slam/cloud_map` | `sensor_msgs/PointCloud2` | `simple_lidar_odom` | (visualization, map_saver) | BestEffort, depth 1 | 1 Hz |
| `/slam/degeneracy_score` | `std_msgs/Float32` | `degeneracy_detector` | `mission_manager` | Reliable, depth 10 | 10 Hz |
| `/slam/degeneracy_warning` | `std_msgs/Bool` | `degeneracy_detector` | `local_planner`, `mission_manager` | Reliable, latched | Event-driven |
| `/octomap_full_color` | `octomap_msgs/Octomap` | `octomap_builder` | `frontier_explorer`, `path_planner`, `collision_checker` | Reliable, depth 1 | 1 Hz |
| `/octomap_2d` | `nav_msgs/OccupancyGrid` | `octomap_builder` | (visualization) | Reliable, depth 1 | 1 Hz |
| `/exploration/frontiers` | `visualization_msgs/MarkerArray` | `frontier_explorer` | (RViz2) | BestEffort, depth 1 | 2 Hz |
| `/exploration/best_frontier` | `geometry_msgs/PoseStamped` | `frontier_explorer` | `path_planner`, `mission_manager` | Reliable, depth 1 | 2 Hz |
| `/exploration/path` | `nav_msgs/Path` | `path_planner` | `local_planner` | Reliable, depth 1 | On-demand |
| `/local_planner/force_vectors` | `visualization_msgs/MarkerArray` | `local_planner` | (RViz2) | BestEffort, depth 1 | 5 Hz |
| `/mission/state` | `std_msgs/String` | `mission_manager` | (logging, RViz2) | Reliable, latched | 5 Hz |
| `/mission/cmd` | `std_msgs/String` | (operator / scripts) | `mission_manager` | Reliable, depth 1 | On-demand |
| `/tf` | `tf2_msgs/TFMessage` | `tf_broadcaster`, `simple_lidar_odom` | All nodes using TF2 | Reliable, depth 100 | 50 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | `tf_broadcaster` | All nodes | Reliable, transient local | On change |

### Services

| Service Name | Type | Server | Client(s) | Description |
|-------------|------|--------|-----------|-------------|
| `/save_map` | `std_srvs/Trigger` | `map_saver` | (CLI / operator) | Saves current OctoMap and point cloud to disk |
| `/plan_path` | `cave_drone_interfaces/PlanPath` | `path_planner` | `mission_manager` | On-demand path plan from current pose to goal |
| `/collision_check` | `cave_drone_interfaces/CollisionCheck` | `collision_checker` | `path_planner`, `local_planner` | Check if a 3D pose or segment is collision-free |
| `/generate_cave` | `cave_drone_interfaces/GenerateCave` | `cave_generator_node` | (launch file / CLI) | Generate a new cave with given parameters |

### Custom Message/Service Types (`cave_drone_interfaces`)

```
# msg/Frontier.msg
std_msgs/Header header
geometry_msgs/Point centroid
float32 utility_score
float32 information_gain
float32 distance
int32 cluster_size

# srv/PlanPath.srv
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
float32 timeout_sec
---
nav_msgs/Path path
bool success
string message

# srv/CollisionCheck.srv
geometry_msgs/Point point
float32 safety_margin
---
bool collision_free
float32 nearest_obstacle_distance

# srv/GenerateCave.srv
int32 seed
int32 num_nodes
float32 bounds_x
float32 bounds_y
float32 bounds_z
---
bool success
string output_dir
string world_file_path
```

---

## 4. TF Tree

The TF tree establishes the complete rigid-body transform chain required for sensor data fusion and navigation. All transforms are maintained in the ROS 2 TF2 library.

```
world (Gazebo inertial frame)
└── map
    └── odom
        └── base_link           (drone body centre, origin at CoM)
            ├── base_footprint  (projection of base_link onto Z=0 plane)
            ├── lidar_link      (Ouster OS0-32 centre; 0.1m above base_link)
            │   └── lidar_optical_frame
            ├── imu_link        (IMU; co-located with base_link in sim)
            └── camera_link     (depth camera; 0.15m forward, 0.05m down)
                └── camera_optical_frame
```

### Transform Sources

| Transform | Publisher Node | Type | Notes |
|-----------|---------------|------|-------|
| `world → map` | `simple_lidar_odom` | Dynamic | Set to identity; `map` is origin of SLAM coordinate system |
| `map → odom` | `simple_lidar_odom` | Dynamic, 10 Hz | Corrected by SLAM loop closure |
| `odom → base_link` | `drone_controller` | Dynamic, 50 Hz | Raw odometry integration |
| `base_link → lidar_link` | `tf_broadcaster` | Static | Fixed offset, loaded from URDF |
| `base_link → imu_link` | `tf_broadcaster` | Static | Fixed offset |
| `base_link → camera_link` | `tf_broadcaster` | Static | Fixed offset |

### Frame Conventions

- **`map`** — Global SLAM frame. Z-up. Origin at the cave entry point (takeoff position).
- **`odom`** — Continuous odometry frame. No jumps but drifts. Used by the local planner.
- **`base_link`** — Body-fixed frame. X forward, Y left, Z up (ROS REP-103).
- **`lidar_link`** — LiDAR sensor frame. Z up, X forward.

---

## 5. Data Flow Diagrams

### 5.1 Exploration Mode (Normal Operation)

```
  Gazebo Harmonic
  ┌──────────────────────────────────┐
  │  LiDAR 10Hz  │  IMU 200Hz        │
  └──────┬───────┴────────┬──────────┘
         │                │
         ▼                ▼
  simple_lidar_odom (ICP + keyframes)
         │
         ├──► /slam/odometry (10Hz) ──────────────────────────┐
         │                                                      │
         ├──► octomap_builder ──► /octomap_full_color (1Hz)   │
         │                              │                      │
         └──► degeneracy_detector       │                      │
               │                        │                      │
               └──► /slam/degeneracy_*  │                      │
                          │              │                      │
                          ▼              ▼                      ▼
                    mission_manager  frontier_explorer    local_planner
                          │              │                      ▲
                          │              ▼                      │
                          │         path_planner                │
                          │              │                      │
                          │              └──► /exploration/path─┘
                          │
                          └──► state: EXPLORE → NAVIGATE → EXPLORE ...
```

### 5.2 Navigation Mode (Path Following)

```
  /exploration/best_frontier (PoseStamped)
         │
         ▼
  path_planner
    ├── Queries /octomap_full_color for collision
    ├── Runs RRT* (max 5000 iter, 10s timeout)
    ├── Falls back to A* on OctoMap if timeout
    └── Publishes /exploration/path (Path)
         │
         ▼
  local_planner (20 Hz)
    ├── Reads next waypoint from path
    ├── Computes APF: F_att = k_att * (goal - pos)
    ├── For each LiDAR point within d0=2.0m:
    │     F_rep += k_rep * (1/d - 1/d0) * (1/d²) * direction
    ├── F_total = F_att + F_rep
    ├── Saturates at max_speed = 1.5 m/s
    └── Publishes /drone/cmd_vel (Twist)
         │
         ▼
  drone_controller (50 Hz)
    ├── PID loop: velocity_error = cmd_vel - current_vel
    └── gz-transport: actuator commands → Gazebo
```

### 5.3 SLAM Test Mode (No Exploration)

```
  ros2 launch slam_pipeline slam_simple.launch.py

  teleop_keyboard ──► /drone/cmd_vel ──► drone_controller ──► Gazebo

  Gazebo ──► /lidar/points ──► simple_lidar_odom ──► /slam/odometry
                                                   ──► /slam/cloud_map
         ──► /drone/ground_truth ──────────────────────────────────────┐
                                                                       │
         ros2 bag record /slam/odometry /drone/ground_truth /tf        │
         evo_ape bag *.bag /drone/ground_truth /slam/odometry ─────────┘
```

---

## 6. Mission Manager State Machine

The `mission_manager` node implements a 7-state finite-state machine that governs the drone's high-level behavior throughout the mission lifecycle.

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                   Mission Manager State Machine                      │
  └─────────────────────────────────────────────────────────────────────┘

             START command                    battery > 10%
                 │                          exploration_alt reached
                 ▼                                 │
     ┌────────┐  cmd:START   ┌─────────┐           │    no frontiers
     │  IDLE  │─────────────►│ TAKEOFF │───────────►  (or cmd:EXPLORE)
     └────────┘              └─────────┘           │
          ▲                                        ▼
          │                               ┌──────────────┐
          │                               │   EXPLORE    │
          │              frontier found   │              │◄─────────┐
          │              (best_frontier   │ frontier_    │          │
          │               published)      │ explorer     │          │
          │                    │          │ running      │          │
          │                    ▼          └──────┬───────┘          │
          │             ┌───────────┐            │ goal selected     │
          │             │ NAVIGATE  │◄───────────┘                  │
          │             │           │                                │
          │             │ path_plan │                                │
          │             │ local_ctrl│──── goal reached ─────────────┘
          │             └─────┬─────┘
          │                   │ battery < 30%                        
          │                   ▼         OR  cmd:RETURN
          │          ┌──────────────────┐
          │          │  RETURN_HOME     │
          │          │                  │
          │          │  RRT* to home    │
          │          │  position        │
          │          └────────┬─────────┘
          │                   │ home reached (< 0.5 m)
          │                   ▼
          │          ┌──────────────────┐
          │          │      LAND        │
          │          │                  │
          │          │  descend at      │
          │          │  0.3 m/s         │
          └──────────┴──────────────────┘
                              │ z < 0.1 m
                              ▼ (back to IDLE)

  ─ ─ ─ ─ ─ ─ ─ ─  Emergency Transitions  ─ ─ ─ ─ ─ ─ ─ ─ ─

  ANY_STATE ──── degeneracy_score < 0.01 ──────►  ┌───────────┐
                     (persistent, 5 scans)         │ EMERGENCY │
  ANY_STATE ──── battery < 10%            ──────►  │           │
                                                   │  hold     │
  ANY_STATE ──── cmd:EMERGENCY            ──────►  │  position │
                                                   └───────────┘
```

### State Descriptions

| State | Entry Condition | Behaviour | Exit Condition |
|-------|----------------|-----------|----------------|
| `IDLE` | Initial state / post-landing | Do nothing; wait for `START` command | `cmd:START` received |
| `TAKEOFF` | `cmd:START` from `IDLE` | Ascend at `takeoff_speed=0.5 m/s` to `exploration_altitude=2.0 m` | Altitude reached |
| `EXPLORE` | Altitude reached / navigation goal completed | Run `frontier_explorer`; select best frontier by utility score | Best frontier selected |
| `NAVIGATE` | Frontier goal selected | Run `path_planner` → `local_planner` to reach frontier | Frontier reached OR battery < 30% |
| `RETURN_HOME` | Battery < 30% OR `cmd:RETURN` | Plan and fly path back to `home_position=[0,0,0]` | Home reached (< 0.5 m) |
| `LAND` | Home reached in `RETURN_HOME` OR `cmd:LAND` | Descend at `landing_speed=0.3 m/s` | Altitude < 0.1 m |
| `EMERGENCY` | Degeneracy score < 0.01 (persistent) OR battery < 10% | Hold position; broadcast alert; await operator `RESET` | `cmd:RESET` from operator |

---

## 7. Timing Diagram

The following diagram shows the typical message flow during one complete exploration cycle: from frontier detection to waypoint reach.

```
Time →  0ms     100ms    200ms    300ms    400ms    500ms   ...   2000ms  2100ms
         │        │        │        │        │        │             │       │
LiDAR    ├──scan──┤        ├──scan──┤        ├──scan──┤             ├──scan─┤
         │                 │                 │                      │
slam_odom│        ├──odom──┤        ├──odom──┤                      │──odom─┤
         │                                                          │
octomap  │                                   ├──────── update ──────┤
frontier │                                            │             │──eval─┤
         │                                            ├─ best_frontier pub
         │                                                          │
path_plan│                                                          ├──RRT*─┐
         │                                                          │       │ (10s max)
         │                                                          │       ▼
         │                                                    ├── /exploration/path
local_pl │  cmd_vel  cmd_vel  cmd_vel  cmd_vel  cmd_vel  ...   cmd_vel  cmd_vel
         │  20Hz     20Hz     20Hz     20Hz     20Hz          20Hz     20Hz

─────────── Key latencies ────────────────────────────────────────────────────
  LiDAR scan → odometry update:         ~25ms  (ICP iteration)
  Odometry → OctoMap update:            ~100ms (ray casting, 1Hz cycle)
  Frontier evaluation:                  ~50ms  (DBSCAN + utility scoring)
  RRT* planning (20m corridor):         ~500ms–5s (5000 iter)
  APF cmd_vel publish:                  ~5ms   (vector computation)
  Total: scan to first motion command:  ~1–6s  (dominated by path planning)
```

---

## 8. Design Decisions and Trade-offs

### 8.1 Why Gazebo Harmonic over Isaac Sim or Unity

The DARPA Subterranean Challenge — the most directly relevant precedent for this research — was run exclusively on Ignition Gazebo (the predecessor to Gazebo Harmonic). All top competing teams (CERBERUS, CSIRO Data61, MARBLE, CTU-CRAS-NORLAB) used the Ignition/Gazebo + ROS ecosystem. The open-source cave tile library, validated SLAM packages, and SubT robot models are all directly reusable. Gazebo Harmonic scored 4.60/5 versus Isaac Sim's 3.72/5 on a weighted evaluation matrix covering LiDAR quality, PX4 integration, ROS 2 maturity, and community support. See [research/sim_platform_comparison.md](../research/sim_platform_comparison.md) for the full comparison.

**Trade-off:** Isaac Sim offers superior photorealism for camera-based SLAM. For a LiDAR-centric thesis, this advantage is outweighed by Gazebo's superior ROS 2 integration and SubT heritage.

### 8.2 Why FAST-LIO2 as Front-End

FAST-LIO2 uses direct scan-to-map registration (no feature extraction), which is critical in featureless cave environments where LOAM-style feature detectors fail to find sufficient edge/plane features. Its iEKF-based tight LiDAR-IMU fusion runs at ~25 ms/scan on a laptop CPU, well within the onboard compute budget. The ikd-Tree data structure enables efficient incremental map updates without rebuilding a voxel grid.

**Trade-off:** FAST-LIO2 has no loop closure. Without RTAB-Map integration, trajectory drift accumulates at ~1–5% of distance traveled. The thesis evaluates both standalone FAST-LIO2 (to measure raw front-end accuracy) and FAST-LIO2 + RTAB-Map (to measure full SLAM quality).

### 8.3 Why RTAB-Map as Back-End

RTAB-Map is the only fully ROS 2 Humble-native SLAM system that provides loop closure, OctoMap output, and a Nav2-compatible 2D occupancy grid simultaneously. Its memory management (Short-Term Memory / Long-Term Memory) prevents working memory from growing unboundedly during long cave traversals. The DLR SCOUT cave rover paper (i-SAIRAS 2024) validated RTAB-Map specifically for cave applications.

**Trade-off:** RTAB-Map's loop closure in purely featureless environments (smooth cave walls, no camera data) relies on ICP-based submap alignment, which can fail in highly repetitive tunnel geometry. Scan Context descriptor integration would improve this but is not natively supported.

### 8.4 Why Frontier-Based Exploration over GBPlanner2/TARE

GBPlanner2 (CERBERUS team) and TARE (CMU, RSS 2021 Best Paper) are the state-of-the-art subterranean exploration planners and significantly outperform frontier-based approaches. However, both are ROS 1-only with no official ROS 2 ports. The thesis implements a utility-scored frontier planner in native ROS 2 as a practical baseline, with the explicit acknowledgment that porting GBPlanner2 would be the next improvement step.

**Trade-off:** The utility-weighted frontier approach covers approximately 60–75% of the cave volume in benchmarks, versus 85–95% for hierarchical planners like TARE. This performance gap is documented as a limitation and future work item in the thesis.

### 8.5 Why a Simple ICP Back-End as Fallback

The `simple_lidar_odom` node provides a self-contained ICP odometry implementation with no external dependencies. This is deliberately kept simple so the system can run without FAST-LIO2 (which requires a custom build) for debugging and CI. The node implements the same interface as FAST-LIO2 (`/slam/odometry`, `/slam/cloud_map`), allowing seamless backend switching via launch file argument.

### 8.6 Separation of Collision Mesh and Visual Mesh

Gazebo physics uses the collision mesh for contact detection. LiDAR ray casting also uses the collision mesh. Using a high-polygon visual mesh for collision is the single most common cause of Gazebo physics performance degradation on large cave environments. CaveDroneSim exports two meshes: a high-detail `.dae` visual mesh (100K–500K triangles) and a simplified `.stl` collision mesh (~20K triangles, generated by Blender decimate modifier). This maintains visual quality while keeping physics at interactive frame rates.

### 8.7 Latched QoS for Critical State Topics

`/mission/state` and `/slam/degeneracy_warning` use latched Reliable QoS (transient_local durability). Late-joining subscribers (e.g., RViz2, logging nodes) receive the last published state immediately on connection, preventing confusing "unknown state" displays during system startup.
