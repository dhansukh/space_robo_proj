# SLAM Configuration and Tuning Guide

**Document type:** Configuration reference and tuning guide  
**Audience:** Thesis authors, SLAM practitioners  
**Cross-references:** [ARCHITECTURE.md](ARCHITECTURE.md) | [THESIS_NOTES.md](THESIS_NOTES.md) | [API_REFERENCE.md](API_REFERENCE.md)

---

## Table of Contents

0. [Quick Start — Running the Full Pipeline](#0-quick-start--running-the-full-pipeline)
1. [SLAM Backends Overview](#1-slam-backends-overview)
2. [Backend 1: Simple ICP (Baseline)](#2-backend-1-simple-icp-baseline)
3. [Backend 2: FAST-LIO2 (Primary)](#3-backend-2-fast-lio2-primary)
4. [Backend 3: RTAB-Map (Full SLAM)](#4-backend-3-rtab-map-full-slam)
5. [When to Use Each Backend](#5-when-to-use-each-backend)
6. [Parameter Tuning for Cave Environments](#6-parameter-tuning-for-cave-environments)
7. [Degeneracy Detection and Handling](#7-degeneracy-detection-and-handling)
8. [Evaluating SLAM Quality](#8-evaluating-slam-quality)
9. [Collecting Ground Truth from Gazebo](#9-collecting-ground-truth-from-gazebo)
10. [Troubleshooting Common SLAM Failures](#10-troubleshooting-common-slam-failures)
11. [Map Quality Improvements — Changelog](#11-map-quality-improvements--changelog)
12. [References](#12-references)

---

## 0. Quick Start — Running the Full Pipeline

All commands are run from the workspace root:
```
~/Downloads/space_robo_proj/cave_drone_sim/
```

### Step 1 — Generate a cave (one-time, skip if `output/cave_flat` exists)

```bash
python3 scripts/generate_cave.py --output ./output/cave_flat
```

### Step 2 — Open four terminals and run each command in order

**Terminal 1 — PX4 SITL + Gazebo**
```bash
./scripts/start_px4_cave.sh
```
Wait until you see `[Gazebo] ... model loaded` before continuing.

**Terminal 2 — ROS 2 drone bridge**
```bash
ros2 launch drone_bringup px4_cave.launch.py use_vel_bridge:=false
```
Wait until `[drone_bringup]` reports the bridge is up.

**Terminal 3 — SLAM pipeline**
```bash
ros2 launch slam_pipeline slam_simple.launch.py
```
The pipeline is ready when you see `[simple_lidar_odom] SimpleLidarOdom ready`.

**Terminal 4 — Coverage mission**
```bash
# Start the coverage planner (3-segment test run, auto-save every 2 min)
python3 scripts/slam_coverage.py --cave ./output/cave_flat --max-segs 3 --auto-save 2

# In a 5th terminal (or after the node is ready), fire the start signal:
ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
```

### Step 3 — View results

Maps are saved automatically to `/tmp/slam_maps/` as:
- `grid_<timestamp>_<n>.png` — 2D occupancy grid (black=wall, light grey=free, mid-grey=unknown)
- `grid_<timestamp>_<n>.pgm` + `.yaml` — nav2 / map_server compatible files
- `cloud_<timestamp>_<n>.pcd` — 3D point cloud

```bash
# Trigger an on-demand save at any time:
ros2 service call /slam/save_map std_srvs/srv/Trigger {}

# Save only the raw point cloud:
ros2 service call /slam/save_pointcloud std_srvs/srv/Trigger {}
```

### Full mission (no segment limit)

```bash
python3 scripts/slam_coverage.py --cave ./output/cave_flat --auto-save 5
ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
```

### Useful slam_coverage.py flags

| Flag | Default | Description |
|------|---------|-------------|
| `--cave PATH` | auto-detect | Cave output directory (must contain `cave_graph.json`) |
| `--max-segs N` | none | Stop after N segments — useful for quick tests |
| `--auto-save MINS` | 0 (disabled) | Save map every MINS minutes during flight |
| `--speed M/S` | 0.3 | Transit speed between waypoints |
| `--scan-secs S` | 20 | Duration of 360° chamber scan |
| `--dry-run` | — | Print the planned route and exit without flying |

### RViz monitoring

```bash
rviz2 -d config/slam_rviz.rviz
```

Key topics to add manually if not in the config:

| Topic | Type | What it shows |
|-------|------|---------------|
| `/slam/map_cloud` | PointCloud2 | Accumulated 3D map |
| `/slam/occupancy_grid` | OccupancyGrid | Live 2D floor plan |
| `/slam/path` | Path | Drone trajectory |
| `/slam/octomap` | MarkerArray | 3D voxel map |
| `/drone/lidar/points` | PointCloud2 | Raw LiDAR (sensor frame) |

---

### Building and Rebuilding the SLAM Package

The `slam_pipeline` package lives under `src/slam_pipeline/` inside the workspace.
All source edits (`.py` files, config YAML, launch files) require a rebuild + install
copy before they take effect at runtime.

#### Normal rebuild after any source edit

```bash
# Run from the workspace root: ~/Downloads/space_robo_proj/cave_drone_sim/
colcon build \
  --base-paths src \
  --packages-select slam_pipeline \
  --allow-overriding slam_pipeline
```

> `--base-paths src` is required because the ROS packages live under `src/`, not at the
> workspace root.  `--allow-overriding` suppresses the warning that `slam_pipeline` is
> already present in the outer install tree.

#### Why colcon sometimes doesn't update installed .py files

Colcon's Python build uses `setup.py install` (or editable mode). On repeated builds it
may detect no change and skip the copy step — leaving the old `.py` files in place.
**Always run the manual copy after `colcon build`** to guarantee the running nodes see
the latest source:

```bash
INSTALL=src/install/slam_pipeline/lib/python3.10/site-packages/slam_pipeline
SRC=src/slam_pipeline/slam_pipeline

cp $SRC/simple_lidar_odom.py $INSTALL/simple_lidar_odom.py
cp $SRC/octomap_builder.py   $INSTALL/octomap_builder.py
cp $SRC/map_saver.py         $INSTALL/map_saver.py

# Clear stale bytecode so Python re-compiles the new source
rm -f $INSTALL/__pycache__/simple_lidar_odom.cpython-310.pyc \
      $INSTALL/__pycache__/octomap_builder.cpython-310.pyc \
      $INSTALL/__pycache__/map_saver.cpython-310.pyc
```

#### Updating the installed config / launch files

Config and launch files are also copied to the install tree at build time and may lag:

```bash
# Config
cp src/slam_pipeline/config/slam_params.yaml \
   src/install/slam_pipeline/share/slam_pipeline/config/slam_params.yaml

# Launch files (if edited)
cp src/slam_pipeline/launch/slam_simple.launch.py \
   src/install/slam_pipeline/share/slam_pipeline/launch/slam_simple.launch.py
```

#### One-liner: full rebuild + install copy + cache clear

```bash
colcon build \
  --base-paths src \
  --packages-select slam_pipeline \
  --allow-overriding slam_pipeline \
&& INSTALL=src/install/slam_pipeline/lib/python3.10/site-packages/slam_pipeline \
&& SRC=src/slam_pipeline/slam_pipeline \
&& cp $SRC/simple_lidar_odom.py $INSTALL/simple_lidar_odom.py \
&& cp $SRC/octomap_builder.py   $INSTALL/octomap_builder.py \
&& cp $SRC/map_saver.py         $INSTALL/map_saver.py \
&& cp src/slam_pipeline/config/slam_params.yaml \
      src/install/slam_pipeline/share/slam_pipeline/config/slam_params.yaml \
&& rm -f $INSTALL/__pycache__/*.cpython-310.pyc \
&& echo "Build + install complete"
```

#### Verify installed files have your changes

```bash
# Check a specific fix is present in the installed file
grep -n "MAX_BODY_RANGE\|current_scan\|v > 0.4\|free_mask" \
  src/install/slam_pipeline/lib/python3.10/site-packages/slam_pipeline/simple_lidar_odom.py \
  src/install/slam_pipeline/lib/python3.10/site-packages/slam_pipeline/octomap_builder.py \
  src/install/slam_pipeline/lib/python3.10/site-packages/slam_pipeline/map_saver.py
```

#### Source the workspace after a full rebuild

If you add new executables or packages, source the install before launching:

```bash
source src/install/setup.bash
```

---

## 1. SLAM Backends Overview

CaveDroneSim supports three interchangeable SLAM backends, all publishing on the same topic interfaces (`/slam/odometry`, `/slam/cloud_map`) for compatibility with the downstream exploration planner.

```
┌──────────────────────────────────────────────────────────────────────┐
│                    SLAM Backend Comparison                            │
│                                                                      │
│  Backend         ROS 2 Humble  Loop Closure  Cave/Degenerate  Speed  │
│  ─────────────   ────────────  ────────────  ──────────────   ─────  │
│  Simple ICP       ✅ Built-in   ❌ None        ⚠️  Moderate   Fast   │
│  FAST-LIO2        ✅ Port avail ❌ None        ✅  Good        Fast   │
│  RTAB-Map         ✅ Native     ✅ ICP-based   ✅  Good       Moderate│
└──────────────────────────────────────────────────────────────────────┘
```

### Comparison Matrix

| Criterion | Simple ICP (now PX4-Anchored) | FAST-LIO2 | RTAB-Map |
|-----------|-----------------------------|-----------|----------|
| External dependencies | None (px4_msgs required) | Custom build | `apt install` |
| Loop closure | ❌ | ❌ | ✅ |
| IMU integration | ✅ (PX4 EKF — tight) | ✅ (tight) | ✅ (loose) |
| Real-time capable | ✅ | ✅ | ✅ (tunable) |
| OctoMap output | ✅ (via octomap_builder) | ✅ (via octomap_builder) | ✅ (native) |
| 2D occupancy grid | ✅ (altitude-tracking Z-slice) | ✅ (Z-slice projection) | ✅ (native) |
| 3D LiDAR support | ✅ (with body-range filter) | ✅ | ✅ |
| Pose source | PX4 NED (GPS/EKF anchor) | LiDAR-IMU iEKF | LiDAR-IMU or FAST-LIO2 |
| Featureless corridor ATE | ~0.05 m (GPS-anchored) | ~0.2 m/100 m | ~0.15 m/100 m (with LC) |
| Memory (100 m traverse) | ~800 MB | ~1.2 GB | ~2 GB |
| Typical use | Simulation baseline, map validation | Primary evaluation | Full SLAM evaluation |

---

## 2. Backend 1: Simple ICP → PX4-Anchored Mapping (Baseline)

**Launch:** `ros2 launch slam_pipeline slam_simple.launch.py`  
**Config:** `slam_pipeline/config/slam_params.yaml`  
**Source:** `slam_pipeline/slam_pipeline/simple_lidar_odom.py`

> **Architecture change (v2):** The original ICP-only pose estimator was replaced with a
> **PX4 NED position anchor**. ICP is retained as a fallback when PX4 position is unavailable.
> This change was necessary because cave tunnel geometry is too symmetric for scan-to-scan
> ICP to converge reliably — every pose along a uniform corridor is ambiguous, causing
> the classic starburst divergence.

### Algorithm (current — v3, after map quality fixes)

1. **LiDAR ingestion:** `/lidar/points` PointCloud2 is voxel-downsampled (`voxel_size=0.1 m`).

2. **Body-range filter:** Points beyond `MAX_BODY_RANGE=10 m` in sensor frame are discarded.  
   *Rationale:* Maximum chamber radius in generated caves is ~8 m. Keeping 10 m gives 2 m margin
   for diagonal tunnel shots, while blocking beams that escape through the cave entrance and
   return from surfaces far outside (those caused the outlier halo visible in earlier maps).*

3. **Statistical Outlier Removal (SOR):** After downsampling, any point with fewer than
   **2 neighbours within 0.6 m** (body frame) is dropped.  
   *Rationale:* Real cave walls form dense clusters after voxel downsampling. Completely isolated
   ghost returns (single-beam reflections, sensor noise) have no neighbours. SOR removes these
   before they reach the octomap, so the log-odds threshold can stay low without single-hit
   artefacts appearing in the map.*

4. **Pose from PX4 NED:** Subscribes to `/fmu/out/vehicle_local_position`.  
   Position `(x=North, y=East, z=-Down)` and `heading` are used directly to build the
   `map ← body_FRD` transform.  
   **Correct rotation matrix** (body FRD → map NED-z-up):
   ```
   R = [[cos(h), -sin(h),  0],    # FRD-x (fwd)   → map North
        [sin(h),  cos(h),  0],    # FRD-y (right)  → map East   ← was WRONG (sign flipped)
        [  0,       0,    -1]]    # FRD-z (down)   → map −Up
   ```
   > **Bug fixed (v3):** The original matrix had `[sin(h), -cos(h)]` for the second row,
   > mapping body-right to West instead of East. This mirrored every LiDAR point across the
   > Y axis, producing a distorted blob instead of the correct cave shape.

   *Falls back to ICP+IMU if `px4_msgs` is not installed or `xy_valid`/`z_valid` are False.*

5. **Keyframe scan published on `/slam/current_scan`:** Each accepted keyframe scan (after SOR,
   transformed to map frame) is published on a separate per-keyframe topic.  
   > **Architecture fix (v3):** Previously the octomap received `/slam/map_cloud` — the
   > *accumulated global point cloud*. This caused every update to raycast from the **current**
   > drone position to **all** past wall points across the whole cave. Rays going backward
   > through already-mapped walls marked solid rock as free space, creating radial starburst
   > spikes. The fix: octomap now subscribes to `/slam/current_scan` (single-keyframe scan)
   > so raycasting only covers what the drone can actually see right now.

6. **Keyframe insertion:** New keyframe when pose changes exceed `keyframe_distance=0.2 m`
   translation **or** `keyframe_rotation=0.087 rad (~5°)` rotation.  
   *5° threshold captures 72 keyframes per 360° chamber scan vs. 24 at the old 15° threshold —
   providing 3× denser wall coverage.*

7. **Global map cloud:** Accumulated world-frame keyframes published on `/slam/map_cloud`
   (visualisation only — no longer used for raycasting).
   Sliding window size is `local_map_size=200`.

8. **TF publication:** Publishes `map → odom → base_link` transforms using the PX4 pose.

### 2D Occupancy Grid Pipeline (octomap_builder → map_saver)

```
/slam/current_scan  ──►  LogOddsVoxelGrid.insert_point_cloud()
                              │  raycast free space + mark occupied endpoints
                              ▼
                         get_2d_slice(z_min = sensor_z − 1.5 m,
                                      z_max = sensor_z + 1.5 m)
                              │  v > 0.4  → occupied (100)
                              │  v < −0.3 → free     (  0)
                              │  else     → unknown   ( −1)
                              ▼
                         _publish_2d_grid()
                              │  binary_dilation(iterations=1) into unknown only
                              │  (does NOT expand into free space)
                              ▼
                    /slam/occupancy_grid  ──►  map_saver
                                                   │  CC filter: remove blobs < 8 px
                                                   │  dilation(1) into unknown only
                                                   │  PGM/PNG:  0=black  205=light-grey  128=mid-grey
                                                   ▼
                                          /tmp/slam_maps/grid_*.png
```

**Why free space matters:** Without free cells, the cave interior renders as unknown (grey) and
wall dots appear against a grey background — impossible to distinguish cave structure from
outliers. With free space, the navigable interior is light grey, walls are black, and unexplored
regions stay mid-grey. This is also the correct `nav_msgs/OccupancyGrid` encoding expected by
Nav2 and `map_server`.

### Limitations

- Uses PX4 EKF position (includes simulated GPS) — not GPS-free. Suitable for simulation
  validation only; replace with FAST-LIO2 or RTAB-Map for GPS-denied real-world deployment.
- No loop closure: any residual EKF drift (typically <0.05 m in SITL) is not corrected.

### Key Parameters (`slam_params.yaml`)

```yaml
simple_lidar_odom:
  ros__parameters:
    voxel_size: 0.1           # Downsampling leaf size (m)
    icp_max_distance: 1.0     # Max correspondence distance (m) — ICP fallback only
    icp_max_iter: 30          # ICP iterations — fallback only
    keyframe_distance: 0.2    # Keyframe translation threshold (m)
    keyframe_rotation: 0.087  # Keyframe rotation threshold (rad ~5°)
    local_map_size: 200       # Sliding-window keyframe count

octomap_builder:
  ros__parameters:
    resolution: 0.1           # Voxel size (m) — 0.1 m for sharp walls
    max_range: 15.0           # Raycasting cap (m) — was 30, reduced to block entrance escapes
    sensor_model_hit: 0.7     # P(occupied | hit)
    sensor_model_miss: 0.4    # P(free | miss)
    z_slice_min: -0.5         # Fallback z-slice (used only before first odom arrives)
    z_slice_max: 2.5
    publish_rate_hz: 1.0
```

---

## 3. Backend 2: FAST-LIO2 (Primary)

**Launch:** `ros2 launch slam_pipeline slam_fastlio.launch.py`  
**Config:** `slam_pipeline/config/fastlio_cave.yaml`  
**External repo:** [github.com/Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)

### Algorithm

FAST-LIO2 (Fast Direct LiDAR-Inertial Odometry) from HKU-MARS Lab performs tight LiDAR-IMU fusion via an iterated Extended Kalman Filter (iEKF):

1. **IMU pre-integration:** Between consecutive LiDAR scans, IMU measurements are integrated to provide a prior pose estimate and to de-skew the point cloud (compensate for motion distortion during the scan sweep).
2. **Scan registration:** The de-skewed scan is registered against the incremental k-d tree map (`ikd-Tree`) using direct point-to-plane matching — no feature extraction. This is the critical advantage over LOAM-based systems in featureless caves.
3. **iEKF update:** The Kalman filter state (pose + velocity + IMU bias) is updated by minimizing the combined LiDAR residual and IMU prior. Multiple EKF iterations are run until convergence.
4. **Map update:** New points are inserted into the ikd-Tree, with dynamic leaf size adaptation for efficient nearest-neighbor queries.

**Performance in caves:** The featureless nature of cave walls is less detrimental to FAST-LIO2 than to feature-based systems because it works directly with raw points. In the LG-SLAM benchmark, FAST-LIO2 achieved 0.11 m ATE on an underground sequence.

### Installation

```bash
# Clone the community ROS 2 port
cd ~/ros2_ws/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Configuration for Cave Environments

The file `slam_pipeline/config/fastlio_cave.yaml` contains the cave-optimized FAST-LIO2 configuration:

```yaml
# fastlio_cave.yaml — Cave-optimized FAST-LIO2 configuration
common:
  lid_topic: "/lidar/points"
  imu_topic: "/imu/data"
  time_sync_en: false         # Topics are pre-synchronized in simulation

preprocess:
  lidar_type: 2               # 2 = Ouster; 1 = Velodyne; 3 = Livox
  scan_line: 32               # Ouster OS0-32 channel count
  blind: 1.5                  # Ignore points within 1.5 m (drone body)
  det_range: 30.0             # Maximum range to register (m)
  feature_enabled: false      # Direct registration; no feature extraction

mapping:
  filter_size_surf: 0.2       # Voxel filter for incoming scan (m)
  filter_size_map: 0.3        # Voxel filter for map (m)
  cube_side_length: 100       # Local map cube size (m) — larger than cave
  acc_cov: 0.1                # Accelerometer noise covariance
  gyr_cov: 0.1                # Gyroscope noise covariance
  b_acc_cov: 0.0001           # Accelerometer bias noise
  b_gyr_cov: 0.0001           # Gyroscope bias noise
  extrinsic_T: [0.0, 0.0, 0.1]   # LiDAR-IMU translation offset (m)
  extrinsic_R: [1,0,0, 0,1,0, 0,0,1]  # LiDAR-IMU rotation (identity in sim)

publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: false     # Publish only keyframe cloud (bandwidth)
  scan_bodyframe_pub_en: false
```

### Key Tuning Parameters

| Parameter | Effect | Cave-specific guidance |
|-----------|--------|----------------------|
| `blind` | Ignore returns within this radius | Must be ≥ drone arm span to exclude body hits |
| `filter_size_surf` | Input scan voxel filter | Smaller (0.1) → better accuracy, more CPU; Larger (0.3) → faster, less accurate |
| `cube_side_length` | Local map size | Set to 2× longest expected cave dimension |
| `acc_cov`, `gyr_cov` | IMU noise model | Increase if IMU shows high noise (check with `ros2 topic hz /imu/data`) |

---

## 4. Backend 3: RTAB-Map (Full SLAM)

**Launch:** `ros2 launch slam_pipeline slam_rtabmap.launch.py`  
**Config:** `slam_pipeline/config/rtabmap_cave.yaml`  
**Package:** `ros-humble-rtabmap-ros` (install via apt)

### Algorithm

RTAB-Map (Real-Time Appearance-Based Mapping) implements a full SLAM pipeline with memory management:

1. **Odometry:** Uses FAST-LIO2 as external odometry (`odom_topic`) rather than internal odometry computation. This hybrid approach leverages FAST-LIO2's superior cave performance while adding RTAB-Map's loop closure.
2. **Keyframe selection:** A new keyframe (memory node) is created when the pose change exceeds a threshold or `RGBD/LinearUpdate`/`RGBD/AngularUpdate` parameters.
3. **Loop closure detection:** ICP-based scan matching against stored LiDAR submaps. The memory management system maintains an active working memory (STM) and transfers old nodes to long-term memory (LTM) to bound computation.
4. **Graph optimization:** On successful loop closure, GTSAM pose graph optimization is performed, correcting the `map → odom` transform.
5. **Map output:** Publishes OctoMap, 2D occupancy grid, and 3D point cloud.

### Configuration for Cave Environments

```yaml
# rtabmap_cave.yaml — Cave LiDAR-only configuration
rtabmap:
  ros__parameters:
    # Input source: LiDAR point cloud + FAST-LIO2 odometry
    subscribe_scan_cloud: true
    subscribe_depth: false
    subscribe_rgbd: false
    odom_topic: "/slam/odometry"          # From FAST-LIO2
    scan_cloud_topic: "/slam/cloud_map"   # From FAST-LIO2

    # Frame IDs
    frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"

    # Memory management
    Mem/IncrementalMemory: "true"
    Mem/InitWMWithAllNodes: "false"
    Mem/UseOdomGravity: "false"

    # Loop closure — ICP for LiDAR-only mode
    Reg/Strategy: "1"                        # 1=ICP (not visual)
    Reg/Force3DoF: "false"                   # Full 6-DOF
    Icp/MaxCorrespondenceDistance: "0.2"     # 20cm — tight for cave walls
    Icp/PointToPlane: "true"
    Icp/Iterations: "30"
    Icp/VoxelSize: "0.1"

    # Proximity-based loop closure (use location proximity, not appearance)
    RGBD/NeighborLinkRefining: "true"
    RGBD/ProximityBySpace: "true"
    RGBD/ProximityMaxGraphDepth: "0"
    RGBD/ProximityPathMaxNeighbors: "5"
    RGBD/LinearUpdate: "0.3"
    RGBD/AngularUpdate: "0.2"

    # 3D occupancy map
    Grid/3D: "true"
    Grid/RangeMax: "20.0"                    # 20m for cave passages
    Grid/CellSize: "0.1"                     # 10 cm OctoMap voxels
    Grid/RayTracing: "true"

    # Publishing rate
    Rtabmap/DetectionRate: "1"               # Hz
    cloud_map_pub_rate: 1.0
```

### RTAB-Map Memory Management

RTAB-Map's memory system is critical for long cave traversals:

```
Working Memory (WM)     Long-Term Memory (LTM)     Retrieve when near
┌────────────────────┐  ┌─────────────────────┐  ◄── location revisited
│  Recent nodes      │  │  All other nodes    │
│  (STM + active LTM)│  │  (compressed)       │
│  ≤ 500 nodes       │  │  unbounded          │
└────────────────────┘  └─────────────────────┘
```

Set `Mem/MaxStMemSize` to control Working Memory size. Default 100 nodes is typically sufficient for caves up to 500m length. For very long traversals, increase to 300.

---

## 5. When to Use Each Backend

| Scenario | Recommended Backend | Rationale |
|----------|--------------------|-----------| 
| Quick integration test or CI | Simple ICP | No external dependencies; fast startup |
| Manual teleoperation testing | Simple ICP or FAST-LIO2 | Robust to slow, deliberate motion |
| Thesis evaluation — odometry accuracy | FAST-LIO2 | Measures front-end quality without loop closure bias |
| Thesis evaluation — full SLAM (ATE) | FAST-LIO2 + RTAB-Map | Loop closure corrects drift; measures complete SLAM quality |
| Long cave traversal (>200 m) | FAST-LIO2 + RTAB-Map | Without loop closure, drift is unacceptable at long range |
| Degeneracy analysis | FAST-LIO2 + degeneracy_detector | FAST-LIO2's eigenvalue covariance exposes degeneracy clearly |
| Debugging cave generation | Simple ICP | Fastest iteration; check that the cave is navigable |

**Switching backends at launch time:**

```bash
# Use simple ICP
ros2 launch cave_drone_sim full_demo.launch.py slam_backend:=simple

# Use FAST-LIO2
ros2 launch cave_drone_sim full_demo.launch.py slam_backend:=fastlio

# Use RTAB-Map (requires FAST-LIO2 running as front-end)
ros2 launch cave_drone_sim full_demo.launch.py slam_backend:=rtabmap
```

---

## 6. Parameter Tuning for Cave Environments

### ICP Correspondence Distance

The `icp_max_distance` parameter (Simple ICP) and `Icp/MaxCorrespondenceDistance` (RTAB-Map) control the maximum distance between matched point pairs. In caves:

- **Too small (< 0.1 m):** ICP fails when the drone moves > 0.1 m between scans (fast flight). Manifests as accumulated zero-displacement updates.
- **Too large (> 0.5 m):** ICP pulls toward false matches in repetitive tunnel geometry. Manifests as sudden pose jumps.
- **Recommended:** `0.2 m` for normal cave exploration at ≤ 1.5 m/s. Increase to `0.4 m` for fast flight; decrease to `0.15 m` for precise mapping.

### Voxel Downsampling

All backends downsample incoming scans. Finer voxels → more accurate scan matching but higher CPU:

| Environment | Recommended `voxel_size` |
|-------------|------------------------|
| Large open chambers | 0.2–0.3 m |
| Narrow corridors | 0.05–0.1 m |
| Default (mixed) | 0.1 m |

### FAST-LIO2 IMU Noise Parameters

If the simulated IMU shows noise artifacts (check `ros2 topic echo /imu/data`), tune:

```yaml
acc_cov: 0.1      # Increase if accelerometer readings are noisy
gyr_cov: 0.1      # Increase if gyroscope readings are noisy
b_acc_cov: 0.0001 # Bias random walk — rarely needs tuning in simulation
b_gyr_cov: 0.0001
```

### OctoMap / 2D Grid Resolution and 3D LiDAR Handling

The `octomap_builder` node has been updated to handle 3D LiDAR correctly:

#### Z-Slice Altitude Tracking

The 2D grid projection now **tracks the drone's actual altitude** from `/slam/odom` instead of
using a hardcoded z range. The slice captures `±z_half=1.5 m` around the current sensor height:

```python
# Dynamic z-slice (replaces static z_slice_min / z_slice_max)
z_min = sensor_z - 1.5
z_max = sensor_z + 1.5
```

*The drone flies at `CRUISE_D = -8 m` NED → map z = +8 m. The old hardcoded slice
`[-0.5, +2.5 m]` was at floor level and missed all walls visible to the drone.*

#### Occupied-Only 2D Grid

Free-space raycasting is **excluded from the 2D grid**. Only occupied cells (log-odds > 0)
are marked; everything else stays unknown (−1). This eliminates the free-space starburst
spokes that appeared when long LiDAR rays swept through tunnel openings:

```python
# get_2d_slice — occupied cells only
relevant = {k: v for k, v in self._grid.items()
            if v > 0.0 and z_min <= k[2] * resolution <= z_max}
```

#### Near-Vertical Ray Filter

Beams with elevation angle > 60° (floor/ceiling hits) are excluded from 2D raycasting.
They are still accumulated in the 3D OctoMap.

```python
elevation = |Δz| / range;  keep if elevation < sin(60°) ≈ 0.866
```

#### Wall Dilation

After building the 2D grid, `binary_dilation(iterations=3)` followed by
`binary_erosion(iterations=1)` fills gaps between sparse LiDAR hits and creates
solid closed wall boundaries (morphological closing).

#### Resolution and Memory

| Resolution | Memory (80×80×25 cave) | Collision check accuracy | Notes |
|-----------|----------------------|-------------------------|-------|
| 0.3 m | ~80 MB | ±0.3 m | Exploration experiments |
| **0.1 m (current default)** | ~2.1 GB | ±0.1 m | **High-fidelity mapping** |

> **Changed from 0.2 m to 0.1 m** for sharper wall outlines. Requires ≥ 8 GB RAM.
> Revert to 0.2 m in `slam_params.yaml` if memory is constrained.

---

## 7. Degeneracy Detection and Handling

**Source file:** `slam_pipeline/slam_pipeline/degeneracy_detector.py`

### What is SLAM Degeneracy?

LiDAR scan matching degenerates when the geometric structure of the environment is insufficient to constrain all 6 degrees of freedom of the pose estimate. In caves, this most commonly occurs in:

- **Long straight corridors:** Point-to-plane ICP cannot constrain motion along the corridor axis (one degree of freedom is unconstrained).
- **Uniform circular tunnels:** All three rotation axes are poorly constrained.
- **Sparse point clouds:** Very narrow passages or long-range scanning with few returns.

### Eigenvalue-Based Detection

The `degeneracy_detector` node computes a score based on the eigenvalue ratio of the point cloud covariance matrix:

```python
# For each incoming scan:
cov = np.cov(downsampled_points.T)              # 3×3 covariance matrix
eigenvalues = np.linalg.eigvalsh(cov)            # [λ_min, λ_mid, λ_max]
score = eigenvalues[0] / eigenvalues[2]          # λ_min / λ_max ∈ [0, 1]
```

**Interpretation:**
- `score ≈ 1.0`: Spherically distributed points → well-constrained (open chamber)
- `score ≈ 0.1`: Planar distribution → one axis poorly constrained (flat floor scan)
- `score ≈ 0.01`: Linear distribution → degenerate corridor
- `score < eigenvalue_threshold (default 0.01)`: Degeneracy warning published

A smoothing window of `window_size=5` scans prevents transient low-score spikes from triggering unnecessary warnings.

### System Response to Degeneracy

When `/slam/degeneracy_warning` is published (Bool: True):

1. **`local_planner`:** Reduces maximum speed to `max_speed × 0.3` (cautious motion to avoid ICP failure from large inter-scan displacement).
2. **`mission_manager`:** If degeneracy persists for more than `slam_degeneracy_threshold` duration, transitions to `EMERGENCY` state and holds position.
3. **FAST-LIO2 (if running):** The iEKF covariance naturally grows in degenerate directions; IMU prediction continues to provide motion estimate along constrained axes.

### Degeneracy Tuning

| Parameter | Default | Effect |
|-----------|---------|--------|
| `eigenvalue_threshold` | `0.01` | Lower → fewer warnings (more false negatives); Higher → more warnings (more false positives) |
| `window_size` | `5` | Larger → smoother score, slower response to sudden degeneracy |
| `voxel_size` | `0.3` | Larger → faster computation; smaller → more accurate score |

Typical scores in different cave geometries:

| Geometry | Expected Score |
|---------|---------------|
| Open chamber (4+ directions) | `0.1 – 0.4` |
| T-junction | `0.05 – 0.15` |
| Straight corridor | `0.005 – 0.02` |
| Vertical shaft | `0.001 – 0.01` |
| Smooth circular tunnel | `< 0.005` |

---

## 8. Evaluating SLAM Quality

The `evo` Python package is the standard tool for SLAM trajectory evaluation. Install:

```bash
pip install evo --upgrade
```

### Absolute Trajectory Error (ATE)

ATE measures global consistency: how far the estimated trajectory deviates from ground truth after optimal rigid-body alignment.

```bash
# Record a bag during exploration:
ros2 bag record /slam/odometry /drone/ground_truth -o evaluation_run_01

# Compute ATE (translational RMSE):
evo_ape bag evaluation_run_01 \
    /drone/ground_truth \
    /slam/odometry \
    --align --correct_scale \
    --plot --plot_mode xyz \
    --save_results results/ate_run_01.zip
```

**Interpreting ATE:**
- `< 0.3 m`: Excellent (suitable for 3D mapping)
- `0.3 – 1.0 m`: Good (acceptable for exploration, not mapping)
- `> 1.0 m`: Poor (significant drift; loop closure failure or degeneracy)

### Relative Pose Error (RPE)

RPE measures local accuracy: how much the trajectory drifts per unit distance.

```bash
evo_rpe bag evaluation_run_01 \
    /drone/ground_truth \
    /slam/odometry \
    --align --delta 1.0 --delta_unit m \
    --plot \
    --save_results results/rpe_run_01.zip
```

**Interpreting RPE (translation, per meter):**
- `< 0.5%` of distance: Excellent
- `0.5 – 2%` of distance: Good
- `> 2%` of distance: Poor drift accumulation

### Comparing Multiple Backends

```bash
# Generate results for all three backends:
evo_ape bag fastlio_run.bag /drone/ground_truth /slam/odometry \
    --align --save_results results/fastlio.zip

evo_ape bag simple_run.bag /drone/ground_truth /slam/odometry \
    --align --save_results results/simple.zip

evo_ape bag rtabmap_run.bag /drone/ground_truth /slam/odometry \
    --align --save_results results/rtabmap.zip

# Compare all three on one plot:
evo_res results/*.zip --plot --save_table table.csv
```

### Map Quality Evaluation

Beyond trajectory error, map quality can be assessed by Chamfer distance between the generated OctoMap point cloud and the ground-truth cave mesh:

```python
# Using Open3D:
import open3d as o3d
estimated = o3d.io.read_point_cloud("slam_map.pcd")
groundtruth = o3d.io.read_triangle_mesh("cave_visual.dae")
gt_cloud = groundtruth.sample_points_uniformly(100000)

# Chamfer distance (mean nearest-neighbor distance, both directions)
dists_est = estimated.compute_point_cloud_distance(gt_cloud)
dists_gt = gt_cloud.compute_point_cloud_distance(estimated)
chamfer = (np.mean(dists_est) + np.mean(dists_gt)) / 2
print(f"Chamfer distance: {chamfer:.3f} m")
```

---

## 9. Collecting Ground Truth from Gazebo

Gazebo publishes the simulated drone's exact pose via a ground truth odometry topic (no noise, no drift):

**Topic:** `/drone/ground_truth` (`nav_msgs/Odometry`)  
**Frame:** `world` → `base_link`  
**Rate:** 50 Hz

### Extracting Ground Truth TF from Gazebo

The `gz_bridge.yaml` configuration bridges Gazebo's exact pose:

```yaml
# gz_bridge.yaml (relevant section)
- ros_topic_name: "/drone/ground_truth"
  gz_topic_name: "/model/cave_drone/pose"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

### Recording Evaluation Data

```bash
# Start the simulation:
ros2 launch cave_drone_sim full_demo.launch.py slam_backend:=fastlio

# In a separate terminal, record evaluation topics:
ros2 bag record \
    /slam/odometry \
    /drone/ground_truth \
    /tf \
    /tf_static \
    /slam/degeneracy_score \
    /mission/state \
    -o run_01_fastlio_seed42

# Run the mission, then stop recording with Ctrl+C
```

### Post-Processing Pipeline

```bash
# 1. Convert bag to TUM format for evo
ros2 run slam_pipeline bag_to_tum.py \
    --bag run_01_fastlio_seed42 \
    --est_topic /slam/odometry \
    --gt_topic /drone/ground_truth \
    --output_dir results/run_01/

# 2. Compute ATE and RPE
evo_ape tum results/run_01/ground_truth.txt results/run_01/estimated.txt \
    --align -v --save_results results/run_01/ate.zip

evo_rpe tum results/run_01/ground_truth.txt results/run_01/estimated.txt \
    --align --delta 1 --delta_unit m --save_results results/run_01/rpe.zip

# 3. Plot trajectory comparison
evo_traj tum results/run_01/ground_truth.txt results/run_01/estimated.txt \
    --plot --ref results/run_01/ground_truth.txt
```

---

## 10. Troubleshooting Common SLAM Failures in Caves

### Symptom: Drone Position Jumps Suddenly

**Cause:** ICP found a false match in a section of repetitive tunnel geometry (perceptual aliasing).  
**Diagnosis:** Check `/slam/odometry` timestamp vs. `/drone/ground_truth` — look for sudden translation > 1 m in one update.  
**Solutions:**
- If using PX4-anchored backend: check ICP jump guard — only ICP results with `fitness ≥ 0.1` AND `jump < 5 m` are accepted.
- Decrease `icp_max_distance` (stricter correspondence matching).
- Enable RTAB-Map proximity loop closure (`RGBD/ProximityBySpace: true`) to reject inconsistent loop closures.
- Add Scan Context descriptor for geometric place recognition.

### Symptom: Map Looks Like a Starburst (Radial Spokes)

**Cause:** `octomap_builder` was subscribed to `/slam/map_cloud` — the *accumulated global
point cloud*. Each update raycasted from the current drone position to **all** past wall points,
including walls in other tunnels behind the drone. These backward rays flew through solid rock
and marked it as free space, producing radial free-space starburst spokes.  
**Solution (applied in current pipeline):**
- `simple_lidar_odom` now publishes `/slam/current_scan` — one keyframe scan per update.
- `octomap_builder` subscribes to `/slam/current_scan` instead of `/slam/map_cloud`.
- Each raycast only covers the view from the current sensor position, never goes backward
  through already-mapped walls.
- Body-range filter caps scan points at `10 m` (was 20 m) blocking entrance escapees.
- `max_range` in `slam_params.yaml` reduced from 30 m to 15 m.

If starburst reappears, check that `/slam/current_scan` is publishing:
```bash
ros2 topic hz /slam/current_scan   # expect ~2–5 Hz during flight
```

### Symptom: Map Blob is Mirrored / Y-Axis Flipped

**Cause:** The `_px4_init_T()` rotation matrix had the wrong sign on the second row, mapping
body-right (East at h=0) to West instead of East. Every LiDAR point was reflected across the
Y axis, turning the cave shape into a mirror image.  
**Solution (applied in current pipeline):** Corrected rotation matrix in
`simple_lidar_odom._px4_init_T()`:
```python
# Correct (current): columns = FRD axes in map frame
R = [[cos(h), -sin(h),  0],
     [sin(h),  cos(h),  0],
     [  0,       0,    -1]]

# Wrong (previous): second row had signs flipped
R = [[cos(h),  sin(h),  0],
     [sin(h), -cos(h),  0],   # ← body-right mapped to West, not East
     [  0,       0,    -1]]
```

### Symptom: 2D Grid Slice Empty / Wrong Location

**Cause:** The static `z_slice_min / z_slice_max` parameters were at floor level
(`[-0.5, +2.5 m]`) while the drone flies at ~8 m altitude (CRUISE_D = −8 m NED → +8 m in
map frame). The slice never captured any drone-level wall hits.  
**Solution (applied in current pipeline):** The z-slice now dynamically tracks the drone's
altitude from `/slam/odom` (±1.5 m around current height). Verify with:
```bash
ros2 topic echo /slam/odom --field pose.pose.position.z
# Should show ~8.0 during flight
```

### Symptom: Map Accumulates Duplicate Walls ("Ghost Effect")

**Cause:** Odometry drift without loop closure; LIO-SAM feature threshold failure.  
**Diagnosis:** Two parallel walls visible in `/slam/cloud_map` topic in RViz2.  
**Solutions:**
- Switch to FAST-LIO2 (direct registration avoids feature-based failures).
- Enable loop closure via RTAB-Map: `RGBD/ProximityBySpace: true`.
- Reduce `octomap_builder` `max_range` to exclude far-range outliers.

### Symptom: SLAM Diverges in Long Corridor

**Cause:** LiDAR degeneracy in straight passages — ICP cannot constrain motion along the corridor axis.  
**Diagnosis:** `/slam/degeneracy_score` drops below 0.01; `/slam/degeneracy_warning: true`.  
**Solutions:**
- The `local_planner` automatically reduces speed (this is correct behavior; check it is responding).
- Increase `icp_max_iter` to 50 — more iterations help in weakly constrained scenarios.
- If using FAST-LIO2, verify IMU data is arriving at 200 Hz (`ros2 topic hz /imu/data`). IMU provides constraint along degenerate axes.
- Consider LiDAR yaw: mounting the LiDAR at 45° tilt provides scan planes perpendicular to the corridor axis, adding constraints.

### Symptom: OctoMap Not Updating

**Cause:** `octomap_builder` not receiving point cloud or odometry.  
**Diagnosis:** `ros2 topic hz /octomap_full_color` returns 0 or very low rate.  
**Solutions:**
- Check `/lidar/points` is publishing: `ros2 topic hz /lidar/points` (expect 10 Hz).
- Check `/slam/odometry` is publishing: `ros2 topic hz /slam/odometry` (expect 10 Hz).
- Verify TF tree: `ros2 run tf2_tools view_frames.py` — `map → odom → base_link → lidar_link` must all be present.
- Check `max_range`: if set too low, no rays complete and no voxels are marked.

### Symptom: FAST-LIO2 Fails to Initialize

**Cause:** IMU and LiDAR data not synchronized, or extrinsic calibration incorrect.  
**Diagnosis:** FAST-LIO2 log shows "IMU not initialized" or "waiting for first IMU measurement."  
**Solutions:**
- Verify `imu_topic` and `lid_topic` match the actual topic names: `ros2 topic list`.
- Ensure drone is stationary at startup for the first 2 seconds (IMU bias estimation).
- Check `extrinsic_T` values match the URDF LiDAR offset: should be `[0.0, 0.0, 0.1]` for the default URDF.
- If topics are delayed, increase `time_sync_en: true` and add a 1-second startup delay.

### Symptom: RTAB-Map Memory Grows Unboundedly

**Cause:** Memory transfer to LTM not triggered; `Mem/MaxStMemSize` too large.  
**Diagnosis:** Monitor `ros2 topic echo /rtabmap/info` — check `odomCacheSize` and `nodesInSTM`.  
**Solutions:**
- Reduce `Mem/MaxStMemSize` from default 100 to 50 for long traversals.
- Check `Mem/IncrementalMemory: true` is set.
- The LTM database is written to `~/.ros/rtabmap.db` by default. Delete this file between experiments to start fresh.

---

## 11. Map Quality Improvements — Changelog

This section documents every code change made to improve map quality, in the order they were
applied. Use it to understand why each parameter has its current value.

### v3 — Current (map quality overhaul)

| # | File | Change | Problem it fixed |
|---|------|--------|-----------------|
| 1 | `simple_lidar_odom.py` `_px4_init_T()` | Rotation matrix row 1: `[s, -c, 0]` → `[-s, c, 0]` (columns now represent FRD axes in map) | Y-axis flip: every LiDAR point was reflected to the wrong side, producing a distorted blob |
| 2 | `simple_lidar_odom.py` `_add_keyframe()` | Added `/slam/current_scan` publisher; per-keyframe scan published before global accumulator | Raycasting the global accumulated cloud created starburst spokes (rays going backward through walls) |
| 3 | `octomap_builder.py` | Subscribe to `/slam/current_scan` instead of `/slam/map_cloud` | Same starburst fix — octomap now receives one scan at a time, not the full history |
| 4 | `simple_lidar_odom.py` `_add_keyframe()` | `MAX_BODY_RANGE`: 20 m → **10 m** | LiDAR beams escaping the cave entrance hit surfaces 15–20 m outside, creating an outlier halo around the map |
| 5 | `simple_lidar_odom.py` `_add_keyframe()` | Statistical Outlier Removal: drop points with `< 2 neighbours in 0.6 m` (body frame, after voxel downsample) | Isolated ghost returns from single-beam reflections survived the range filter and accumulated as scattered dots |
| 6 | `octomap_builder.py` `get_2d_slice()` | Include free cells (`v < -0.3`) in output as `0`; occupied threshold `v > 0.4` | Cave interior rendered as unknown (grey) — indistinguishable from walls. Free space makes the navigable floor plan visible |
| 7 | `octomap_builder.py` `_publish_2d_grid()` | Dilation: 2 iterations → **1 iteration**, applied only into unknown cells (not free) | 2 iterations expanded each stray dot into a 5×5 pixel blob; now only fills genuine 1-pixel gaps between wall returns |
| 8 | `map_saver.py` `save_2d_grid_pgm()` | Replaced double-dilation with: 1 dilation into unknown + CC filter (remove blobs < 8 px) | Map_saver was re-dilating an already-dilated grid (total 4 iterations), bloating every outlier into a large blob |
| 9 | `map_saver.py` `save_2d_grid_pgm()` | PNG/PGM encoding: free=205, occupied=0, unknown=128 | Previous version wrote only 0/128, hiding all free-space information from the saved image |
| 10 | `slam_params.yaml` | `octomap_builder.max_range`: 30 m → **15 m** | Largest chamber radius is 7.7 m. 30 m allowed long-range phantom hits from outside the cave to accumulate |

### v2 — PX4 anchor + keyframe densification

| # | File | Change | Problem it fixed |
|---|------|--------|-----------------|
| 1 | `simple_lidar_odom.py` | Replaced ICP-only pose with PX4 NED anchor | Symmetric cave tunnels caused ICP to diverge (perceptual aliasing); PX4 EKF provides ground-truth-quality pose |
| 2 | `slam_params.yaml` | `keyframe_distance`: 0.5 → 0.2 m; `keyframe_rotation`: 0.26 → 0.087 rad | Too-sparse keyframes left large gaps between wall returns; 5° rotation captures 72 frames per 360° scan |
| 3 | `slam_params.yaml` | `local_map_size`: 50 → 200 | Small window dropped old keyframes during long missions; 200 retains full coverage |

---

## 12. References

1. **FAST-LIO2:** Xu, W. et al. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry." IEEE Transactions on Robotics, 2022. [github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)

2. **RTAB-Map:** Labbé, M., Michaud, F. "RTAB-Map as an Open-Source Lidar and Visual Simultaneous Localization and Mapping Library for Large-Scale and Long-Term Online Operation." Journal of Field Robotics, 2019. [github.com/introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros)

3. **FAST-LIO2 ROS 2 port:** Ericsii. [github.com/Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)

4. **evo trajectory evaluation:** Grupp, M. [michaelgrupp.github.io/evo](https://michaelgrupp.github.io/evo/)

5. **DARPA SubT SLAM survey:** Ohradzansky, M. et al. "Present and Future of SLAM in Extreme Underground Environments." IEEE T-RO, 2023. [arxiv.org/abs/2208.01787](https://arxiv.org/abs/2208.01787)

6. **LG-SLAM benchmark:** Garcia-Espinosa, R. et al. "LG-SLAM: A Versatile and Robust Framework for Range-Inertial SLAM." arXiv 2407.14797. [arxiv.org/abs/2407.14797](https://arxiv.org/abs/2407.14797)

7. **Degeneracy detection:** Sun, Z. et al. "A Real-time Degeneracy Sensing and Compensation Method." arXiv 2412.07513. [arxiv.org/abs/2412.07513](https://arxiv.org/abs/2412.07513)

8. **DLR SCOUT cave rover SLAM:** Koch, J.F. "SLAM for SCOUT: A ROS2-Based Multi-Sensor SLAM System for a Cave Rover." i-SAIRAS 2024. [elib.dlr.de/210182](https://elib.dlr.de/210182/1/SLAM%20for%20SCOUT.pdf)

9. **FAST-LIVO2:** Zheng, C. et al. "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry." arXiv 2408.14035. [arxiv.org/abs/2408.14035](https://arxiv.org/abs/2408.14035)

10. **SC-LIO-SAM:** Kim, G. "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map." IROS 2018. [github.com/gisbi-kim/SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)
