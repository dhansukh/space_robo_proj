# LiDAR SLAM for Autonomous Drone Navigation in GPS-Denied Underground Cave Environments
## A Comprehensive Research Report for a ROS 2 Thesis Project

**Date:** April 2026  
**Scope:** LiDAR SLAM, Visual SLAM, multi-sensor fusion, 3D mapping representations, and evaluation methodology for cave drone applications

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Problem Context: SLAM in Caves](#problem-context)
3. [SLAM System Comparison Matrix](#comparison-matrix)
4. [Detailed System Evaluations](#detailed-evaluations)
   - FAST-LIO2
   - LIO-SAM
   - KISS-ICP
   - Cartographer (Google)
   - hdl_graph_slam
   - ORB-SLAM3
   - RTAB-Map
5. [Lessons from DARPA SubT Challenge](#darpa-subt)
6. [Multi-Sensor Fusion Approaches](#multi-sensor-fusion)
7. [3D Mapping Representations: OctoMap vs Voxblox vs TSDF](#mapping-representations)
8. [SLAM Evaluation Methodology](#evaluation-methodology)
9. [Recommendation for This Project](#recommendation)
10. [Suggested Architecture](#suggested-architecture)
11. [References](#references)

---

## 1. Executive Summary <a name="executive-summary"></a>

For an autonomous drone navigating GPS-denied underground cave environments in ROS 2, the optimal SLAM approach is a **layered architecture**:

1. **Primary front-end odometry:** FAST-LIO2 (with an optional upgrade to FAST-LIVO2 for visual-aided degeneration recovery)
2. **SLAM back-end with loop closure:** LIO-SAM (ROS 2 port) with Scan Context descriptor, or RTAB-Map configured in LiDAR mode
3. **3D occupancy mapping:** Voxblox (TSDF + ESDF) for local planning; OctoMap for Nav2 compatibility
4. **Degeneracy handling:** Eigenvalue-based degeneracy detection + IMU fallback

Pure LiDAR odometry systems (FAST-LIO2, KISS-ICP) offer best raw odometry performance but **lack loop closure** — critical for caves where drift accumulates. LIO-SAM provides loop closure but is more sensitive to featureless geometry. RTAB-Map offers the most complete, ROS 2-native SLAM pipeline with strong loop closure, at moderate computational cost.

---

## 2. Problem Context: SLAM in Caves <a name="problem-context"></a>

### Why Caves Are Particularly Hard for SLAM

| Challenge | Impact on SLAM |
|-----------|---------------|
| **No GPS** | No absolute position reference; all positioning is dead-reckoning + loop closure |
| **Featureless walls** | LiDAR scan matching degenerates in long, smooth passages — symmetric geometry causes ambiguous ICP solutions |
| **Repetitive geometry** | Tunnels/passages look identical, causing false loop closure matches |
| **Dust, moisture, particles** | Causes LiDAR false returns; visual SLAM fails entirely |
| **Confined space** | Drone maneuverability limited; aggressive motion causes IMU saturation |
| **Variable illumination** | Dark sections cripple visual/RGB-D SLAM |
| **Long distances without revisitation** | Drift accumulates; loop closure may never fire |
| **3D complexity** | Cave floors slope, passages go vertical — 2D SLAM is inadequate |
| **Drone vibration** | High-frequency IMU noise, LiDAR motion distortion |

### Key SLAM Requirements for This Application

- **3D pose estimation** (6-DOF, not 2D): mandatory for drone
- **LiDAR-IMU tight coupling**: compensates for vibration and motion blur
- **Loop closure**: essential for long cave traversals
- **Degeneracy detection**: prevents catastrophic failure in featureless corridors
- **Real-time, onboard computation**: ~10W power envelope typical for drone compute
- **ROS 2 Humble compatibility**: thesis requirement
- **Output usable by Nav2** or a 3D navigation stack

---

## 3. SLAM System Comparison Matrix <a name="comparison-matrix"></a>

### Core Comparison

| System | Type | ROS 2 Status | Loop Closure | Cave/Featureless | Compute (drone-viable?) | 3D Map Output | Nav2 Integration |
|--------|------|-------------|-------------|-----------------|------------------------|---------------|-----------------|
| **FAST-LIO2** | LiDAR-Inertial Odometry | ✅ Humble (community port) | ❌ None | ⚠️ Good odometry, degeneration risk | ✅ ARM-capable (~6ms/scan) | PCD point cloud | Indirect (needs bridge) |
| **LIO-SAM** | Tightly-coupled LIO + SLAM | ✅ ROS 2 port available | ✅ ICP-based (radius search) | ⚠️ Feature-dependent; ICP fails in repetitive scenes | ✅ Moderate (4 cores) | PCD + pose graph | Indirect |
| **KISS-ICP** | LiDAR Odometry only | ✅ Native ROS 2 | ❌ None | ⚠️ Fails in low-structure scenes (KITTI Stairs: ×) | ✅ Very lightweight | Sparse PCD | Indirect |
| **Cartographer** | 2D/3D SLAM | ✅ Humble (cartographer_ros) | ✅ Branch-and-bound | ⚠️ Weak on 3D map output despite 6DOF pose | ✅ Moderate | 2D occupancy grid (3D pose only) | ✅ Direct (slam_toolbox-compatible pattern) |
| **hdl_graph_slam** | 3D Graph SLAM | ❌ ROS 1 only | ✅ NDT-based + g2o | ✅ Best raw scan matching accuracy | ⚠️ High CPU (dense GICP) | PCD | ❌ Requires bridge |
| **ORB-SLAM3** | Visual/VI SLAM | ⚠️ Community wrappers (Humble) | ✅ BoW-based | ❌ Fails in low-light/dust/featureless | ✅ Low (vision only) | Sparse 3D map | ❌ Needs custom integration |
| **RTAB-Map** | Multi-modal SLAM | ✅ Native ROS 2 Humble | ✅ Appearance-based (multi-modal) | ✅ Works with LiDAR; degrades without visual features | ⚠️ Moderate-High (LiDAR mode) | PCD + OctoMap + mesh | ✅ Direct via `/map` + costmap |

### Loop Closure Quality in Featureless Environments

| System | Loop Closure Method | Featureless Robustness | Perceptual Aliasing Resistance |
|--------|--------------------|-----------------------|-------------------------------|
| LIO-SAM | Euclidean radius + ICP | Low — ICP fails in repetitive geometry | ❌ Poor |
| RTAB-Map | Visual bag-of-words + LiDAR ICP | Medium with LiDAR; Low without visual | ⚠️ Moderate |
| hdl_graph_slam | NDT + g2o | Medium | ⚠️ Moderate |
| Cartographer | Branch-and-bound submap | Medium (geometry-dependent) | ⚠️ Moderate |
| LIO-SAM + Scan Context | Scan Context + ICP | **High** | ✅ Good |
| RTAB-Map + Scan Context | Scan Context (feature request) | **High** (if integrated) | ✅ Good |

### Computational Profile

| System | Typical CPU Load | Min RAM | GPU Required? | Embedded ARM? |
|--------|-----------------|---------|---------------|---------------|
| FAST-LIO2 | 1 core, ~25ms/scan | 4 GB | No | ✅ Yes (TX2, Pi 4B, Orin) |
| KISS-ICP | Very low | 2 GB | No | ✅ Yes |
| LIO-SAM | 4 cores recommended | 8 GB | No | ⚠️ Marginal (Orin NX) |
| RTAB-Map (LiDAR) | 2–4 cores | 8 GB | Optional (CUDA) | ⚠️ Marginal |
| Cartographer | 2 cores | 4 GB | No | ✅ Yes |
| hdl_graph_slam | 4+ cores (dense GICP) | 16 GB | No | ❌ Too heavy |
| ORB-SLAM3 | 2 cores | 4 GB | Optional | ⚠️ Marginal |

---

## 4. Detailed System Evaluations <a name="detailed-evaluations"></a>

### 4.1 FAST-LIO2

**What it is:** Direct LiDAR-inertial odometry framework from HKU-MARS Lab. Uses an iterated Extended Kalman Filter (iEKF) to tightly fuse LiDAR raw points with IMU, without feature extraction. Maintains a dynamic map via an incremental k-d tree (ikd-Tree).

**ROS 2 Compatibility:**
- Official repository (`hku-mars/FAST_LIO`) targets ROS 1.
- Several community ROS 2 Humble ports exist: `Ericsii/FAST_LIO_ROS2`, `Taeyoung96/FAST_LIO_ROS2`. These are maintained but not official.
- `FAST-LIVO2` (the visual+LiDAR+IMU successor) has a ROS 2 Humble migration issue open (March 2025) indicating active community work.
- Recommend using Docker-containerized Humble build for stability.

**Performance in Featureless Environments:**
- FAST-LIO2 uses direct scan-to-map registration without feature extraction — this is an advantage over LOAM-based systems in low-feature environments.
- In the LG-SLAM benchmark, FAST-LIO2 achieved 0.11 m ATE on an underground sequence and 0.16 m average ATE across all sequences — competitive performance.
- **Critical weakness:** No loop closure. In long cave traversals, drift accumulates without correction. Benchmark showed it failed on Stairs sequence (FAST-LIO2 result: ×) due to lack of loop correction capability.
- Degeneracy in long, uniform corridors: the iEKF still relies on point-to-plane correspondence — uniform surfaces cause poor conditioning.

**Computational Requirements:**
- Scan update: ~25 ms full iEKF iteration, ~6 ms on ARM (Khadas VIM3).
- Verified on: Nvidia TX2, Raspberry Pi 4B (8GB), DJI Manifold 2-C (i7-8550U).
- **Drone-viable:** Yes. Among the lightest full LIO systems.

**3D Mapping Output:** Dense/semi-dense point cloud via ikd-Tree. Does not natively produce occupancy grids, OctoMap, or ESDF — requires downstream processing.

**Nav2 Integration:** Indirect. Publish `/tf` (map→odom→base_link) and point cloud; use `octomap_server2` or `rtabmap_ros` to convert to costmap.

**Drift over Long Distances:** Significant without loop closure. In practice, 1–5% of total distance traversed is typical drift — unacceptable for >50m cave traversals without correction.

**Dust/Dynamic Environments:** IMU fallback helps during momentary LiDAR degradation. Intensity thresholding of near-range returns can filter dust to some degree. No semantic filtering.

**Verdict for Cave Drone:** ✅ Excellent front-end odometry. ❌ Must pair with loop closure back-end.

---

### 4.2 LIO-SAM

**What it is:** Tightly-coupled LiDAR-inertial odometry via Smoothing and Mapping, from MIT SPARK Lab (Tixiao Shan). Uses LOAM-style feature extraction (edge + plane), IMU pre-integration, and iSAM2/GTSAM for incremental smoothing with loop closure.

**ROS 2 Compatibility:**
- Official ROS 2 branch exists in `TixiaoShan/LIO-SAM`.
- Also used in DARPA SubT by Team MARBLE (ROS 1, but now ported).
- Supports Velodyne, Ouster, Livox sensor types.

**Performance in Featureless Environments:**
- **Critical weakness:** Feature-based (LOAM-style). Requires sufficient edge and plane features. In degenerate environments (smooth cave walls, long straight passages), feature extraction fails — the `edgeFeatureMinValidNum: 10` and `surfFeatureMinValidNum: 100` thresholds may not be met.
- Research shows LIO-SAM "undergoes severe degeneracy" and "trajectories show significant errors" in degenerate sequences; maps show "clear ghosting effect."
- Loop closure uses Euclidean radius search + ICP: "In a highly reproducible environment, this method does not yield the desired results."
- Can be augmented with **Scan Context** (SC-LIO-SAM) for more robust place recognition.

**Computational Requirements:**
- 4 cores recommended. Heavier than FAST-LIO2 due to feature extraction + GTSAM optimization.
- Viable on Jetson Orin NX (16GB), marginal on smaller platforms.

**3D Mapping Output:** Dense point cloud (PCD). Pose graph with keyframes. Can integrate with OctoMap downstream.

**Nav2 Integration:** Indirect. Publish `/map` → `/odom` TF chain.

**Drift over Long Distances:** Loop closure significantly reduces drift when correctly triggered. Without loop closure (or with failed loop closure in featureless areas), drift accumulates rapidly.

**Dust/Dynamic Environments:** No built-in filtering. Dynamic objects appear as ghost features.

**Verdict for Cave Drone:** ⚠️ Useful with Scan Context augmentation; degeneration risk in smooth caves. Use SC-LIO-SAM variant.

---

### 4.3 KISS-ICP

**What it is:** "Keep It Small and Simple" LiDAR odometry from University of Bonn. Point-to-point ICP with constant velocity motion model for de-skewing, voxel grid downsampling, adaptive threshold. Extremely simple, robust across many sensor types.

**ROS 2 Compatibility:**
- ✅ Native ROS 2 support (Python, C++, ROS1, ROS2). Officially maintained.
- `ros2 launch kiss_icp odometry.launch.py` — plug-and-play.

**Performance in Featureless Environments:**
- **Fails in low-structure scenes.** Benchmark results: 0.67 m average ATE, failed (×) on the Stairs sequence. Outperformed by FAST-LIO2 on underground sequences.
- No feature extraction — designed to be geometry-agnostic, but this means it cannot compensate for degenerate ICP convergence.
- Best used as baseline or in well-structured environments.

**Computational Requirements:**
- Extremely lightweight. Suitable for any drone compute platform.

**3D Mapping Output:** Sparse point cloud only. No SLAM back-end.

**Nav2 Integration:** Odometry output only; no map. Must combine with separate SLAM system.

**Verdict for Cave Drone:** ❌ Insufficient alone; useful as lightweight baseline or in structured portions of caves. Not recommended as primary system.

---

### 4.4 Cartographer (Google)

**What it is:** Google's SLAM system supporting 2D and 3D SLAM with branch-and-bound loop closure detection and submap-based mapping. Uses Ceres optimization backend.

**ROS 2 Compatibility:**
- ✅ Available for ROS 2 Humble via `cartographer_ros` package (`ros-humble-cartographer-ros`).
- **Note:** Google has not actively maintained Cartographer since ~2022. Community considers it "not being maintained anymore." The Humble packages are available but bug fixes are community-driven.

**Performance in Featureless Environments:**
- 3D Cartographer provides 6-DOF pose estimation but **does not generate a true 3D volumetric map** — only a 2D occupancy grid despite 3D localization. This is a significant limitation for drone applications.
- In benchmarks (Impact of 3D LiDAR Resolution paper), Cartographer outperformed SC-LeGO-LOAM and SC-LIO-SAM on KITTI sequences 05 and 08 (structured urban).
- Loop closure uses branch-and-bound with Scan Match scores — works in geometrically distinctive environments but struggles in uniform caves.

**Computational Requirements:**
- Moderate (~2 cores). Ceres solver is efficient.

**3D Mapping Output:** 2D occupancy grid + 6DOF trajectory. No 3D volumetric map.

**Nav2 Integration:** ✅ Best of all systems for 2D Nav2; publishes `/map` directly.

**Drift:** Manageable with loop closure, but the lack of 3D map output limits utility.

**Verdict for Cave Drone:** ❌ 3D limitations and maintenance concerns make it unsuitable as primary system for 3D drone navigation. Consider only for 2D floor-plane mapping as complement.

---

### 4.5 hdl_graph_slam

**What it is:** 3D Graph SLAM using NDT (Normal Distributions Transform) scan matching for odometry + g2o pose graph optimization with loop closure from Kenji Koide (AIST).

**ROS 2 Compatibility:**
- ❌ **ROS 1 only.** The original repository (`koide3/hdl_graph_slam`) targets ROS Noetic.
- The Autoware documentation lists it as "ROS 1" only.
- Community forks exist attempting ROS 2 migration, but none are officially supported or well-maintained.
- **This is a blocking issue for a ROS 2 thesis project.**

**Performance in Featureless Environments:**
- Uses dense point cloud registration (FAST-GICP) — no feature extraction. This is theoretically more robust than feature-based methods.
- In benchmarks, hdl_graph_slam achieves **lowest RPE** (best scan-to-scan accuracy) but struggles with loop closure: "absence of a more robust loop closure algorithm led to less favorable overall results" and "failed to complete the loop" in KITTI sequence 00.
- "Diverged the most from ground-truth trajectory" on straight segments.
- Loop closure detection quality is weaker than SC-LIO-SAM or Cartographer.

**Computational Requirements:**
- High — dense GICP on full point clouds is expensive. Timing increases significantly with 128-channel LiDAR.

**3D Mapping Output:** Dense 3D point cloud (PCD). Supports GPS/IMU/floor constraints in pose graph.

**Verdict for Cave Drone:** ❌ ROS 1 only (blocking). Conceptually interesting but loop closure weakness and ROS 2 gap make it impractical.

---

### 4.6 ORB-SLAM3

**What it is:** Feature-based visual SLAM system from University of Zaragoza. Supports monocular, stereo, RGB-D, and visual-inertial configurations. Uses ORB feature descriptors, DBoW2 bag-of-words for loop closure, and full bundle adjustment.

**ROS 2 Compatibility:**
- ⚠️ No official ROS 2 package. Several community wrappers:
  - `gjcliff/ORB_SLAM3_ROS2` — monocular + IMU-monocular for Humble (December 2024)
  - Various wrappers for RealSense D435i with Humble
  - LinkedIn announcement (September 2025): new ROS 2 wrapper with CUDA support released
- All are community-maintained, varying quality.

**Performance in Featureless Environments:**
- **Major weakness for caves.** ORB-SLAM3 requires textured environments for ORB feature extraction.
- Research confirms: "The texture of the corridor in those datasets is very weak. During the turn, the camera is very close to the white wall surface, resulting in a very small number of feature points or even none at all. This causes purely visual SLAM tracking to fail."
- In dark caves or dusty environments, camera-based SLAM is unreliable.
- ORB-SLAM3 achieves 3.6 cm accuracy on EuRoC drone dataset — excellent accuracy in structured environments.

**Computational Requirements:**
- Low-moderate (2 cores, vision only).

**3D Mapping Output:** Sparse feature map. Not suitable for dense 3D occupancy.

**Nav2 Integration:** ❌ Requires significant custom work. No direct `/map` topic.

**Dynamic Environments:** Struggles with moving objects unless semantic masking is applied.

**Verdict for Cave Drone:** ❌ Primary SLAM choice. May serve as **visual backup/complement** for degeneracy recovery in textured sections. Do not rely on in dark or dusty cave conditions.

---

### 4.7 RTAB-Map (Real-Time Appearance-Based Mapping)

**What it is:** Multi-session, multi-modal SLAM library from IntRoLab (Université de Sherbrooke). Supports monocular, stereo, RGB-D, and LiDAR inputs with appearance-based loop closure. Has a memory management system that limits the map in working memory for real-time operation.

**ROS 2 Compatibility:**
- ✅ **Native ROS 2 Humble support.** `rtabmap_ros` package maintained by IntRoLab. Available via `apt install ros-humble-rtabmap-ros`.
- Used in the DLR SCOUT cave rover paper (i-SAIRAS 2024) specifically because it is "tightly integrated into the ROS2 middleware" and "features out-of-the-box integration of RGB-D cameras."
- LiDAR-only mode: use `subscribe_scan_cloud:=true` without camera input.

**Performance in Featureless Environments:**
- **LiDAR mode is viable in caves.** Loop closure uses ICP on LiDAR submaps when no visual features are available.
- Visual loop closure is stronger when camera data is available (textured sections).
- A community issue (October 2025) requests integration of Scan Context for LiDAR-based loop closure — currently not natively supported; this would significantly improve underground performance.
- Memory management (STM/LTM) helps with long traversals.
- DLR cave mapping experiment: ATE 41mm (live) / 232mm (real-time), RPE 230mm / 199mm — reasonable for a visual-only system; LiDAR mode expected to improve this.

**Computational Requirements:**
- Moderate-high in full visual+LiDAR mode. LiDAR-only is lighter.
- Can be configured to reduce map complexity for embedded hardware.

**3D Mapping Output:**
- ✅ Point cloud (PCD)
- ✅ OctoMap output (via `rtabmap_ros` node)
- ✅ 3D mesh
- ✅ 2D occupancy grid

**Nav2 Integration:**
- ✅ Best integration of all compared systems. Publishes `/map` (2D occupancy) directly usable by Nav2.
- For 3D Nav2 navigation (Spatio-Temporal Voxel Layer or NVBlox), the OctoMap output can be used.

**Drift:** Loop closure actively corrects drift. Performance depends on loop closure firing correctly.

**Dust/Dynamic Environments:** Dynamic object detection with motion filtering. LiDAR less affected by dust than camera.

**Verdict for Cave Drone:** ✅ **Best complete SLAM pipeline for ROS 2 cave drone.** Especially strong on integration, flexibility, and loop closure. Weakest point is loop closure quality in fully featureless environments without camera data.

---

## 5. Lessons from DARPA SubT Challenge <a name="darpa-subt"></a>

The DARPA Subterranean Challenge (2018–2021) is the most relevant real-world benchmark for cave/underground SLAM. Teams navigated tunnels, urban underground, and natural caves with robots in GPS-denied, often dusty, dark conditions.

### Key SLAM Systems by Team

| Team | Finish | SLAM System | Key Technical Choices |
|------|--------|-------------|----------------------|
| **CERBERUS** (Winner, 23 pts) | 1st | CompSLAM + M3RM | LOAM-based LiDAR odometry; GTSAM factor graph; BRISK visual loop closure; loose multi-modal fusion (visual, thermal, LiDAR, IMU) |
| **CSIRO Data61** (2nd, 23 pts) | 2nd | Wildcat | Surfel-based LiDAR-inertial odometry; Cauchy M-estimator for robustness; peer-to-peer map sharing |
| **MARBLE** (3rd, 18 pts) | 3rd | **LIO-SAM** + OctoMap fork | Tightly-coupled LiDAR-IMU; Ouster OS1 LiDAR |
| **Explorer** (Tunnel winner) | 4th | Super Odometry | IMU-centric LOAM-visual-inertial; radius + BoW loop closure; GICP |
| **CoSTAR** (Urban winner) | 5th | LAMP + LOCUS | Multi-robot; GNN loop prioritization; GNC outlier rejection |
| **CTU-CRAS-Norlab** | 6th | A-LOAM + ICP | No loop closure on UAVs; UAV used LOAM + Kalman |

### Critical Lessons for Cave Drone SLAM

1. **LiDAR is king.** All teams used LiDAR as their primary sensor. Visual/thermal was secondary — used for redundancy, especially in obscurants (smoke, dust). ["LIDAR-centric SLAM solutions — the go-to approach for virtually all teams."]

2. **Multi-modal redundancy is essential.** CERBERUS won by using multiple sensor modalities simultaneously, so that failure of any single sensor (dust blocking camera, LiDAR degeneration in a corridor) was compensated by others.

3. **IMU is critical for UAVs.** CTU-CRAS used LOAM + Kalman filter state estimation for UAVs specifically. IMU de-skewing of point clouds is mandatory on drone platforms.

4. **Loop closure is complex underground.** Most teams used proximity-based (Euclidean radius) loop closure — simple but effective when the robot physically revisits a location. BoW and junction-based methods were added for robustness.

5. **Degeneracy is a constant threat.** Long straight tunnels cause LiDAR scan matching to fail. Solutions: eigenvalue analysis of the Hessian, IMU fallback when degenerate, visual aiding.

6. **Dust/fog/smoke requires sensor fusion.** Pure visual SLAM failed in smoke-filled sections. LiDAR was more robust but also degraded. Multi-modal fusion (particularly thermal camera) provided backup.

7. **MARBLE used LIO-SAM directly** — validating its use for underground SLAM with appropriate sensor hardware. OctoMap was used for the 3D map.

8. **Factor graph back-ends (GTSAM) dominated.** GTSAM-based pose graph optimization was the standard backend across most teams.

9. **Computation constraints were real.** Aerial platforms required carefully optimized SLAM stacks. CTU-CRAS ran LOAM without loop closure on UAVs to stay within computational budget.

10. **Map sharing between robots** required compression (DRACO by CERBERUS), peer-to-peer protocols, and outlier-robust back-end optimization — relevant if multi-drone setups are considered.

---

## 6. Multi-Sensor Fusion Approaches <a name="multi-sensor-fusion"></a>

### LiDAR + IMU (Core)

**Tight coupling** (FAST-LIO2, LIO-SAM) integrates raw LiDAR points and IMU measurements in a single optimization:
- IMU pre-integration removes point cloud motion distortion (de-skewing)
- IMU provides prediction during LiDAR scan gaps
- Faster initialization and recovery from tracking loss

**Loose coupling** (CompSLAM, classic SLAM toolbox + robot_localization) estimates poses independently and fuses at the pose level:
- More modular, easier to implement
- Less accurate but more fault-tolerant

**Recommended:** Tight coupling via FAST-LIO2 as the primary front-end.

### LiDAR + IMU + Depth Camera

FAST-LIVO2 (from the FAST-LIO2 team, HKU-MARS) extends the tight-coupling to include camera:
- Sequential iEKF update: LiDAR first, then visual direct photometric alignment
- Runs on ARM platforms (Jetson Orin NX, RK3588)
- Verified: "Stably map and return to origin in extremely degraded and GPS-denied tunnel environments over 25 minutes"
- Directly addresses cave degeneracy: LiDAR handles geometry, camera handles texture, IMU handles dynamic motion
- **Note:** The camera is a complement, not a replacement — it adds texture information in low-feature LiDAR environments

### Fusion Architecture for Cave Drone

```
Sensors: 3D LiDAR (Ouster OS0/OS1) + IMU (internal/external) + Depth Camera (optional)

Layer 1: Front-End Odometry
  └── FAST-LIO2 (or FAST-LIVO2): raw LiDAR + IMU → pose estimate + dense point cloud

Layer 2: SLAM Back-End
  └── RTAB-Map or SC-LIO-SAM: keyframe selection + loop closure → corrected pose graph

Layer 3: 3D Map Representation
  └── Voxblox (TSDF/ESDF): local planning map
  └── OctoMap: global occupancy for Nav2

Layer 4: Navigation
  └── Nav2 + 3D costmap (STVL plugin) → path planning
```

### Degeneracy Handling

When LiDAR scan matching degenerates (featureless corridor):
1. **Eigenvalue analysis** of the Hessian matrix — small eigenvalues indicate degenerate directions
2. **DBSCAN-based degeneracy detection** (from arXiv 2412.07513) — environment-adaptive thresholding
3. **IMU compensation** — project IMU state estimate onto degenerate directions to prevent drift
4. **Visual fallback** — if camera data available, use visual odometry to supplement

CMU's "Visual-Aided LiDAR-Inertial Odometry in Challenging Subterranean Environments" demonstrated that visual aiding can overcome LiDAR failure in "elevator shafts, long corridors" — directly applicable to caves.

---

## 7. 3D Mapping Representations: OctoMap vs Voxblox vs TSDF <a name="mapping-representations"></a>

### Comparison Table

| Property | OctoMap | Voxblox (TSDF+ESDF) | TSDF (generic, e.g. KinectFusion) |
|----------|---------|--------------------|------------------------------------|
| **Data structure** | Octree (probabilistic occupancy) | Voxel hash + TSDF/ESDF | Voxel grid (fixed) |
| **Map query complexity** | O(log n) | O(1) — voxel hashing | O(1) |
| **TSDF integration speed** | Baseline | Up to **20× faster** raycasting, **2× faster** than grouped OctoMap | Varies |
| **Distance field accuracy** | Poor (occupancy-based) | ✅ High (ESDF from TSDF) | Moderate |
| **Local planning suitability** | ⚠️ Slow for ESDF queries | ✅ Best — ESDF enables fast gradient descent path planning | ✅ Good |
| **Nav2 compatibility** | ✅ Direct (`octomap_server` → costmap) | ⚠️ Requires adapter (nvblox_ros, or custom) | ❌ No direct support |
| **ROS 2 support** | ✅ `octomap_server2` | ✅ `voxblox_ros` (community) | Varies |
| **Memory use** | Adaptive (octree prunes empty) | Moderate (hash table) | High (fixed grid) |
| **Drone onboard suitability** | ✅ Proven | ✅ Original paper: runs at 4Hz on Intel i7 onboard MAV | ⚠️ Memory-intensive |
| **Dynamic map updates** | ✅ Probabilistic ray clearing | ✅ TSDF weight decay | ❌ Typically batch |
| **Mesh visualization** | ⚠️ Requires marching cubes post-process | ✅ Native mesh output | ✅ Via marching cubes |

### Recommendation

**For this project, use a two-layer approach:**

1. **Voxblox** (TSDF + ESDF) for **local 3D planning** around the drone:
   - 20cm voxel size for TSDF, 4Hz ESDF updates
   - Gradient-based trajectory optimization for collision avoidance
   - Validated on MAV hardware (Asctec Firefly) at matching update budget
   - Run incremental ESDF: "order of magnitude faster than batch"

2. **OctoMap** for **global occupancy mapping** and Nav2 integration:
   - `octomap_server2` for ROS 2
   - 10cm resolution for detailed obstacle representation
   - Use RTAB-Map's built-in OctoMap output or `octomap_server2` fed by the SLAM point cloud

**NVBlox** (NVIDIA Isaac ROS) is an emerging alternative combining TSDF and OctoMap patterns with GPU acceleration — suitable if a Jetson GPU is available, but adds NVIDIA SDK dependency.

---

## 8. SLAM Evaluation Methodology <a name="evaluation-methodology"></a>

### Standard Metrics

#### Absolute Trajectory Error (ATE)
- Measures **global consistency** of the estimated trajectory against ground truth.
- Computed by aligning estimated and ground-truth trajectories (via SE(3) or Sim(3) transformation) then computing RMSE of translational differences.
- Formula: `ATE_RMSE = sqrt(1/n * Σ ||trans(F_i)||²)` where `F_i` is the error matrix at frame `i`.
- **Best metric for evaluating loop closure effectiveness** — a successful loop closure dramatically reduces ATE.

#### Relative Pose Error (RPE)
- Measures **local drift** over fixed time intervals.
- Evaluates odometric accuracy between consecutive poses (e.g., over 1-second windows).
- Captures: how much drift accumulates per unit distance/time.
- **Best metric for evaluating odometry front-end quality.**

#### Additional Metrics
- **Map quality:** Chamfer distance between estimated and ground-truth point clouds
- **Loop closure precision/recall:** For evaluating place recognition algorithms
- **Processing time per frame:** For real-time viability assessment
- **CPU/memory utilization:** Onboard compute viability

### Evaluation Tools

| Tool | Purpose | Notes |
|------|---------|-------|
| **evo** (`michaelgrupp/evo`) | ATE + RPE computation, trajectory visualization | Python; standard in SLAM research; supports ROS bag, TUM, KITTI formats |
| **rpg_trajectory_evaluation** | RPE with multiple alignment methods | CMU/ETH; used in VIO benchmarks |
| **Gazebo Classic / GZ Sim** | Cave simulation | Use SubT Challenge Gazebo worlds (available on GitHub) |
| **SubT-MRS Dataset** | Real underground dataset | Multiple environments, LiDAR+IMU+camera; best available for cave SLAM |
| **Newer College Dataset** | Urban/indoor | LiDAR only; useful for baseline testing |
| **EuRoC MAV** | Aerial platform | Visual-inertial; good for ORB-SLAM3 and visual testing |

### Simulation Approach for Cave SLAM

1. **Use DARPA SubT Gazebo worlds** — these are publicly available from the SubT Challenge virtual competition. Include tunnel, urban, and cave environments with realistic geometry.
   - GitHub: `osrf/subt` (SubT challenge simulator)
   - Includes: dust particles, variable lighting, narrow passages

2. **Ground truth extraction:**
   - In Gazebo: `gazebo_ros` ground truth plugin publishes exact pose via `/ground_truth/odom`
   - Compare against SLAM output using `evo_ape` (ATE) and `evo_rpe` (RPE)

3. **Simulation workflow:**
   ```bash
   # Terminal 1: Launch SubT cave world
   ros2 launch subt_gazebo cave_world.launch.py
   
   # Terminal 2: Run SLAM
   ros2 launch fast_lio mapping_ouster64.launch.py
   
   # Terminal 3: Fly drone and record
   ros2 bag record /tf /tf_static /ground_truth/odom /points /imu
   
   # Post-analysis:
   evo_ape bag data.bag /ground_truth/odom /slam/odometry --align --plot
   evo_rpe bag data.bag /ground_truth/odom /slam/odometry --delta 1.0
   ```

4. **Key test scenarios for cave drone:**
   - Long straight corridor (degeneracy test)
   - Loop closure test (revisit same location)
   - Vertical traverse (elevator shaft — high degeneracy)
   - Dust particle injection (LiDAR degradation test)
   - Aggressive maneuvers (IMU saturation test)

---

## 9. Recommendation for This Project <a name="recommendation"></a>

### Tier 1 Recommendation: FAST-LIO2 + RTAB-Map

**Best overall combination for ROS 2 thesis on cave drone SLAM.**

| Criterion | Assessment |
|-----------|-----------|
| ROS 2 Humble support | ✅ Both well-supported |
| Cave/featureless performance | ✅ FAST-LIO2 handles degeneration better than feature-based systems |
| Loop closure | ✅ RTAB-Map provides robust multi-modal loop closure |
| Compute on drone | ✅ FAST-LIO2 is ARM-capable; RTAB-Map can be tuned |
| 3D map output | ✅ RTAB-Map outputs PCD + OctoMap + 2D grid |
| Nav2 integration | ✅ RTAB-Map publishes directly to Nav2-compatible topics |
| Academic precedent | ✅ Both heavily cited; DLR used RTAB-Map for cave rover |
| Thesis value | ✅ Novel combination, not just off-the-shelf |

**Implementation approach:**
1. Run FAST-LIO2 as the high-rate (100Hz) odometry front-end
2. Feed FAST-LIO2 odometry + point cloud into RTAB-Map as external odometry with LiDAR scan input
3. RTAB-Map performs keyframe selection, loop closure, and global map optimization
4. Output: OctoMap for Nav2 costmap + PCD for visualization

### Tier 2 Alternative: SC-LIO-SAM

If tighter LiDAR-IMU integration in the SLAM back-end is preferred:

- Use **LIO-SAM + Scan Context** (SC-LIO-SAM) for loop closure robust to featureless geometry
- Add DBSCAN-based degeneracy detection (arXiv 2412.07513) to handle corridor degeneration
- Pair with Voxblox for 3D local mapping

**Caveat:** LIO-SAM's feature extraction can still struggle in very smooth caves. Requires careful parameter tuning (`edgeFeatureMinValidNum`, `surfFeatureMinValidNum`).

### Tier 3 Advanced Option: FAST-LIVO2

If a camera is available on the drone:
- **FAST-LIVO2** (LiDAR + Visual + IMU tight coupling from HKU-MARS)
- Addresses degeneracy via camera-aided correction in featureless zones
- Runs on Jetson Orin NX in real-time
- Proven in tunnel environments >25 minutes without drift
- ROS 2 port: in progress (March 2025 Humble migration issue); may require custom build

### Systems to Avoid

| System | Reason |
|--------|--------|
| hdl_graph_slam | ROS 1 only — not viable for ROS 2 thesis |
| KISS-ICP (alone) | No loop closure; fails in low-structure scenes |
| ORB-SLAM3 (alone) | Visual SLAM fails in dark/dusty caves |
| Cartographer (alone) | No true 3D volumetric map; maintenance concerns |

---

## 10. Suggested Architecture <a name="suggested-architecture"></a>

```
┌────────────────────────────────────────────────────────────────────┐
│                    Cave Drone SLAM Architecture                      │
│                    (ROS 2 Humble, Thesis Project)                    │
└────────────────────────────────────────────────────────────────────┘

Hardware:
  ├── 3D LiDAR: Ouster OS0-64 or OS1-64 (hemispherical FoV, lightweight)
  ├── IMU: ICM-42688 or VectorNav VN-100 (tightly-coupled)
  ├── Depth Camera: Intel RealSense D435i (optional, for visual aiding)
  └── Compute: NVIDIA Jetson Orin NX 16GB (or similar)

ROS 2 Node Graph:
  ┌─────────────────────────────────────────────────────┐
  │  SENSORS                                             │
  │  /velodyne_points or /ouster/points (PointCloud2)   │
  │  /imu/data (Imu)                                     │
  │  /camera/depth/color/points (optional, PointCloud2) │
  └────────────────┬────────────────────────────────────┘
                   │
  ┌────────────────▼────────────────────────────────────┐
  │  FRONT-END: FAST-LIO2 (fast_lio node)               │
  │  • Tight LiDAR-IMU fusion via iEKF                  │
  │  • Output: /Odometry (100Hz), /cloud_registered     │
  │  • Degeneracy detection: eigenvalue monitor         │
  └────────────────┬────────────────────────────────────┘
                   │
  ┌────────────────▼────────────────────────────────────┐
  │  SLAM BACK-END: RTAB-Map (rtabmap node)             │
  │  • Input: /Odometry + /cloud_registered             │
  │  • Loop closure: ICP on LiDAR submaps               │
  │  • Memory management for long traversals            │
  │  • Output: /rtabmap/mapData, /rtabmap/cloud_map     │
  └────────────────┬────────────────────────────────────┘
                   │
  ┌────────────────▼────────────────────────────────────┐
  │  3D MAP: Dual Representation                         │
  │  ├── OctoMap (/rtabmap/octomap_full_color)           │
  │  │   → /map (2D occupancy for Nav2)                  │
  │  └── Voxblox (voxblox_ros node)                      │
  │      → ESDF for 3D collision avoidance               │
  └────────────────┬────────────────────────────────────┘
                   │
  ┌────────────────▼────────────────────────────────────┐
  │  NAVIGATION: Nav2 Stack                              │
  │  • Global planner: Navfn / Smac Planner             │
  │  • Local planner: DWB or TEB                         │
  │  • Costmap: STVL (3D voxel layer) + static layer    │
  │  • TF: map → odom → base_link (from RTAB-Map)       │
  └─────────────────────────────────────────────────────┘

Key ROS 2 Packages Required:
  ros-humble-rtabmap-ros     # SLAM back-end
  ros-humble-octomap-ros     # OctoMap visualization
  ros-humble-nav2-*          # Navigation stack
  fast_lio (custom build)    # Front-end odometry
  voxblox_ros (custom build) # ESDF mapping
  evo (pip)                  # Trajectory evaluation
```

### Parameter Tuning for Cave Environments

**FAST-LIO2 (config/ouster64.yaml):**
```yaml
filter_size_surf: 0.2      # Larger voxel — featureless environments need less filtering
filter_size_map: 0.3       # Map resolution
cube_side_length: 100      # 100m local map cube (adjust for cave size)
preprocess/blind: 1.5      # Blind zone — ignore returns <1.5m (drone body)
```

**RTAB-Map (for LiDAR underground):**
```yaml
Reg/Strategy: 1             # ICP registration (not visual)
Icp/MaxCorrespondenceDistance: 0.2   # 20cm for cave scan matching
RGBD/NeighborLinkRefining: true     # Refine odometry with ICP
RGBD/ProximityBySpace: true          # Proximity-based loop closure
Grid/3D: true                        # Enable 3D occupancy grid
Grid/RangeMax: 20.0                  # 20m range for cave passages
Mem/UseOdomGravity: false            # No GPS, use LiDAR gravity estimation
```

---

## 11. References <a name="references"></a>

1. Xu, W., et al. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry." IEEE Transactions on Robotics, 2022. GitHub: https://github.com/hku-mars/FAST_LIO

2. Shan, T., et al. "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping." IROS 2020. GitHub: https://github.com/TixiaoShan/LIO-SAM

3. Vizzo, I., et al. "KISS-ICP: In Defense of Point-to-Point ICP." IEEE RA-L, 2023. https://github.com/PRBonn/kiss-icp

4. Hess, W., et al. "Real-Time Loop Closure in 2D LIDAR SLAM." ICRA 2016. (Cartographer). https://github.com/cartographer-project/cartographer

5. Koide, K., et al. "A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement." (hdl_graph_slam). https://github.com/koide3/hdl_graph_slam

6. Campos, C., et al. "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM." IEEE T-RO, 2021. https://arxiv.org/abs/2007.11898

7. Labbé, M., Michaud, F. "RTAB-Map as an Open-Source Lidar and Visual Simultaneous Localization and Mapping Library for Large-Scale and Long-Term Online Operation." JFR, 2019. https://github.com/introlab/rtabmap_ros

8. Ohradzansky, M., et al. "Present and Future of SLAM in Extreme Underground Environments." IEEE T-RO, 2023. https://www-robotics.jpl.nasa.gov/media/documents/2208.01787.pdf [DARPA SubT survey]

9. DARPA. "Subterranean Challenge Final Event Results." 2021. https://www.darpa.mil/news/2021/subterranean-challenge-winners

10. Hoeller, H., et al. "Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning." IROS 2017. https://helenol.github.io/publications/iros_2017_voxblox.pdf

11. Hornung, A., et al. "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees." AURO 2013.

12. Koch, J.F. "SLAM for SCOUT: A ROS2-Based Multi-Sensor SLAM System for a Cave Rover." i-SAIRAS 2024. https://elib.dlr.de/210182/1/SLAM%20for%20SCOUT.pdf

13. Zheng, C., et al. "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry." arXiv 2408.14035. https://arxiv.org/html/2408.14035v2

14. Garcia-Espinosa, R., et al. "LG-SLAM: A Versatile and Robust Framework for Range-Inertial SLAM." arXiv 2407.14797. https://arxiv.org/html/2407.14797v2

15. Sun, Z., et al. "A Real-time Degeneracy Sensing and Compensation Method." arXiv 2412.07513. https://arxiv.org/html/2412.07513v1

16. Grupp, M. "evo: A Python package for the evaluation of odometry and SLAM." https://michaelgrupp.github.io/evo/

17. FAST-LIO2 ROS 2 port (Ericsii): https://github.com/Ericsii/FAST_LIO_ROS2

18. LIO-SAM ROS 2 + Scan Context: https://github.com/gisbi-kim/SC-LIO-SAM

19. RTAB-Map ROS 2: https://github.com/introlab/rtabmap_ros

20. SubT Challenge Simulator: https://github.com/osrf/subt
