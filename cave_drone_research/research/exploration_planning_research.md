# Autonomous Exploration & Path Planning for Drones in Cave Environments
## Research Report for ROS 2 Thesis Project

**Context:** A drone must autonomously explore and navigate procedurally-generated cave environments in a GPS-denied setting using ROS 2.

---

## Table of Contents

1. [Exploration Strategy Algorithms](#1-exploration-strategy-algorithms)
   - 1.1 [Frontier-Based Exploration](#11-frontier-based-exploration)
   - 1.2 [Next-Best-View Planning (NBVP)](#12-next-best-view-planning-nbvp)
   - 1.3 [GBPlanner / GBPlanner2](#13-gbplanner--gbplanner2)
   - 1.4 [FUEL](#14-fuel-fast-uav-exploration)
   - 1.5 [TARE](#15-tare-technology-aware-robot-exploration)
   - 1.6 [MBPlanner](#16-mbplanner-motion-primitive-based-planner)
2. [Comparative Summary Table](#2-comparative-summary-table)
3. [Goal-Directed Path Planning](#3-goal-directed-path-planning)
   - 3.1 [RRT* / Informed RRT*](#31-rrt--informed-rrt)
   - 3.2 [A* on OctoMap / Voxel Grids](#32-a-on-octomap--voxel-grids)
   - 3.3 [Model Predictive Control (MPC)](#33-model-predictive-control-mpc)
   - 3.4 [PX4 Avoidance Package](#34-px4-avoidance-package)
   - 3.5 [Nav2 Adaptation for 3D/Aerial](#35-nav2-adaptation-for-3daerial)
4. [Full Autonomy Stack Architecture](#4-full-autonomy-stack-architecture)
   - 4.1 [Stack Layers](#41-stack-layers-global--local--controller)
   - 4.2 [SLAM Integration](#42-slam-integration)
   - 4.3 [Return-to-Base Behavior](#43-return-to-base-behavior)
   - 4.4 [Communication Relay](#44-communication-relay-for-deep-cave-exploration)
   - 4.5 [Emergency Behaviors](#45-emergency-behaviors)
5. [Recommended Architecture for Thesis](#5-recommended-architecture-for-thesis)
6. [Key Papers Reference List](#6-key-papers-reference-list)

---

## 1. Exploration Strategy Algorithms

### 1.1 Frontier-Based Exploration

**Concept:** Frontiers are the boundaries between known free space and unknown space. The robot repeatedly identifies frontiers, selects the most informative one (typically by distance and expected information gain), navigates to it, and updates the map. The process repeats until no frontiers remain.

**Key Papers:**
- Yamauchi (1997) — original seminal paper defining frontier-based exploration
- [Frontier-based autonomous exploration implementation (arXiv 2018)](https://arxiv.org/abs/1806.03581) — Wavefront Frontier Detector (WFD) implementation in ROS
- [Fast Frontier-Based Information-Driven Autonomous Exploration with an MAV (ICRA 2020)](https://arxiv.org/abs/2002.04440) — tight integration of octree mapping, frontier extraction, and motion planning; hybrid frontier+sampling approach
- [3D Reactive Control and Frontier-Based Exploration for Unstructured Environments (arXiv 2021)](https://arxiv.org/abs/2108.00380) — two-layer architecture (frontier-based goal selection + artificial potential field obstacle avoidance) for extremely cluttered environments

**ROS 2 Compatibility:**
- [`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2) — port of `explore_lite` to ROS 2 (Humble/Jazzy); 2D frontier-based exploration using Nav2
- Pure frontier-based exploration is relatively easy to implement from scratch in ROS 2 given occupancy grid or OctoMap data
- Works natively with Nav2 for 2D; 3D frontier detection requires custom nodes or integration with Voxblox/OctoMap

**Performance in Narrow/Confined Spaces:**
- **Weakness:** Standard frontier selection (nearest frontier) performs poorly in narrow passages because the robot may oscillate between competing nearby frontiers and may not efficiently resolve dead-end corridors
- **Mitigation:** Utility-based frontier selection (weighting by distance, information gain, and heading) improves corridor traversal
- A potential function approach using direct depth sensor measurements alongside map-based planning helps avoid small obstacles not captured in the map

**Dead-End / Backtracking Handling:**
- Frontiers naturally expire when all neighboring voxels become known; the planner moves to the next closest frontier
- **Key issue:** Without a global graph, frontier selection can be myopic — the robot may inefficiently re-traverse explored corridors to reach remaining frontiers
- **Solution:** Topological maps or hierarchical frontier structures (as in FUEL/TARE) are needed to avoid redundant travel

**SLAM Integration:**
- Depends entirely on the underlying map representation; works with OctoMap, Voxblox ESDF/TSDF, or occupancy grid
- Frontiers are extracted directly from the online map update

**Computational Requirements:**
- Frontier detection is lightweight (O(n) over frontier cells)
- Wavefront-based detection ~6–7× faster than naive methods
- Real-time capable on embedded processors

---

### 1.2 Next-Best-View Planning (NBVP)

**Concept:** Information-theoretic approach where candidate viewpoints are sampled (typically via RRT-style random trees) and the viewpoint that maximizes expected information gain (e.g., unmapped voxels visible from that pose) is selected as the next goal.

**Key Papers:**
- Bircher et al. (ICRA 2016) — "Receding Horizon Next-Best-View Planner for 3D Exploration" — foundational NBVP paper, commonly used as baseline
- [Tree-Based Next-Best-Trajectory for 3D UAV Exploration — ERRT (IEEE TRO 2024)](https://ieeexplore.ieee.org/document/10582913/) — Exploration-RRT combining sampling, safe path planning, and information gain maximization; field-verified in subterranean/narrow environments; **fully ROS-integrated**
- [Efficient 3D Exploration with Distributed Multi-UAV Teams (Drones 2024)](https://www.mdpi.com/2504-446X/8/11/630) — hybrid frontier+NBV for multi-UAV

**ROS 2 Compatibility:**
- The original NBVP (Bircher 2016) is ROS 1 only; no official ROS 2 port
- ERRT (Lindqvist et al., 2024) claims full ROS integration with field deployment in subterranean environments; **check for ROS 2 branch**
- Implementing NBVP from scratch in ROS 2 using OMPL is feasible and common

**Performance in Narrow/Confined Spaces:**
- **Better than pure frontier** for confined spaces because the sampling strategy can place candidate viewpoints inside narrow passages rather than just at the frontier boundary
- ERRT specifically evaluated in "constrained and narrow subterranean and GPS-denied environments" with real field experiments
- Information gain metric naturally discourages re-visiting known areas

**Dead-End / Backtracking Handling:**
- RRT tree growth can reach dead ends and backtrack to parent nodes naturally
- A receding horizon formulation avoids committing to long-range plans that may be invalid
- **Weakness:** Can be slow (tree must be rebuilt or extended at each step)

**SLAM Integration:**
- Requires volumetric map with raycasting capability (OctoMap or Voxblox ESDF)
- Raycasting used to compute information gain per candidate viewpoint

**Computational Requirements:**
- Moderate to high — raycasting for each sampled viewpoint is expensive
- Tree size must be bounded for real-time performance
- Typically 5–15 Hz planning on modern CPUs; depth-limited tree helps

---

### 1.3 GBPlanner / GBPlanner2

**Concept:** A bifurcated local/global graph-based planner purpose-built for subterranean environments. The **local planner** uses a rapidly-exploring random graph (RRG) to find collision-free paths that maximize exploration gain within a local subspace while respecting robot dynamics. The **global planner** incrementally builds a sparse graph and is triggered when the local planner reaches a dead end, needs to reposition the robot to a distant frontier, or must return home.

**Key Papers:**
- [Graph-based Subterranean Exploration Path Planning using Aerial and Legged Robots — Dang et al., Journal of Field Robotics 2020](https://onlinelibrary.wiley.com/doi/10.1002/rob.21993) — original GBPlanner; field-tested in underground mines (US, Switzerland), DARPA SubT Tunnel Circuit
- [Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots — Kulkarni et al., ICRA 2022](https://ieeexplore.ieee.org/document/9812401) — GBPlanner2 + COHORT multi-robot coordination; [PDF](https://marco-tranzatto.github.io/publications/fr_2021_gbplanner_2.pdf)
- [CERBERUS: Autonomous Legged and Aerial Robotic Exploration in the DARPA SubT Challenge (arXiv 2022)](https://arxiv.org/abs/2201.07067) — Team CERBERUS field report, including ANYmal legged robots + aerial robots with GBPlanner2

**GitHub:**
- [`ntnu-arl/gbplanner_ros`](https://github.com/ntnu-arl/gbplanner_ros) — 729 stars, BSD-3-Clause, C++ — branch `gbplanner2` is the latest version
- **ROS 1 only (Melodic/Noetic)** — no official ROS 2 port as of research date
- Mapping backend: OctoMap

**ROS 2 Compatibility:**
- **Not natively ROS 2 compatible**
- Options for thesis: (a) use `ros1_bridge` (as done in M-TARE), (b) port critical components, (c) use as reference implementation for custom ROS 2 reimplementation

**Performance in Narrow/Confined Spaces:**
- Purpose-designed for underground mines and tunnels: tested in passages as narrow as 0.8m with 0.55m-wide collision-tolerant MAV
- GBPlanner2 significantly improved handling of "challenging subterranean geometries and steep slopes" over GBPlanner1
- Field-verified in mines, bunkers, urban underground, DARPA SubT cave circuit, and multi-level power plants

**Dead-End / Backtracking Handling:**
- **Explicitly handled** by the global planning layer: when the local planner cannot find useful frontiers (dead end), the global planner is triggered to reposition the robot to a previously identified frontier node in the global graph
- The global graph incrementally records all visited positions and known frontiers, enabling efficient backtracking

**SLAM Integration:**
- Designed to work with LiDAR-based SLAM (tested with LOAM variants and CompSLAM)
- Map is maintained as OctoMap; frontiers extracted from OctoMap voxels

**Computational Requirements:**
- Moderate; local RRG rebuilds at 1–2 Hz; global graph is sparse and efficient
- Tested on Intel NUC-class hardware onboard aerial robots

---

### 1.4 FUEL (Fast UAV Exploration)

**Concept:** FUEL maintains a **Frontier Information Structure (FIS)** — a data structure that incrementally tracks frontier clusters, their centroids, viewpoints, and yaw angles as the map is explored. A hierarchical planner then: (1) solves a global TSP-like path over all frontier clusters, (2) refines a local set of viewpoints, (3) generates minimum-time B-spline trajectories.

**Key Papers:**
- [FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning — Zhou et al., IEEE RA-L 2021](https://ieeexplore.ieee.org/document/9324988/) — main paper; [arXiv preprint](https://arxiv.org/abs/2010.11561)
- Demonstrated **3–8× faster** exploration than state-of-the-art at time of publication

**GitHub:**
- [`HKUST-Aerial-Robotics/FUEL`](https://github.com/HKUST-Aerial-Robotics/FUEL) — active repository
- Multi-UAV extension also released (2023)
- **ROS 1 only (Melodic/Noetic)** — a ROS 2 port [has been discussed in issues](https://github.com/HKUST-Aerial-Robotics/FUEL/issues/59) but no official port exists
- Dependencies: nlopt, CUDA (optional, CPU-only mode added August 2021)

**ROS 2 Compatibility:**
- **Not natively ROS 2 compatible**
- Given the active community, porting is feasible; key dependencies (nlopt, PCL, Eigen) are ROS 2 compatible
- Alternative: [FASTER](https://github.com/mit-acl/faster) and [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) are spiritually similar and have more recent ROS 2 community attention

**Performance in Narrow/Confined Spaces:**
- FUEL is primarily benchmarked in office-like environments (cluttered open spaces); less specifically tailored to narrow passages than GBPlanner/MBPlanner
- FIS efficiently handles environments where frontiers form linear chains (as in corridors)
- The minimum-time trajectory generator explicitly respects flight envelope and obstacle clearance

**Dead-End / Backtracking Handling:**
- The global TSP-like path over FIS ensures the drone revisits all frontier regions without redundant travel
- FIS incrementally removes frontiers as they are observed, so dead ends are handled by the global re-routing to remaining FIS entries
- **Better than naive frontier methods** at minimizing total travel distance and back-and-forth

**SLAM Integration:**
- Uses a custom ESDF-based volumetric map (built on Voxblox-style TSDF/ESDF)
- The FIS is tightly coupled to the map update loop

**Computational Requirements:**
- Low-moderate; FIS updates are incremental (O(changed voxels))
- TSP global planning is approximated (greedy nearest-neighbor + 2-opt refinement), typically <10ms
- Minimum-time trajectory generation: ~20–50ms
- Suitable for onboard computing (tested on Intel NUC-class hardware)

---

### 1.5 TARE (Technology-Aware Robot Exploration)

**Concept:** A hierarchical framework with two processing levels operating simultaneously: (1) a **dense local level** maintains a detailed map and computes an accurate path within a local planning horizon; (2) a **sparse global level** maintains a coarse map of the entire explored area and computes a coarse global coverage path. Both paths are joined at the local horizon boundary, prioritizing near-vehicle detail while maintaining global coverage awareness.

**Key Papers:**
- [TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments — Cao et al., RSS 2021](https://www.ri.cmu.edu/app/uploads/2021/06/RSS_2021.pdf) — **Best Paper Award and Best System Paper Award at RSS 2021**
- [Exploring the Most Sectors at the DARPA SubT Challenge Finals — Cao et al., Field Robotics 2023](https://ieeexplore.ieee.org/document/10882594/) — CMU-OSU team report; 26/28 sectors explored, "Most Sectors Explored" award

**GitHub:**
- [`caochao39/tare_planner`](https://github.com/caochao39/tare_planner) — TARE single-robot
- [`caochao39/mtare_planner`](https://github.com/caochao39/mtare_planner) — M-TARE multi-robot; **uses `ros1_bridge` to connect ROS 1 TARE to ROS 2 inter-robot communication messages**
- **ROS 1 only (Melodic/Noetic)** for core planner; OR-Tools dependency; now ARM-compatible (previously AMD64 only)

**ROS 2 Compatibility:**
- **Not natively ROS 2 compatible** for the core planner
- M-TARE uses `ros1_bridge` for inter-robot ROS 2 messages — this is the clearest path to integration in a ROS 2 system
- The CMU Autonomous Exploration Development Environment ([cmu-exploration.com](https://www.cmu-exploration.com)) provides simulation environments compatible with TARE

**Performance in Narrow/Confined Spaces:**
- DARPA SubT final event: Louisville Mega Cavern (KY) — real cave environment; robots traveled 886m+ fully autonomously
- Specifically supports a `tunnel` simulation environment
- Dense local map ensures tight obstacle clearance near the vehicle
- Sparse global map maintains coarse coverage picture

**Dead-End / Backtracking Handling:**
- The coarse global path ensures the robot doesn't get "stuck" locally — it can always reference the global coverage plan to navigate away from dead ends
- After local exploration is exhausted, the global plan redirects the robot to uncovered remote regions

**SLAM Integration:**
- Integrates with the CMU exploration development stack; uses LiDAR-based localization (LOAM/LeGO-LOAM compatible)
- OR-Tools used for global path optimization (TSP-style)

**Computational Requirements:**
- Single CPU thread; tested on 4.1GHz i7
- Uses less than 50% of computation of state-of-the-art methods (per paper)
- ARM-compatible now available

---

### 1.6 MBPlanner (Motion Primitive-Based Planner)

**Concept:** Uses a library of **motion primitives** (pre-computed dynamically feasible trajectory segments) to search the configuration space efficiently. For each primitive, the planner evaluates exploration gain (unmapped volume visible along the trajectory), collision safety, and dynamic feasibility. This tightly couples path planning with UAV flight dynamics, making it especially suitable for **fast and agile exploration** in confined spaces.

**Key Papers:**
- [Motion Primitives-based Path Planning for Fast and Agile Exploration using Aerial Robots — Dharmadhikari et al., ICRA 2020](https://ieeexplore.ieee.org/document/9196964/) — main paper; tested in underground mines at up to 2m/s in passages as narrow as 0.8m
- [Appendix: Global Planning Extension for MBPlanner (arXiv 2020)](https://arxiv.org/abs/2012.03228) — adds global planning layer for large-scale exploration

**GitHub:**
- [`ntnu-arl/mbplanner_ros`](https://github.com/ntnu-arl/mbplanner_ros) — by NTNU Autonomous Robots Lab (same group as GBPlanner)
- Mapping backends: **Voxblox** (default, TSDF/ESDF) or **OctoMap**
- Supports DARPA SubT Cave Circuit simulation environments
- **ROS 1 only (Melodic/Noetic)**

**ROS 2 Compatibility:**
- **Not natively ROS 2 compatible**
- Same migration path as GBPlanner: `ros1_bridge` or manual port (Voxblox and OctoMap have ROS 2 support)
- A ROS 2 compatible motion-primitive library: [`sikang/motion_primitive_library`](https://github.com/sikang/motion_primitive_library) (MRSL MPL) — quadrotor-specific, more basic

**Performance in Narrow/Confined Spaces:**
- **Best-in-class for narrow confined environments** at the time of publication
- 0.55m-wide collision-tolerant MAV in 0.8m-wide mine drifts at 2m/s
- Motion primitives inherently respect actuator limits and velocity constraints — no infeasible trajectory commands
- Explicitly designed for "fast and agile" behavior in subterranean settings

**Dead-End / Backtracking Handling:**
- When all local primitives lead to occupied or unknown space, the local planner signals failure
- The global planning extension (from the appendix paper) then repositions the robot to a distant frontier via the global graph
- This bifurcated approach is shared with GBPlanner and is well-validated

**SLAM Integration:**
- Works with Voxblox (primary) or OctoMap; uses ESDF for collision checking (fast distance queries)
- Used alongside LiDAR SLAM (LOAM family) in SubT deployments

**Computational Requirements:**
- Low — motion primitive evaluation is fast (table lookup + collision check via ESDF distance query)
- Real-time at 10–20 Hz on Intel NUC
- ESDF distance queries are O(1) per point

---

## 2. Comparative Summary Table

| Planner | ROS 2 Native | Primary Use Case | Narrow Spaces | Dead-End Handling | SLAM Integration | Computation | SubT Validated |
|---|---|---|---|---|---|---|---|
| **Frontier-Based** | ✅ (m-explore-ros2) | General 2D/3D | Fair | Poor (greedy) | Any occupancy map | Very Low | No |
| **NBVP** | ❌ (manual port needed) | 3D volumetric | Good | Moderate (tree backtrack) | OctoMap/Voxblox | Moderate-High | ICRA 2016 baseline |
| **GBPlanner2** | ❌ (ROS1 bridge) | Subterranean tunnels | Excellent | Excellent (global graph) | OctoMap | Moderate | ✅ DARPA SubT (CERBERUS) |
| **FUEL** | ❌ (ROS1, port possible) | UAV indoor/outdoor | Good | Good (FIS global path) | Custom ESDF | Low-Moderate | Not SubT-specific |
| **TARE** | ❌ (ROS1 bridge) | Complex 3D any platform | Very Good | Good (sparse global) | LiDAR SLAM | Very Low | ✅ DARPA SubT Finals (Best Paper) |
| **MBPlanner** | ❌ (ROS1, Voxblox) | Agile UAV in mines | **Best** | Good (global graph) | Voxblox ESDF | Low | ✅ DARPA SubT (NTNU) |

**For procedurally-generated caves in a ROS 2 thesis:**
- **Primary recommendation:** Port **GBPlanner2** or **MBPlanner** via `ros1_bridge` or manual port — both are purpose-designed for the exact scenario
- **Secondary recommendation:** Implement **FUEL**-style FIS with hierarchical planning natively in ROS 2 (the concepts are well-described in the paper)
- **Quick start:** Use **frontier-based exploration** (`m-explore-ros2` + Nav2) for early prototyping, then migrate to a richer planner

---

## 3. Goal-Directed Path Planning

### 3.1 RRT* / Informed RRT*

**Concept:** Rapidly-exploring Random Tree Star (RRT*) is an asymptotically optimal sampling-based planner. **Informed RRT*** (Gammell et al.) accelerates convergence by restricting sampling to an ellipsoidal subset of the state space that can improve the current best solution. In 3D, the subset becomes an oblique cylinder, reducing search space by 75–84% versus general RRT* in typical drone scenarios.

**Key Papers:**
- [Informed RRT* paper — Gammell et al., IROS 2014](https://doi.org/10.1109/IROS.2014.6942976)
- [3D Informed RRT* for UAV Obstacle Avoidance (UCL/ROBIO 2018)](https://discovery.ucl.ac.uk/10121867/3/Pawar_ROBIO_paper_2018.pdf) — achieves 16–25% smaller search space vs. RRT* in 3D multi-obstacle environments
- [3D Exploration and Navigation with Optimal-RRT Planners (PMC/Sensors 2019)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6983016/) — full autonomous navigation and exploration pipeline using RRT* with OctoMap
- [Implementation of Informed RRT* on OctoMap from RTAB-Map (Scribd summary)](https://www.scribd.com/document/504551795/Implementation-of-Informed-RRT-on-a-Pre-Mapped-Octomap-Generated-by-RTAB-Map) — quadcopter navigation; OMPL + FCL + OctoMap integration; <1 second planning time

**ROS 2 Integration:**
- [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/) — the standard implementation; available for ROS 2 via `moveit2`
- OctoMap collision checking with FCL (Flexible Collision Library)
- Can query ESDF from Voxblox for faster collision checking than raw OctoMap raycasting
- Direct ROS 2 path: `ompl` → `fcl` → `octomap_server2` for map; publish `nav_msgs/Path`

**Performance in Caves:**
- Works well for goal-directed navigation (A→B) once the map is partially known
- **Not suited for exploration** (no information gain metric) — use for local/global re-navigation
- Path quality converges toward optimal with more computation time (anytime property)
- For 3D cave: resolution ~0.1m OctoMap, typical planning time <1s for 20–30m corridors

**Computational Requirements:**
- Low-moderate; single planning call typically <500ms for small environments
- Use lazy collision checking for speed; batch-validate path segments

---

### 3.2 A* on OctoMap / Voxel Grids

**Concept:** Graph-search over a discretized 3D occupancy grid (OctoMap or voxel grid). A* uses a heuristic (Euclidean distance to goal) to guide search efficiently. Guaranteed optimal path on the discretized grid.

**Key Papers / Implementations:**
- [OctoMap Library](https://octomap.github.io/) — standard probabilistic 3D mapping; `octomap_server2` available for ROS 2
- [A* in 3D OctoMap for drone navigation (GAAS tutorial)](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-part-4-stereo-depth-estimation-octomap-and-path-planning) — full pipeline: stereo camera → depth → OctoMap → 3D A* → path pruning → MAVROS waypoints
- [LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map](https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map) — ROS 2 package to convert OctoMap to 2D occupancy + 3D path conversion for UAVs; explicitly supports aerial robots with collision spheres
- [Autonomous Drone Navigation with A* in ROS2 + MAVROS](https://www.youtube.com/watch?v=pmalu7tcEw4) — direct demonstration integrating A* path planning with ROS2 + MAVROS + PX4

**ROS 2 Integration:**
- `octomap_server2` is available for ROS 2 (Humble/Jazzy)
- Custom A* over OctoMap: query occupied/free status per voxel; straightforward C++ or Python node
- **Lazy Theta*** is an improvement over A* for 3D: allows any-angle paths, fewer waypoints
- Inflate obstacles by robot radius before planning for safe clearance

**Performance in Caves:**
- A* on 3D OctoMap with 0.1m resolution is memory and compute intensive in large caves
- **Use multi-resolution OctoMap** (coarser resolution far from robot, finer near robot)
- Path pruning essential: raw A* paths have many redundant intermediate waypoints
- Corridor-following behavior emerges naturally since free voxels define the navigable space

**Computational Requirements:**
- Depends heavily on map resolution and environment size
- 0.1m resolution OctoMap: A* in a 20×20×5m volume: ~100ms–2s (unbounded cave growth is problematic)
- **Prefer ESDF-based RRT*** for larger volumes; use 3D A* for short-range re-planning

---

### 3.3 Model Predictive Control (MPC)

**Concept:** MPC formulates trajectory generation/tracking as a receding-horizon optimal control problem. At each step, a finite-horizon trajectory is optimized considering drone dynamics, obstacle constraints, and tracking error. Only the first control action is applied; the horizon shifts forward. This provides both planning and control in a single unified framework.

**Key Papers:**
- [Custom NMPC for Obstacle Avoidance on DJI Matrice 100 (arXiv 2024)](https://arxiv.org/abs/2410.02732) — B-spline reference trajectories, CasADi for real-time optimization, indoor/outdoor experiments; code public
- [NMPC-Based Collision Avoidance for UAV in Narrow-Cluttered Environments (Semantic Scholar)](https://pdfs.semanticscholar.org/e109/62cfab3bc6f5f550bb81df717ec41228d8b9.pdf) — LiDAR-based DBSCAN clustering of obstacles as MPC constraints; **positive performance in narrow-cluttered environments**
- [Model Predictive Contouring Control (MPCC) for Unstructured Environments (IROS 2019)](https://www.youtube.com/watch?v=crGTsiiilHo) — MPCC with convex free-space regions; real-time at 30+ Hz; robot-agnostic
- [Optimal Motion Planning in GPS-Denied Environments Using Nonlinear MPC (Sensors 2021)](https://www.mdpi.com/1424-8220/21/16/5547) — graph-based global planner + NMPC local planner for subterranean GPS-denied environments; directly relevant

**ROS 2 Integration:**
- CasADi (used in multiple NMPC papers) has Python/C++ interfaces compatible with ROS 2 nodes
- [ACADO Toolkit](https://acado.github.io/) and [acados](https://acados.org/) are faster MPC solvers with ROS 2 community support
- Typical architecture: global planner provides reference path → MPC tracks path while avoiding local obstacles

**Performance in Caves:**
- MPC is ideal as a **local obstacle avoidance layer** for narrow passages
- Can enforce velocity limits, maximum tilt angles, and minimum clearance constraints simultaneously
- Receding horizon naturally handles dynamic re-planning when new obstacles are detected
- Response to SLAM drift: MPC uses current pose estimate; SLAM failures propagate as incorrect obstacles

**Computational Requirements:**
- NMPC: 10–50ms per solve step with CasADi/acados on a modern CPU (i7/Jetson Orin)
- Horizon length trade-off: 10–20 timesteps × 0.1s = 1–2s prediction is typical for drone MPC
- Consider using simpler linear MPC or DWA for very low-compute platforms

---

### 3.4 PX4 Avoidance Package

**Concept:** [`PX4/avoidance`](https://github.com/PX4/avoidance) provides a local planner and global planner for obstacle avoidance that interfaces with PX4 via MAVLink. The local planner emits setpoints at ~30Hz (up to 3m/s); the global planner emits at ~10Hz (1–1.5m/s). Uses depth camera input; OctoMap as internal map.

**GitHub:** [`PX4/avoidance`](https://github.com/PX4/avoidance)

**ROS 2 Compatibility:**
- **Core package is ROS 1 only**
- A ROS 2 branch exists via Auterion but is described as "very outdated and may not work on Humble" (per PX4 forum 2023)
- PX4 v1.14+ uses uXRCE-DDS for ROS 2 communication — the newer paradigm replaces MAVROS with native ROS 2 topics
- **Recommended alternative for ROS 2 thesis:** Use PX4 with uXRCE-DDS + custom local avoidance node (e.g., NMPC-based) instead of the legacy avoidance package
- [`aerial-autonomy-stack`](https://arxiv.org/html/2602.07264v1) (arXiv 2026) — new open-source autopilot-agnostic ROS 2 framework for PX4 and ArduPilot with LiDAR SLAM; directly relevant

**Performance in Caves:**
- Local planner: sphere-based collision avoidance using depth camera; struggles in very narrow passages (<1.5× robot width) due to lack of spatial reasoning
- **Not recommended as the primary avoidance layer for cave exploration** — too reactive, low speed
- Better used as a safety fallback layer beneath a more intelligent planner

---

### 3.5 Nav2 Adaptation for 3D/Aerial Navigation

**Concept:** Nav2 is the ROS 2 navigation stack, designed for 2D ground robots. Adapting it to 3D aerial navigation requires replacing the 2D costmap with a 3D representation and the controller with a drone-appropriate algorithm.

**Key Resources:**
- [`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2) — frontier exploration for ROS 2 using Nav2; tested on Humble/Jazzy; provides the exploration layer
- [`darshmenon/rosnav`](https://github.com/darshmenon/rosnav) — ROS 2 navigation with SLAM via Nav2; frontier-based exploration; Humble/Jazzy compatible
- [LTU-RAI OctoMap→2D conversion](https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map) — converts OctoMap to 2D occupancy for UAVs with 3D path conversion; ROS 2 package

**Limitations for Aerial/3D:**
- Nav2's costmap system is fundamentally 2D (with optional 3D inflation layers)
- The default DWB local planner assumes ground robot kinematics
- **For a thesis drone in a cave:** Nav2 is useful for 2D slice-based navigation (keep the drone at a fixed altitude and use 2D Nav2), but is insufficient for full 3D exploration

**Recommended Approach for ROS 2 Thesis:**
- Use Nav2 for the 2D "goal navigation" layer if you fix altitude
- For true 3D navigation, build a custom planner pipeline: OctoMap → RRT*/A* → MPC controller → MAVROS/uXRCE-DDS → PX4
- `nav2_smac_planner` (hybrid A*) can be extended for 3D by creating a custom state space plugin

---

## 4. Full Autonomy Stack Architecture

### 4.1 Stack Layers: Global → Local → Controller

```
┌─────────────────────────────────────────────────────────┐
│                    MISSION LAYER                         │
│  State machine: EXPLORE → NAVIGATE → RETURN HOME → LAND │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│              GLOBAL EXPLORATION PLANNER                  │
│  Options: GBPlanner2 / TARE / FUEL / MBPlanner           │
│  Input: 3D map (OctoMap/Voxblox ESDF), robot pose        │
│  Output: sequence of waypoints / frontier goals          │
│  Rate: 0.5–2 Hz                                          │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│              GLOBAL PATH PLANNER                         │
│  Options: RRT* / Informed RRT* / A* on OctoMap           │
│  Input: current pose, goal waypoint, 3D map              │
│  Output: collision-free 3D geometric path                │
│  Rate: 1–5 Hz (replanned on map update or new goal)      │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│              LOCAL OBSTACLE AVOIDANCE                    │
│  Options: NMPC / DWA-3D / Potential Fields               │
│  Input: reference path, live sensor data (LiDAR/depth)   │
│  Output: velocity setpoints / trajectory segment        │
│  Rate: 10–30 Hz                                          │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│              DRONE CONTROLLER (PX4 / ArduPilot)          │
│  Attitude, thrust, velocity tracking                     │
│  Interface: MAVROS (ROS1) or uXRCE-DDS (ROS2 native)     │
│  Rate: 50–250 Hz                                         │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│              SLAM / LOCALIZATION                         │
│  Options: FAST-LIO2, FAST-LIVO2, LIO-SAM, LOAM          │
│  Output: 6-DOF pose + 3D point cloud map                 │
│  Rate: 10–20 Hz                                          │
└─────────────────────────────────────────────────────────┘
```

**Data flow:**
- SLAM provides pose to all layers and contributes to the volumetric map
- The global exploration planner queries the map for frontiers/information gain
- The path planner queries the map for collision checking
- The local planner reads live sensor data for dynamic obstacle avoidance
- The controller receives velocity/position setpoints and closes the control loop at high frequency

---

### 4.2 SLAM Integration

**Recommended SLAM systems for caves (GPS-denied, dark, narrow):**

| SLAM System | Sensor | ROS 2 | Key Advantage |
|---|---|---|---|
| **FAST-LIO2** | LiDAR + IMU | ✅ native | Very fast; works in narrow tunnels; no feature extraction needed |
| **FAST-LIVO2** | LiDAR + IMU + camera | ✅ native | Higher accuracy; visual texture helps in sparse LiDAR environments |
| **LIO-SAM** | LiDAR + IMU + GPS (opt.) | ✅ available | Loop closure support; better long-range drift correction |
| **LOAM / LeGO-LOAM** | LiDAR | ROS1 primarily | Classic SubT choice; less computational overhead |
| **SLAM Toolbox** | 2D LiDAR | ✅ native ROS2 | Only 2D; not suitable for 3D cave exploration |

**FAST-LIO2 is the recommended choice** for this thesis:
- Tested in narrow horizontal and vertical tunnels ([YouTube demo](https://www.youtube.com/watch?v=emiSJMcA8yM))
- Uses incremental ikd-Tree for efficient point insertion and map updates
- Directly feeds point cloud to OctoMap or Voxblox for the exploration planner

**Map representations:**
- **OctoMap** (`octomap_server2`, ROS 2 native): probabilistic 3D occupancy; good for sparse environments; A*/RRT* collision queries
- **Voxblox** (Oleynikova et al.): TSDF + ESDF; fast distance queries; used by MBPlanner and FUEL; [ROS 2 port available](https://github.com/ethz-asl/voxblox)
- **Bonxai** (newer): faster than OctoMap; mentioned in PX4 community as drop-in replacement

---

### 4.3 Return-to-Base Behavior

**Problem:** When battery is critically low in a deep cave, the drone must find its way back to the entry point reliably — the path may be long, winding, and partially blocked.

**Approaches from SubT research:**

1. **Graph-based return path (GBPlanner/TARE):**
   - The global graph built during exploration records the traversed topology
   - Return path = shortest path in the global graph from current position to home node
   - **Advantage:** Guaranteed navigable (drone already traversed this path forward)
   - **Implementation:** Dijkstra/A* on the sparse global graph; typically very fast

2. **Battery-aware planning:**
   - Maintain a real-time estimate of battery-remaining vs. return-energy cost
   - [Multi-Objective Risk Assessment for Exploration Planning (arXiv 2024)](https://arxiv.org/abs/2410.03917) — accounts for battery life, travel distance, and risk simultaneously in cave environments
   - Trigger RTH when: `battery_remaining < return_cost × safety_margin`
   - `return_cost` estimated from path length × power-per-meter model

3. **Dynamic RTH decision:**
   - Monitor battery at 1 Hz
   - Compute shortest return path distance from global graph
   - Estimate flight time (distance / average_speed) + hover overhead
   - Add 20–30% safety margin
   - Trigger RTH state machine when margin is reached

**State Machine for Return:**
```
EXPLORE → [battery_threshold_reached] → RETURN_HOME
RETURN_HOME → [at_home ± 0.5m] → LAND
RETURN_HOME → [battery_critical] → EMERGENCY_LAND_IN_PLACE
```

**ROS 2 Implementation:**
- `BehaviorTree.CPP` (used by Nav2) can model this state machine
- Battery topic: `/mavros/battery` or `/fmu/out/battery_status` (uXRCE-DDS)
- Return path: query global exploration graph; publish as `nav_msgs/Path`

---

### 4.4 Communication Relay for Deep Cave Exploration

**Problem:** Radio signals attenuate rapidly around cave bends. After ~50–100m into a complex cave, line-of-sight to the operator is lost. This affects: telemetry, emergency overrides, map data exfiltration.

**Solutions from DARPA SubT:**

1. **Droppable communication nodes (breadcrumbs):**
   - Drone carries a dispenser of small WiFi/UWB mesh nodes
   - Drops a node when signal strength (RSSI) drops below a threshold
   - Each node relays to the previous one → mesh chain back to base
   - **CHORD system (Team CoSTAR/JPL)** — won 1st place in SubT Urban Circuit; MANET radios (Silvus Technologies SC4240E); demonstrated 100s of meters into underground environments
   - [`ACHORD` paper (arXiv 2022)](https://arxiv.org/abs/2206.02245) — formal description of autonomous relay dropping and bandwidth prioritization

2. **Autonomous operation without communication:**
   - The drone must operate fully autonomously without any operator commands
   - All exploration decisions, path planning, and RTH must run onboard
   - Map data is stored onboard and exfiltrated when communication is restored (near a relay node or at entry)

3. **Communication-aware planning:**
   - Planner includes a "signal strength" constraint: don't move into areas where the relay chain would break
   - Or: plan relay drops proactively at topological decision points (junctions, long straights)

**For thesis (simulated cave):**
- If using a single simulated drone without multi-robot concerns, skip physical relays
- Implement a `communication_range_monitor` node that simulates signal loss based on path length/topology
- Plan relay drops as part of the mission if multi-drone or human-in-loop communication is in scope

---

### 4.5 Emergency Behaviors

#### Lost Communication

```
IF no_heartbeat_from_operator > 30s:
    → Continue autonomous exploration (if battery sufficient)
    → OR → Hold position and wait T_wait seconds
    → OR → Begin return-to-home immediately
```

In SubT systems, robots operate fully autonomously with no communication for extended periods. The recommended behavior for a thesis drone: **continue autonomous exploration** unless battery triggers RTH.

#### SLAM Failure Detection and Recovery

SLAM failure manifests as:
- Sudden large pose jump (> 0.5m in a single update)
- Covariance explosion in the pose estimate
- LiDAR scan registration fails (FAST-LIO2 reports this via health flag)
- Inconsistency between IMU dead-reckoning and SLAM pose

**Detection:**
```python
# ROS 2 node: slam_health_monitor
if abs(pose_delta) > SLAM_JUMP_THRESHOLD:
    publish_slam_failure()
if slam_covariance > COVARIANCE_THRESHOLD:
    publish_slam_failure()
```

**Recovery strategies:**
1. **Hold position and re-initialize SLAM** with the current point cloud as the initial map
2. **Back up along last known safe path** (reverse the last N waypoints)
3. **Rotate in place** to accumulate point cloud for re-localization
4. **SLAM loop closure trigger** — if LIVO-SAM/LIO-SAM is used, force a relocalization attempt

**Map backup strategy:**
- Serialize the current OctoMap to disk at regular intervals (every 30 seconds)
- On SLAM failure, restore the last known good map and restart from the last known good pose
- This prevents losing all map data if SLAM crashes

#### Collision Imminent

```
IF distance_to_obstacle < COLLISION_THRESHOLD (e.g., 0.3m):
    → Emergency hover / stop
    → Override all planner commands
    → MPC with hard collision constraints active
```

This should be implemented as a hardware-priority node that reads the depth sensor directly and publishes emergency stop commands at the highest ROS 2 priority.

---

## 5. Recommended Architecture for Thesis

**Given:** ROS 2, procedurally-generated caves, single drone, thesis timeline

### Phased Approach

**Phase 1: Basic Autonomy (weeks 1–6)**
- SLAM: FAST-LIO2 (ROS 2 native)
- Map: OctoMap via `octomap_server2`
- Exploration: `m-explore-ros2` (frontier-based, Nav2-integrated)
- Path planning: A* via Nav2 SMAC planner
- Controller: PX4 via MAVROS or uXRCE-DDS
- Goal: drone explores a simple cave, builds map, returns home

**Phase 2: Subterranean Planner Integration (weeks 7–14)**
- Replace frontier-based exploration with **TARE** (via `ros1_bridge`) or a ROS 2 reimplementation of GBPlanner2 concepts
- Replace A* path planner with **Informed RRT*** on ESDF (Voxblox)
- Add battery-aware RTH state machine
- Goal: efficient exploration of multi-branched cave with dead-end handling

**Phase 3: Robustness and Emergency Behaviors (weeks 15–20)**
- Add SLAM failure detection and recovery
- Implement NMPC local obstacle avoidance layer
- Add communication simulation + relay drop behavior (optional)
- Evaluate: coverage rate, exploration time, path efficiency, recovery success rate

### Technology Stack Summary

```
ROS 2 (Humble or Jazzy)
├── SLAM: FAST-LIO2 → point cloud + pose
├── Map: Voxblox (ESDF) → collision + frontiers
├── Exploration: GBPlanner2 (ros1_bridge) or custom FIS
├── Global Path: Informed RRT* (OMPL + FCL)
├── Local Avoidance: NMPC (CasADi/acados)
├── Mission State Machine: BehaviorTree.CPP
├── Drone Controller: PX4 via uXRCE-DDS
└── Simulation: Gazebo Harmonic + custom cave generator
```

### Simulation Setup

- **Gazebo Harmonic** (ROS 2 Jazzy compatible) or **Gazebo Classic** (Humble compatible)
- Generate procedural caves with Python (perlin noise heightmaps, L-systems for branching)
- Export to SDF/URDF mesh for Gazebo
- Use `aerial-autonomy-stack` ([arXiv 2026](https://arxiv.org/html/2602.07264v1)) for integrated PX4+ROS2+LiDAR simulation framework

---

## 6. Key Papers Reference List

### Exploration Planners

1. Dang, T., Tranzatto, M., et al. (2020). **Graph-based subterranean exploration path planning using aerial and legged robots.** *Journal of Field Robotics, 37*(8), 1363–1388. https://doi.org/10.1002/rob.21993

2. Kulkarni, M., Dharmadhikari, M., et al. (2022). **Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots.** *ICRA 2022*. https://doi.org/10.1109/ICRA46639.2022.9812401

3. Zhou, B., Zhang, Y., Chen, X., & Shen, S. (2021). **FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning.** *IEEE RA-L, 6*(2), 779–786. https://doi.org/10.1109/LRA.2021.3051563. [arXiv](https://arxiv.org/abs/2010.11561)

4. Cao, C., Zhu, H., Choset, H., & Zhang, J. (2021). **TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments.** *RSS 2021* (**Best Paper Award**). https://www.ri.cmu.edu/app/uploads/2021/06/RSS_2021.pdf

5. Dharmadhikari, M., Dang, T., et al. (2020). **Motion Primitives-based Path Planning for Fast and Agile Exploration using Aerial Robots.** *ICRA 2020*. https://doi.org/10.1109/ICRA40945.2020.9196964

6. Bircher, A., et al. (2016). **Receding Horizon "Next-Best-View" Planner for 3D Exploration.** *ICRA 2016.* (NBVP baseline)

7. Lindqvist, B., Patel, A., et al. (2024). **A Tree-Based Next-Best-Trajectory Method for 3D UAV Exploration.** *IEEE TRO*. https://doi.org/10.1109/TRO.2024.3422052

8. Tranzatto, M., et al. (2022). **CERBERUS: Autonomous Legged and Aerial Robotic Exploration in the DARPA SubT Challenge.** [arXiv:2201.07067](https://arxiv.org/abs/2201.07067)

### Path Planning

9. Gammell, J.D., et al. (2014). **Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic.** *IROS 2014.*

10. Dang, T., Mascarich, F., et al. (2019). **Graph-Based Path Planning for Autonomous Robotic Exploration in Subterranean Environments.** *IROS 2019.* https://doi.org/10.1109/IROS40897.2019.8968151

11. Dharmadhikari, M., & Alexis, K. (2020). **Appendix for MBPlanner — Global Planning Integration.** [arXiv:2012.03228](https://arxiv.org/abs/2012.03228)

### MPC / Control

12. Bui, D.-N., et al. (2024). **Model Predictive Control for Optimal Motion Planning of Unmanned Aerial Vehicles.** [arXiv:2410.09799](https://arxiv.org/abs/2410.09799)

13. Barczyk, M., & Al Younes, Y. (2021). **Optimal Motion Planning in GPS-Denied Environments Using Nonlinear Model Predictive Horizon.** *Sensors, 21*(16), 5547. https://doi.org/10.3390/s21165547

### SLAM

14. Xu, W., et al. (2022). **FAST-LIO2: Fast Direct LiDAR-Inertial Odometry.** *IEEE TRO.* https://doi.org/10.1109/TRO.2022.3141876. [arXiv](https://arxiv.org/abs/2107.06829)

15. Zheng, C., et al. (2024). **FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.** [arXiv:2408.14035](https://arxiv.org/html/2408.14035v2)

### SubT Challenge Reports

16. Agha, A., et al. (2022). **NeBula: TEAM CoSTAR's Robotic Autonomy Solution that Won Phase II of DARPA Subterranean Challenge.** *Field Robotics.* https://doi.org/10.55417/fr.2022047. [arXiv](https://arxiv.org/abs/2103.11470)

17. Cao, C., Nogueira, L., et al. (2023). **Exploring the Most Sectors at the DARPA SubT Challenge Finals.** *Field Robotics.* https://doi.org/10.55417/fr.2023025

18. Rouček, T., et al. (2021). **System for multi-robotic exploration of underground environments — CTU-CRAS-NORLAB in DARPA SubT.** [arXiv:2110.05911](https://arxiv.org/abs/2110.05911)

### Communication in Caves

19. Saboia, M., et al. (2022). **ACHORD: Communication-Aware Multi-Robot Coordination With Intermittent Connectivity.** [arXiv:2206.02245](https://arxiv.org/abs/2206.02245)

20. Anonymous (2023). **A Hansel & Gretel breadcrumb-style dynamically deployed communication network for subterranean exploration.** *Advances in Space Research.* https://pmc.ncbi.nlm.nih.gov/articles/PMC10399462/

### Software / Frameworks

21. `ntnu-arl/gbplanner_ros` (GBPlanner2): https://github.com/ntnu-arl/gbplanner_ros
22. `ntnu-arl/mbplanner_ros` (MBPlanner): https://github.com/ntnu-arl/mbplanner_ros
23. `caochao39/tare_planner` (TARE): https://github.com/caochao39/tare_planner
24. `HKUST-Aerial-Robotics/FUEL` (FUEL): https://github.com/HKUST-Aerial-Robotics/FUEL
25. `robo-friends/m-explore-ros2` (frontier ROS 2): https://github.com/robo-friends/m-explore-ros2
26. `LTU-RAI OctoMap→2D for UAV` (ROS 2): https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map
27. `aerial-autonomy-stack` (ROS 2 + PX4/ArduPilot): https://arxiv.org/html/2602.07264v1

---

*Report compiled April 2026. All referenced GitHub repositories should be re-checked for updates; the field advances rapidly.*
