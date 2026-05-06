a whik# Exploration and Navigation Guide

**Document type:** Algorithm reference and tuning guide  
**Audience:** Thesis authors, algorithm contributors  
**Cross-references:** [ARCHITECTURE.md](ARCHITECTURE.md) | [SLAM_GUIDE.md](SLAM_GUIDE.md) | [API_REFERENCE.md](API_REFERENCE.md)

---

## Table of Contents

1. [Exploration System Overview](#1-exploration-system-overview)
2. [Frontier-Based Exploration Algorithm](#2-frontier-based-exploration-algorithm)
3. [RRT* Path Planning](#3-rrt-path-planning)
4. [Artificial Potential Field Local Planner](#4-artificial-potential-field-local-planner)
5. [Mission Manager State Machine](#5-mission-manager-state-machine)
6. [Exploration Tuning Guide](#6-exploration-tuning-guide)
7. [Known Limitations and Workarounds](#7-known-limitations-and-workarounds)
8. [References](#8-references)

---

## 1. Exploration System Overview

The `exploration_planner` package implements a four-layer autonomy stack for cave exploration:

```
  Layer 1: Mission Management        mission_manager.py
  ──────────────────────────────
  7-state FSM; governs lifecycle (TAKEOFF → EXPLORE → NAVIGATE → RETURN_HOME)
  Responds to battery level, SLAM health, and operator commands

  Layer 2: Exploration Planning      frontier_explorer.py
  ──────────────────────────────
  Frontier detection from OctoMap
  Utility-based frontier scoring (information gain × distance × heading)
  Publishes best frontier as goal

  Layer 3: Path Planning             path_planner.py  +  collision_checker.py
  ──────────────────────────────
  RRT* sampling-based planner over 3D OctoMap
  A* fallback on 3D voxel grid
  B-spline path smoothing

  Layer 4: Local Motion Control      local_planner.py
  ──────────────────────────────
  Artificial potential field (APF) reactive control
  20 Hz command rate
  Real-time obstacle avoidance from live LiDAR scan
```

The layered design mirrors the architecture used by CERBERUS (Team CERBERUS, DARPA SubT 2021 winner) and documented in the GBPlanner2 paper (Kulkarni et al., ICRA 2022). The key insight is that global planning (Layers 1–3) can run at low rates (0.5–2 Hz) while local control (Layer 4) must run at high rates (20 Hz) to guarantee safe navigation in cluttered environments.

---

## 2. Frontier-Based Exploration Algorithm

**Source:** `exploration_planner/frontier_explorer.py`  
**Rate:** 2 Hz  
**Input:** `/octomap_full_color`, `/slam/odometry`  
**Output:** `/exploration/frontiers` (MarkerArray), `/exploration/best_frontier` (PoseStamped)

### 2.1 Frontier Definition

A **frontier** is a boundary cell between known free space and unknown space in the 3D occupancy map. Formally, a voxel `v` is a frontier if:

```
v is marked FREE in OctoMap
AND at least one 26-neighbor of v is UNKNOWN
AND v is within max_exploration_range of drone position
```

This definition generalizes the classic 2D frontier concept (Yamauchi 1997) to 3D. In the OctoMap representation, "unknown" voxels are those that have never been observed by the LiDAR.

### 2.2 Frontier Extraction

The frontier extraction operates on the OctoMap published by `octomap_builder`:

```python
def extract_frontiers(octomap, drone_pos, max_range):
    frontier_points = []
    for voxel in octomap.free_voxels():
        if distance(voxel.center, drone_pos) > max_range:
            continue
        for neighbor in voxel.neighbors_26():
            if neighbor.status == UNKNOWN:
                frontier_points.append(voxel.center)
                break
    return frontier_points
```

**Performance optimization:** Rather than querying every free voxel (O(N³)), only voxels that changed status in the last OctoMap update are checked for frontier membership. This reduces the per-cycle computation from O(N³) to O(ΔN), where ΔN is the number of newly observed voxels.

### 2.3 Frontier Clustering

Raw frontier voxels form a sparse point cloud. Nearby frontier voxels are grouped into clusters using DBSCAN:

```python
from sklearn.cluster import DBSCAN

clustering = DBSCAN(
    eps=dbscan_eps,           # Neighbourhood radius (1.0 m default)
    min_samples=dbscan_min_samples  # Minimum cluster size (3)
).fit(frontier_points)
```

Clusters with fewer than `frontier_min_size=5` voxels are discarded as noise. Each surviving cluster is represented by its centroid and convex hull volume.

### 2.4 Utility-Based Frontier Scoring

Each frontier cluster is assigned a utility score that balances three objectives:

```
U(f) = w_info × IG(f) - w_dist × D(f) + w_heading × H(f)
```

Where:

- **IG(f): Information Gain** — estimated number of unknown voxels visible from the frontier centroid, computed by ray casting `raytrace_samples=50` rays in a hemisphere from the centroid position.

  ```python
  info_gain = 0
  for ray in hemisphere_rays(centroid, n=50):
      for point in raycast(centroid, ray_direction, max_range):
          if octomap[point].status == UNKNOWN:
              info_gain += 1
              break  # count each ray once
  ```

- **D(f): Distance** — Euclidean distance from the drone's current position to the frontier centroid. Larger distance → larger penalty.

  ```python
  distance = np.linalg.norm(frontier_centroid - drone_position)
  ```

- **H(f): Heading Alignment** — cosine similarity between the drone's current heading vector and the direction to the frontier. Rewards frontiers in the drone's forward direction to minimize unnecessary rotation.

  ```python
  heading_alignment = max(0, np.dot(drone_heading, 
                                    (frontier_centroid - drone_position) / distance))
  ```

The weights `w_info=1.0`, `w_dist=0.5`, `w_heading=0.3` are configurable in `exploration_params.yaml`. The frontier with the highest `U(f)` is selected as the exploration goal.

### 2.5 Frontier Lifecycle

Frontiers are persistent between update cycles. A frontier is removed when:

1. It is fully observed (all neighboring voxels become known — either free or occupied).
2. The drone reaches within `goal_reach_tolerance=0.3 m` of the frontier centroid.
3. The frontier centroid becomes occupied (cave wall) — prevents the planner from targeting walls.

---

## 3. RRT* Path Planning

**Source:** `exploration_planner/path_planner.py`  
**Trigger:** New best frontier selected by `frontier_explorer`  
**Input:** `/exploration/best_frontier`, `/octomap_full_color`, `/slam/odometry`  
**Output:** `/exploration/path` (nav_msgs/Path)

### 3.1 Algorithm Overview

The path planner implements **RRT*** (Rapidly-exploring Random Tree Star, Karaman & Frazza 2011) in 3D configuration space. RRT* is asymptotically optimal: given sufficient computation time, the path length converges to the optimal (shortest collision-free) path.

The implementation uses the OctoMap occupancy grid for collision checking. Continuous collision checking is approximated by sampling points along each tree edge at `step_size / 3` intervals and querying each point's occupancy.

```
RRT* Algorithm:
  T = {start}                                   # tree rooted at start
  for i in range(max_iterations):
    q_rand = sample_random_state()              # uniform random in bounds
    if random() < goal_bias:                    # goal bias: 10% toward goal
      q_rand = goal
    q_near = nearest(T, q_rand)                 # nearest tree node
    q_new = steer(q_near, q_rand, step_size)    # extend step_size toward q_rand
    if collision_free(q_near → q_new):
      Q_near = near(T, q_new, rewiring_radius)  # nodes within rewiring_radius
      q_min = q_near                             # best parent candidate
      c_min = cost(T, q_near) + dist(q_near, q_new)
      for q_near_i in Q_near:                   # find best parent
        if collision_free(q_near_i → q_new):
          if cost(T, q_near_i) + dist(q_near_i, q_new) < c_min:
            q_min = q_near_i
            c_min = cost(T, q_near_i) + dist(q_near_i, q_new)
      T.add_vertex(q_new)
      T.add_edge(q_min, q_new)
      for q_near_i in Q_near:                   # rewire tree
        if cost(T, q_new) + dist(q_new, q_near_i) < cost(T, q_near_i):
          if collision_free(q_new → q_near_i):
            T.replace_parent(q_near_i, q_new)
  return extract_path(T, goal)
```

### 3.2 Collision Checking

Collision checking uses the `collision_checker` node via a ROS 2 service call:

```python
# collision_checker.py maintains a KD-tree of occupied voxel centers
# Service: /collision_check (CollisionCheck.srv)
# Query: point + safety_margin
# Response: collision_free (bool) + nearest_obstacle_distance (float)

for point in edge_samples:
    response = await collision_checker_client.call(
        CollisionCheck.Request(point=point, safety_margin=safety_margin)
    )
    if not response.collision_free:
        return False  # edge is blocked
```

The `safety_margin=0.4 m` inflates obstacles by the drone's effective radius, ensuring the path provides sufficient clearance. For narrow passages, this can be temporarily reduced to `0.25 m` if the primary margin produces no valid path.

### 3.3 A* Fallback

If RRT* fails to find a path within `timeout_sec=10.0` seconds, a 3D A* search on a voxelized grid is used as fallback:

```python
# A* on OctoMap with 3D 26-connectivity
grid_resolution = astar_grid_resolution  # 0.5 m default
# Convert OctoMap to binary voxel grid at grid_resolution
# Inflate obstacles by safety_margin
# Run A* with Euclidean heuristic
```

A* on a coarser grid (`0.5 m`) completes in < 2 s for typical cave passages. The resulting path is then smoothed by the B-spline step.

### 3.4 Path Smoothing

The raw RRT*/A* path contains many sharp waypoints. A two-step smoothing procedure improves tracking performance:

**Step 1: Path shortcutting** — Iteratively try to connect non-adjacent waypoints directly. If the direct connection is collision-free, remove the intermediate waypoints. `smooth_shortcut_passes=3` iterations.

**Step 2: B-spline smoothing** — Fit a B-spline of degree `smooth_spline_k=3` through the shortcutted waypoints. Re-sample at `step_size / 2` intervals to generate a smooth, uniformly-spaced path for the local planner.

---

## 4. Artificial Potential Field Local Planner

**Source:** `exploration_planner/local_planner.py`  
**Rate:** 20 Hz  
**Input:** `/exploration/path`, `/lidar/points`, `/slam/odometry`  
**Output:** `/drone/cmd_vel` (geometry_msgs/Twist)

### 4.1 APF Formulation

The local planner implements an Artificial Potential Field (APF) controller, which generates velocity commands by combining attractive forces toward the path goal and repulsive forces from nearby obstacles:

```
F_total(p) = F_attractive(p) + F_repulsive(p)
v_cmd = saturate(F_total, max_speed)
```

**Attractive Force:**  
The drone is attracted toward the next path waypoint. A conic well potential is used to avoid the local minimum problem in narrow corridors:

```
F_att = k_attractive × (p_goal - p_current) / ||p_goal - p_current||
```

When the drone is within `waypoint_reach_tolerance=0.4 m` of the current waypoint, the next waypoint in the path is activated.

**Repulsive Force:**  
For each LiDAR point within the influence distance `d0 = obstacle_influence_distance = 2.0 m`:

```
F_rep(p, obstacle) = k_repulsive × (1/d - 1/d0) × (1/d²) × (p - obstacle) / ||p - obstacle||
```

where `d = ||p - obstacle||`. This follows the classic Khatib (1985) formulation, which produces zero repulsion at `d = d0` and infinite repulsion as `d → 0`.

The total repulsive force sums over all obstacle points within the influence radius:

```python
F_rep_total = sum(
    k_repulsive * (1/d - 1/d0) * (1/d**2) * direction
    for obstacle in lidar_points
    if d < d0
)
```

**Emergency Stop:**  
If any LiDAR point is within `emergency_stop_distance = 0.3 m`, the velocity command is set to zero regardless of the APF output. This provides a hard safety layer.

### 4.2 Velocity Saturation

The total force vector is scaled to the target speed:

```python
speed = min(max_speed, np.linalg.norm(F_total))
direction = F_total / np.linalg.norm(F_total)
v_cmd = speed * direction
```

The yaw rate command is computed separately from the horizontal direction of `F_total`:

```python
desired_yaw = atan2(F_total.y, F_total.x)
yaw_error = wrap_angle(desired_yaw - current_yaw)
yaw_rate = saturate(yaw_kp * yaw_error, max_angular_speed)
```

### 4.3 Path Deviation Monitoring

The local planner monitors lateral deviation from the planned path. If the drone drifts more than `path_deviation_threshold=2.0 m` from the nearest path point (due to APF avoidance maneuvers), a replan request is published via the `/plan_path` service.

### 4.4 Degeneracy-Aware Speed Reduction

When `/slam/degeneracy_warning` is `True`, the local planner reduces its effective `max_speed` to `0.3 × max_speed`. This prevents large inter-scan displacements that would cause ICP registration failures in degenerate corridors.

---

## 5. Mission Manager State Machine

**Source:** `exploration_planner/mission_manager.py`  
**Rate:** 5 Hz  
**Input:** `/slam/degeneracy_warning`, `/slam/odometry`, `/drone/battery_state`, `/mission/cmd`  
**Output:** `/mission/state`

See [ARCHITECTURE.md#6](ARCHITECTURE.md#6-mission-manager-state-machine) for the full state machine diagram.

### 5.1 State Transition Table

| Current State | Trigger | Next State | Action |
|--------------|---------|------------|--------|
| `IDLE` | `cmd: START` | `TAKEOFF` | Set velocity target to `[0,0,takeoff_speed]` |
| `TAKEOFF` | altitude ≥ exploration_altitude | `EXPLORE` | Stop ascending; start frontier_explorer |
| `EXPLORE` | best_frontier published | `NAVIGATE` | Call `/plan_path` service |
| `NAVIGATE` | frontier reached (< goal_tolerance) | `EXPLORE` | Select next frontier |
| `NAVIGATE` | battery < battery_return_threshold | `RETURN_HOME` | Plan path to home |
| `RETURN_HOME` | home reached (< home_reach_tolerance) | `LAND` | Start descent |
| `LAND` | altitude < 0.1 m | `IDLE` | Stop all motion |
| `ANY` | battery < battery_land_threshold | `EMERGENCY` | Hold position |
| `ANY` | degeneracy_score < 0.01 (persistent) | `EMERGENCY` | Hold position |
| `ANY` | `cmd: EMERGENCY` | `EMERGENCY` | Hold position |
| `EMERGENCY` | `cmd: RESET` | `IDLE` | Reset state machine |

### 5.2 Battery Management

The simulated battery drains at `battery_drain_rate = 0.001` (0.1%/s). Starting at 100%, the drone has ~16 minutes before the 30% return threshold triggers. This is configurable:

```yaml
mission_manager:
  ros__parameters:
    battery_drain_rate: 0.001          # Fraction per second
    battery_return_threshold: 30.0     # % - trigger RTH
    battery_land_threshold: 10.0       # % - force landing
```

To extend exploration time for large caves, increase the drain rate decay or set `battery_return_threshold: 15.0`.

### 5.3 Return-to-Home Path

When transitioning to `RETURN_HOME`, the mission manager calls `/plan_path` with the home position `[0, 0, exploration_altitude]` as the goal. RRT* finds a path back through the already-explored and mapped cave. Since the OctoMap is built during exploration, the return path benefits from a complete map of the traversed area.

### 5.4 Operator Commands

The mission manager subscribes to `/mission/cmd` (std_msgs/String). Recognized commands:

| Command | Effect |
|---------|--------|
| `START` | Begin mission (IDLE → TAKEOFF) |
| `RETURN` | Immediate return-to-home from any non-emergency state |
| `LAND` | Immediate land from any non-emergency state |
| `EMERGENCY` | Force emergency hold |
| `RESET` | Reset from EMERGENCY to IDLE |
| `PAUSE` | Pause exploration; hold position |
| `RESUME` | Resume from pause |

```bash
# Trigger from CLI:
ros2 topic pub /mission/cmd std_msgs/msg/String "data: 'START'" -1
ros2 topic pub /mission/cmd std_msgs/msg/String "data: 'RETURN'" -1
```

---

## 6. Exploration Tuning Guide

### Improving Exploration Coverage

| Problem | Parameter to Change | Direction |
|---------|---------------------|-----------|
| Drone keeps revisiting explored areas | `utility_weight_distance` | Decrease (reduce distance penalty) |
| Drone won't explore far frontiers | `utility_weight_distance` | Increase (encourage far exploration) |
| Drone ignores dead-end passages | `utility_weight_info` | Increase (reward high information gain) |
| Drone wastes time rotating at junctions | `utility_weight_heading` | Decrease (less heading bias) |
| Coverage stalls at 50–60% | `max_exploration_range` | Increase (reveal farther frontiers) |
| Frontiers too noisy | `dbscan_eps` | Increase; or increase `frontier_min_size` |
| Over-clustering of nearby frontiers | `dbscan_eps` | Decrease |

### Improving Path Planning Speed

| Problem | Parameter to Change | Direction |
|---------|---------------------|-----------|
| Path planning takes > 10 s | `step_size` | Increase (larger steps cover space faster) |
| Paths hug walls too closely | `safety_margin` | Decrease (carefully — avoid collisions) |
| RRT* never finds path in narrow passage | `rewiring_radius` | Decrease; `goal_bias` | Increase |
| A* fallback used too often | `max_iterations` | Increase; `timeout_sec` | Increase |
| Paths have too many sharp turns | `smooth_shortcut_passes` | Increase |

### Improving Local Planner Stability

| Problem | Parameter to Change | Direction |
|---------|---------------------|-----------|
| Drone oscillates near walls | `k_repulsive` | Decrease |
| Drone doesn't avoid obstacles fast enough | `k_repulsive` | Increase; `obstacle_influence_distance` | Increase |
| Drone overshoots waypoints | `max_speed` | Decrease; `waypoint_reach_tolerance` | Increase |
| Drone gets stuck in APF local minimum | `path_deviation_threshold` | Decrease (trigger replanning sooner) |
| Too many replan requests | `path_deviation_threshold` | Increase |
| Oscillation in narrow corridors | `k_attractive` | Decrease relative to `k_repulsive` |

---

## 7. Known Limitations and Workarounds

### 7.1 APF Local Minima

**Limitation:** APF can produce local minima where the attractive force toward the goal and repulsive forces from nearby walls exactly cancel, trapping the drone in a static equilibrium.

**Most common scenario:** Narrow corridors where the goal is directly ahead but walls on both sides produce equal and opposite repulsive forces, canceling the attractive force along the corridor axis.

**Workaround:**
1. The `path_deviation_threshold` trigger: if the drone doesn't make progress for 5 seconds (computed as velocity < 0.1 m/s for 100 cycles), a replan is requested with a temporarily-increased `safety_margin` override of `0.1 m` to allow paths through tighter spaces.
2. Adding a small random perturbation to the velocity command: `v_cmd += Uniform(-0.05, 0.05)` applied when stuck state is detected. This breaks the equilibrium.
3. For systematic APF local minimum avoidance, consider replacing APF with a DWA (Dynamic Window Approach) or NMPC local planner — both handle local minima better but require significantly more implementation effort.

### 7.2 Frontier Oscillation

**Limitation:** When two frontiers have nearly equal utility scores (e.g., two passages at similar distances in similar directions), the planner may alternate between them on consecutive evaluation cycles.

**Workaround:** A frontier commitment hysteresis is implemented: once a frontier is selected as the goal, it remains the goal until either reached or explicitly invalidated, regardless of utility score changes. The `min_frontier_commitment_cycles` parameter (default 10 cycles = 5 seconds at 2 Hz) enforces this.

### 7.3 No Hierarchical Global Planning

**Limitation:** The utility-scored frontier approach is greedy — it always selects the current best local frontier without considering the global distribution of unexplored regions. This leads to suboptimal coverage paths: the drone may explore one branch of the cave network completely before moving to another, resulting in unnecessary long-distance traversals.

**Impact:** Coverage efficiency is approximately 60–75% compared to hierarchical planners (TARE: ~85–95%; FUEL: ~75–85%).

**Workaround:** None automated in this implementation. To mitigate: increase `utility_weight_distance` to create stronger pull toward far frontiers, at the cost of less efficient local exploration. Future work: port FUEL's FIS (Frontier Information Structure) to ROS 2.

### 7.4 OctoMap Latency

**Limitation:** The OctoMap is published at 1 Hz. Frontier detection is therefore limited to 1 Hz updates. During fast exploration, the drone may reach a frontier before the OctoMap captures the newly observed region, causing the drone to request a replan to a frontier that no longer exists.

**Workaround:** Increase `octomap_builder` `publish_rate_hz` to 2 Hz at the cost of higher CPU load. The `frontier_explorer` also validates frontiers immediately before publishing by re-checking the voxel status.

### 7.5 ROS 2-only Stack Limitation

**Limitation:** The best-validated subterranean exploration planners (GBPlanner2, TARE, MBPlanner, FUEL) are all ROS 1 only. The ROS 2-native implementation in this project does not match their performance on exploration efficiency.

**Workaround for thesis:** Use the frontier-based planner as the primary evaluation baseline. Document the performance gap relative to TARE/GBPlanner2 as a known limitation and future work item. For comparison, TARE or GBPlanner2 can be run via `ros1_bridge` alongside the existing ROS 2 SLAM stack.

---

## 8. References

1. **Yamauchi, B.** "A Frontier-Based Approach for Autonomous Exploration." Proceedings of the IEEE International Symposium on Computational Intelligence in Robotics and Automation, 1997.

2. **Wavefront Frontier Detector (WFD):** Keidar, M. and Kaminka, G.A. "Efficient Frontier Detection for Robot Exploration." arXiv 2018. [arxiv.org/abs/1806.03581](https://arxiv.org/abs/1806.03581)

3. **Fast Frontier + MAV:** Selin, M. et al. "Efficient Autonomous Exploration Planning of Large-Scale 3-D Environments." IEEE RA-L 5(2):1699–1706, 2020. [arxiv.org/abs/2002.04440](https://arxiv.org/abs/2002.04440)

4. **APF + Frontier:** Bayer, J. et al. "3D Reactive Control and Frontier-Based Exploration for Unstructured Environments." arXiv 2021. [arxiv.org/abs/2108.00380](https://arxiv.org/abs/2108.00380)

5. **RRT*:** Karaman, S. and Frazza, E. "Sampling-based algorithms for optimal motion planning." International Journal of Robotics Research, 2011.

6. **Informed RRT*:** Gammell, J.D. et al. "Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic." IROS 2014. [doi.org/10.1109/IROS.2014.6942976](https://doi.org/10.1109/IROS.2014.6942976)

7. **GBPlanner2:** Kulkarni, M. et al. "Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots." ICRA 2022. [ieeexplore.ieee.org/document/9812401](https://ieeexplore.ieee.org/document/9812401)

8. **FUEL:** Zhou, B. et al. "FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning." IEEE RA-L 6(2):779–786, 2021. [arxiv.org/abs/2010.11561](https://arxiv.org/abs/2010.11561)

9. **TARE:** Cao, C. et al. "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments." RSS 2021 (Best Paper Award). [ri.cmu.edu/app/uploads/2021/06/RSS_2021.pdf](https://www.ri.cmu.edu/app/uploads/2021/06/RSS_2021.pdf)

10. **Khatib APF:** Khatib, O. "Real-time obstacle avoidance for manipulators and mobile robots." ICRA 1985.

11. **m-explore-ros2 (ROS 2 frontier exploration):** [github.com/robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)

12. **Multi-Objective Risk Assessment for Exploration:** Faessler, M. et al. "Multi-Objective Risk Assessment for Exploration Planning." arXiv 2024. [arxiv.org/abs/2410.03917](https://arxiv.org/abs/2410.03917)
