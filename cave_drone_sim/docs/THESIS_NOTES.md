# Thesis Research Notes and Context

**Document type:** Thesis research companion  
**Audience:** Author, thesis advisor  
**Note:** This document is not intended for public distribution. It synthesizes research context, frames contributions, and records evaluation design decisions.  
**Cross-references:** [SLAM_GUIDE.md](SLAM_GUIDE.md) | [EXPLORATION_GUIDE.md](EXPLORATION_GUIDE.md) | [ARCHITECTURE.md](ARCHITECTURE.md)

---

## Table of Contents

1. [Research Context and Motivation](#1-research-context-and-motivation)
2. [Literature Review Summary](#2-literature-review-summary)
3. [Relation to DARPA SubT Challenge](#3-relation-to-darpa-subt-challenge)
4. [Novel Contributions vs. Existing Work](#4-novel-contributions-vs-existing-work)
5. [Evaluation Methodology](#5-evaluation-methodology)
6. [Suggested Thesis Chapter Outline](#6-suggested-thesis-chapter-outline)
7. [Key References Organized by Topic](#7-key-references-organized-by-topic)

---

## 1. Research Context and Motivation

Autonomous robotic exploration of GPS-denied underground environments is an open problem with significant practical applications: search and rescue after mine collapses, geological survey, planetary lava tube exploration, and infrastructure inspection. The DARPA Subterranean Challenge (2018–2021), an $82M DARPA program specifically targeting this problem, demonstrated both the feasibility and the remaining gaps in autonomous underground navigation.

The challenge problem combines several intersecting hard subproblems:

1. **3D LiDAR SLAM without GPS** — All position estimation must be derived from sensor observations. Caves present particularly adversarial conditions: featureless walls cause scan-matching degeneracy; repetitive tunnel geometry causes loop closure confusion; dust and vibration degrade sensor data.

2. **Autonomous exploration without prior maps** — The robot must simultaneously build a map and use it to make navigation decisions (SLAM + exploration). The optimal exploration strategy in a graph-like underground network is computationally hard (related to the Traveling Salesman Problem).

3. **Resource-constrained operation** — Aerial robots carry strict payload and power budgets. SLAM and planning algorithms must run onboard within tight computational envelopes.

This thesis contributes a complete simulation platform for evaluating these algorithms — CaveDroneSim — and uses it to systematically benchmark SLAM backends and exploration strategies across a controlled set of procedurally-generated cave environments.

---

## 2. Literature Review Summary

### 2.1 Simulation Platforms

**Primary recommendation: Gazebo Harmonic + ROS 2 Humble.**

The comprehensive survey ([research/sim_platform_comparison.md](../research/sim_platform_comparison.md)) evaluated Gazebo Harmonic, NVIDIA Isaac Sim, and Unity against six criteria weighted by thesis relevance. Gazebo Harmonic scored 4.60/5 versus Isaac Sim 3.72/5 and Unity 2.87/5. The decisive factors:

- **SubT heritage:** The DARPA SubT Challenge was run on Ignition Gazebo (Gazebo Harmonic's predecessor). All open-source cave assets, validated SLAM configurations, and robot models are native to this ecosystem.
- **PX4 SITL integration:** Gazebo Harmonic is PX4's official primary SITL target. The `x500` quadrotor model is directly usable.
- **Reproducibility:** Fully open-source, no GPU tier requirement (though GPU helps). Thesis reviewers can reproduce results without NVIDIA RTX hardware ownership.

**Key source:** Open Robotics, "SubT Part 3: The Simulator" (2022). URL: https://www.openrobotics.org/blog/2022/2/3/subt-part-3-the-simulator

### 2.2 Cave Generation

Six procedural generation paradigms were surveyed ([research/cave_generation_research.md](../research/cave_generation_research.md)):

| Method | SLAM Suitability | Notes |
|--------|-----------------|-------|
| Graph-based (Prim's MST) | ★★★★★ | Direct topology control; used in this thesis |
| Space Colonization | ★★★★☆ | Most organic geometry; less controllable |
| SDF + Marching Cubes | ★★★☆☆ | Good surface detail; poor topology control alone |
| Spline Sweep | ★★★★★ | Clean passages; used as geometry step in this thesis |
| Cellular Automata | ★★☆☆☆ | 2D useful; 3D limited — used only for wall texturing |
| Wave Function Collapse | ★★★☆☆ | Best for mine-like structured environments |

The hybrid Graph + Spline + SDF + Marching Cubes pipeline used in CaveDroneSim is the closest implementation to PLUME (Gabry, arXiv 2025), which was validated with Gazebo + RTAB-Map LiDAR SLAM.

**Key gap vs. PLUME:** PLUME uses Blender as the mesh generation backend, while CaveDroneSim uses Python/scikit-image. The advantage is that CaveDroneSim is fully scriptable and parallelizable (no GUI required), at the cost of some geometric complexity.

### 2.3 SLAM for Cave Environments

The comprehensive SLAM survey ([research/slam_comparison.md](../research/slam_comparison.md)) compared 7 SLAM systems:

**Key finding:** The recommended architecture is **FAST-LIO2 (front-end) + RTAB-Map (back-end)**:

- FAST-LIO2 uses direct scan-to-map registration without feature extraction, performing better in featureless environments than LOAM-based systems. Achieves 0.11 m ATE underground (LG-SLAM benchmark).
- RTAB-Map is the only fully ROS 2-native system with loop closure + OctoMap + Nav2 compatibility. Used by DLR for the SCOUT cave rover (i-SAIRAS 2024).
- Neither FAST-LIO2 alone nor RTAB-Map alone is sufficient: FAST-LIO2 drifts without loop closure; RTAB-Map's loop closure in LiDAR-only mode is weaker than FAST-LIO2's odometry accuracy.

**Critical lessons from DARPA SubT:**
- LiDAR is the primary sensor for all top teams. Visual/thermal is backup.
- IMU is mandatory on UAV platforms for de-skewing and motion bridging.
- Degeneracy detection (eigenvalue analysis) is a critical safety component.
- CERBERUS won by using multi-modal sensor fusion (LiDAR + IMU + visual + thermal) so that single-sensor failure was always compensated.

### 2.4 Exploration Planning

The exploration survey ([research/exploration_planning_research.md](../research/exploration_planning_research.md)) identified three tiers:

**Tier 1 (state of the art, ROS 1 only):** GBPlanner2 (CERBERUS team), TARE (CMU, RSS 2021 Best Paper), MBPlanner (NTNU, SubT). These are purpose-designed for subterranean UAV exploration with validated SubT performance. All require `ros1_bridge` for ROS 2 use.

**Tier 2 (good performance, ROS 2 available):** FUEL (HKUST) — 3–8× faster than state-of-the-art at publication. No official ROS 2 port but community effort ongoing.

**Tier 3 (baseline, ROS 2 native):** Frontier-based with utility scoring (`m-explore-ros2`, or custom as in this thesis). Simpler but lower coverage efficiency.

**Thesis choice:** Utility-scored frontier exploration (Tier 3) implemented natively in ROS 2. This is explicitly framed as a baseline, with the performance gap to Tier 1 documented as a finding and future work item.

---

## 3. Relation to DARPA SubT Challenge

The DARPA Subterranean Challenge (2018–2021) is the most directly relevant prior work for this thesis. Key parallels:

| SubT Challenge | This Thesis |
|----------------|-------------|
| GPS-denied cave environments | GPS-denied procedurally-generated caves (Gazebo) |
| LiDAR-primary sensing | Ouster OS0-32 LiDAR simulation |
| Aerial + ground robots | Aerial drone only |
| Competition-validated SLAM (CERBERUS: CompSLAM + M3RM) | FAST-LIO2 + RTAB-Map |
| Graph-based exploration (GBPlanner2) | Frontier-based exploration (baseline) |
| OctoMap 3D occupancy | OctoMap via `octomap_builder` |
| Real underground environments | Procedurally generated (systematic evaluation) |

**Advantages of this thesis over SubT:**
1. **Controlled experiments:** Procedural generation allows systematic variation of cave parameters (complexity, size, surface roughness, loop count) with known ground truth. SubT competitions used fixed, undisclosed environments.
2. **Automated evaluation:** Ground truth pose from Gazebo enables automated ATE/RPE computation without mocap or external tracking. SubT teams had to use manually placed artifact positions as ground truth.
3. **Reproducibility:** Any researcher can regenerate the exact same cave environment with the same seed. SubT cave worlds are publicly available but not systematically varied.

**Limitations relative to SubT:**
1. Simulated LiDAR (Gaussian noise) vs. real-world LiDAR artifacts (retroreflective materials, dust, water droplets).
2. No multi-robot coordination.
3. No communication relay deployment.
4. Procedurally generated caves may not capture all geometric features of real caves (speleothems, scalloped walls, phreatic tube cross-sections).

---

## 4. Novel Contributions vs. Existing Work

### What Is Novel

1. **Automated parametric cave generation pipeline for SLAM benchmarking:** CaveDroneSim provides the first fully-scriptable, seed-deterministic 3D cave generation pipeline that outputs directly to Gazebo SDF format with configurable topology, surface roughness, and obstacle density. Previous tools (PLUME, SubT tile system) either require Blender GUI or use pre-made static tiles.

2. **Systematic comparison of SLAM backends across diverse procedural caves:** Prior SLAM evaluations use fixed benchmark datasets (KITTI, EuRoC, Newer College) or fixed SubT competition worlds. This thesis enables evaluation across a factorial design: N seeds × M surface roughness levels × K SLAM backends — providing statistical power not possible with a single dataset.

3. **Integrated ROS 2-native SLAM + exploration + mission management pipeline:** While individual components (FAST-LIO2, RTAB-Map, frontier exploration) are prior work, their integration in a unified, ROS 2-native, fully-documented simulation platform specifically for cave UAVs is new. Existing simulation setups either use ROS 1 or lack documentation.

4. **Degeneracy-aware exploration:** The integration of eigenvalue-based degeneracy detection with exploration speed reduction and mission state management is novel at the system integration level. Individual components (degeneracy detection: Sun et al. 2024) are prior work.

### What Is NOT Novel

- The cave generation algorithms themselves (Prim's MST, marching cubes, Simplex noise) are standard.
- FAST-LIO2, RTAB-Map, frontier exploration are established prior work.
- Gazebo + ROS 2 + PX4 SITL as a drone simulation platform is well-established.
- OctoMap-based mapping is standard in the field.

**Thesis framing:** The contribution is at the **system integration and experimental evaluation** level, not at the algorithm invention level. This is appropriate for a master's thesis in the simulation/evaluation track.

---

## 5. Evaluation Methodology

### 5.1 Metrics

| Metric | Notation | Tool | What It Measures |
|--------|----------|------|-----------------|
| Absolute Trajectory Error (RMSE) | ATE | `evo_ape` | Global SLAM accuracy; loop closure effectiveness |
| Relative Pose Error (translational, per meter) | RPE | `evo_rpe` | Local odometry drift rate |
| Exploration coverage percentage | Coverage% | Custom script | Fraction of cave volume explored |
| Map accuracy (Chamfer distance) | CD | Open3D | Geometric accuracy of the 3D map |
| Computation time per scan | T_scan | ROS 2 timing | Real-time viability |
| Frontier selection efficiency | FSE | Custom script | Mean distance to selected frontier / mean distance to nearest frontier (ideally 1.0) |
| Mission duration | T_mission | ROS 2 bag | Total time from takeoff to 80% coverage |
| Degeneracy event count | N_degen | Custom script | Number of degeneracy warnings per mission |

### 5.2 Experimental Design

**Primary experiment (SLAM comparison):**

```
Design: N_caves × M_backends × K_repetitions
  N_caves = 5 (seeds: 0, 42, 99, 7, 123)
  M_backends = 3 (Simple ICP, FAST-LIO2, FAST-LIO2 + RTAB-Map)
  K_repetitions = 3 (for statistical significance)

Total runs: 5 × 3 × 3 = 45

Metrics: ATE (primary), RPE, T_scan, N_degen

Controlled variables:
  - Cave: seed=42, num_nodes=20, bounds_x=y=80m, bounds_z=25m
  - Drone trajectory: deterministic (same seed → same frontier sequence)
  - SLAM parameters: default values from config files
```

**Secondary experiment (cave complexity):**

```
Design: N_complexity × M_backends
  N_complexity = 5 (num_nodes: 8, 12, 20, 30, 40)
  M_backends = 3

Metric: ATE vs. num_nodes (does SLAM accuracy degrade with cave complexity?)
```

**Tertiary experiment (surface roughness):**

```
Design: N_roughness × M_backends
  N_roughness = 5 (noise_amplitude: 0.0, 0.1, 0.25, 0.4, 0.6)
  M_backends = 3

Metric: ATE, RPE, N_degen vs. noise_amplitude
  (tests degeneracy detector calibration across roughness levels)
```

**Exploration experiment:**

```
Design: N_caves × K_repetitions
  N_caves = 5, K_repetitions = 3

Metrics: Coverage%, Mission duration, FSE
  (baseline: utility-weighted frontier)
  (comparison: random frontier selection as ablation)
```

### 5.3 Running the Experiments

```bash
# Primary experiment: all 45 SLAM runs
python3 scripts/run_slam_experiment.py \
    --seeds 0 42 99 7 123 \
    --backends simple fastlio rtabmap \
    --repetitions 3 \
    --output_dir results/slam_comparison/

# Analysis and plotting
python3 scripts/analyze_results.py \
    --results_dir results/slam_comparison/ \
    --output_dir results/figures/
```

### 5.4 Statistical Analysis

With K=3 repetitions, use:
- **Median** rather than mean for ATE/RPE (robust to outliers from SLAM divergence events).
- **Interquartile range (IQR)** as variability measure.
- **Wilcoxon signed-rank test** for pairwise backend comparison (non-parametric; appropriate for small N).
- Report results as: `median ± IQR (m)`.

---

## 6. Suggested Thesis Chapter Outline

```
Chapter 1: Introduction
  1.1 Problem statement: autonomous exploration of GPS-denied cave environments
  1.2 Motivation: search and rescue, geological survey, planetary exploration
  1.3 DARPA SubT Challenge as primary benchmark context
  1.4 Research questions
      RQ1: How does SLAM accuracy (ATE, RPE) vary across cave complexity and 
           surface roughness in simulation?
      RQ2: Does the FAST-LIO2 + RTAB-Map combination provide statistically
           significantly better ATE than FAST-LIO2 alone in cave environments?
      RQ3: How does utility-weighted frontier selection compare to random 
           frontier selection in exploration coverage efficiency?
  1.5 Thesis contributions (as listed in §4 of this document)
  1.6 Thesis structure

Chapter 2: Background and Related Work
  2.1 LiDAR SLAM principles (ICP, point-to-plane, iEKF)
  2.2 SLAM systems comparison (FAST-LIO2, LIO-SAM, KISS-ICP, RTAB-Map)
  2.3 Exploration planning (frontier-based, GBPlanner2, TARE, FUEL)
  2.4 DARPA SubT Challenge — teams, methods, outcomes
  2.5 Simulation platforms (Gazebo, Isaac Sim comparison)
  2.6 Procedural cave generation (survey of six methods)

Chapter 3: System Design and Implementation
  3.1 System architecture overview
  3.2 Simulation environment: Gazebo Harmonic + PX4 SITL
  3.3 Cave generation pipeline (graph → SDF → marching cubes)
  3.4 SLAM pipeline: Simple ICP, FAST-LIO2, RTAB-Map configurations
  3.5 Degeneracy detection and handling
  3.6 Exploration and navigation (frontier explorer, RRT*, APF)
  3.7 Mission management FSM
  3.8 Evaluation infrastructure (ground truth extraction, evo)

Chapter 4: Experimental Results
  4.1 Experiment setup (hardware, software versions, random seeds)
  4.2 SLAM evaluation across cave configurations (primary experiment)
      4.2.1 ATE results: backends × cave seeds
      4.2.2 RPE results: backends × cave seeds
      4.2.3 Degeneracy events vs. surface roughness
      4.2.4 Computation timing analysis
  4.3 Cave complexity analysis (secondary experiment)
  4.4 Surface roughness analysis (tertiary experiment)
  4.5 Exploration coverage results

Chapter 5: Discussion
  5.1 SLAM accuracy findings: why does FAST-LIO2 + RTAB-Map outperform?
  5.2 Degeneracy patterns: where and when does SLAM fail in caves?
  5.3 Exploration efficiency: limitations of greedy frontier selection
  5.4 Comparison to SubT ground truth results
  5.5 Threats to validity

Chapter 6: Conclusion and Future Work
  6.1 Summary of contributions
  6.2 Answers to research questions
  6.3 Future work
      - GBPlanner2 or TARE port to ROS 2
      - FAST-LIVO2 (LiDAR + Visual + IMU) for degeneracy recovery
      - Multi-drone exploration
      - Sim-to-real transfer with real Ouster LiDAR
      - Scan Context loop closure integration in RTAB-Map

Appendix A: CaveDroneSim Package Documentation
Appendix B: Full Results Tables
Appendix C: Cave Parameter Configurations
```

---

## 7. Key References Organized by Topic

### Simulation Platforms

1. Open Robotics. "DARPA SubT Final Competition." 2021. https://www.openrobotics.org/blog/2021/9/27/darpa-subt-final-competition
2. Open Robotics. "SubT Part 3: The Simulator." 2022. https://www.openrobotics.org/blog/2022/2/3/subt-part-3-the-simulator
3. Müller, M. et al. "Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation." ICUAS 2023. arXiv:2307.05263. https://arxiv.org/abs/2307.05263
4. PX4 Simulation Documentation. https://docs.px4.io/main/en/simulation/
5. McGuire Robotics. "PX4 Simulation Survey." 2025. https://www.mcguirerobotics.com/px4_sim_research_report/

### Cave Generation

6. Gabry, G. "PLUME: Procedural Level Unified Mesh Environment." arXiv 2025. https://github.com/Gabryss/P.L.U.M.E
7. Mark, J. et al. "Procedural Generation of 3D Caves for Games on the GPU." FDG 2015. http://www.fdg2015.org/papers/fdg2015_paper_80.pdf
8. Koval, A. et al. "A Subterranean Virtual Cave World for Gazebo based on the DARPA SubT Challenge." arXiv 2020. https://arxiv.org/abs/2004.08452
9. Cui, H. et al. "Procedural Generation of 3D Cave Models with Stalactites and Stalagmites." IJCSNS 2011. http://paper.ijcsns.org/07_book/201108/20110812.pdf
10. Villar López, F., Chover, M. "Procedural Generation of 3D Maps with Wave Function Collapse." Eurographics 2025. https://diglib.eg.org/bitstream/handle/10.2312/ceig20251107/ceig20251107.pdf

### SLAM — Core Papers

11. Xu, W. et al. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry." IEEE Transactions on Robotics 38(4):2053–2073, 2022. https://github.com/hku-mars/FAST_LIO
12. Shan, T. et al. "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping." IROS 2020. https://github.com/TixiaoShan/LIO-SAM
13. Vizzo, I. et al. "KISS-ICP: In Defense of Point-to-Point ICP — Simple, Accurate, and Robust Registration if Done the Right Way." IEEE RA-L 8(2):1029–1036, 2023. https://github.com/PRBonn/kiss-icp
14. Labbé, M., Michaud, F. "RTAB-Map as an Open-Source Lidar and Visual Simultaneous Localization and Mapping Library for Large-Scale and Long-Term Online Operation." JFR 36(2):416–446, 2019. https://github.com/introlab/rtabmap_ros
15. Zheng, C. et al. "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry." arXiv 2408.14035, 2024. https://arxiv.org/abs/2408.14035

### SLAM — Underground / Degeneracy

16. Ohradzansky, M. et al. "Present and Future of SLAM in Extreme Underground Environments." IEEE T-RO, 2023. https://arxiv.org/abs/2208.01787
17. Sun, Z. et al. "A Real-time Degeneracy Sensing and Compensation Method for Inertial Aided LiDAR SLAM." arXiv 2412.07513, 2024. https://arxiv.org/abs/2412.07513
18. Koch, J.F. "SLAM for SCOUT: A ROS2-Based Multi-Sensor SLAM System for a Cave Rover." i-SAIRAS 2024. https://elib.dlr.de/210182/
19. Garcia-Espinosa, R. et al. "LG-SLAM: A Versatile and Robust Framework for Range-Inertial SLAM." arXiv 2407.14797, 2024. https://arxiv.org/abs/2407.14797

### Mapping Representations

20. Hornung, A. et al. "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees." AURO 34(3):189–206, 2013.
21. Oleynikova, H. et al. "Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning." IROS 2017. https://helenol.github.io/publications/iros_2017_voxblox.pdf

### Exploration Planning

22. Yamauchi, B. "A Frontier-Based Approach for Autonomous Exploration." CIRA 1997.
23. Zhou, B. et al. "FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning." IEEE RA-L 6(2):779–786, 2021. https://arxiv.org/abs/2010.11561
24. Cao, C. et al. "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments." RSS 2021 (Best Paper). https://www.ri.cmu.edu/app/uploads/2021/06/RSS_2021.pdf
25. Dang, T. et al. "Graph-based Subterranean Exploration Path Planning using Aerial and Legged Robots." JFR 37(8):1481–1509, 2020. https://onlinelibrary.wiley.com/doi/10.1002/rob.21993
26. Kulkarni, M. et al. "Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots." ICRA 2022. https://ieeexplore.ieee.org/document/9812401
27. Dharmadhikari, M. et al. "Motion Primitives-based Path Planning for Fast and Agile Exploration using Aerial Robots." ICRA 2020. https://ieeexplore.ieee.org/document/9196964/
28. Karaman, S., Frazza, E. "Sampling-based algorithms for optimal motion planning." IJRR 30(7):846–894, 2011.

### Path Planning and Control

29. Khatib, O. "Real-time obstacle avoidance for manipulators and mobile robots." ICRA 1985.
30. Gammell, J.D. et al. "Informed RRT*: Optimal Sampling-based Path Planning." IROS 2014. https://doi.org/10.1109/IROS.2014.6942976

### DARPA SubT Teams

31. Tranzatto, M. et al. "CERBERUS: Autonomous Legged and Aerial Robotic Exploration in the DARPA SubT Challenge." arXiv 2201.07067. https://arxiv.org/abs/2201.07067
32. DARPA. "Team CERBERUS and Team Dynamo Win DARPA SubT Challenge." 2021. https://www.darpa.mil/news/2021/subterranean-challenge-winners
33. Cao, C. et al. "Exploring the Most Sectors at the DARPA SubT Challenge Finals." Field Robotics, 2023. https://ieeexplore.ieee.org/document/10882594/

### Evaluation Tools

34. Grupp, M. "evo: A Python package for the evaluation of odometry and SLAM." 2017. https://michaelgrupp.github.io/evo/
