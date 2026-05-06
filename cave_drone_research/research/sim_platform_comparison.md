# Simulation Platform Comparison for Autonomous Drone LiDAR SLAM in GPS-Denied Caves
**Thesis Research Brief | ROS 2 Integration | April 2026**

---

## Executive Summary

For a thesis project involving an autonomous drone navigating procedurally-generated caves with LiDAR SLAM in a GPS-denied environment using ROS 2, the research clearly points to **Gazebo Harmonic + ROS 2 as the primary recommendation**, with Isaac Sim as a high-value secondary option if an RTX GPU is available.

The DARPA Subterranean (SubT) Challenge — the single most relevant precedent for underground cave drone SLAM research — was run exclusively on **Ignition Gazebo** (the predecessor to Gazebo Harmonic). Every major academic and DARPA-funded team building LiDAR SLAM for caves used the Gazebo/Ignition ecosystem with ROS. The SubT virtual testbed, cave tile libraries, open-source drone models, and community-developed SLAM packages are all directly reusable in a Gazebo Harmonic + ROS 2 project.

---

## Table of Contents

1. [Platform-by-Platform Evaluation](#platform-evaluation)
2. [Comparative Scoring Matrix](#scoring-matrix)
3. [DARPA SubT Challenge Analysis](#darpa-subt)
4. [Open-Source Cave Simulation Resources](#open-source-resources)
5. [PX4 SITL vs ArduPilot SITL Compatibility](#autopilot-comparison)
6. [Recommendation with Justification](#recommendation)
7. [Recommended Stack and Setup Path](#setup-path)

---

## 1. Platform-by-Platform Evaluation <a name="platform-evaluation"></a>

### 1.1 Gazebo Harmonic + ROS 2

**Overview:** Gazebo Harmonic (gz-sim 8.x) is the current LTS release of the Ignition/Gazebo lineage, maintained by Open Robotics (now part of Intrinsic). It is the official default simulator for both PX4 and ArduPilot SITL. Gazebo Harmonic with ROS 2 Jazzy is the canonical combination as of 2025–2026.

#### LiDAR Sensor Simulation Quality
- **Built-in GPU LiDAR (`gpu_lidar`)**: Supports configurable ray counts, FOV, Gaussian noise models (mean + standard deviation), update rate, and range limits. Produces `sensor_msgs/PointCloud2` and `sensor_msgs/LaserScan`. Noise modeling is simple Gaussian but is adequate for most SLAM evaluation.
- **RobotecAI RGL Gazebo Plugin**: An open-source NVIDIA OptiX/CUDA-accelerated LiDAR plugin that delivers **~4–4.5× faster** ray casting than the built-in `gpu_lidar`. It supports hardware ray-traced reflections (using RTX cores), realistic LiDAR presets (Velodyne VLP-16, VLP-32, Ouster OS1/OS2, Hesai, etc.), and custom firing pattern files. For complex cave mesh geometries, RGL's ray-traced approach avoids the rasterization artifacts that plague software renderers in 360° scanning scenarios. A GSoC 2025 project additionally produced `gz-wgpu-rt-lidar`, a vendor-agnostic (NVIDIA/AMD/Intel) ray-traced LiDAR plugin maintaining near-constant render time even as scene vertex count scales into the millions — directly relevant to large cave meshes.
  - *Sources: [RGLGazeboPlugin GitHub](https://github.com/RobotecAI/RGLGazeboPlugin), [Robotec.AI blog](https://www.robotec.ai/blog/robotics/automated-future-with-rgl-robotec-gpu-lidar), [GSoC 2025 discourse](https://discourse.openrobotics.org/t/gsoc-2025-ray-tracing-enabled-faster-than-real-time-gpu-based-lidar-plugin-for-gazebo/50714)*

#### Depth Camera Simulation
- **Built-in RGBD (`camera` sensor with depth)**: Gazebo ships with a depth camera sensor plugin that outputs `sensor_msgs/Image` (depth) and `sensor_msgs/PointCloud2`. Frame conventions require manual TF correction.
- Fully documented workflow for ROS 2 topic bridging via `ros_gz_bridge`. Supports configurable noise, FOV, clipping, and update rate.
- Usable directly with stereo/RGBD-based SLAM packages such as RTAB-Map and OpenVINS.

#### Drone Physics Fidelity
- **PX4 SITL**: The official PX4 documentation lists Gazebo (Harmonic) as the primary SITL target. Communication uses `gz_bridge` (Gazebo transport), not MAVLink — directly improving timing accuracy. Lockstep synchronization is supported. The `x500` quadrotor model is actively maintained. Integration via Micro XRCE-DDS to ROS 2 is well-documented and battle-tested.
- **ArduPilot SITL**: Fully supported via the `ardupilot_gz` plugin, which targets Gazebo Harmonic as the recommended version. The `iris_maze` example ships with a 360° LiDAR-equipped drone in a maze world, which is directly relevant to cave navigation. ArduPilot's gazebo plugin communicates over a JSON/TCP interface.
- **Multirotor plugin**: The built-in `MulticopterMotorModel` plugin simulates rotor aerodynamics including blade flapping, drag, and ground effect. Physics are adequate for indoor/cave navigation algorithm validation, though not as high-fidelity as a JSBSim blade-element model.
  - *Sources: [PX4 Simulation Docs](https://docs.px4.io/main/en/simulation/), [ArduPilot Gazebo Docs](https://ardupilot.org/dev/docs/sitl-with-gazebo.html), [PX4 Survey — McGuire Robotics](https://www.mcguirerobotics.com/px4_sim_research_report/)*

#### Performance with Large/Complex Meshes
- **Concern**: Standard `gpu_lidar` in Gazebo Harmonic uses rasterization and can exhibit significant slow-down with high-polygon cave meshes, particularly for 360° LiDAR sweeps.
- **Mitigation**: The RGL plugin uses CUDA/OptiX ray-tracing with BVH acceleration structures, meaning render time scales much more favorably with mesh complexity. The `gz-wgpu-rt-lidar` plugin shows near-flat render time scaling for millions of vertices.
- SubT cave worlds were built from ~73 tiled meshes ranging from 1.5–5.2 km in length. These ran on cloud infrastructure with Gazebo Ignition/Harmonic without requiring specialized hardware per node.
- Procedural cave generation can be done by tiling SDF meshes — the same approach used in SubT. Tools exist to convert Blender/photogrammetry point clouds to Gazebo SDF.
  - *Sources: [Open Robotics SubT Part 3](https://www.openrobotics.org/blog/2022/2/3/subt-part-3-the-simulator), [SubT Gazebo Community Meeting 2022](https://www.youtube.com/watch?v=V-QcACKjZlM)*

#### Ease of Importing Custom Mesh Environments
- Environments defined in SDF (Simulation Description Format) — XML-based, well-documented.
- Meshes can be imported as Collada (`.dae`), OBJ, or STL. Blender → SDF exporter exists.
- SubT community developed detailed guides for optimizing cave mesh tiles for Gazebo physics.
- **Procedural generation**: Tile-based world generation scripts can compose SDF files programmatically from a tile library. This is the exact methodology used for SubT caves.

#### ROS 2 Integration Maturity
- **Best-in-class.** `ros_gz_bridge` is actively maintained and provides bidirectional topic bridging for all Gazebo sensor types. `ros_gz_image`, `ros_gz_sim`, and `ros_gz_interfaces` are available in binary packages for ROS 2 Humble and Jazzy.
- PX4's `px4_msgs` and Micro XRCE-DDS agent provide native ROS 2 integration without MAVLink.
- All standard robotics packages (Nav2, SLAM Toolbox, RTAB-Map, LIO-SAM, FAST-LIO) have been validated in this stack.
- **Known issues**: Some users report ROS 2 bridge reliability issues (sensor data visible in Gazebo not reaching ROS 2 topics), and `std::bad_alloc` errors when mixing Foxglove bridge with the PX4 bridge. These are documented community issues with known workarounds.
  - *Sources: [PX4 Survey](https://www.mcguirerobotics.com/px4_sim_research_report/), [Open Robotics Discourse (Jazzy)](https://discourse.openrobotics.org/t/gsoc-2024-migration-of-project-dave-to-ros-2-and-harmonic-launch-files-robot-models-and-sensor-plugins/39321)*

#### Community Support for Drone SLAM
- **Largest community by far.** The SubT challenge produced hundreds of open-source repos targeting Gazebo + ROS for underground exploration.
- Active GitHub repositories: `sjtu_drone` (ROS 2 quadcopter), `px4_sim_ros2`, SubT tech repo (`osrf/subt`), LTU-RAI cave world.
- FAST-LIO2, LIO-SAM, LOAM, KISS-ICP, and RTAB-Map all have tested configurations for Gazebo + ROS 2.
- Academic papers heavily favor Gazebo as the validation environment.

#### GPU Requirements and System Requirements
- **Minimum**: Modern CPU (8+ cores), 16 GB RAM, NVIDIA GPU with CUDA for `gpu_lidar` / RGL. 
- **Without GPU**: Software rendering works for visualization but LiDAR performance is degraded. The `cpu_lidar` plugin is slow for dense point clouds.
- **With RGL**: CUDA-enabled GPU required; RTX cores improve performance but are not mandatory.
- **OS**: Ubuntu 22.04 or 24.04 (primary support); binary packages available.
- **Cost**: Fully open-source, free.

---

### 1.2 NVIDIA Isaac Sim + ROS 2

**Overview:** Isaac Sim (currently v4.5/5.x as of 2025–2026) is built on NVIDIA Omniverse and uses PhysX 5.0 for physics, RTX rendering for photorealistic sensors, and USD (Universal Scene Description) as its scene format. The **Pegasus Simulator** (open-source, BSD-3 licensed) is the key extension for multirotor drone simulation with PX4 integration.

#### LiDAR Sensor Simulation Quality
- **RTX LiDAR sensors**: Isaac Sim uses a camera-based rendering pipeline internally for LiDAR — each LiDAR is actually rendered via ray tracing through the RTX pipeline. This gives high visual fidelity (material reflectance, return intensity) but requires an RT Core-capable GPU.
- Supports both solid-state and rotating LiDAR configurations. Many real-world LiDAR presets are available (Hesai XT32, VLP-16, etc.).
- Outputs `LaserScan` and `PointCloud2` via OmniGraph action graphs to ROS 2 topics.
- Noise models: Gaussian noise is configurable, though not as straightforward to tune as in Gazebo's XML-based config.
- **Key limitation**: Each RTX LiDAR must be attached to its own viewport. Docking/undocking the UI while LiDAR simulation runs can crash the application. Multiple simultaneous LiDARs require careful setup.
  - *Sources: [Isaac Sim RTX Lidar Docs](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)*

#### Depth Camera Simulation
- **Photorealistic RGBD**: Isaac Sim's depth camera produces photorealistic RGB + depth images via RTX path tracing. This is a major advantage for vision-based SLAM (ORB-SLAM3, OpenVINS) where domain randomization for sim-to-real transfer matters.
- Outputs `sensor_msgs/Image` and `sensor_msgs/CameraInfo` over ROS 2.
- Isaac ROS Visual SLAM (`isaac_ros_visual_slam`) is a GPU-accelerated ROS 2 VSLAM package that integrates with this camera pipeline, enabling real-time VSLAM on NVIDIA hardware.
  - *Sources: [Isaac ROS VSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html), [GPS-Denied VSLAM project](https://www.andrewbernas.com/docs/projects/robots/vslam)*

#### Drone Physics Fidelity
- **Pegasus Simulator** (v4.5.1 for Isaac Sim 4.5.0, October 2025) provides multirotor dynamics with:
  - Quadratic thrust curve (T = k·ω²), reactive torque modeling
  - Full IMU, barometer, magnetometer, GPS simulation with configurable noise
  - MAVLink communication with PX4 SITL via lockstep synchronization
  - ROS 2 Humble integration with MAVROS and Micro XRCE-DDS bridge
  - Multi-vehicle support (5+ drones tested in swarm configurations)
- **ArduPilot**: Experimental ArduPilot interface exists in Pegasus but was not tested in v4.5.0 release (October 2025).
- Physics quality is comparable to Gazebo for multirotor dynamics; the bigger advantage is rendering quality for camera-based sensors.
  - *Sources: [Pegasus Simulator GitHub](https://github.com/PegasusSimulator/PegasusSimulator), [VividCloud Isaac Sim PX4](https://www.vividcloud.com/hexacopter-flight-simulation-with-px4-integration-in-nvidia-isaac-sim/), [Pegasus arXiv paper](https://arxiv.org/html/2307.05263v2)*

#### Performance with Large/Complex Meshes
- **Advantage over Gazebo for large scenes**: Isaac Sim is built on Omniverse/USD and is architected for large-scale scenes (autonomous vehicle testing, warehouse robotics). It handles multi-kilometer environments without the polygon-count performance cliff that Gazebo exhibits.
- Scene load times are significant (57–575 seconds for a full warehouse scene depending on platform). Once loaded, FPS is 85–120+ for typical scenes with RTX hardware.
- **Memory**: Large meshes with many materials increase VRAM consumption significantly. A complex cave world could require 12–16+ GB VRAM.
- Procedural mesh generation can be done in Python via USD APIs (`UsdGeom.Mesh`), and meshes can be imported from OBJ, FBX, STL, or glTF via the Mesh Importer.
  - *Sources: [Isaac Sim Benchmarks](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/reference_material/benchmarks.html), [Isaac Sim Performance Handbook](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/reference_material/sim_performance_optimization_handbook.html), [NVIDIA Forum procedural mesh](https://forums.developer.nvidia.com/t/importing-a-procedurally-generated-mesh-into-isaac-sim/262938)*

#### Ease of Importing Custom Mesh Environments
- Assets must be converted to USD format. NVIDIA provides importers for URDF, MJCF, OBJ/FBX/STL/glTF.
- **Procedural cave**: Python + USD API approach is viable but requires more engineering effort than Gazebo's SDF XML approach. For a thesis, this is a non-trivial upfront investment.
- Collision baking (calling `UsdPhysics.CollisionAPI.Apply()`) must be explicitly done for each mesh; falling through floors without it is a common pitfall.
  - *Sources: [Isaac Lab Asset Import](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)*

#### ROS 2 Integration Maturity
- **Functional but more complex**. ROS 2 integration in Isaac Sim is via OmniGraph action graphs — visual node-based programming that wraps ROS 2 publishers/subscribers. This is less intuitive than Gazebo's topic bridge YAML configs.
- Isaac ROS packages (Visual SLAM, DNN Inference, Obstacle Detection) are well-maintained GPU-accelerated ROS 2 packages that run on Jetson and desktop NVIDIA GPUs.
- ROS 2 bridge is stable for typical sensor streams, but setting up multi-sensor configurations requires careful OmniGraph wiring.
- **Not fully open-source**: Isaac Sim is free for individuals/academia but requires accepting NVIDIA's EULA. No source code access.
  - *Sources: [Isaac Sim RTX Lidar Tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)*

#### Community Support for Drone SLAM
- Growing rapidly since 2023. Pegasus Simulator has active development. PX4 community survey (McGuire 2025) shows Isaac Sim as the 4th most-used simulator after Gazebo variants.
- However, **cave/underground drone SLAM resources are sparse** in the Isaac Sim ecosystem. No equivalent of the SubT cave world library exists.
- For vision-based SLAM or RL-based navigation, Isaac Sim has better community resources.

#### GPU Requirements and System Requirements
- **Isaac Sim 5.x (2025–2026)**:
  - Minimum: RTX 4080, 16 GB VRAM, 32 GB RAM
  - Recommended: RTX 5080, 16 GB VRAM, 64 GB RAM
  - Ideal: RTX PRO 6000 Blackwell, 48 GB VRAM, 64 GB RAM
- **Requires RT Core-capable GPU** (RTX series). A100/H100 data center GPUs are explicitly unsupported.
- **Internet required** during startup for asset streaming from NVIDIA's Nucleus server.
- **Cost**: Free for individuals/academia under NVIDIA EULA.
  - *Sources: [Isaac Sim 5.1 Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)*

---

### 1.3 Unity + ROS 2 (Unity Robotics Hub / ros2-for-unity)

**Overview:** Unity supports ROS 2 via the Unity Robotics Hub (open-source, C# TCP bridge) and the `ros2-for-unity` package (more performant, native DDS). Unity uses PhysX for physics and its own rendering pipeline. LiDAR and sensor simulation require custom plugins.

#### LiDAR Sensor Simulation Quality
- **No native LiDAR plugin**: Unity does not include a built-in LiDAR sensor. Custom ray cast–based implementations are required, or the **Robotec GPU Lidar (RGL)** plugin (which also targets Unity alongside Gazebo and O3DE) can be used.
- RGL for Unity provides the same CUDA-accelerated ray casting as for Gazebo, including realistic LiDAR presets and noise models.
- Without RGL, Unity ray cast LiDAR implementations are CPU-bound and slow for high-density 3D LiDAR.
- **Key gap**: No ecosystem of pre-built, validated LiDAR configurations exists in Unity for drone SLAM workflows. Configuration is bespoke per project.
  - *Sources: [RobotecGPULidar GitHub](https://github.com/RobotecAI/RobotecGPULidar), [Robotec.AI blog](https://www.robotec.ai/blog/robotics/automated-future-with-rgl-robotec-gpu-lidar)*

#### Depth Camera Simulation
- Unity's rendering pipeline (URP/HDRP) produces high-quality depth maps. The Unity Robotics Hub includes a simple RGBD camera that publishes over ROS 2.
- Photorealism is comparable to or better than Isaac Sim in some scenarios due to Unity's mature shader ecosystem.
- Depth accuracy for close-range cave surfaces is generally good, but lacks the physically-based material models that Isaac Sim provides.

#### Drone Physics Fidelity
- **Challenging PX4 SITL integration**: Unity has no official PX4 plugin. Integration typically requires a workaround: Unity → WebSocket → Simulator (Go/Python) → ROS 2 → PX4, or using the `jMAVSim`-style MAVLink bridge.
- One community approach routes: Unity (physics + poses) → custom MAVLink bridge → PX4 SITL. This is functional but brittle and requires significant custom code.
- **ArduPilot thesis project (University of Thessaly)** built a Unity + ArduPilot + MATLAB integration, demonstrating it is possible but requiring substantial engineering effort.
- Unity's PhysX provides reasonable quadrotor dynamics, but rotor aerodynamics must be hand-coded in C#.
  - *Sources: [PX4 Unity discussion](https://discuss.px4.io/t/integrate-px4-and-unity/42508), [Unity+ArduPilot+MATLAB thesis](https://ir.lib.uth.gr/xmlui/bitstream/handle/11615/85750/31127.pdf)*

#### Performance with Large/Complex Meshes
- Unity's rendering engine excels at large, complex environments (it was designed for games). Procedural mesh generation via Unity's `Mesh` API is well-supported.
- Physics performance with complex collision meshes degrades under heavy polygon counts — convex decomposition (VHACD) is typically needed.
- For a thesis project, importing procedural cave meshes is straightforward using Unity's Mesh API or ProBuilder.
- Unity does not have Gazebo's "stuck at large environment physics" limitations.

#### Ease of Importing Custom Mesh Environments
- **Best-in-class for mesh import**: Unity supports FBX, OBJ, glTF, and custom mesh APIs directly in the editor. Procedural generation is natural in C#.
- ProBuilder allows in-editor mesh sculpting, useful for cave-like environments.
- Converting Blender exports to Unity is well-documented in the game development community.

#### ROS 2 Integration Maturity
- **Least mature of the three**. Unity Robotics Hub uses a TCP-based ROS-TCP-Connector, which adds latency and lacks QoS control. `ros2-for-unity` (Robotic Systems Lab, ETH Zurich fork) uses native DDS but has less maintenance momentum.
- ROS 2 message types are supported through C# code generation, but the ecosystem of ROS 2 packages validated with Unity is much smaller than Gazebo's.
- Offboard control through ROS 2 in Unity setups shows inconsistent behavior in community reports.
  - *Sources: [Unity ROS 2 announcement](https://unity.com/news/unity-announces-support-ros-2), [PX4 simulation survey](https://www.mcguirerobotics.com/px4_sim_research_report/)*

#### Community Support for Drone SLAM
- **Minimal for underground/cave SLAM**. No SubT-equivalent resource library, no community cave worlds, no validated LiDAR SLAM workflows.
- Unity drone simulation community is primarily focused on outdoor environments and visual navigation.
- Using Unity for underground LiDAR SLAM in ROS 2 would require building essentially all tooling from scratch.

#### GPU Requirements and System Requirements
- No RTX requirement: any modern GPU works (NVIDIA, AMD, Intel Arc).
- For RGL: CUDA-enabled GPU required.
- Minimum: 8 GB RAM, 4 GB VRAM; Recommended: 16+ GB RAM, 8+ GB VRAM.
- **Cost**: Free for student/academic use (Unity Personal/Student license). Free for projects under $200K annual revenue.

---

## 2. Comparative Scoring Matrix <a name="scoring-matrix"></a>

Scores are 1–5 (5 = best for this use case).

| Criterion | Weight | Gazebo Harmonic | Isaac Sim | Unity |
|---|---|---|---|---|
| LiDAR simulation quality & noise models | 15% | **5** | 4 | 3 |
| Depth camera simulation | 8% | 4 | **5** | 4 |
| Drone physics fidelity (PX4 SITL) | 15% | **5** | 4 | 2 |
| Performance with large/complex meshes | 10% | 3† | **5** | 4 |
| Ease of custom mesh import | 10% | 4 | 3 | **5** |
| ROS 2 integration maturity | 15% | **5** | 4 | 2 |
| Community support for drone SLAM/caves | 15% | **5** | 3 | 1 |
| GPU requirements (accessibility) | 7% | **4** | 2 | 4 |
| Open-source / thesis reproducibility | 5% | **5** | 3 | 4 |
| **Weighted Total** | **100%** | **4.60** | **3.72** | **2.87** |

_†With RGLGazeboPlugin, Gazebo's mesh performance score rises to 4 for high-complexity scenes._

---

## 3. DARPA SubT Challenge Analysis <a name="darpa-subt"></a>

The DARPA Subterranean (SubT) Challenge (2018–2021) is the most directly relevant precedent for this thesis. It was an $82M competition specifically targeting autonomous robotic exploration of GPS-denied underground environments including caves.

### Virtual Competition Platform

**The official simulator for the SubT Virtual Competition was Ignition Gazebo** (the direct predecessor to Gazebo Harmonic). Open Robotics was the official simulation provider for the virtual track. All virtual competition competitors used Ignition Gazebo with ROS.

Key facts:
- The SubT Virtual Testbed ran on Ignition Gazebo with a cloud infrastructure (Cloudsim) using 1,824 cloud machines for the finals.
- Cave worlds were built from 73 mesh tiles with procedural composition — exactly the tile-based procedural generation relevant to a thesis cave environment.
- Plugins developed included: thermal camera, dynamic rock falls, communication model for GPS-denied radio propagation, particle emitters (dust/smoke).
- All SubT worlds and robot models are publicly available on [Ignition Fuel](https://app.gazebosim.org/fuel/models) and the SubT tech repo.
  - *Sources: [Open Robotics SubT Final Competition](https://www.openrobotics.org/blog/2021/9/27/darpa-subt-final-competition), [Open Robotics SubT Part 3](https://www.openrobotics.org/blog/2022/2/3/subt-part-3-the-simulator)*

### Top Teams and Their Software

| Rank | Team | System Track Result | Simulation Used |
|---|---|---|---|
| 1st (Systems) | **CERBERUS** (NTNU, ETH Zurich, Berkeley, Oxford, Flyability) | 23 artifacts, $2M prize | Ignition Gazebo + ROS |
| 2nd (Systems) | **CSIRO Data61** (CSIRO + Emesent + Georgia Tech) | 23 artifacts, $1M prize | Ignition Gazebo + ROS |
| 3rd (Systems) | **MARBLE** (CU Boulder) | 18 artifacts, $500K prize | Ignition Gazebo + ROS |
| 1st (Virtual) | **Dynamo** (Keybotic) | 223 pts, $750K prize | Ignition Gazebo |
| 2nd (Virtual) | **CTU-CRAS-NORLAB** (Czech/Canadian) | 215 pts, $500K prize | Ignition Gazebo |
| 3rd (Virtual) | **Coordinated Robotics** | 212 pts, $250K prize | Ignition Gazebo |

> "The Gazebo simulations we created for SubT mirror the real world competition in almost every detail, as they should, given that we cloned the competition arenas and robots using the latest photogrammetry and laser scanning techniques."  
> — Open Robotics, SubT Final Competition blog post

The SubT challenge also produced a landmark survey on underground SLAM: *"Present and Future of SLAM in Extreme Underground Environments"* (JPL/NASA, arXiv:2208.01787), which catalogues LiDAR-centric SLAM systems validated in Gazebo environments.

**None of the top teams used Isaac Sim or Unity.**

---

## 4. Open-Source Cave Simulation Resources <a name="open-source-resources"></a>

### Directly Usable in Gazebo Harmonic / ROS 2

| Resource | Description | URL |
|---|---|---|
| **SubT Tech Repo** (OSRF) | Complete cave/tunnel/urban world SDF tiles, robot models (aerial + ground), ROS bridge examples | [github.com/osrf/subt](https://github.com/osrf/subt) |
| **SubT Hello World** | Starter tutorials for SubT testbed with ROS | [github.com/osrf/subt_hello_world](https://github.com/osrf/subt_hello_world) |
| **LTU-RAI gazebo_cave_world** | Cave world with DARPA tiles + custom models, stalactites/stalagmites, vertical shafts, AprilTag gates, ArduPilot drone evaluation area | [github.com/LTU-RAI/gazebo_cave_world](https://github.com/LTU-RAI/gazebo_cave_world) |
| **CTU-MRS SubT Ignition Resources** | Model and world files for Ignition Gazebo from SubT virtual challenge | [github.com/ctu-mrs/subt_ign_resources](https://github.com/ctu-mrs/subt_ign_resources) |
| **sjtu_drone (ROS 2)** | Quadrotor simulation in Gazebo for ROS 2 (NovoG93 fork with ROS 2 branch) | [github.com/NovoG93/sjtu_drone](https://github.com/NovoG93/sjtu_drone) |
| **px4_sim_ros2** | PX4 + Gazebo Harmonic + ROS 2 Docker ecosystem | [github.com/ParsaKhaledi/px4_sim_ros2](https://github.com/ParsaKhaledi/px4_sim_ros2) |
| **RGL Gazebo Plugin** | NVIDIA OptiX GPU LiDAR plugin for Gazebo Harmonic | [github.com/RobotecAI/RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) |

### SubT Cave World Details (arXiv:2004.08452)

A paper from Luleå University of Technology ("A Subterranean Virtual Cave World for Gazebo based on the DARPA SubT Challenge") documents open-source cave world creation methodology using SubT tiles, directly applicable to a thesis cave simulation setup.

### LiDAR SLAM Packages Validated with Gazebo + ROS 2

| Package | Type | Notes |
|---|---|---|
| **FAST-LIO2** | LiDAR-inertial odometry | Widely used in SubT-inspired research; supports Ouster, Velodyne, Livox |
| **LIO-SAM** | LiDAR-inertial SLAM | Graph-based, good for loop closure in caves |
| **KISS-ICP** | Point-to-point ICP odometry | Minimal-dependency, fast prototyping |
| **RTAB-Map** | Multi-modal SLAM | Supports LiDAR + RGBD, active ROS 2 maintenance |
| **Cartographer 3D** | Google's SLAM | Configurable for 3D cave mapping |
| **lidarslam_ros2** | 3D LiDAR SLAM for ROS 2 | [github.com/rsasaki0109/lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) |

---

## 5. PX4 SITL vs ArduPilot SITL Compatibility <a name="autopilot-comparison"></a>

### Gazebo Harmonic

| Feature | PX4 SITL | ArduPilot SITL |
|---|---|---|
| Integration method | `gz_bridge` (Gazebo transport, no MAVLink overhead) | JSON/TCP plugin (`ardupilot_gazebo`) |
| Communication with ROS 2 | Micro XRCE-DDS → `px4_msgs` | DDS-XRCE → `ardupilot_msgs` or MAVROS |
| Lockstep support | ✅ Full lockstep | ✅ Supported |
| Drone models | `x500`, `iris`, many more | `iris`, `zephyr` included |
| Community for SLAM | Extremely large (PX4 is the default for research drones) | Large (more used in hobby/applied contexts) |
| Recommended Gazebo version | Harmonic (official) | Harmonic (recommended, via `ardupilot_gz`) |
| Documentation quality | Excellent (official PX4 docs) | Good (ArduPilot dev docs) |
| ROS 2 Jazzy support | ✅ | ✅ |

**Recommendation for thesis**: Use **PX4 SITL** with Gazebo Harmonic. The `px4_msgs` + Micro XRCE-DDS bridge is the de facto standard for autonomous drone research in ROS 2, has better documentation, and aligns with most academic robotics publications. PX4's `x500` model in Gazebo Harmonic is the current default.

If ArduPilot is preferred (e.g., for compatibility with specific hardware), the `ardupilot_gz` package provides an equally capable SITL setup with Gazebo Harmonic.

### NVIDIA Isaac Sim

| Feature | PX4 SITL | ArduPilot SITL |
|---|---|---|
| Integration method | MAVLink via Pegasus Simulator | Experimental (not tested in Pegasus v4.5) |
| Communication with ROS 2 | MAVROS or Micro XRCE-DDS (via Pegasus) | Experimental |
| Lockstep support | ✅ Implemented | Not validated |
| Drone models | Quadrotor, hexacopter (Pegasus) | Limited |
| Community for SLAM | Sparse for underground SLAM | Very sparse |

### Unity

| Feature | PX4 SITL | ArduPilot SITL |
|---|---|---|
| Integration method | Custom WebSocket/bridge (no official plugin) | ArduPilot JSON interface (some research projects) |
| Communication with ROS 2 | Complex bridging required | Complex bridging required |
| Lockstep support | ❌ Not native | Partial (custom implementation needed) |
| Drone models | Hand-coded C# physics | ArduPilot MATLAB thesis demonstrated feasibility |
| Community for SLAM | Minimal | Minimal |

*Sources: [PX4 Simulation Docs](https://docs.px4.io/main/en/simulation/), [ArduPilot ROS 2 SITL](https://ardupilot.org/dev/docs/ros2-sitl.html), [ArduPilot Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html), [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator)*

---

## 6. Recommendation with Justification <a name="recommendation"></a>

### Primary Recommendation: Gazebo Harmonic + ROS 2 + PX4 SITL

**Confidence: High**

#### Key Justifications

**1. Direct relevance to DARPA SubT — the gold standard for your problem domain**  
The SubT challenge was the world's largest, most funded effort to solve exactly the problem your thesis addresses: autonomous robot navigation in GPS-denied underground caves using LiDAR SLAM. The official simulator for all virtual competition was Ignition Gazebo (now Gazebo Harmonic). The cave world tile library, all open-source implementations from top teams (CERBERUS, CTU-CRAS-NORLAB, Coordinated Robotics), and the associated SLAM packages were all developed in this ecosystem. Choosing Gazebo Harmonic means you inherit five years of directly applicable open-source infrastructure.

**2. Best ROS 2 integration maturity**  
Gazebo Harmonic has the most mature, well-documented, and battle-tested ROS 2 integration. The `ros_gz_bridge` is actively maintained. PX4 SITL integration (via Micro XRCE-DDS) is the most-used and best-documented autopilot simulation path for academic drone research.

**3. GPU-accessible LiDAR simulation via RGL**  
With the RGLGazeboPlugin, you get GPU-accelerated hardware ray-traced LiDAR that is ~4× faster than the built-in option and scales well to high-polygon cave meshes. This is open-source and free. A 2025 GSoC project additionally produced a vendor-agnostic (NVIDIA/AMD/Intel) ray-traced LiDAR plugin.

**4. Reproducibility and thesis robustness**  
Fully open-source stack. Reviewers can reproduce results without NVIDIA GPU ownership or commercial licenses. The SubT cave worlds are publicly available — using them immediately situates your work within an established benchmark context.

**5. Community and existing cave SLAM resources**  
The LTU-RAI cave world, SubT SDF tile library, CTU-MRS resources, sjtu_drone, and all the SLAM packages above are directly reusable. You are not building from scratch.

#### When to Consider Isaac Sim Instead (or as a complement)

Consider **Isaac Sim as a secondary validation environment** if:
- You have an RTX 4080+ GPU and 64 GB RAM available
- Your thesis includes a camera/vision component (depth camera quality is superior)
- You plan to use reinforcement learning or need domain randomization for sim-to-real transfer
- You want to benchmark LiDAR SLAM performance in photorealistic conditions
- Your target hardware is NVIDIA Jetson-based (the Isaac ROS stack aligns well)

**Do not use Isaac Sim as the primary environment** if:
- Your GPU has less than 16 GB VRAM
- Reproducibility without specialized hardware is important for your committee
- You want to leverage SubT cave world assets directly (format conversion required)

#### Unity: Not Recommended for This Use Case

Unity requires building essentially the entire sensor simulation and autopilot integration pipeline from scratch. There are no existing cave SLAM resources in Unity + ROS 2. PX4 SITL integration is fragile and undocumented. The development overhead would significantly detract from the actual research contribution.

---

## 7. Recommended Stack and Setup Path <a name="setup-path"></a>

### Recommended Software Stack

```
Operating System:    Ubuntu 22.04 LTS (Jammy) or Ubuntu 24.04 LTS (Noble)
ROS 2 Distribution:  Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
Simulator:           Gazebo Harmonic (gz-sim 8.x)
Bridge:              ros_gz_bridge (ros-<distro>-ros-gz-bridge)
Autopilot:           PX4 Autopilot v1.15.x (SITL)
Autopilot Bridge:    Micro XRCE-DDS Agent + px4_msgs
Drone Model:         PX4 x500 quadrotor
LiDAR Plugin:        RGLGazeboPlugin (RobotecAI, CUDA GPU)
                     OR built-in gz gpu_lidar (any CUDA GPU)
Cave World:          LTU-RAI gazebo_cave_world + SubT SDF tiles
SLAM:                FAST-LIO2 (LiDAR-inertial) or LIO-SAM
Visualization:       RViz2
QGC:                 QGroundControl (ground station)
```

### Key GitHub Repositories to Clone First

1. `PX4/PX4-Autopilot` — autopilot firmware with Gazebo SITL support
2. `PX4/px4_msgs` — ROS 2 message definitions
3. `eProsima/Micro-XRCE-DDS-Agent` — DDS bridge
4. `RobotecAI/RGLGazeboPlugin` — GPU LiDAR
5. `LTU-RAI/gazebo_cave_world` — cave world tiles
6. `FAST-LIO` (Hku-Mars) or `LIO-SAM` (TixiaoShan) — SLAM
7. `osrf/subt` — SubT tech repo for additional cave tiles and robot models

### Suggested Thesis Validation Sequence

1. **Phase 1**: Validate drone flight in Gazebo Harmonic with PX4 SITL in a simple enclosed world
2. **Phase 2**: Add RGL LiDAR sensor, validate point cloud output vs. RViz2
3. **Phase 3**: Integrate FAST-LIO2 or LIO-SAM, validate odometry in a simple box world
4. **Phase 4**: Load LTU-RAI cave world, run full LiDAR SLAM pipeline
5. **Phase 5**: Implement procedural tile-based cave generation (SubT tile methodology)
6. **Phase 6**: Benchmark SLAM across multiple procedurally generated caves

### Minimum Hardware Requirements for Comfortable Development

| Component | Minimum | Recommended |
|---|---|---|
| CPU | 8-core (Intel i7/AMD Ryzen 7) | 12+ core (i9/Ryzen 9) |
| RAM | 16 GB | 32 GB |
| GPU | NVIDIA GTX 1080 (CUDA, 8 GB VRAM) | NVIDIA RTX 3070+ (8+ GB VRAM) |
| Storage | 100 GB SSD | 500 GB NVMe SSD |

*If using RGL Gazebo Plugin: CUDA-capable NVIDIA GPU required. RTX GPU for hardware ray tracing recommended but not required — CUDA-only mode available.*

---

## References

1. Open Robotics, "DARPA SubT Final Competition" (2021) — https://www.openrobotics.org/blog/2021/9/27/darpa-subt-final-competition
2. Open Robotics, "SubT Part 3: The Simulator" (2022) — https://www.openrobotics.org/blog/2022/2/3/subt-part-3-the-simulator
3. DARPA, "Team CERBERUS and Team Dynamo Win DARPA SubT Challenge" (2021) — https://www.darpa.mil/news/2021/subterranean-challenge-winners
4. Pegasus Simulator arXiv paper (ICUAS 2024) — https://arxiv.org/html/2307.05263v2
5. PX4 Simulation Documentation — https://docs.px4.io/main/en/simulation/
6. ArduPilot SITL with Gazebo Docs — https://ardupilot.org/dev/docs/sitl-with-gazebo.html
7. ArduPilot ROS 2 SITL Docs — https://ardupilot.org/dev/docs/ros2-sitl.html
8. RGLGazeboPlugin GitHub — https://github.com/RobotecAI/RGLGazeboPlugin
9. RobotecGPULidar GitHub — https://github.com/RobotecAI/RobotecGPULidar
10. Isaac Sim RTX LiDAR Tutorial — https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html
11. Isaac Sim 5.1 Requirements — https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
12. Isaac Sim Performance Benchmarks — https://docs.isaacsim.omniverse.nvidia.com/4.5.0/reference_material/benchmarks.html
13. Isaac ROS Visual SLAM — https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
14. Pegasus Simulator GitHub — https://github.com/PegasusSimulator/PegasusSimulator
15. VividCloud Isaac Sim + PX4 Hexacopter — https://www.vividcloud.com/hexacopter-flight-simulation-with-px4-integration-in-nvidia-isaac-sim/
16. LTU-RAI Cave World — https://github.com/LTU-RAI/gazebo_cave_world
17. McGuire Robotics PX4 Sim Survey (2025) — https://www.mcguirerobotics.com/px4_sim_research_report/
18. SubT Hello World — https://github.com/osrf/subt_hello_world
19. CTU-MRS SubT Ignition Resources — https://github.com/ctu-mrs/subt_ign_resources
20. Unity ROS 2 Announcement — https://unity.com/news/unity-announces-support-ros-2
21. SubT Cave World arXiv paper — https://arxiv.org/abs/2004.08452
22. Present and Future of SLAM in Extreme Underground Environments (JPL/NASA) — https://www-robotics.jpl.nasa.gov/media/documents/2208.01787.pdf
23. GSoC 2025 GPU LiDAR for Gazebo — https://discourse.openrobotics.org/t/gsoc-2025-ray-tracing-enabled-faster-than-real-time-gpu-based-lidar-plugin-for-gazebo/50714
24. GPS-Denied UAV Visual SLAM (Bernas, 2025) — https://www.andrewbernas.com/docs/projects/robots/vslam
25. px4_sim_ros2 GitHub — https://github.com/ParsaKhaledi/px4_sim_ros2
26. Comparative Review of Drone Simulators (KPI Ukraine) — https://ela.kpi.ua/bitstreams/6ac76caf-05ce-4dc0-95cf-c2b0644958b1/download
27. RIIS LLC: Custom Flight Modes Using PX4 and ROS 2 — https://www.riis.com/blog/custom-flight-modes-using-px4-and-ros2

---

*Document prepared: April 2026 | Research methodology: systematic web search, primary source analysis, GitHub repository review, NVIDIA/Open Robotics documentation review*
