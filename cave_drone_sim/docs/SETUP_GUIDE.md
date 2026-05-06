# Setup Guide: Ubuntu 22.04 Fresh Installation

**Document type:** Step-by-step installation guide  
**Target OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)  
**ROS 2 distribution:** Humble Hawksbill  
**Gazebo version:** Harmonic (gz-sim 8.x)  
**Cross-references:** [README.md](../README.md) | [ARCHITECTURE.md](ARCHITECTURE.md)

---

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)
2. [ROS 2 Humble Installation](#2-ros-2-humble-installation)
3. [Gazebo Harmonic Installation](#3-gazebo-harmonic-installation)
4. [ROS 2 — Gazebo Bridge](#4-ros-2--gazebo-bridge)
5. [Python Dependencies](#5-python-dependencies)
6. [FAST-LIO2 Installation](#6-fast-lio2-installation)
7. [RTAB-Map Installation](#7-rtab-map-installation)
8. [Building the CaveDroneSim Workspace](#8-building-the-cavedronesim-workspace)
9. [Verifying the Installation](#9-verifying-the-installation)
10. [Docker Alternative](#10-docker-alternative)
11. [Common Issues and Fixes](#11-common-issues-and-fixes)

---

## 1. Hardware Requirements

### Minimum

| Component | Minimum | Notes |
|-----------|---------|-------|
| CPU | 8-core (Intel i7 / AMD Ryzen 7) | 12+ cores recommended |
| RAM | 16 GB | 32 GB for high-resolution caves |
| GPU | NVIDIA GTX 1080, 8 GB VRAM, CUDA 11.x | Required for `gpu_lidar` in Gazebo |
| Storage | 50 GB free SSD | NVMe recommended |
| OS | Ubuntu 22.04 LTS | Other distros not tested |

### Recommended

| Component | Recommended |
|-----------|-------------|
| CPU | 12-core Intel i9 / AMD Ryzen 9 |
| RAM | 32 GB DDR5 |
| GPU | NVIDIA RTX 3070 or higher, 8+ GB VRAM |
| Storage | 200 GB NVMe SSD |

> **Note on GPU:** The Gazebo `gpu_lidar` plugin requires a CUDA-capable NVIDIA GPU. AMD and Intel GPUs are not supported for hardware-accelerated LiDAR simulation. Without a GPU, the `cpu_lidar` plugin can be used but is ~10× slower and may not achieve real-time simulation.

---

## 2. ROS 2 Humble Installation

Follow the official ROS 2 Humble installation instructions for Ubuntu 22.04.

```bash
# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key and repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc for persistence)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 --version
# Expected: ros2 cli version X.X.X (from ros2cli-X.X.X)
```

---

## 3. Gazebo Harmonic Installation

Gazebo Harmonic (gz-sim 8.x) is installed separately from ROS 2.

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install lsb-release gnupg -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic -y

# Verify installation
gz sim --version
# Expected: Gazebo Sim 8.X.X
```

---

## 4. ROS 2 — Gazebo Bridge

The `ros_gz` bridge translates between Gazebo transport messages and ROS 2 messages.

```bash
sudo apt install ros-humble-ros-gz -y

# This installs:
# ros-humble-ros-gz-bridge   — topic bridging
# ros-humble-ros-gz-sim      — launch utilities
# ros-humble-ros-gz-image    — image conversion
# ros-humble-ros-gz-interfaces — message type definitions
```

Verify:
```bash
ros2 pkg list | grep ros_gz
# Expected: ros_gz_bridge, ros_gz_sim, etc.
```

---

## 5. Python Dependencies

```bash
# Core scientific Python
sudo apt install python3-pip python3-numpy python3-scipy -y

# Install Python packages
pip3 install \
    numpy \
    scipy \
    scikit-image \
    scikit-learn \
    trimesh \
    open3d \
    noise \
    matplotlib \
    pyyaml \
    evo

# Verify key installs
python3 -c "import skimage; print('scikit-image:', skimage.__version__)"
python3 -c "import trimesh; print('trimesh:', trimesh.__version__)"
python3 -c "import open3d; print('open3d:', open3d.__version__)"
python3 -c "import evo; print('evo installed')"
```

---

## 6. FAST-LIO2 Installation

FAST-LIO2 requires a community-maintained ROS 2 port. The official upstream repository is ROS 1 only.

```bash
# Create or use an existing ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the community ROS 2 port
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git

# Install dependencies
cd ~/ros2_ws
sudo apt install libeigen3-dev libpcl-dev -y
rosdep install --from-paths src --ignore-src -r -y

# Build FAST-LIO2 (Release mode — required for real-time performance)
colcon build \
    --packages-select fast_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> **Build time:** Approximately 2–5 minutes depending on hardware. If the build fails with Eigen errors, ensure `libeigen3-dev` is installed before building.

Verify:
```bash
ros2 pkg list | grep fast_lio
# Expected: fast_lio
```

---

## 7. RTAB-Map Installation

RTAB-Map has a native ROS 2 Humble binary package:

```bash
sudo apt install ros-humble-rtabmap-ros -y

# Optional: OctoMap visualization
sudo apt install ros-humble-octomap-ros ros-humble-octomap-rviz-plugins -y
```

Verify:
```bash
ros2 pkg list | grep rtabmap
# Expected: rtabmap_launch, rtabmap_msgs, rtabmap_ros, rtabmap_slam, rtabmap_util, rtabmap_viz
```

---

## 8. Building the CaveDroneSim Workspace

```bash
# Clone the repository
cd ~
git clone https://github.com/your-org/cave_drone_sim.git
cd cave_drone_sim

# Install ROS 2 package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install additional ROS 2 packages
sudo apt install \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    -y

# Build the workspace
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source src/install/setup.bash
echo "source ~/cave_drone_sim/src/install/setup.bash" >> ~/.bashrc
```

> **Note on `--symlink-install`:** This creates symbolic links instead of copies for Python files, allowing you to edit source files without rebuilding. Recommended during development. For a clean release build, omit this flag.

---

## 9. Verifying the Installation

Run the following checks in order. Each should complete without errors.

### 9.1 Cave Generation Test

```bash
source install/setup.bash

# Generate a small cave (should complete in < 10 s)
ros2 run cave_generator generator_node \
    --ros-args \
    -p num_nodes:=8 \
    -p bounds_x:=40.0 \
    -p bounds_y:=40.0 \
    -p bounds_z:=15.0 \
    -p resolution:=0.5 \
    -p export_gazebo:=false \
    -p seed:=0

# Expected output:
# [INFO] [cave_generator_node]: Graph generated: 8 nodes, 8 edges
# [INFO] [cave_generator_node]: SDF field computed: 80x80x30 grid
# [INFO] [cave_generator_node]: Mesh extracted: 45230 vertices, 90460 faces
# [INFO] [cave_generator_node]: Cave generation complete in 2.8s
```

### 9.2 Gazebo Launch Test

```bash
# Launch Gazebo Harmonic with empty world
ros2 launch drone_bringup drone_only.launch.py

# Expected: Gazebo window opens with drone model visible
# In another terminal:
ros2 topic hz /lidar/points
# Expected: rate: ~10 Hz

ros2 topic hz /imu/data
# Expected: rate: ~200 Hz
```

### 9.3 SLAM Test

```bash
# In terminal 1: Launch simulation
ros2 launch drone_bringup drone_only.launch.py

# In terminal 2: Launch SLAM
ros2 launch slam_pipeline slam_simple.launch.py

# In terminal 3: Drive manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/drone/cmd_vel

# In terminal 4: Check odometry
ros2 topic hz /slam/odometry
# Expected: rate: ~10 Hz

ros2 topic hz /octomap_full_color
# Expected: rate: ~1 Hz
```

### 9.4 Full Demo Test

```bash
# Launch the complete system
ros2 launch cave_drone_sim full_demo.launch.py seed:=0 slam_backend:=simple

# Wait for cave generation (~3 s)
# Watch RViz2 — drone should take off and start exploring
# Check mission state:
ros2 topic echo /mission/state
# Expected: transitions through IDLE → TAKEOFF → EXPLORE → NAVIGATE ...
```

---

## 10. Docker Alternative

A Dockerfile is provided that bundles all dependencies. This is recommended for reproducible deployments and CI/CD.

### Build the Docker Image

```dockerfile
# Dockerfile (located at cave_drone_sim/Dockerfile)
FROM osrf/ros:humble-desktop

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gz \
    ros-humble-rtabmap-ros \
    ros-humble-octomap-ros \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy scipy scikit-image scikit-learn trimesh open3d noise evo

# Build CaveDroneSim (FAST-LIO2 installed separately)
WORKDIR /ros2_ws
COPY . /ros2_ws/src/cave_drone_sim/
RUN colcon build --base-paths src --symlink-install

SHELL ["/bin/bash", "-c"]
RUN echo "source /ros2_ws/src/install/setup.bash" >> /root/.bashrc
```

```bash
# Build image
docker build -t cave_drone_sim:humble .

# Run with GPU support and X11 forwarding
docker run --gpus all \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --rm -it cave_drone_sim:humble \
    bash -c "ros2 launch cave_drone_sim full_demo.launch.py seed:=42"
```

### Using the Pre-built Image

```bash
docker pull ghcr.io/your-org/cave_drone_sim:latest
docker run --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    --rm -it ghcr.io/your-org/cave_drone_sim:latest
```

---

## 11. Common Issues and Fixes

### Issue: Gazebo does not start (hangs at "Loading world")

**Cause:** Missing GPU driver or CUDA not initialized.  
**Fix:**
```bash
# Check NVIDIA driver
nvidia-smi
# If not found, install NVIDIA driver:
sudo ubuntu-drivers autoinstall
sudo reboot

# Check CUDA
nvcc --version
# If missing: sudo apt install cuda-toolkit-12-x

# Set DISPLAY for headless environments:
export DISPLAY=:0
```

### Issue: `ros_gz_bridge` reports "Topic type mismatch"

**Cause:** Message type mismatch between Gazebo and ROS 2 bridge configuration.  
**Fix:** Check `drone_bringup/config/gz_bridge.yaml` — ensure `ros_type_name` and `gz_type_name` match exactly. Verify with:
```bash
gz topic -l   # List Gazebo topics
ros2 topic list  # List ROS 2 topics
```

### Issue: `colcon build` fails with "Could not find a package configuration file for Eigen3"

**Fix:**
```bash
sudo apt install libeigen3-dev
# Add to CMakeLists.txt search path if needed:
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
```

### Issue: FAST-LIO2 "waiting for first IMU measurement"

**Cause:** IMU topic not publishing or topic name mismatch.  
**Fix:**
```bash
ros2 topic echo /imu/data --once
# If no output, check gz_bridge.yaml IMU bridge entry
# Verify: ros2 topic hz /imu/data  (should be ~200 Hz)
```

### Issue: OctoMap visualization shows nothing in RViz2

**Cause:** Frame ID mismatch or OctoMap not yet initialized.  
**Fix:**
```bash
# Check OctoMap is being published:
ros2 topic echo /octomap_full_color --field header.frame_id
# Expected: "map"

# In RViz2: Fixed Frame must be set to "map"
# Add OctoMap display: Add → By topic → /octomap_full_color → OccupancyGrid

# Check TF tree:
ros2 run tf2_tools view_frames.py
# Inspect ~/frames.pdf — "map" must be present
```

### Issue: Cave generation fails with MemoryError

**Cause:** Voxel grid too large for available RAM.  
**Fix:** Increase `resolution` or decrease `bounds_*`:
```bash
# Use 0.5 m resolution for large caves during development:
ros2 launch cave_generator generate_cave.launch.py resolution:=0.5
```

### Issue: RViz2 "No map received" for exploration display

**Cause:** `frontier_explorer` has not published frontiers yet; OctoMap not populated.  
**Fix:** Wait 30–60 seconds after takeoff for the drone to observe enough of the cave for frontiers to appear. Check:
```bash
ros2 topic echo /exploration/frontiers --once
```

### Issue: `evo_ape` reports "No common timestamps"

**Cause:** ROS 2 bag topic timestamps don't align.  
**Fix:** Use the bag-to-TUM conversion script that handles timestamp alignment:
```bash
ros2 run slam_pipeline bag_to_tum.py \
    --bag <bag_dir> \
    --est_topic /slam/odometry \
    --gt_topic /drone/ground_truth \
    --max_diff 0.05 \
    --output_dir results/
```

### Issue: Drone falls through the floor in Gazebo

**Cause:** Cave collision mesh is not watertight (holes in the mesh).  
**Fix:**
```bash
# Regenerate with mesh repair enabled (default is on)
# Check that trimesh repair ran successfully — look for log output:
# [INFO] [cave_generator_node]: Mesh repaired: 0 holes filled, 0 degenerate faces removed
# If holes > 0: decrease resolution (finer grid produces fewer boundary artifacts)
```

---

*Last updated: April 2026. For setup issues not covered here, file a GitHub issue with your Ubuntu version, GPU model, and the full error log.*
