# Space Robo Project - Cave Drone Simulation

This repository contains the `cave_drone_sim` full-stack autonomous drone exploration platform for GPS-denied underground cave environments. It features procedurally-generated 3D cave worlds, LiDAR-inertial SLAM, and a frontier-based exploration planner.

Included in this repository is the pre-generated `cave_flat` dataset, so you can immediately clone the repository and run SLAM without needing to generate your own cave from scratch.

---

## 1. Quick Start: Running the Full SLAM Pipeline

To run the pipeline using the pre-generated `cave_flat` map, open four separate terminals and run the following commands in order from the `cave_drone_sim/` workspace root:

**Terminal 1 — PX4 SITL + Gazebo Simulation**
```bash
cd cave_drone_sim
./scripts/start_px4_cave.sh
```
*(Wait until you see `[Gazebo] ... model loaded` before continuing.)*

**Terminal 2 — ROS 2 Drone Bridge**
```bash
cd cave_drone_sim
ros2 launch drone_bringup px4_cave.launch.py use_vel_bridge:=false
```

**Terminal 3 — SLAM Pipeline**
```bash
cd cave_drone_sim
ros2 launch slam_pipeline slam_simple.launch.py
```

**Terminal 4 — Coverage Mission**
```bash
cd cave_drone_sim
# Start the coverage planner (auto-saving maps every 2 minutes)
python3 scripts/slam_coverage.py --cave ./output/cave_flat --max-segs 3 --auto-save 2

# Fire the start signal (run this in another terminal or after the node is ready)
ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
```

Maps (2D grids and 3D point clouds) are automatically saved to `/tmp/slam_maps/`.

---

## 2. Generating Custom 3D Cave Maps

You can procedurally generate new 3D caves with adjustable parameters (e.g., bounds, roughness, stalactite density). 

**Using the Standalone Script:**
```bash
cd cave_drone_sim
python3 scripts/generate_cave.py --seed 99 --nodes 30 --output ./output/my_custom_cave
```

**Using ROS 2 Launch (Includes Gazebo Export):**
```bash
cd cave_drone_sim
ros2 launch cave_generator generate_cave.launch.py \
    seed:=99 num_nodes:=30 bounds_x:=120.0 bounds_z:=30.0 \
    noise_amplitude:=0.4 stalactite_density:=0.5
```

---

## 3. Opening the 3D Map in Gazebo

To launch a full simulation with a specific generated cave map (for instance, seed `42`), you can run the full demo launch file:

```bash
cd cave_drone_sim
ros2 launch cave_drone_sim full_demo.launch.py seed:=42 slam_backend:=simple
```

**Manual Flight Test (No Exploration):**
If you want to just fly around the cave manually to test odometry:
```bash
# Terminal 1: Launch drone in Gazebo
cd cave_drone_sim
ros2 launch drone_bringup drone_only.launch.py

# Terminal 2: Manual Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/drone/cmd_vel
```

---

## Further Reading

For more technical details, refer to the individual guides located in the `cave_drone_sim/docs/` directory:
- `SLAM_GUIDE.md`: Deep dive into SLAM backends and tuning.
- `CAVE_GENERATION.md`: How the procedural generation pipeline works.
- `SETUP_GUIDE.md`: Installation instructions for Ubuntu 22.04.
- `EXPLORATION_GUIDE.md`: RRT* and APF path planning breakdown.
