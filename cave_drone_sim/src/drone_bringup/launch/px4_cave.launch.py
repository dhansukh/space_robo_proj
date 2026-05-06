"""
px4_cave.launch.py
==================
ROS 2 launch file for the PX4 SITL cave exploration setup.

This launch file starts the ROS 2 side of the simulation:
  1. Micro XRCE-DDS Agent  — bridges PX4 ↔ ROS 2 (px4_msgs)
  2. ros_gz_bridge          — bridges Gazebo sensor topics to ROS 2
  3. Static TF broadcaster  — sensor frame transforms
  4. (Optional) RViz2       — pre-configured visualization

It does NOT start Gazebo or PX4 — those are launched separately via:
  ./scripts/start_px4_cave.sh

Launch arguments
----------------
use_rviz        : 'true' / 'false'  (default: true)
use_sim_time    : 'true' / 'false'  (default: true)
bridge_config   : path to gz_bridge yaml (default: package default)
use_vel_bridge  : 'true' / 'false'  (default: true)
    Set to 'false' when using cave_explorer.py or square_flight.py —
    those scripts publish their own OffboardControlMode directly to PX4.
    Running px4_vel_bridge alongside them causes a mode conflict
    (velocity vs position) that prevents takeoff.

Usage
-----
    # Terminal 1: Start PX4 + Gazebo
    ./scripts/start_px4_cave.sh

    # Terminal 2a: Full stack (mission_manager controls flight)
    ros2 launch drone_bringup px4_cave.launch.py

    # Terminal 2b: Bridge only (cave_explorer.py / square_flight.py controls flight)
    ros2 launch drone_bringup px4_cave.launch.py use_vel_bridge:=false

    # Terminal 3 (optional): Verify
    ros2 topic list | grep -E "fmu|lidar|imu|clock"
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build and return the PX4 cave bringup LaunchDescription."""

    pkg_share = get_package_share_directory('drone_bringup')

    # Default paths
    default_bridge = os.path.join(pkg_share, 'config', 'gz_bridge_px4.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 visualization',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock (/clock topic)',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=default_bridge,
            description='Path to ros_gz_bridge YAML config',
        ),
        DeclareLaunchArgument(
            'use_vel_bridge',
            default_value='true',
            description=(
                'Launch px4_vel_bridge (cmd_vel → PX4 velocity setpoints). '
                'Set false when cave_explorer.py or square_flight.py is used '
                'so they can publish position setpoints without conflict.'
            ),
        ),
    ]

    use_rviz       = LaunchConfiguration('use_rviz')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    bridge_config  = LaunchConfiguration('bridge_config')
    use_vel_bridge = LaunchConfiguration('use_vel_bridge')

    # ------------------------------------------------------------------
    # 1. Micro XRCE-DDS Agent
    #    Bridges PX4 uORB ↔ ROS 2 px4_msgs topics
    #    Listens on UDP port 8888 (PX4 SITL default)
    # ------------------------------------------------------------------
    micro_xrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4',
            '-p', '8888',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 2. ros_gz_bridge  (sensor topics + clock)
    # ------------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. Static TF broadcaster (sensor frames relative to base_link)
    # ------------------------------------------------------------------
    tf_broadcaster = Node(
        package='drone_bringup',
        executable='tf_broadcaster',
        name='drone_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 4. Depth image colorizer (32FC1 → mono8 for RViz Image display)
    # ------------------------------------------------------------------
    depth_colorizer = Node(
        package='drone_bringup',
        executable='depth_colorizer',
        name='depth_colorizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 5. PX4 velocity bridge (/cmd_vel → offboard TrajectorySetpoint)
    #    Disabled when use_vel_bridge:=false (e.g. cave_explorer.py runs)
    # ------------------------------------------------------------------
    px4_vel_bridge = Node(
        package='drone_bringup',
        executable='px4_vel_bridge',
        name='px4_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_vel_bridge),
    )

    # ------------------------------------------------------------------
    # 6. (Optional) RViz2
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # Assemble
    # ------------------------------------------------------------------
    return LaunchDescription(
        args
        + [
            micro_xrce_agent,
            ros_gz_bridge,
            tf_broadcaster,
            depth_colorizer,
            px4_vel_bridge,
            rviz,
        ]
    )
