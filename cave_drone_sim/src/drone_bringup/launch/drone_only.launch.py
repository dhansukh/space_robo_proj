"""
drone_only.launch.py
====================
Launches the drone ROS 2 nodes **without** starting Gazebo.

Use this launch file when Gazebo is already running (e.g. launched separately
or by another launch file) and you only need the drone software stack:

    * robot_state_publisher  — URDF + joint-state publication
    * drone_tf_broadcaster   — static sensor-frame TF transforms
    * drone_controller       — velocity PID + /drone/odom publisher
    * ros_gz_bridge          — topic bridge from Gazebo to ROS 2

This is useful for:
* Iterating on control logic without restarting Gazebo.
* Running multiple drones (call this launch file once per drone, varying
  the ``drone_ns`` namespace argument).
* CI/CD pipelines that launch the world separately.

Launch arguments
----------------
use_sim_time  : 'true' / 'false'  (default: true)
params_file   : path to drone_params.yaml (default: package default)
drone_ns      : ROS namespace for all nodes (default: '')
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build and return the drone-only LaunchDescription."""

    pkg_share = get_package_share_directory('drone_bringup')

    default_urdf   = os.path.join(pkg_share, 'urdf', 'cave_drone.urdf.xacro')
    default_params = os.path.join(pkg_share, 'config', 'drone_params.yaml')
    default_bridge = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock (/clock topic)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to drone_params.yaml parameter file',
        ),
        DeclareLaunchArgument(
            'drone_ns',
            default_value='',
            description='Optional ROS 2 namespace for all nodes',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    drone_ns     = LaunchConfiguration('drone_ns')

    # ------------------------------------------------------------------
    # robot_state_publisher
    # ------------------------------------------------------------------
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        default_urdf,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=drone_ns,
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Static TF broadcaster (sensor frames)
    # ------------------------------------------------------------------
    tf_broadcaster = Node(
        package='drone_bringup',
        executable='tf_broadcaster',
        name='drone_tf_broadcaster',
        namespace=drone_ns,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # Drone velocity controller
    # ------------------------------------------------------------------
    drone_controller = Node(
        package='drone_bringup',
        executable='drone_controller',
        name='drone_controller',
        namespace=drone_ns,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # ros_gz_bridge  (connect Gazebo sensor topics to ROS 2)
    # ------------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace=drone_ns,
        output='screen',
        parameters=[
            {
                'config_file': default_bridge,
                'use_sim_time': use_sim_time,
                'qos_overrides./drone/lidar/points.publisher.reliability': 'best_effort',
            }
        ],
    )

    # ------------------------------------------------------------------
    # Assemble
    # ------------------------------------------------------------------
    return LaunchDescription(
        args
        + [
            robot_state_publisher,
            tf_broadcaster,
            drone_controller,
            ros_gz_bridge,
        ]
    )
