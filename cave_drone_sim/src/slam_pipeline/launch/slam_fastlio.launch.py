#!/usr/bin/env python3
"""
slam_fastlio.launch.py
======================
Launch file for the FAST-LIO2-backed SLAM pipeline.

Assumes FAST-LIO2 (fast_lio package) is installed in the workspace.
See: https://github.com/hku-mars/FAST_LIO

FAST-LIO2 provides:
  /Odometry       → remapped to /slam/odom
  /cloud_registered → remapped to /slam/map_cloud

We then launch:
  octomap_builder     — 3-D occupancy mapping from FAST-LIO2 output
  degeneracy_detector — geometry health monitor (raw LiDAR)
  map_saver           — on-demand map persistence

Usage
-----
  ros2 launch slam_pipeline slam_fastlio.launch.py
  ros2 launch slam_pipeline slam_fastlio.launch.py \
      fastlio_config:=/path/to/your/lio_config.yaml

Prerequisites
-------------
  sudo apt install ros-$ROS_DISTRO-fast-lio  # or build from source
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare('slam_pipeline')

    # ── Launch arguments ──────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
        description='SLAM pipeline parameters',
    )
    fastlio_config_arg = DeclareLaunchArgument(
        'fastlio_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'fastlio_cave.yaml']),
        description='FAST-LIO2 configuration file',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/slam_maps',
        description='Directory for saved maps',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    params_file   = LaunchConfiguration('params_file')
    fastlio_cfg   = LaunchConfiguration('fastlio_config')
    output_dir    = LaunchConfiguration('output_dir')
    use_sim_time  = LaunchConfiguration('use_sim_time')

    # ── FAST-LIO2 node ────────────────────────────────────────────────────
    # FAST-LIO2 publishes on its own topic namespace; we remap to our
    # convention so downstream nodes need no modification.
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[
            fastlio_cfg,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # FAST-LIO2 → our convention
            ('/Odometry',          '/slam/odom'),
            ('/cloud_registered',  '/slam/map_cloud'),
            # Input: our LiDAR topic → FAST-LIO2 expected topic
            ('/drone/lidar/points', '/livox/lidar'),
            ('/drone/imu',          '/livox/imu'),
        ],
    )

    # ── Downstream nodes ──────────────────────────────────────────────────

    octomap_builder_node = Node(
        package='slam_pipeline',
        executable='octomap_builder',
        name='octomap_builder',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    degeneracy_detector_node = Node(
        package='slam_pipeline',
        executable='degeneracy_detector',
        name='degeneracy_detector',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    map_saver_node = Node(
        package='slam_pipeline',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'output_dir': output_dir,
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        fastlio_config_arg,
        output_dir_arg,
        use_sim_time_arg,
        LogInfo(msg='[slam_fastlio] Launching FAST-LIO2 + OctoMap pipeline...'),
        fastlio_node,
        octomap_builder_node,
        degeneracy_detector_node,
        map_saver_node,
        LogInfo(msg='[slam_fastlio] All nodes launched.'),
    ])
