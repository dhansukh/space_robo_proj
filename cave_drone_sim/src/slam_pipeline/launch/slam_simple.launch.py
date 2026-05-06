#!/usr/bin/env python3
"""
slam_simple.launch.py
=====================
Launches the self-contained SLAM pipeline with no external SLAM
dependencies.  All processing is done by the nodes in this package.

Nodes started
-------------
  simple_lidar_odom    — ICP-based LiDAR odometry
  octomap_builder      — 3-D occupancy mapping
  degeneracy_detector  — geometry health monitor
  map_saver            — on-demand map persistence

Usage
-----
  ros2 launch slam_pipeline slam_simple.launch.py
  ros2 launch slam_pipeline slam_simple.launch.py output_dir:=/home/user/maps
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare('slam_pipeline')

    # ── Launch arguments ──────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
        description='Path to the SLAM parameter file',
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
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )

    params_file    = LaunchConfiguration('params_file')
    output_dir     = LaunchConfiguration('output_dir')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    log_level      = LaunchConfiguration('log_level')

    # ── Nodes ─────────────────────────────────────────────────────────────

    simple_lidar_odom_node = Node(
        package='slam_pipeline',
        executable='simple_lidar_odom',
        name='simple_lidar_odom',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/drone/lidar/points', '/lidar/points'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    octomap_builder_node = Node(
        package='slam_pipeline',
        executable='octomap_builder',
        name='octomap_builder',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
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
        arguments=['--ros-args', '--log-level', log_level],
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
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        params_file_arg,
        output_dir_arg,
        use_sim_time_arg,
        log_level_arg,
        LogInfo(msg='[slam_simple] Starting self-contained SLAM pipeline...'),
        simple_lidar_odom_node,
        octomap_builder_node,
        degeneracy_detector_node,
        map_saver_node,
        LogInfo(msg='[slam_simple] All nodes launched.'),
    ])
