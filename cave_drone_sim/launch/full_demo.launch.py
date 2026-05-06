#!/usr/bin/env python3
"""
Full Cave Drone Exploration Demo
=================================
Generates a cave, spawns drone, runs SLAM, and starts autonomous exploration.

Launch sequence
---------------
  t=0s   Gazebo + drone bringup (drone_sim.launch.py)
  t=5s   SLAM pipeline (slam_simple / slam_fastlio / slam_rtabmap)
  t=8s   Exploration planner (explore.launch.py)

Usage
-----
  ros2 launch cave_drone_sim full_demo.launch.py
  ros2 launch cave_drone_sim full_demo.launch.py cave_seed:=42 num_nodes:=30
  ros2 launch cave_drone_sim full_demo.launch.py slam_backend:=rtabmap
  ros2 launch cave_drone_sim full_demo.launch.py use_rviz:=false log_level:=debug
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build and return the full demo LaunchDescription."""

    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    drone_bringup_share   = FindPackageShare('drone_bringup')
    slam_pipeline_share   = FindPackageShare('slam_pipeline')
    exploration_share     = FindPackageShare('exploration_planner')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'cave_seed',
            default_value='42',
            description='Random seed used when the cave was generated',
        ),
        DeclareLaunchArgument(
            'num_nodes',
            default_value='25',
            description='Number of cave chambers (used by cave_generator)',
        ),
        DeclareLaunchArgument(
            'cave_bounds',
            default_value='80 80 25',
            description='Bounding box of the cave in metres: "BX BY BZ"',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description=(
                'Path to a pre-generated .sdf world file. '
                'When empty the default output path produced by run_demo.sh is used.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_backend',
            default_value='simple',
            choices=['simple', 'fastlio', 'rtabmap'],
            description='SLAM backend: simple | fastlio | rtabmap',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualisation',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level: debug | info | warn | error',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/slam_maps',
            description='Directory for saved SLAM maps',
        ),
    ]

    # LaunchConfiguration references
    world_file    = LaunchConfiguration('world_file')
    slam_backend  = LaunchConfiguration('slam_backend')
    use_rviz      = LaunchConfiguration('use_rviz')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    log_level     = LaunchConfiguration('log_level')
    output_dir    = LaunchConfiguration('output_dir')

    # ------------------------------------------------------------------
    # Step 1 – Gazebo + drone bringup  (t = 0 s)
    # ------------------------------------------------------------------
    drone_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([drone_bringup_share, 'launch', 'drone_sim.launch.py'])
        ]),
        launch_arguments={
            'world_file':   world_file,
            'use_rviz':     use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ------------------------------------------------------------------
    # Step 2 – SLAM pipeline  (t = 5 s)
    # Backend is selected at launch time via slam_backend argument.
    # ------------------------------------------------------------------

    slam_simple_launch = GroupAction(
        condition=LaunchConfigurationEquals('slam_backend', 'simple'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([slam_pipeline_share, 'launch', 'slam_simple.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'log_level':    log_level,
                    'output_dir':   output_dir,
                }.items(),
            )
        ],
    )

    slam_fastlio_launch = GroupAction(
        condition=LaunchConfigurationEquals('slam_backend', 'fastlio'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([slam_pipeline_share, 'launch', 'slam_fastlio.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'output_dir':   output_dir,
                }.items(),
            )
        ],
    )

    slam_rtabmap_launch = GroupAction(
        condition=LaunchConfigurationEquals('slam_backend', 'rtabmap'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([slam_pipeline_share, 'launch', 'slam_rtabmap.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'output_dir':   output_dir,
                }.items(),
            )
        ],
    )

    slam_group = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[full_demo] ── Step 2: Starting SLAM pipeline ──'),
            slam_simple_launch,
            slam_fastlio_launch,
            slam_rtabmap_launch,
        ],
    )

    # ------------------------------------------------------------------
    # Step 3 – Exploration planner  (t = 8 s)
    # Waits for SLAM to initialise before sending goals.
    # ------------------------------------------------------------------
    exploration_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[full_demo] ── Step 3: Starting exploration planner ──'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([exploration_share, 'launch', 'explore.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'log_level':    log_level,
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------------------------
    # Assemble LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        args
        + [
            LogInfo(msg='[full_demo] ════════════════════════════════════════════'),
            LogInfo(msg='[full_demo]   Cave Drone Exploration Demo – Full Stack  '),
            LogInfo(msg='[full_demo] ════════════════════════════════════════════'),
            LogInfo(msg='[full_demo] ── Step 1: Starting Gazebo + drone bringup ──'),
            drone_sim_launch,
            slam_group,
            exploration_launch,
        ]
    )
