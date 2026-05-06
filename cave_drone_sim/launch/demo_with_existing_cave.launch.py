#!/usr/bin/env python3
"""
demo_with_existing_cave.launch.py
==================================
Launch the full exploration demo with a **pre-generated** cave world file.
Cave generation is skipped — pass the path to an .sdf world directly.

This is useful when you already have a cave world on disk and want to
iterate quickly on SLAM or exploration parameters without re-generating.

Usage
-----
  ros2 launch cave_drone_sim demo_with_existing_cave.launch.py \\
      world_file:=/path/to/cave_world.sdf

  ros2 launch cave_drone_sim demo_with_existing_cave.launch.py \\
      world_file:=/tmp/cave_001/cave_world.sdf \\
      slam_backend:=rtabmap \\
      use_rviz:=false
"""

from __future__ import annotations

import os

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
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build and return the demo-with-existing-cave LaunchDescription."""

    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    drone_bringup_share = FindPackageShare('drone_bringup')
    slam_pipeline_share = FindPackageShare('slam_pipeline')
    exploration_share   = FindPackageShare('exploration_planner')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'world_file',
            description='Full path to the pre-generated .sdf world file (required)',
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
        DeclareLaunchArgument(
            'drone_name',
            default_value='cave_drone',
            description='Gazebo model name for the drone',
        ),
    ]

    # LaunchConfiguration references
    world_file   = LaunchConfiguration('world_file')
    slam_backend = LaunchConfiguration('slam_backend')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level    = LaunchConfiguration('log_level')
    output_dir   = LaunchConfiguration('output_dir')
    drone_name   = LaunchConfiguration('drone_name')

    # ------------------------------------------------------------------
    # Step 1 – Gazebo + drone bringup  (t = 0 s)
    # ------------------------------------------------------------------
    drone_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([drone_bringup_share, 'launch', 'drone_sim.launch.py'])
        ]),
        launch_arguments={
            'world_file':   world_file,
            'drone_name':   drone_name,
            'use_rviz':     use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ------------------------------------------------------------------
    # Step 2 – SLAM pipeline  (t = 5 s)
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
            LogInfo(msg='[demo_existing] ── Step 2: Starting SLAM pipeline ──'),
            slam_simple_launch,
            slam_fastlio_launch,
            slam_rtabmap_launch,
        ],
    )

    # ------------------------------------------------------------------
    # Step 3 – Exploration planner  (t = 8 s)
    # ------------------------------------------------------------------
    exploration_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[demo_existing] ── Step 3: Starting exploration planner ──'),
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
            LogInfo(msg='[demo_existing] ═══════════════════════════════════════════════'),
            LogInfo(msg='[demo_existing]   Cave Drone Demo – Pre-generated Cave World   '),
            LogInfo(msg='[demo_existing] ═══════════════════════════════════════════════'),
            LogInfo(msg='[demo_existing] ── Step 1: Starting Gazebo + drone bringup ──'),
            drone_sim_launch,
            slam_group,
            exploration_launch,
        ]
    )
