#!/usr/bin/env python3
"""
slam_test.launch.py
====================
SLAM validation launch – no autonomous exploration.

Starts Gazebo, the drone, and the selected SLAM backend, then leaves you
in control.  Drive the drone manually with teleop_twist_keyboard (see the
helper script scripts/teleop_helper.sh) and verify that the occupancy map
and odometry look correct before running the full autonomy stack.

Launch sequence
---------------
  t=0s   Gazebo + drone bringup
  t=5s   SLAM pipeline

No exploration planner is started.  The drone sits idle until you publish
velocity commands to /cmd_vel.

Usage
-----
  # In terminal 1 – start the SLAM test:
  ros2 launch cave_drone_sim slam_test.launch.py

  # In terminal 2 – drive the drone manually:
  bash scripts/teleop_helper.sh

  # Or launch teleop directly:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r cmd_vel:=/cmd_vel

Launch arguments
----------------
  world_file    : path to .sdf world (default: drone_bringup empty.sdf)
  slam_backend  : simple | fastlio | rtabmap           (default: simple)
  use_rviz      : true | false                         (default: true)
  use_sim_time  : true | false                         (default: true)
  log_level     : debug | info | warn | error          (default: info)
  output_dir    : directory for map saves              (default: /tmp/slam_maps)
"""

from __future__ import annotations

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
    """Build and return the SLAM-test LaunchDescription."""

    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    drone_bringup_share = FindPackageShare('drone_bringup')
    slam_pipeline_share = FindPackageShare('slam_pipeline')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Path to a .sdf world file. Defaults to the empty world.',
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
    world_file   = LaunchConfiguration('world_file')
    slam_backend = LaunchConfiguration('slam_backend')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level    = LaunchConfiguration('log_level')
    output_dir   = LaunchConfiguration('output_dir')

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
            LogInfo(msg='[slam_test] ── Step 2: Starting SLAM pipeline ──'),
            slam_simple_launch,
            slam_fastlio_launch,
            slam_rtabmap_launch,
        ],
    )

    # ------------------------------------------------------------------
    # Assemble LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        args
        + [
            LogInfo(msg='[slam_test] ══════════════════════════════════════════'),
            LogInfo(msg='[slam_test]   SLAM Validation Mode – Manual Teleop    '),
            LogInfo(msg='[slam_test] ══════════════════════════════════════════'),
            LogInfo(msg='[slam_test]  Tip: run  bash scripts/teleop_helper.sh  '),
            LogInfo(msg='[slam_test]       in a second terminal to drive drone '),
            LogInfo(msg='[slam_test] ══════════════════════════════════════════'),
            LogInfo(msg='[slam_test] ── Step 1: Starting Gazebo + drone bringup ──'),
            drone_sim_launch,
            slam_group,
            LogInfo(msg='[slam_test] No exploration planner – drive manually'),
        ]
    )
