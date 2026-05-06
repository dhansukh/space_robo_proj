"""
explore.launch.py
=================
Full autonomous exploration launch file.

Launches all five nodes with shared parameters from exploration_params.yaml:
  1. collision_checker  — shared KD-tree obstacle map
  2. frontier_explorer  — frontier detection and goal selection
  3. path_planner       — RRT* / A* global path planning
  4. local_planner      — APF local obstacle avoidance
  5. mission_manager    — mission state machine

Usage
-----
  ros2 launch exploration_planner explore.launch.py
  ros2 launch exploration_planner explore.launch.py params_file:=/path/to/custom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('exploration_planner')
    default_params = os.path.join(pkg_share, 'config', 'exploration_params.yaml')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to exploration parameters YAML file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level: debug, info, warn, error',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    collision_checker_node = Node(
        package='exploration_planner',
        executable='collision_checker',
        name='collision_checker',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/slam/occupied_cells', '/slam/occupied_cells'),
        ],
    )

    frontier_explorer_node = Node(
        package='exploration_planner',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/slam/occupied_cells', '/slam/occupied_cells'),
            ('/slam/odom',           '/slam/odom'),
            ('/slam/octomap',        '/slam/octomap'),
        ],
    )

    path_planner_node = Node(
        package='exploration_planner',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/slam/occupied_cells', '/slam/occupied_cells'),
        ],
    )

    local_planner_node = Node(
        package='exploration_planner',
        executable='local_planner',
        name='local_planner',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/drone/lidar/points', '/drone/lidar/points'),
            ('/planning/path',      '/planning/path'),
            ('/slam/odom',          '/slam/odom'),
        ],
    )

    mission_manager_node = Node(
        package='exploration_planner',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/slam/odom',              '/slam/odom'),
            ('/slam/degeneracy_score',  '/slam/degeneracy_score'),
            ('/exploration/status',     '/exploration/status'),
            ('/local_planner/status',   '/local_planner/status'),
            ('/mission/start',          '/mission/start'),
            ('/mission/goal',           '/mission/goal'),
            ('/drone/battery',          '/drone/battery'),
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        log_level_arg,

        LogInfo(msg='[explore.launch] Starting full exploration stack...'),

        collision_checker_node,
        frontier_explorer_node,
        path_planner_node,
        local_planner_node,
        mission_manager_node,

        LogInfo(msg='[explore.launch] All nodes launched.'),
    ])
