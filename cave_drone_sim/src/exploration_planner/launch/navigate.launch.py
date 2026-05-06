"""
navigate.launch.py
==================
Goal-directed navigation launch file (no frontier exploration).

Launches:
  1. collision_checker  — shared KD-tree obstacle map
  2. path_planner       — RRT* / A* global planning
  3. local_planner      — APF local obstacle avoidance
  4. mission_manager    — state machine starting in NAVIGATING mode

The frontier_explorer node is intentionally omitted.  Send a goal via:
  ros2 topic pub /mission/goal geometry_msgs/msg/PoseStamped ...

Usage
-----
  ros2 launch exploration_planner navigate.launch.py
  ros2 launch exploration_planner navigate.launch.py \
      goal_x:=10.0 goal_y:=5.0 goal_z:=2.0
  ros2 launch exploration_planner navigate.launch.py \
      params_file:=/path/to/custom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    # Optional pre-set goal position
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='',
                                        description='Goal X (optional)')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='',
                                        description='Goal Y (optional)')
    goal_z_arg = DeclareLaunchArgument('goal_z', default_value='2.0',
                                        description='Goal Z altitude')
    return_home_arg = DeclareLaunchArgument(
        'return_home',
        default_value='true',
        description='Return to home after reaching goal',
    )

    params_file   = LaunchConfiguration('params_file')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    log_level     = LaunchConfiguration('log_level')

    # ------------------------------------------------------------------
    # Navigation-only nodes
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
    )

    # Mission manager starts directly in NAVIGATING state (no exploration)
    mission_manager_node = Node(
        package='exploration_planner',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time':  use_sim_time,
                'initial_state': 'IDLE',   # still IDLE; TAKEOFF triggered by /mission/start
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        log_level_arg,
        goal_x_arg,
        goal_y_arg,
        goal_z_arg,
        return_home_arg,

        LogInfo(msg='[navigate.launch] Starting navigation-only stack '
                    '(frontier explorer disabled)...'),

        collision_checker_node,
        path_planner_node,
        local_planner_node,
        mission_manager_node,

        LogInfo(msg='[navigate.launch] Nodes launched. '
                    'Send /mission/start (Empty) to begin takeoff, then '
                    '/mission/goal (PoseStamped) to navigate.'),
    ])
