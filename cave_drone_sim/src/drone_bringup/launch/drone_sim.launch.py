"""
drone_sim.launch.py
===================
Complete bringup launch file for simulating the cave exploration drone in
Gazebo Harmonic.

What this launches
------------------
1. Gazebo Harmonic (gz sim) with a user-specified world file.
2. robot_state_publisher  — publishes the URDF and static joint states.
3. drone_bringup/tf_broadcaster  — sensor-frame static TF.
4. drone_bringup/drone_controller  — velocity PID + odometry.
5. ros_gz_bridge  — bridges all sensor topics between Gazebo and ROS 2.
6. Drone spawner  — spawns the URDF model into the running Gazebo world.
7. (Optional) RViz2  — pre-configured visualisation.

Launch arguments
----------------
world_file    : path to .sdf world file          (default: empty.sdf)
drone_name    : Gazebo model name                (default: cave_drone)
x, y, z       : spawn position                   (default: 0 0 0.5)
roll, pitch, yaw : spawn orientation in radians  (default: 0 0 0)
use_rviz      : 'true' / 'false'                 (default: true)
use_sim_time  : 'true' / 'false'                 (default: true)
params_file   : path to drone_params.yaml        (default: package default)

Usage
-----
    ros2 launch drone_bringup drone_sim.launch.py world_file:=/path/to/cave.sdf
    ros2 launch drone_bringup drone_sim.launch.py use_rviz:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build and return the complete simulation LaunchDescription."""

    pkg_share = get_package_share_directory('drone_bringup')

    # ------------------------------------------------------------------
    # Default paths
    # ------------------------------------------------------------------
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    default_urdf = os.path.join(pkg_share, 'urdf', 'cave_drone.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    default_params = os.path.join(pkg_share, 'config', 'drone_params.yaml')
    default_bridge = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    models_dir = os.path.join(pkg_share, 'models')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'world_file',
            default_value=default_world,
            description='Path to the Gazebo world SDF file',
        ),
        DeclareLaunchArgument(
            'drone_name',
            default_value='cave_drone',
            description='Name of the drone model in Gazebo',
        ),
        DeclareLaunchArgument('x', default_value='0.0',
                              description='Spawn X position (m)'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='Spawn Y position (m)'),
        DeclareLaunchArgument('z', default_value='0.5',
                              description='Spawn Z position (m)'),
        DeclareLaunchArgument('roll',  default_value='0.0',
                              description='Spawn roll  (rad)'),
        DeclareLaunchArgument('pitch', default_value='0.0',
                              description='Spawn pitch (rad)'),
        DeclareLaunchArgument('yaw',   default_value='0.0',
                              description='Spawn yaw   (rad)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 visualisation',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to drone_params.yaml',
        ),
    ]

    # LaunchConfiguration references
    world_file    = LaunchConfiguration('world_file')
    drone_name    = LaunchConfiguration('drone_name')
    spawn_x       = LaunchConfiguration('x')
    spawn_y       = LaunchConfiguration('y')
    spawn_z       = LaunchConfiguration('z')
    spawn_roll    = LaunchConfiguration('roll')
    spawn_pitch   = LaunchConfiguration('pitch')
    spawn_yaw     = LaunchConfiguration('yaw')
    use_rviz      = LaunchConfiguration('use_rviz')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    params_file   = LaunchConfiguration('params_file')

    # ------------------------------------------------------------------
    # Make Gazebo aware of our model directory
    # ------------------------------------------------------------------
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir,
    )

    # ------------------------------------------------------------------
    # 1. Gazebo Harmonic
    # ------------------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': models_dir},
    )

    # ------------------------------------------------------------------
    # 2. robot_state_publisher  (converts xacro → URDF, publishes /tf)
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
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. Static TF broadcaster (sensor frames)
    # ------------------------------------------------------------------
    tf_broadcaster = Node(
        package='drone_bringup',
        executable='tf_broadcaster',
        name='drone_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 4. Drone velocity controller
    # ------------------------------------------------------------------
    drone_controller = Node(
        package='drone_bringup',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 5. ros_gz_bridge  (sensor topics + clock + cmd_vel)
    # ------------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
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
    # 6. Spawn drone into Gazebo
    #    Delayed by 3 s to give Gazebo time to load the world.
    # ------------------------------------------------------------------
    spawn_drone = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_cave_drone',
                output='screen',
                arguments=[
                    '-world', 'cave_world',
                    '-name',  drone_name,
                    '-topic', '/robot_description',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-R', spawn_roll,
                    '-P', spawn_pitch,
                    '-Y', spawn_yaw,
                ],
            )
        ],
    )

    # ------------------------------------------------------------------
    # 7. (Optional) RViz2
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
    # Assemble LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        args
        + [
            set_gz_model_path,
            gazebo,
            robot_state_publisher,
            tf_broadcaster,
            drone_controller,
            ros_gz_bridge,
            spawn_drone,
            rviz,
        ]
    )
