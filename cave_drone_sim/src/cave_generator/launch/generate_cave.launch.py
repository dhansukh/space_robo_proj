"""
generate_cave.launch.py
========================

ROS 2 launch file for the cave_generator_node.

Usage
-----
::

    ros2 launch cave_generator generate_cave.launch.py
    ros2 launch cave_generator generate_cave.launch.py seed:=123 num_nodes:=30
    ros2 launch cave_generator generate_cave.launch.py config_file:=/path/to/cave.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the LaunchDescription for the cave_generator_node."""

    # ---- Default config file path ----
    pkg_share = get_package_share_directory("cave_generator")
    default_config = os.path.join(pkg_share, "config", "default_cave.yaml")

    # ---- Launch arguments ----
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the cave generator YAML config file.",
    )
    declare_seed = DeclareLaunchArgument(
        "seed",
        default_value="42",
        description="Random seed for deterministic cave generation.",
    )
    declare_num_nodes = DeclareLaunchArgument(
        "num_nodes",
        default_value="20",
        description="Number of cave chambers/junctions.",
    )
    declare_resolution = DeclareLaunchArgument(
        "resolution",
        default_value="0.3",
        description="Voxel grid resolution in metres.",
    )
    declare_output_dir = DeclareLaunchArgument(
        "output_dir",
        default_value="/tmp/cave_generator_output",
        description="Output directory for generated files.",
    )
    declare_export_gazebo = DeclareLaunchArgument(
        "export_gazebo",
        default_value="true",
        description="Whether to export Gazebo model and world files.",
    )
    declare_num_loops = DeclareLaunchArgument(
        "num_extra_loops",
        default_value="3",
        description="Number of extra loop edges for SLAM loop-closure testing.",
    )

    # ---- Node ----
    cave_node = Node(
        package="cave_generator",
        executable="cave_generator_node",
        name="cave_generator_node",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "seed": LaunchConfiguration("seed"),
                "num_nodes": LaunchConfiguration("num_nodes"),
                "resolution": LaunchConfiguration("resolution"),
                "output_dir": LaunchConfiguration("output_dir"),
                "export_gazebo": LaunchConfiguration("export_gazebo"),
                "num_extra_loops": LaunchConfiguration("num_extra_loops"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_config_file,
        declare_seed,
        declare_num_nodes,
        declare_resolution,
        declare_output_dir,
        declare_export_gazebo,
        declare_num_loops,
        cave_node,
    ])
