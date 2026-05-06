#!/usr/bin/env python3
"""
slam_rtabmap.launch.py
======================
Launch file for RTAB-Map in LiDAR-only SLAM mode with loop closure,
optimised for cave exploration.

RTAB-Map provides:
  /rtabmap/odom          → remapped to /slam/odom
  /rtabmap/cloud_map     → remapped to /slam/map_cloud

We then launch:
  octomap_builder     — 3-D occupancy mapping from RTAB-Map output
  degeneracy_detector — geometry health monitor
  map_saver           — on-demand map persistence

Usage
-----
  # With full GUI:
  ros2 launch slam_pipeline slam_rtabmap.launch.py

  # Headless:
  ros2 launch slam_pipeline slam_rtabmap.launch.py \
      rtabmap_viz:=false

Prerequisites
-------------
  sudo apt install ros-$ROS_DISTRO-rtabmap-ros

References
----------
  http://wiki.ros.org/rtabmap_ros
  https://github.com/introlab/rtabmap_ros
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare('slam_pipeline')

    # ── Launch arguments ──────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
        description='Pipeline parameter file',
    )
    rtabmap_params_arg = DeclareLaunchArgument(
        'rtabmap_params',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rtabmap_cave.yaml']),
        description='RTAB-Map parameter file',
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
    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map visualizer',
    )
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='/tmp/rtabmap.db',
        description='Path to RTAB-Map database file',
    )

    params_file    = LaunchConfiguration('params_file')
    rtabmap_params = LaunchConfiguration('rtabmap_params')
    output_dir     = LaunchConfiguration('output_dir')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    rtabmap_viz    = LaunchConfiguration('rtabmap_viz')
    database_path  = LaunchConfiguration('database_path')

    # ── RTAB-Map ICP odometry node ────────────────────────────────────────
    # Provides wheel/IMU-free odometry from LiDAR scans
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id':         'base_link',
            'odom_frame_id':    'odom',
            'publish_tf':       True,
            # ICP parameters tuned for cave LiDAR
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '30',
            'Icp/VoxelSize':                 '0.1',
            'Icp/MaxTranslation':            '2.0',
            'Icp/MaxRotation':               '0.78',   # ~45 deg per scan
            'OdomF2M/MaxSize':               '10000',
            'OdomF2M/BundleAdjustment':      '0',
            'approx_sync':                   True,
        }],
        remappings=[
            ('scan_cloud', '/drone/lidar/points'),
            ('odom',       '/slam/icp_odom'),
        ],
    )

    # ── RTAB-Map SLAM node ────────────────────────────────────────────────
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {
                'use_sim_time':         use_sim_time,
                'database_path':        database_path,
                'frame_id':             'base_link',
                'odom_frame_id':        'odom',
                'map_frame_id':         'map',
                'subscribe_scan_cloud': True,
                'subscribe_depth':      False,
                'subscribe_rgb':        False,
                'subscribe_stereo':     False,

                # Loop closure settings for cave environment
                # Large search radius since caves have repeated geometry
                'Mem/STMSize':                   '30',
                'Mem/NotLinkedNodesKept':        'true',
                'Rtabmap/TimeThr':               '0',     # no time limit
                'Rtabmap/DetectionRate':         '1',
                'Rtabmap/LoopThr':               '0.11',

                # ICP loop closure refinement
                'Reg/Strategy':                  '1',     # 1 = ICP
                'Icp/MaxCorrespondenceDistance': '1.5',
                'Icp/PointToPlane':              'true',
                'Icp/Iterations':                '30',
                'Icp/VoxelSize':                 '0.1',

                # 3-D map settings
                'Grid/3D':                       'true',
                'Grid/CellSize':                 '0.2',
                'Grid/RangeMax':                 '30.0',
                'Grid/RayTracing':               'true',

                # Visual vocabulary (disabled for LiDAR-only)
                'Kp/MaxFeatures':                '-1',
                'Vis/MaxFeatures':               '0',

                # Robust pose graph optimisation
                'Optimizer/Strategy':            '1',     # g2o
                'Optimizer/Robust':              'true',
                'Optimizer/Epsilon':             '0.0001',
            },
        ],
        remappings=[
            # Input
            ('scan_cloud', '/drone/lidar/points'),
            ('odom',       '/slam/icp_odom'),
            # Output → our convention
            ('/rtabmap/odom',      '/slam/odom'),
            ('/rtabmap/cloud_map', '/slam/map_cloud'),
        ],
        arguments=['--delete_db_on_start'],
    )

    # ── RTAB-Map visualizer (optional) ────────────────────────────────────
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(rtabmap_viz),
        parameters=[{
            'use_sim_time':         use_sim_time,
            'frame_id':             'base_link',
            'subscribe_scan_cloud': True,
            'odom_frame_id':        'odom',
        }],
        remappings=[
            ('scan_cloud',         '/drone/lidar/points'),
            ('/rtabmap/odom',      '/slam/odom'),
            ('/rtabmap/cloud_map', '/slam/map_cloud'),
        ],
    )

    # ── Downstream pipeline nodes ─────────────────────────────────────────

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
                'output_dir':   output_dir,
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        rtabmap_params_arg,
        output_dir_arg,
        use_sim_time_arg,
        rtabmap_viz_arg,
        database_path_arg,
        LogInfo(msg='[slam_rtabmap] Launching RTAB-Map LiDAR SLAM pipeline...'),
        icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
        octomap_builder_node,
        degeneracy_detector_node,
        map_saver_node,
        LogInfo(msg='[slam_rtabmap] All nodes launched.'),
    ])
