"""
slam_pipeline
=============
ROS 2 SLAM pipeline for autonomous drone cave exploration.

Nodes
-----
- simple_lidar_odom  : Self-contained ICP-based LiDAR odometry (fallback)
- octomap_builder    : 3D occupancy mapping from SLAM output
- degeneracy_detector: Detects geometrically degenerate environments
- map_saver          : Persists maps to disk on-demand
"""
