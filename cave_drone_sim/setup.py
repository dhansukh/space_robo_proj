"""
cave_drone_sim – top-level integration package
================================================
This package provides the master launch files that tie together the four
sub-packages:
  • cave_generator       – procedural cave environment
  • drone_bringup        – Gazebo simulation + drone
  • slam_pipeline        – LiDAR SLAM (simple / FAST-LIO2 / RTAB-Map)
  • exploration_planner  – autonomous frontier exploration

Build with:
    colcon build --symlink-install

After building, launch with:
    source install/setup.bash
    ros2 launch cave_drone_sim full_demo.launch.py
"""

import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'cave_drone_sim'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        # Register with ament index so ros2 can find this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        # Install package.xml
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Install all launch files
        (os.path.join('share', PACKAGE_NAME, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thesis Author',
    maintainer_email='thesis@example.com',
    description='Integration launch package for the Cave Drone Simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
