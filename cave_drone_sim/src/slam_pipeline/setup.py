from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cave Drone Team',
    maintainer_email='drone@cave-exploration.local',
    description='ROS 2 SLAM pipeline for autonomous drone cave exploration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_lidar_odom = slam_pipeline.simple_lidar_odom:main',
            'octomap_builder    = slam_pipeline.octomap_builder:main',
            'degeneracy_detector = slam_pipeline.degeneracy_detector:main',
            'map_saver          = slam_pipeline.map_saver:main',
        ],
    },
)
