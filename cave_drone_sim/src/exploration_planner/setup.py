from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exploration_planner'

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
    maintainer='Thesis Author',
    maintainer_email='drone@thesis.local',
    description='Autonomous 3D exploration and navigation planner for cave drone missions.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = exploration_planner.frontier_explorer:main',
            'path_planner      = exploration_planner.path_planner:main',
            'local_planner     = exploration_planner.local_planner:main',
            'mission_manager   = exploration_planner.mission_manager:main',
            'collision_checker = exploration_planner.collision_checker:main',
        ],
    },
)
