"""
setup.py
========
ament_python build configuration for drone_bringup.

Data files (URDF, SDF, launch files, config, models) are installed into
the share directory so they are accessible at runtime via
``ament_index_python.packages.get_package_share_directory('drone_bringup')``.
"""

import os
from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'drone_bringup'


def _glob_data(source_glob: str, install_dir: str) -> tuple[str, list[str]]:
    """Return a data_files entry tuple (install_dir, [matched_files])."""
    return (install_dir, glob(source_glob))


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index entry
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),

        # package.xml
        ('share/' + PACKAGE_NAME, ['package.xml']),

        # Launch files
        _glob_data('launch/*.py',
                   os.path.join('share', PACKAGE_NAME, 'launch')),

        # URDF / xacro
        _glob_data('urdf/*.xacro',
                   os.path.join('share', PACKAGE_NAME, 'urdf')),
        _glob_data('urdf/*.urdf',
                   os.path.join('share', PACKAGE_NAME, 'urdf')),

        # Gazebo models
        _glob_data('models/cave_drone/*',
                   os.path.join('share', PACKAGE_NAME, 'models', 'cave_drone')),

        # Config files
        _glob_data('config/*.yaml',
                   os.path.join('share', PACKAGE_NAME, 'config')),
        _glob_data('config/*.rviz',
                   os.path.join('share', PACKAGE_NAME, 'config')),

        # World files
        _glob_data('worlds/*.sdf',
                   os.path.join('share', PACKAGE_NAME, 'worlds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_bringup',
    maintainer_email='thesis@cave-exploration.sim',
    description=(
        'Bringup package for an autonomous cave-exploration drone '
        'simulated in Gazebo Harmonic.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node executables registered by ament
            'drone_controller  = drone_bringup.drone_controller:main',
            'tf_broadcaster    = drone_bringup.tf_broadcaster:main',
            'depth_colorizer   = drone_bringup.depth_colorizer:main',
            'px4_vel_bridge    = drone_bringup.px4_vel_bridge:main',
        ],
    },
)
