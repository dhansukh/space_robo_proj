from setuptools import setup, find_packages
import os
from glob import glob

package_name = "cave_generator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thesis Author",
    maintainer_email="thesis@example.com",
    description="Procedural 3D cave generator for drone simulation in Gazebo Harmonic.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cave_generator_node = cave_generator.generator_node:main",
        ],
    },
)
