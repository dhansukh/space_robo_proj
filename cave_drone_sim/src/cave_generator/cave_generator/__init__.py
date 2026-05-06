"""
cave_generator
==============

Procedural 3D cave environment generator for ROS 2 / Gazebo Harmonic drone simulation.

Pipeline:
    1. Graph topology (CaveGraph)   - randomized Prim's MST + loop edges
    2. Mesh generation (CaveMeshGenerator) - SDF carving + Simplex noise + marching cubes
    3. Obstacle placement (ObstaclePlacer) - stalactites, rubble
    4. Gazebo export (GazeboExporter) - model.sdf, model.config, world file

Quick usage::

    from cave_generator.cave_graph import CaveGraph
    from cave_generator.cave_mesh import CaveMeshGenerator
    from cave_generator.obstacle_placer import ObstaclePlacer
    from cave_generator.gazebo_exporter import GazeboExporter

    graph = CaveGraph(seed=42)
    graph.generate_topology(num_nodes=20, bounds_3d=(80, 80, 25))
    gen = CaveMeshGenerator(graph, resolution=0.3, seed=42)
    visual_mesh, collision_mesh = gen.generate()
    exporter = GazeboExporter()
    exporter.export_cave_model(visual_mesh, collision_mesh, "./output/cave_model_001")
"""

from cave_generator.cave_graph import CaveGraph
from cave_generator.cave_mesh import CaveMeshGenerator
from cave_generator.obstacle_placer import ObstaclePlacer
from cave_generator.gazebo_exporter import GazeboExporter

__all__ = [
    "CaveGraph",
    "CaveMeshGenerator",
    "ObstaclePlacer",
    "GazeboExporter",
]
