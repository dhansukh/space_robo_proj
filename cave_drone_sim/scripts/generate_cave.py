#!/usr/bin/env python3
"""
generate_cave.py
================

Standalone CLI script for procedural cave generation.  **No ROS required.**

Runs the full pipeline:
  1. Generate cave graph topology
  2. Build 3-D mesh (SDF + marching cubes)
  3. Place stalactites and rubble
  4. Export Gazebo model (model.sdf, model.config) and world file
  5. Save debug visualisation images (if matplotlib is available)

Usage
-----
::

    python3 generate_cave.py --nodes 25 --bounds 80 80 25 --seed 42 \\
        --output ./output/cave_001

    python3 generate_cave.py --help

All output is written to the directory given by ``--output``.

Output layout
-------------
::

    output/cave_001/
        cave_graph.json          ← graph topology (nodes + edges)
        cave_model_001/
            model.config
            model.sdf
            meshes/
                cave_visual.obj
                cave_collision.stl
        cave_world.sdf
        obstacles/
            stalactite_0000.stl
            ...
            rubble_0000.stl
            ...
        debug/
            graph.png
            splines.png
            sdf_slice.png
            composite.png
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from typing import Tuple

# Add the package to path when running without installation
_script_dir = os.path.dirname(os.path.abspath(__file__))
_pkg_src = os.path.join(_script_dir, "..", "src", "cave_generator")
if os.path.isdir(_pkg_src):
    sys.path.insert(0, _pkg_src)

from cave_generator.cave_graph import CaveGraph
from cave_generator.cave_mesh import CaveMeshGenerator
from cave_generator.obstacle_placer import ObstaclePlacer
from cave_generator.gazebo_exporter import GazeboExporter

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("generate_cave")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="generate_cave",
        description=(
            "Procedural 3-D cave generator for Gazebo Harmonic drone simulation.\n"
            "Outputs a Gazebo model directory and world SDF file."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Topology
    topo = p.add_argument_group("Cave topology")
    topo.add_argument(
        "--nodes", "-n", type=int, default=20, metavar="N",
        help="Number of cave chambers/junctions (default: 20).",
    )
    topo.add_argument(
        "--bounds", nargs=3, type=float, default=[80.0, 80.0, 5.0],
        metavar=("BX", "BY", "BZ"),
        help="Bounding box in metres: width_x width_y tunnel_height_z (default: 80 80 5).",
    )
    topo.add_argument(
        "--loops", type=int, default=3, metavar="K",
        help="Extra loop edges added on top of the MST (default: 3).",
    )
    topo.add_argument(
        "--min-edge", type=float, default=8.0, metavar="M",
        help="Minimum edge length in metres (default: 8.0).",
    )
    topo.add_argument(
        "--max-edge", type=float, default=30.0, metavar="M",
        help="Maximum edge length in metres (default: 30.0).",
    )

    # Mesh
    mesh = p.add_argument_group("Mesh generation")
    mesh.add_argument(
        "--resolution", "-r", type=float, default=0.3, metavar="RES",
        help="Voxel grid resolution in metres (default: 0.3).",
    )
    mesh.add_argument(
        "--noise-amplitude", type=float, default=0.25, metavar="A",
        help="Simplex noise amplitude as fraction of min tunnel radius (default: 0.25).",
    )
    mesh.add_argument(
        "--noise-frequency", type=float, default=0.08, metavar="F",
        help="Simplex noise spatial frequency (default: 0.08).",
    )
    mesh.add_argument(
        "--collision-faces", type=int, default=8000, metavar="CF",
        help="Target face count for decimated collision mesh (default: 8000).",
    )

    # Obstacles
    obs = p.add_argument_group("Obstacle placement")
    obs.add_argument(
        "--stalactite-density", type=float, default=0.3, metavar="D",
        help="Stalactite coverage density [0-1] (default: 0.3).",
    )
    obs.add_argument(
        "--rubble-density", type=float, default=0.2, metavar="D",
        help="Rubble coverage density [0-1] (default: 0.2).",
    )
    obs.add_argument(
        "--no-obstacles", action="store_true",
        help="Skip obstacle placement.",
    )

    # Output
    out = p.add_argument_group("Output")
    out.add_argument(
        "--output", "-o", type=str, default="./output/cave_001", metavar="DIR",
        help="Output directory (default: ./output/cave_001).",
    )
    out.add_argument(
        "--model-name", type=str, default="cave_model_001", metavar="NAME",
        help="Gazebo model directory name (default: cave_model_001).",
    )
    out.add_argument(
        "--no-gazebo", action="store_true",
        help="Skip Gazebo export.",
    )
    out.add_argument(
        "--visualize", action="store_true",
        help="Generate debug visualisation images (requires matplotlib).",
    )

    # Misc
    misc = p.add_argument_group("Misc")
    misc.add_argument(
        "--seed", "-s", type=int, default=42, metavar="SEED",
        help="Random seed for deterministic generation (default: 42).",
    )
    misc.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable DEBUG logging.",
    )

    return p


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    """
    Execute the full cave generation pipeline.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed CLI arguments.

    Returns
    -------
    int
        Exit code (0 = success).
    """
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    output_dir = os.path.abspath(args.output)
    os.makedirs(output_dir, exist_ok=True)

    bounds: Tuple[float, float, float] = (
        float(args.bounds[0]),
        float(args.bounds[1]),
        float(args.bounds[2]),
    )

    t_total = time.time()

    # -------------------------------------------------------------------------
    # 1. Graph topology
    # -------------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 1/5  Cave graph topology")
    logger.info("=" * 60)
    t0 = time.time()

    cave_graph = CaveGraph(seed=args.seed)
    cave_graph.generate_topology(
        num_nodes=args.nodes,
        bounds_3d=bounds,
        min_edge_length=args.min_edge,
        max_edge_length=args.max_edge,
        num_extra_loops=args.loops,
    )
    graph_json_path = os.path.join(output_dir, "cave_graph.json")
    cave_graph.to_json(graph_json_path)

    logger.info("%s", cave_graph.summary())
    logger.info("Graph JSON written to: %s", graph_json_path)
    logger.info("Step 1 done in %.1f s", time.time() - t0)

    # -------------------------------------------------------------------------
    # 2. Mesh generation
    # -------------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 2/5  Mesh generation (SDF + marching cubes)")
    logger.info("=" * 60)
    t0 = time.time()

    gen = CaveMeshGenerator(
        cave_graph,
        resolution=args.resolution,
        seed=args.seed,
        noise_amplitude=args.noise_amplitude,
        noise_frequency=args.noise_frequency,
        collision_target_faces=args.collision_faces,
    )
    visual_mesh, collision_mesh = gen.generate()

    logger.info(
        "Visual mesh:    %d vertices, %d faces",
        len(visual_mesh.vertices), len(visual_mesh.faces),
    )
    logger.info(
        "Collision mesh: %d vertices, %d faces",
        len(collision_mesh.vertices), len(collision_mesh.faces),
    )

    # Export tunnel centerline splines for SLAM path planning
    import json as _json
    centerlines = gen.get_centerlines()
    centerlines_path = os.path.join(output_dir, "centerlines.json")
    with open(centerlines_path, "w") as _fh:
        _json.dump(centerlines, _fh)
    logger.info(
        "Centerlines: %d edges → %s",
        len(centerlines.get("edges", {})), centerlines_path,
    )

    logger.info("Step 2 done in %.1f s", time.time() - t0)

    # -------------------------------------------------------------------------
    # 3. Obstacle placement
    # -------------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 3/5  Obstacle placement")
    logger.info("=" * 60)
    t0 = time.time()

    if not args.no_obstacles:
        placer = ObstaclePlacer(seed=args.seed)
        stalactites = placer.place_stalactites(
            visual_mesh, density=args.stalactite_density
        )
        rubble = placer.place_rubble(visual_mesh, density=args.rubble_density)

        obstacles_dir = os.path.join(output_dir, "obstacles")
        placer.export_obstacles(stalactites, obstacles_dir, prefix="stalactite")
        placer.export_obstacles(rubble, obstacles_dir, prefix="rubble")

        logger.info(
            "Placed %d stalactites + %d rubble rocks  →  %s",
            len(stalactites), len(rubble), obstacles_dir,
        )
    else:
        logger.info("Obstacle placement skipped (--no-obstacles).")

    logger.info("Step 3 done in %.1f s", time.time() - t0)

    # -------------------------------------------------------------------------
    # 4. Gazebo export
    # -------------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 4/5  Gazebo export")
    logger.info("=" * 60)
    t0 = time.time()

    if not args.no_gazebo:
        model_dir = os.path.join(output_dir, args.model_name)
        exporter = GazeboExporter()
        exporter.export_cave_model(
            visual_mesh, collision_mesh, model_dir, model_name=args.model_name
        )

        # Compute Z-offset so the cave's lowest point sits at the ground plane (z=0)
        mesh_min_z = float(visual_mesh.vertices[:, 2].min())
        cave_z_offset = -mesh_min_z  # lift the cave up

        entrance_node = cave_graph.get_entrance_node()
        ep = cave_graph.graph.nodes[entrance_node]["position"]
        # Spawn drone inside the cave: account for the Z-offset
        spawn_z = 1.0 + cave_z_offset
        spawn_pose = (ep[0], ep[1], spawn_z, 0.0, 0.0, 0.0)

        world_path = os.path.join(output_dir, "cave_world.sdf")
        exporter.export_world_file(
            model_dir,
            drone_spawn_pose=spawn_pose,
            output_path=world_path,
            cave_z_offset=cave_z_offset,
        )

        logger.info("Gazebo model: %s", model_dir)
        logger.info("World SDF:    %s", world_path)
        logger.info("Cave Z-offset: %.2f m (mesh min_z was %.2f)", cave_z_offset, mesh_min_z)
        logger.info(
            "Drone spawn: x=%.1f y=%.1f z=%.1f",
            spawn_pose[0], spawn_pose[1], spawn_pose[2],
        )
    else:
        logger.info("Gazebo export skipped (--no-gazebo).")

    logger.info("Step 4 done in %.1f s", time.time() - t0)

    # -------------------------------------------------------------------------
    # 5. Debug visualisation
    # -------------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 5/5  Debug visualisation")
    logger.info("=" * 60)
    t0 = time.time()

    if args.visualize:
        _generate_visualisations(cave_graph, gen, output_dir)
    else:
        logger.info("Skipping visualisation (pass --visualize to enable).")

    logger.info("Step 5 done in %.1f s", time.time() - t0)

    # -------------------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------------------
    elapsed = time.time() - t_total
    logger.info("=" * 60)
    logger.info("Cave generation complete in %.1f s", elapsed)
    logger.info("Output directory: %s", output_dir)
    logger.info("=" * 60)
    _print_summary(output_dir, args)
    return 0


def _generate_visualisations(
    cave_graph: CaveGraph,
    gen: CaveMeshGenerator,
    output_dir: str,
) -> None:
    """Attempt to generate matplotlib debug images; warn if matplotlib missing."""
    try:
        from cave_generator.visualizer import (
            plot_graph,
            plot_splines,
            plot_sdf_cross_section,
            plot_all,
        )
    except ImportError:
        logger.warning("matplotlib not available; skipping visualisation.")
        return

    debug_dir = os.path.join(output_dir, "debug")
    os.makedirs(debug_dir, exist_ok=True)

    # Graph plot
    fig = plot_graph(cave_graph)
    fig.savefig(os.path.join(debug_dir, "graph.png"), dpi=120, bbox_inches="tight")
    fig_close(fig)

    # Splines
    splines = gen.get_splines()
    fig = plot_splines(cave_graph, splines)
    fig.savefig(os.path.join(debug_dir, "splines.png"), dpi=120, bbox_inches="tight")
    fig_close(fig)

    # SDF slice (only if SDF is available)
    if gen._sdf is not None:
        fig = plot_sdf_cross_section(
            gen._sdf, gen._grid_origin, gen.resolution, axis=2
        )
        fig.savefig(os.path.join(debug_dir, "sdf_slice.png"), dpi=120, bbox_inches="tight")
        fig_close(fig)

        # Composite
        fig = plot_all(
            cave_graph,
            splines,
            gen._sdf,
            gen._grid_origin,
            gen.resolution,
            save_path=os.path.join(debug_dir, "composite.png"),
        )
        fig_close(fig)

    logger.info("Debug images written to: %s", debug_dir)


def fig_close(fig) -> None:  # type: ignore[no-untyped-def]
    """Close a matplotlib figure to free memory."""
    try:
        import matplotlib.pyplot as plt
        plt.close(fig)
    except Exception:
        pass


def _print_summary(output_dir: str, args: argparse.Namespace) -> None:
    """Print a brief summary of all output files."""
    print("\n--- Generated files ---")
    for root, dirs, files in os.walk(output_dir):
        dirs.sort()
        rel = os.path.relpath(root, output_dir)
        prefix = "" if rel == "." else f"{rel}/"
        for f in sorted(files):
            full = os.path.join(root, f)
            size_kb = os.path.getsize(full) / 1024.0
            print(f"  {prefix}{f}  ({size_kb:.1f} KB)")
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
