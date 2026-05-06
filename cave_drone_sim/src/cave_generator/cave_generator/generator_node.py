"""
generator_node.py
=================

ROS 2 node ``cave_generator_node``:

- Generates a cave on startup (configurable via parameters).
- Listens on ``/generate_cave`` (std_msgs/String, JSON config) to regenerate.
- Publishes the cave graph to ``/cave_graph`` (visualization_msgs/MarkerArray)
  for RViz inspection.
- Publishes a mesh resource marker to ``/cave_mesh``
  (visualization_msgs/Marker).

Parameters (ROS 2 declare_parameter)
--------------------------------------
num_nodes          : int   = 20
bounds_x           : float = 80.0
bounds_y           : float = 80.0
bounds_z           : float = 25.0
resolution         : float = 0.3
seed               : int   = 42
noise_amplitude    : float = 0.25
noise_frequency    : float = 0.08
num_extra_loops    : int   = 3
stalactite_density : float = 0.3
rubble_density     : float = 0.2
output_dir         : str   = "/tmp/cave_generator_output"
export_gazebo      : bool  = True
"""

from __future__ import annotations

import json
import logging
import os
import threading
from typing import Any, Dict, Optional

# ROS 2 imports — these will fail if rclpy is not installed.
# The rest of the package is still usable without ROS.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS_AVAILABLE = False
    # Provide stub so the module can at least be imported outside ROS
    class Node:  # type: ignore
        pass

from cave_generator.cave_graph import CaveGraph
from cave_generator.cave_mesh import CaveMeshGenerator
from cave_generator.obstacle_placer import ObstaclePlacer
from cave_generator.gazebo_exporter import GazeboExporter

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Colour constants for RViz markers
# ---------------------------------------------------------------------------
_COLOUR_NODE = (0.2, 0.8, 1.0, 0.9)    # RGBA: cyan
_COLOUR_EDGE = (0.9, 0.6, 0.2, 0.8)    # orange
_COLOUR_DEAD = (1.0, 0.2, 0.2, 0.9)    # red


class CaveGeneratorNode(Node):
    """
    ROS 2 lifecycle-compatible cave generator node.

    On startup it runs the full cave generation pipeline and publishes
    visualisation markers to RViz.  It also exposes a string subscriber
    that accepts JSON reconfiguration at runtime.
    """

    def __init__(self) -> None:
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "rclpy is not available.  Install ROS 2 before running the node."
            )
        super().__init__("cave_generator_node")

        # ---- Declare parameters ----
        self.declare_parameter("num_nodes", 20)
        self.declare_parameter("bounds_x", 80.0)
        self.declare_parameter("bounds_y", 80.0)
        self.declare_parameter("bounds_z", 25.0)
        self.declare_parameter("resolution", 0.3)
        self.declare_parameter("seed", 42)
        self.declare_parameter("noise_amplitude", 0.25)
        self.declare_parameter("noise_frequency", 0.08)
        self.declare_parameter("num_extra_loops", 3)
        self.declare_parameter("stalactite_density", 0.3)
        self.declare_parameter("rubble_density", 0.2)
        self.declare_parameter("output_dir", "/tmp/cave_generator_output")
        self.declare_parameter("export_gazebo", True)

        # ---- Publishers ----
        self._graph_pub = self.create_publisher(
            MarkerArray, "/cave_graph", qos_profile=10
        )
        self._mesh_pub = self.create_publisher(
            Marker, "/cave_mesh", qos_profile=10
        )

        # ---- Subscriber ----
        self._gen_sub = self.create_subscription(
            String,
            "/generate_cave",
            self._on_generate_request,
            qos_profile=10,
        )

        # ---- Timer: re-publish markers at 1 Hz ----
        self._publish_timer = self.create_timer(1.0, self._publish_markers)

        self._cave_graph: Optional[CaveGraph] = None
        self._output_dir: Optional[str] = None
        self._lock = threading.Lock()

        # Generate on startup
        self.get_logger().info("CaveGeneratorNode initialised — generating cave …")
        self._generate(self._collect_params())

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_generate_request(self, msg: String) -> None:
        """
        Handle a ``/generate_cave`` message.

        The message data should be a JSON object with any subset of the
        parameter keys.  Missing keys fall back to current parameter values.
        """
        try:
            overrides: Dict[str, Any] = json.loads(msg.data) if msg.data.strip() else {}
        except json.JSONDecodeError as exc:
            self.get_logger().error("Invalid JSON on /generate_cave: %s", exc)
            return

        params = self._collect_params()
        params.update(overrides)
        self.get_logger().info(
            "Received /generate_cave request: %s", json.dumps(overrides)
        )
        # Run in a background thread so we don't block the ROS executor
        thread = threading.Thread(target=self._generate, args=(params,), daemon=True)
        thread.start()

    def _publish_markers(self) -> None:
        """Periodically re-publish the cave graph marker array."""
        with self._lock:
            if self._cave_graph is None:
                return
            graph = self._cave_graph
            output_dir = self._output_dir

        marker_array = self._graph_to_marker_array(graph)
        self._graph_pub.publish(marker_array)

        if output_dir is not None:
            mesh_marker = self._make_mesh_marker(output_dir)
            if mesh_marker is not None:
                self._mesh_pub.publish(mesh_marker)

    # ------------------------------------------------------------------
    # Generation pipeline
    # ------------------------------------------------------------------

    def _generate(self, params: Dict[str, Any]) -> None:
        """Run full cave generation pipeline with *params*."""
        self.get_logger().info("Starting cave generation …")

        seed = int(params["seed"])
        num_nodes = int(params["num_nodes"])
        bounds = (
            float(params["bounds_x"]),
            float(params["bounds_y"]),
            float(params["bounds_z"]),
        )
        resolution = float(params["resolution"])
        noise_amplitude = float(params["noise_amplitude"])
        noise_frequency = float(params["noise_frequency"])
        num_extra_loops = int(params["num_extra_loops"])
        stalactite_density = float(params["stalactite_density"])
        rubble_density = float(params["rubble_density"])
        output_dir = str(params["output_dir"])
        export_gazebo = bool(params["export_gazebo"])

        os.makedirs(output_dir, exist_ok=True)

        # 1. Graph
        cave_graph = CaveGraph(seed=seed)
        cave_graph.generate_topology(
            num_nodes=num_nodes,
            bounds_3d=bounds,
            num_extra_loops=num_extra_loops,
        )
        cave_graph.to_json(os.path.join(output_dir, "cave_graph.json"))
        self.get_logger().info("Graph: %s", cave_graph.summary())

        # 2. Mesh
        gen = CaveMeshGenerator(
            cave_graph,
            resolution=resolution,
            seed=seed,
            noise_amplitude=noise_amplitude,
            noise_frequency=noise_frequency,
        )
        visual_mesh, collision_mesh = gen.generate()
        self.get_logger().info(
            "Mesh: %d visual faces, %d collision faces",
            len(visual_mesh.faces),
            len(collision_mesh.faces),
        )

        # Export tunnel centerline splines for SLAM path planning
        centerlines = gen.get_centerlines()
        centerlines_path = os.path.join(output_dir, "centerlines.json")
        with open(centerlines_path, "w") as fh:
            json.dump(centerlines, fh)
        self.get_logger().info(
            "Centerlines: %d edges → %s",
            len(centerlines.get("edges", {})), centerlines_path,
        )

        # 3. Obstacles
        placer = ObstaclePlacer(seed=seed)
        stalactites = placer.place_stalactites(visual_mesh, density=stalactite_density)
        rubble = placer.place_rubble(visual_mesh, density=rubble_density)
        obstacles_dir = os.path.join(output_dir, "obstacles")
        placer.export_obstacles(stalactites, obstacles_dir, prefix="stalactite")
        placer.export_obstacles(rubble, obstacles_dir, prefix="rubble")

        # 4. Gazebo export
        if export_gazebo:
            model_dir = os.path.join(output_dir, "cave_model_001")
            exporter = GazeboExporter()
            exporter.export_cave_model(visual_mesh, collision_mesh, model_dir)

            entrance = cave_graph.get_entrance_node()
            ep = cave_graph.graph.nodes[entrance]["position"]
            spawn_pose = (ep[0], ep[1], ep[2] + 2.0, 0.0, 0.0, 0.0)
            exporter.export_world_file(
                model_dir,
                drone_spawn_pose=spawn_pose,
                output_path=os.path.join(output_dir, "cave_world.sdf"),
            )
            self.get_logger().info("Gazebo files written to: %s", model_dir)

        with self._lock:
            self._cave_graph = cave_graph
            self._output_dir = output_dir if export_gazebo else None

        self.get_logger().info("Cave generation complete.")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _collect_params(self) -> Dict[str, Any]:
        """Read all declared parameters into a plain dict."""
        return {
            "num_nodes": self.get_parameter("num_nodes").get_parameter_value().integer_value,
            "bounds_x": self.get_parameter("bounds_x").get_parameter_value().double_value,
            "bounds_y": self.get_parameter("bounds_y").get_parameter_value().double_value,
            "bounds_z": self.get_parameter("bounds_z").get_parameter_value().double_value,
            "resolution": self.get_parameter("resolution").get_parameter_value().double_value,
            "seed": self.get_parameter("seed").get_parameter_value().integer_value,
            "noise_amplitude": self.get_parameter("noise_amplitude").get_parameter_value().double_value,
            "noise_frequency": self.get_parameter("noise_frequency").get_parameter_value().double_value,
            "num_extra_loops": self.get_parameter("num_extra_loops").get_parameter_value().integer_value,
            "stalactite_density": self.get_parameter("stalactite_density").get_parameter_value().double_value,
            "rubble_density": self.get_parameter("rubble_density").get_parameter_value().double_value,
            "output_dir": self.get_parameter("output_dir").get_parameter_value().string_value,
            "export_gazebo": self.get_parameter("export_gazebo").get_parameter_value().bool_value,
        }

    # ------------------------------------------------------------------
    # Marker construction
    # ------------------------------------------------------------------

    def _graph_to_marker_array(self, graph: CaveGraph) -> MarkerArray:
        """Convert a CaveGraph to a MarkerArray for RViz."""
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = "map"

        # Node spheres
        for node_id in graph.graph.nodes:
            props = graph.graph.nodes[node_id]
            pos = props["position"]
            is_dead = props["is_dead_end"]
            r_col, g_col, b_col, a_col = _COLOUR_DEAD if is_dead else _COLOUR_NODE

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "cave_nodes"
            marker.id = node_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            r = props["radius"]
            marker.scale.x = marker.scale.y = marker.scale.z = float(r)
            marker.color.r = r_col
            marker.color.g = g_col
            marker.color.b = b_col
            marker.color.a = a_col
            array.markers.append(marker)

        # Edge lines
        edge_marker = Marker()
        edge_marker.header.frame_id = frame_id
        edge_marker.header.stamp = stamp
        edge_marker.ns = "cave_edges"
        edge_marker.id = 0
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.3
        edge_marker.color.r = _COLOUR_EDGE[0]
        edge_marker.color.g = _COLOUR_EDGE[1]
        edge_marker.color.b = _COLOUR_EDGE[2]
        edge_marker.color.a = _COLOUR_EDGE[3]
        edge_marker.pose.orientation.w = 1.0

        for u, v in graph.graph.edges:
            pu = graph.graph.nodes[u]["position"]
            pv = graph.graph.nodes[v]["position"]
            p_start = Point(x=float(pu[0]), y=float(pu[1]), z=float(pu[2]))
            p_end = Point(x=float(pv[0]), y=float(pv[1]), z=float(pv[2]))
            edge_marker.points.append(p_start)
            edge_marker.points.append(p_end)

        array.markers.append(edge_marker)
        return array

    def _make_mesh_marker(self, output_dir: str) -> Optional[Marker]:
        """Build a mesh resource Marker pointing to the exported visual mesh."""
        mesh_path = os.path.join(output_dir, "cave_model_001", "meshes", "cave_visual.obj")
        if not os.path.isfile(mesh_path):
            return None

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cave_mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.45
        marker.color.b = 0.4
        marker.color.a = 0.85
        marker.mesh_resource = f"file://{mesh_path}"
        marker.mesh_use_embedded_materials = False
        return marker


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """ROS 2 node entry point."""
    if not _ROS_AVAILABLE:
        print("rclpy not available.  Cannot start cave_generator_node.")
        return
    rclpy.init(args=args)
    node = CaveGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
