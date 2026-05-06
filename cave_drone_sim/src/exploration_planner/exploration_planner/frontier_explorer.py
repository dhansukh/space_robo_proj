"""
frontier_explorer.py
====================
Frontier-based 3D exploration planner for cave drone missions.

Architecture
------------
* Maintains a 3-D occupancy grid whose cells are tagged FREE / OCCUPIED / UNKNOWN.
* Detects *frontier voxels*: FREE cells adjacent to at least one UNKNOWN cell.
* Clusters frontier voxels with DBSCAN, producing a compact set of frontier
  regions with centroid and size.
* Scores each region with a composite utility function and publishes the best
  region as a navigation goal.

Subscriptions
-------------
/slam/occupied_cells  (sensor_msgs/PointCloud2)     — occupied voxels
/slam/odom            (nav_msgs/Odometry)            — current pose
/slam/octomap         (visualization_msgs/MarkerArray) — free/unknown info

Publications
------------
/exploration/frontiers  (visualization_msgs/MarkerArray) — frontier spheres
/exploration/goal       (geometry_msgs/PoseStamped)      — best frontier goal
/exploration/status     (std_msgs/String)                — state string
"""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Duration


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

class CellState(Enum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class VoxelGrid:
    """
    Sparse 3-D voxel grid backed by a dictionary.

    Keys are integer (ix, iy, iz) tuples; values are CellState enum members.
    The grid is thread-safe via an internal RLock.
    """

    def __init__(self, resolution: float = 0.3) -> None:
        self.resolution: float = resolution
        self._cells: Dict[Tuple[int, int, int], CellState] = {}
        self._lock: threading.RLock = threading.RLock()

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_index(self, point: np.ndarray) -> Tuple[int, int, int]:
        """Convert world-frame (x, y, z) to grid index tuple."""
        p = np.asarray(point, dtype=np.float64)
        return (
            int(np.floor(p[0] / self.resolution)),
            int(np.floor(p[1] / self.resolution)),
            int(np.floor(p[2] / self.resolution)),
        )

    def index_to_world(self, idx: Tuple[int, int, int]) -> np.ndarray:
        """Return the centre of grid cell *idx* in world frame."""
        r = self.resolution
        return np.array([
            (idx[0] + 0.5) * r,
            (idx[1] + 0.5) * r,
            (idx[2] + 0.5) * r,
        ], dtype=np.float64)

    # ------------------------------------------------------------------
    # Mutation
    # ------------------------------------------------------------------

    def set(self, idx: Tuple[int, int, int], state: CellState) -> None:
        with self._lock:
            self._cells[idx] = state

    def mark_occupied(self, point: np.ndarray) -> None:
        with self._lock:
            self._cells[self.world_to_index(point)] = CellState.OCCUPIED

    def mark_free(self, point: np.ndarray) -> None:
        with self._lock:
            idx = self.world_to_index(point)
            # Don't downgrade OCCUPIED cells
            if self._cells.get(idx) != CellState.OCCUPIED:
                self._cells[idx] = CellState.FREE

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get(self, idx: Tuple[int, int, int]) -> CellState:
        with self._lock:
            return self._cells.get(idx, CellState.UNKNOWN)

    def get_all_by_state(self, state: CellState) -> List[np.ndarray]:
        """Return world-frame centres of all cells in *state*."""
        with self._lock:
            return [
                self.index_to_world(k)
                for k, v in self._cells.items()
                if v == state
            ]

    def get_all_indices_by_state(
        self, state: CellState
    ) -> List[Tuple[int, int, int]]:
        with self._lock:
            return [k for k, v in self._cells.items() if v == state]

    def count(self) -> int:
        with self._lock:
            return len(self._cells)


# ---------------------------------------------------------------------------
# Frontier region
# ---------------------------------------------------------------------------

class FrontierRegion:
    """A cluster of frontier voxels."""

    def __init__(
        self,
        centroid: np.ndarray,
        size: int,
        voxels: np.ndarray,
    ) -> None:
        self.centroid: np.ndarray = centroid   # shape (3,)
        self.size: int = size
        self.voxels: np.ndarray = voxels       # shape (N, 3)
        self.utility: float = 0.0
        self.info_gain: float = 0.0
        self.distance: float = 0.0
        self.heading_alignment: float = 0.0


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class FrontierExplorer(Node):
    """
    ROS 2 node — frontier-based 3-D exploration planner.
    """

    _FACE_OFFSETS: List[Tuple[int, int, int]] = [
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    ]

    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('voxel_resolution', 0.3)
        self.declare_parameter('frontier_min_size', 5)
        self.declare_parameter('utility_weight_info', 1.0)
        self.declare_parameter('utility_weight_distance', 0.5)
        self.declare_parameter('utility_weight_heading', 0.3)
        self.declare_parameter('max_exploration_range', 50.0)
        self.declare_parameter('dbscan_eps', 1.0)
        self.declare_parameter('dbscan_min_samples', 3)
        self.declare_parameter('raytrace_samples', 50)
        self.declare_parameter('publish_rate', 2.0)

        res = self.get_parameter('voxel_resolution').get_parameter_value().double_value
        self._min_size: int = (
            self.get_parameter('frontier_min_size').get_parameter_value().integer_value
        )
        self._w1: float = (
            self.get_parameter('utility_weight_info').get_parameter_value().double_value
        )
        self._w2: float = (
            self.get_parameter('utility_weight_distance').get_parameter_value().double_value
        )
        self._w3: float = (
            self.get_parameter('utility_weight_heading').get_parameter_value().double_value
        )
        self._max_range: float = (
            self.get_parameter('max_exploration_range').get_parameter_value().double_value
        )
        self._dbscan_eps: float = (
            self.get_parameter('dbscan_eps').get_parameter_value().double_value
        )
        self._dbscan_min_samples: int = (
            self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value
        )
        self._raytrace_samples: int = (
            self.get_parameter('raytrace_samples').get_parameter_value().integer_value
        )
        rate: float = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._grid: VoxelGrid = VoxelGrid(resolution=res)
        self._pose: Optional[np.ndarray] = None          # (x, y, z)
        self._heading: Optional[np.ndarray] = None       # unit vector
        self._lock: threading.RLock = threading.RLock()

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self._occ_sub = self.create_subscription(
            PointCloud2, '/slam/occupied_cells',
            self._occupied_cb, sensor_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/slam/odom',
            self._odom_cb, sensor_qos,
        )
        self._octomap_sub = self.create_subscription(
            MarkerArray, '/slam/octomap',
            self._octomap_cb, sensor_qos,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._frontier_pub = self.create_publisher(
            MarkerArray, '/exploration/frontiers', reliable_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, '/exploration/goal', reliable_qos
        )
        self._status_pub = self.create_publisher(
            String, '/exploration/status', reliable_qos
        )

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / rate, self._explore_tick)

        self.get_logger().info('FrontierExplorer initialised — waiting for SLAM data.')

    # ======================================================================
    # Subscription callbacks
    # ======================================================================

    def _occupied_cb(self, msg: PointCloud2) -> None:
        """Mark voxels reported by SLAM as OCCUPIED."""
        try:
            # Use positional indexing to handle both plain tuples and structured arrays
            pts = np.array(
                [[p[0], p[1], p[2]] for p in
                 pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                dtype=np.float64,
            )
            if pts.shape[0] == 0:
                return
            for pt in pts[:, :3]:
                self._grid.mark_occupied(pt)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'FrontierExplorer: occupied_cb error: {exc}')

    def _odom_cb(self, msg: Odometry) -> None:
        """Update current pose and heading."""
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            self._pose = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            # Derive heading (yaw) from quaternion → forward vector in xy-plane
            # Forward vector = R * [1, 0, 0]
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = float(np.arctan2(siny_cosp, cosy_cosp))
            self._heading = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float64)

        # Mark current position as free in the grid
        self._grid.mark_free(self._pose)

    def _octomap_cb(self, msg: MarkerArray) -> None:
        """
        Interpret the MarkerArray from OctoMap:
        * Namespace 'free' → mark cells as FREE
        * Namespace 'occupied' (or default) → mark as OCCUPIED
        We use the cube centre positions.
        """
        for marker in msg.markers:
            state = (
                CellState.FREE
                if 'free' in marker.ns.lower()
                else CellState.OCCUPIED
            )
            for pt in marker.points:
                pos = np.array([pt.x, pt.y, pt.z], dtype=np.float64)
                if state == CellState.FREE:
                    self._grid.mark_free(pos)
                else:
                    self._grid.mark_occupied(pos)

    # ======================================================================
    # Core exploration tick
    # ======================================================================

    def _explore_tick(self) -> None:
        """Periodically detect frontiers, score them, and publish the best goal."""
        with self._lock:
            pose = self._pose
            heading = self._heading

        if pose is None:
            self._publish_status('waiting_for_slam')
            return

        frontiers = self._detect_frontiers(pose)

        if not frontiers:
            self._publish_status('no_frontiers')
            self._publish_frontier_markers([])
            self.get_logger().info('FrontierExplorer: no frontiers found.')
            return

        # Score and sort
        self._score_frontiers(frontiers, pose, heading)
        frontiers.sort(key=lambda f: f.utility, reverse=True)

        best = frontiers[0]
        self.get_logger().info(
            f'FrontierExplorer: {len(frontiers)} frontiers — best utility='
            f'{best.utility:.3f} at {best.centroid}'
        )

        self._publish_goal(best.centroid)
        self._publish_frontier_markers(frontiers)
        self._publish_status('exploring')

    # ======================================================================
    # Frontier detection
    # ======================================================================

    def _detect_frontiers(self, pose: np.ndarray) -> List[FrontierRegion]:
        """
        1. Collect all FREE cells within max_range.
        2. Identify those adjacent to at least one UNKNOWN cell.
        3. Cluster with DBSCAN.
        4. Return list of FrontierRegion.
        """
        free_indices = self._grid.get_all_indices_by_state(CellState.FREE)
        if not free_indices:
            return []

        frontier_voxels: List[np.ndarray] = []
        pose_idx = self._grid.world_to_index(pose)

        for idx in free_indices:
            # Range filter (fast integer check)
            dx = (idx[0] - pose_idx[0]) * self._grid.resolution
            dy = (idx[1] - pose_idx[1]) * self._grid.resolution
            dz = (idx[2] - pose_idx[2]) * self._grid.resolution
            if dx*dx + dy*dy + dz*dz > self._max_range ** 2:
                continue

            # Adjacency check: is any face-neighbour UNKNOWN?
            for off in self._FACE_OFFSETS:
                nb = (idx[0] + off[0], idx[1] + off[1], idx[2] + off[2])
                if self._grid.get(nb) == CellState.UNKNOWN:
                    frontier_voxels.append(self._grid.index_to_world(idx))
                    break

        if not frontier_voxels:
            return []

        pts = np.array(frontier_voxels, dtype=np.float64)

        # DBSCAN clustering
        db = DBSCAN(eps=self._dbscan_eps, min_samples=self._dbscan_min_samples).fit(pts)
        labels = db.labels_
        unique_labels = set(labels) - {-1}

        regions: List[FrontierRegion] = []
        for lbl in unique_labels:
            mask = labels == lbl
            cluster_pts = pts[mask]
            if len(cluster_pts) < self._min_size:
                continue
            centroid = cluster_pts.mean(axis=0)
            regions.append(FrontierRegion(centroid, int(len(cluster_pts)), cluster_pts))

        return regions

    # ======================================================================
    # Frontier scoring
    # ======================================================================

    def _score_frontiers(
        self,
        frontiers: List[FrontierRegion],
        pose: np.ndarray,
        heading: Optional[np.ndarray],
    ) -> None:
        """Compute utility scores in place."""
        distances = np.array([np.linalg.norm(f.centroid - pose) for f in frontiers])
        max_dist = float(distances.max()) if distances.max() > 0 else 1.0

        info_gains = np.array([self._estimate_info_gain(f.centroid) for f in frontiers])
        max_gain = float(info_gains.max()) if info_gains.max() > 0 else 1.0

        for i, f in enumerate(frontiers):
            f.distance = float(distances[i])
            f.info_gain = float(info_gains[i])

            norm_gain = f.info_gain / max_gain
            norm_dist = f.distance / max_dist

            if heading is not None:
                direction = f.centroid - pose
                d_norm = np.linalg.norm(direction)
                if d_norm > 1e-6:
                    direction /= d_norm
                    f.heading_alignment = float(np.dot(heading, direction))
                else:
                    f.heading_alignment = 0.0
            else:
                f.heading_alignment = 0.0

            f.utility = (
                self._w1 * norm_gain
                - self._w2 * norm_dist
                + self._w3 * f.heading_alignment
            )

    def _estimate_info_gain(self, centroid: np.ndarray) -> float:
        """
        Estimate information gain at *centroid* by casting random rays and
        counting UNKNOWN voxels hit before reaching an OCCUPIED one.
        """
        # Generate random ray directions on a sphere
        phi = np.random.uniform(0, 2 * np.pi, self._raytrace_samples)
        costheta = np.random.uniform(-1, 1, self._raytrace_samples)
        sintheta = np.sqrt(1 - costheta ** 2)
        dirs = np.stack([
            sintheta * np.cos(phi),
            sintheta * np.sin(phi),
            costheta,
        ], axis=1)  # shape (N, 3)

        unknown_count = 0
        max_ray_len = 5.0  # metres
        step = self._grid.resolution

        for direction in dirs:
            t = step
            while t <= max_ray_len:
                sample = centroid + t * direction
                idx = self._grid.world_to_index(sample)
                state = self._grid.get(idx)
                if state == CellState.UNKNOWN:
                    unknown_count += 1
                elif state == CellState.OCCUPIED:
                    break
                t += step

        return float(unknown_count)

    # ======================================================================
    # Publishers
    # ======================================================================

    def _publish_goal(self, position: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_frontier_markers(self, frontiers: List[FrontierRegion]) -> None:
        """Publish MarkerArray of spheres, colour-coded by utility."""
        array = MarkerArray()

        # Delete all previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        array.markers.append(delete_marker)

        if not frontiers:
            self._frontier_pub.publish(array)
            return

        utilities = [f.utility for f in frontiers]
        u_min, u_max = min(utilities), max(utilities)
        u_range = u_max - u_min if u_max > u_min else 1.0

        now = self.get_clock().now().to_msg()

        for i, f in enumerate(frontiers):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'map'
            marker.ns = 'frontiers'
            marker.id = i + 1  # id=0 reserved for DELETEALL
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(f.centroid[0])
            marker.pose.position.y = float(f.centroid[1])
            marker.pose.position.z = float(f.centroid[2])
            marker.pose.orientation.w = 1.0

            # Scale proportional to size
            scale = float(np.clip(0.3 + 0.05 * f.size, 0.3, 2.0))
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            # Colour: green (high utility) → red (low utility)
            t = (f.utility - u_min) / u_range
            marker.color.r = float(1.0 - t)
            marker.color.g = float(t)
            marker.color.b = 0.0
            marker.color.a = 0.8

            lifetime = Duration()
            lifetime.sec = 2
            marker.lifetime = lifetime

            array.markers.append(marker)

        self._frontier_pub.publish(array)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
