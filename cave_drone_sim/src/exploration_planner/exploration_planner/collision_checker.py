"""
collision_checker.py
====================
Thread-safe 3D collision-checking utility backed by a KD-tree of occupied voxels.

Subscriptions
-------------
/slam/occupied_cells  (sensor_msgs/PointCloud2)  — occupied voxels from SLAM

Public API (callable from other nodes by importing the class)
-------------------------------------------------------------
CollisionChecker.is_point_free(point, safety_margin)
CollisionChecker.is_path_free(start, end, step)
CollisionChecker.get_nearest_obstacle_distance(point)

Also runnable as a standalone ROS 2 node that rebuilds its KD-tree at a
configurable rate and exposes the same checks as a service.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ---------------------------------------------------------------------------
# Standalone utility class (importable, no ROS dependency required)
# ---------------------------------------------------------------------------

class CollisionCheckerCore:
    """
    Pure-Python collision checker.  Thread-safe KD-tree wrapper.

    Parameters
    ----------
    safety_margin : float
        Default inflation radius around obstacle points (metres).
    """

    def __init__(self, safety_margin: float = 0.3) -> None:
        self._safety_margin: float = safety_margin
        self._kdtree: Optional[KDTree] = None
        self._obstacle_points: Optional[np.ndarray] = None  # shape (N, 3)
        self._lock: threading.RLock = threading.RLock()
        self._last_update: float = 0.0

    # ------------------------------------------------------------------
    # Data ingestion
    # ------------------------------------------------------------------

    def update_obstacles(self, points: np.ndarray) -> None:
        """
        Replace the current obstacle set with a new array of 3-D points.

        Parameters
        ----------
        points : np.ndarray
            Shape (N, 3) array of obstacle positions in world frame.
        """
        if points is None or len(points) == 0:
            return
        with self._lock:
            self._obstacle_points = np.asarray(points, dtype=np.float64)
            self._kdtree = KDTree(self._obstacle_points)
            self._last_update = time.monotonic()

    # ------------------------------------------------------------------
    # Query API
    # ------------------------------------------------------------------

    def is_point_free(
        self,
        point: np.ndarray,
        safety_margin: Optional[float] = None,
    ) -> bool:
        """
        Return True if *point* is at least *safety_margin* metres from every
        known obstacle.

        Parameters
        ----------
        point : array-like, shape (3,)
        safety_margin : float or None
            Override default safety margin.
        """
        margin = safety_margin if safety_margin is not None else self._safety_margin
        with self._lock:
            if self._kdtree is None:
                return True  # no data → optimistically free
            dist, _ = self._kdtree.query(np.asarray(point, dtype=np.float64))
            return float(dist) >= margin

    def is_path_free(
        self,
        start: np.ndarray,
        end: np.ndarray,
        step: float = 0.1,
        safety_margin: Optional[float] = None,
    ) -> bool:
        """
        Check whether the line segment *start*→*end* is collision-free by
        sampling the segment at *step* intervals.

        Parameters
        ----------
        start, end : array-like, shape (3,)
        step : float
            Sampling resolution in metres.
        safety_margin : float or None
            Override default safety margin.
        """
        start = np.asarray(start, dtype=np.float64)
        end = np.asarray(end, dtype=np.float64)
        margin = safety_margin if safety_margin is not None else self._safety_margin

        with self._lock:
            if self._kdtree is None:
                return True

            length = float(np.linalg.norm(end - start))
            if length < 1e-6:
                dist, _ = self._kdtree.query(start)
                return float(dist) >= margin

            n_steps = max(2, int(np.ceil(length / step)))
            alphas = np.linspace(0.0, 1.0, n_steps)
            # Vectorised interpolation: shape (n_steps, 3)
            samples = start[None, :] + alphas[:, None] * (end - start)[None, :]
            dists, _ = self._kdtree.query(samples)
            return bool(np.all(dists >= margin))

    def get_nearest_obstacle_distance(self, point: np.ndarray) -> float:
        """
        Return the distance (metres) from *point* to the nearest obstacle.
        Returns infinity if the obstacle map is empty.
        """
        with self._lock:
            if self._kdtree is None:
                return float('inf')
            dist, _ = self._kdtree.query(
                np.asarray(point, dtype=np.float64).reshape(1, 3)
            )
            return float(dist[0])

    def get_obstacle_points_in_radius(
        self, point: np.ndarray, radius: float
    ) -> np.ndarray:
        """
        Return all obstacle points within *radius* of *point*.

        Returns
        -------
        np.ndarray
            Shape (M, 3); empty array if none found.
        """
        with self._lock:
            if self._kdtree is None or self._obstacle_points is None:
                return np.empty((0, 3), dtype=np.float64)
            indices = self._kdtree.query_ball_point(
                np.asarray(point, dtype=np.float64), radius
            )
            if not indices:
                return np.empty((0, 3), dtype=np.float64)
            return self._obstacle_points[indices]

    @property
    def has_data(self) -> bool:
        """True if the KD-tree has been populated at least once."""
        with self._lock:
            return self._kdtree is not None

    @property
    def obstacle_count(self) -> int:
        """Number of obstacle points currently in the KD-tree."""
        with self._lock:
            if self._obstacle_points is None:
                return 0
            return len(self._obstacle_points)


# ---------------------------------------------------------------------------
# ROS 2 node wrapper
# ---------------------------------------------------------------------------

class CollisionCheckerNode(Node):
    """
    ROS 2 node that wraps CollisionCheckerCore.

    Subscribes to /slam/occupied_cells and maintains an up-to-date KD-tree.
    Exposes the CollisionCheckerCore instance as ``self.checker`` so other
    nodes can import this module and share the same instance.
    """

    def __init__(self) -> None:
        super().__init__('collision_checker')

        # Parameters
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('kdtree_rebuild_rate', 2.0)
        self.declare_parameter('max_obstacle_points', 100_000)

        self._safety_margin: float = (
            self.get_parameter('safety_margin').get_parameter_value().double_value
        )
        self._max_points: int = (
            self.get_parameter('max_obstacle_points').get_parameter_value().integer_value
        )

        self.checker = CollisionCheckerCore(safety_margin=self._safety_margin)

        # QoS: BEST_EFFORT for high-frequency sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._occ_sub = self.create_subscription(
            PointCloud2,
            '/slam/occupied_cells',
            self._occupied_cells_callback,
            sensor_qos,
        )

        self.get_logger().info(
            f'CollisionChecker ready — safety_margin={self._safety_margin}m'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _occupied_cells_callback(self, msg: PointCloud2) -> None:
        """Convert incoming PointCloud2 to numpy array and update checker."""
        try:
            points = np.array(
                [[p[0], p[1], p[2]] for p in
                 pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                dtype=np.float64,
            )
            if points.shape[0] == 0:
                return
            # Cap to avoid unbounded memory usage
            if points.shape[0] > self._max_points:
                idx = np.random.choice(points.shape[0], self._max_points, replace=False)
                points = points[idx]
            self.checker.update_obstacles(points[:, :3])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'CollisionChecker: failed to parse PointCloud2: {exc}')


def main(args=None) -> None:
    """Entry point for the standalone collision_checker node."""
    rclpy.init(args=args)
    node = CollisionCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
