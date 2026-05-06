#!/usr/bin/env python3
"""
octomap_builder.py
==================
3D occupancy mapping node that consumes SLAM output and builds an
OctoMap-style volumetric representation.

Uses the octomap-python bindings when available; otherwise implements
a compatible log-odds voxel grid in pure numpy.

Subscriptions
-------------
  /slam/map_cloud  (sensor_msgs/PointCloud2)
  /slam/odom       (nav_msgs/Odometry)

Publications
------------
  /slam/octomap         (visualization_msgs/MarkerArray)  — 3D RViz viz
  /slam/occupancy_grid  (nav_msgs/OccupancyGrid)          — 2D Nav2 slice
  /slam/occupied_cells  (sensor_msgs/PointCloud2)         — occupied voxels
"""

from __future__ import annotations

import math
import struct
import threading
from typing import Dict, Optional, Tuple

import numpy as np
try:
    from scipy.ndimage import binary_dilation as _scipy_dilate
    _HAS_SCIPY_ND = True
except ImportError:
    _HAS_SCIPY_ND = False
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------
# Optional octomap-python bindings
# ---------------------------------------------------------------------------
try:
    import octomap  # type: ignore
    _HAS_OCTOMAP = True
except ImportError:
    _HAS_OCTOMAP = False

# ---------------------------------------------------------------------------
# Point-cloud helpers (inline to avoid circular imports)
# ---------------------------------------------------------------------------

def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Fast PointCloud2 → (N, 3) float32 extraction."""
    try:
        fields_sorted = sorted(msg.fields, key=lambda f: f.offset)
        dt_fields = []
        prev_end = 0
        for f in fields_sorted:
            if f.offset > prev_end:
                dt_fields.append(('_pad_%d' % f.offset, np.uint8, f.offset - prev_end))
            if f.datatype == PointField.FLOAT32:
                dt_fields.append((f.name, np.float32))
                prev_end = f.offset + 4
            elif f.datatype == PointField.FLOAT64:
                dt_fields.append((f.name, np.float64))
                prev_end = f.offset + 8
            else:
                dt_fields.append((f.name, np.uint8))
                prev_end = f.offset + 1
        arr = np.frombuffer(bytes(msg.data), dtype=np.dtype(dt_fields))
        xyz = np.column_stack([arr['x'].astype(np.float32),
                               arr['y'].astype(np.float32),
                               arr['z'].astype(np.float32)])
        return xyz[np.isfinite(xyz).all(axis=1)]
    except Exception:
        return np.zeros((0, 3), dtype=np.float32)


def _xyz_to_pc2(
    points: np.ndarray, frame_id: str, stamp: RosTime
) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = bytearray(points.astype(np.float32).tobytes())
    return msg


# ---------------------------------------------------------------------------
# Pure-numpy log-odds occupancy grid
# ---------------------------------------------------------------------------

class LogOddsVoxelGrid:
    """
    3-D log-odds voxel occupancy grid.

    Voxels are stored in a dict keyed by integer (ix, iy, iz) tuple to
    allow unbounded growth without pre-allocation.
    """

    # Log-odds probabilities
    L_OCC_DEFAULT  = math.log(0.7 / 0.3)    # ~0.85
    L_FREE_DEFAULT = math.log(0.4 / 0.6)    # ~-0.41
    L_MIN = -2.0
    L_MAX =  3.5
    P_OCC_THRESHOLD = 0.5                    # log-odds = 0.0

    def __init__(
        self,
        resolution: float = 0.2,
        max_range: float = 30.0,
        l_occ: float = L_OCC_DEFAULT,
        l_free: float = L_FREE_DEFAULT,
    ) -> None:
        self.resolution = resolution
        self.max_range = max_range
        self.l_occ = l_occ
        self.l_free = l_free
        # voxel key → log-odds value
        self._grid: Dict[Tuple[int, int, int], float] = {}
        self._lock = threading.Lock()

    # ── Coordinate helpers ────────────────────────────────────────────────

    def _to_key(self, point: np.ndarray) -> Tuple[int, int, int]:
        return (
            int(math.floor(point[0] / self.resolution)),
            int(math.floor(point[1] / self.resolution)),
            int(math.floor(point[2] / self.resolution)),
        )

    def _key_to_center(self, key: Tuple[int, int, int]) -> np.ndarray:
        return np.array([
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution,
        ], dtype=np.float32)

    # ── Raycasting ────────────────────────────────────────────────────────

    def _bresenham3d(
        self,
        origin: np.ndarray,
        endpoint: np.ndarray,
    ) -> list[Tuple[int, int, int]]:
        """
        3-D Bresenham / DDA ray traversal through the voxel grid.
        Returns all voxels intersected *before* the endpoint.
        """
        o = origin / self.resolution
        e = endpoint / self.resolution
        d = e - o
        length = np.linalg.norm(d)
        if length < 1e-6:
            return []

        steps = max(int(length * 2), 1)   # 2 samples per voxel side
        cells = []
        prev = None
        for i in range(steps):
            t = i / steps
            p = o + d * t
            key = (int(math.floor(p[0])),
                   int(math.floor(p[1])),
                   int(math.floor(p[2])))
            if key != prev:
                cells.append(key)
                prev = key
        return cells

    def insert_point_cloud(
        self,
        points: np.ndarray,
        sensor_origin: np.ndarray,
    ) -> None:
        """
        Update log-odds grid from a new scan.

        Parameters
        ----------
        points        : (N, 3) array in *world/map* frame
        sensor_origin : (3,) sensor position in same frame
        """
        with self._lock:
            for pt in points:
                diff = pt - sensor_origin
                dist = float(np.linalg.norm(diff))
                if dist > self.max_range or dist < 0.1:
                    continue

                # Free-space rays
                free_cells = self._bresenham3d(sensor_origin, pt)
                for key in free_cells:
                    self._grid[key] = max(
                        self.L_MIN,
                        self._grid.get(key, 0.0) + self.l_free)

                # Occupied endpoint
                occ_key = self._to_key(pt)
                self._grid[occ_key] = min(
                    self.L_MAX,
                    self._grid.get(occ_key, 0.0) + self.l_occ)

    # ── Queries ───────────────────────────────────────────────────────────

    def get_occupied_centers(self) -> np.ndarray:
        """Return (N, 3) array of occupied voxel centres."""
        with self._lock:
            occupied = [
                self._key_to_center(k)
                for k, v in self._grid.items()
                if v > 0.0   # log-odds > 0 ⟺ p > 0.5
            ]
        if not occupied:
            return np.zeros((0, 3), dtype=np.float32)
        return np.array(occupied, dtype=np.float32)

    def get_free_centers(self) -> np.ndarray:
        """Return (N, 3) array of free voxel centres (log-odds < 0)."""
        with self._lock:
            free = [
                self._key_to_center(k)
                for k, v in self._grid.items()
                if v < 0.0
            ]
        if not free:
            return np.zeros((0, 3), dtype=np.float32)
        return np.array(free, dtype=np.float32)

    def get_2d_slice(
        self, z_min: float = -0.5, z_max: float = 2.5
    ) -> Tuple[np.ndarray, Tuple[float, float]]:
        """
        Project occupied voxels within [z_min, z_max] to a 2-D grid.
        Only occupied cells (log-odds > 0) are marked; everything else
        stays UNKNOWN (-1) to avoid free-space raycasting starburst.
        """
        with self._lock:
            # Only keep cells with positive log-odds (occupied wall hits)
            relevant = {k: v for k, v in self._grid.items()
                        if v > 0.0 and z_min <= k[2] * self.resolution <= z_max}

        if not relevant:
            return np.full((10, 10), -1, dtype=np.int8), (0.0, 0.0)

        ixs = [k[0] for k in relevant]
        iys = [k[1] for k in relevant]
        ix_min, ix_max = min(ixs), max(ixs)
        iy_min, iy_max = min(iys), max(iys)
        W = ix_max - ix_min + 1
        H = iy_max - iy_min + 1
        grid_2d = np.full((H, W), -1, dtype=np.int8)  # all unknown by default

        for (kx, ky, _) in relevant:
            xi = kx - ix_min
            yi = ky - iy_min
            if 0 <= yi < H and 0 <= xi < W:
                grid_2d[yi, xi] = 100  # mark only occupied cells

        origin = (ix_min * self.resolution, iy_min * self.resolution)
        return grid_2d, origin


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class OctomapBuilder(Node):
    """
    Builds a 3-D occupancy map from SLAM point-cloud output.

    When octomap-python is installed the official C++ OctoMap library
    is used via its Python bindings.  Otherwise a compatible log-odds
    implementation in pure numpy is used transparently.
    """

    def __init__(self) -> None:
        super().__init__('octomap_builder')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('sensor_model_hit', 0.7)
        self.declare_parameter('sensor_model_miss', 0.4)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('z_slice_min', -0.5)
        self.declare_parameter('z_slice_max', 2.5)

        self._resolution = self.get_parameter('resolution').value
        self._max_range = self.get_parameter('max_range').value
        self._hit = self.get_parameter('sensor_model_hit').value
        self._miss = self.get_parameter('sensor_model_miss').value
        self._map_frame = self.get_parameter('map_frame').value
        self._pub_rate = self.get_parameter('publish_rate_hz').value
        self._z_min = self.get_parameter('z_slice_min').value
        self._z_max = self.get_parameter('z_slice_max').value
        # How far above/below the drone to include in the 2D slice
        # (overrides z_slice_min/max when tracking sensor height)
        self._z_half = 1.5  # metres each side of drone height

        # ── Map backend ───────────────────────────────────────────────────
        l_occ = math.log(self._hit / (1.0 - self._hit))
        l_free = math.log(self._miss / (1.0 - self._miss))

        if _HAS_OCTOMAP:
            self._octree = octomap.OcTree(self._resolution)
            self.get_logger().info(
                f'OctomapBuilder using octomap-python (res={self._resolution}m)')
        else:
            self._octree = LogOddsVoxelGrid(
                resolution=self._resolution,
                max_range=self._max_range,
                l_occ=l_occ,
                l_free=l_free,
            )
            self.get_logger().info(
                f'OctomapBuilder using numpy log-odds grid '
                f'(res={self._resolution}m) — install octomap-python for native support')

        # Current sensor origin — updated from /slam/odom
        self._sensor_origin = np.zeros(3, dtype=np.float64)
        self._sensor_z = None   # None until first odom received
        self._lock = threading.Lock()
        self._pending_cloud: Optional[np.ndarray] = None

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        # ── Subscribers ───────────────────────────────────────────────────
        # Subscribe to per-keyframe scans (NOT the accumulated /slam/map_cloud).
        # Raycasting the full accumulated cloud from the current position creates
        # free-space rays through already-mapped walls → starburst artefact.
        self.create_subscription(
            PointCloud2, '/slam/current_scan', self._cloud_callback, sensor_qos)
        self.create_subscription(
            Odometry, '/slam/odom', self._odom_callback, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self._marker_pub = self.create_publisher(
            MarkerArray, '/slam/octomap', map_qos)
        self._grid_pub = self.create_publisher(
            OccupancyGrid, '/slam/occupancy_grid', map_qos)
        self._cells_pub = self.create_publisher(
            PointCloud2, '/slam/occupied_cells', map_qos)

        # ── Publish timer ─────────────────────────────────────────────────
        period = 1.0 / max(self._pub_rate, 0.1)
        self.create_timer(period, self._publish_callback)

        self._update_count = 0

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            p = msg.pose.pose.position
            self._sensor_origin = np.array([p.x, p.y, p.z], dtype=np.float64)
            self._sensor_z = float(p.z)   # track drone height for z-slice

    def _cloud_callback(self, msg: PointCloud2) -> None:
        pts = _pc2_to_xyz(msg)
        if len(pts) == 0:
            return

        with self._lock:
            origin = self._sensor_origin.copy()

        self._insert_scan(pts, origin)
        self._update_count += 1
        if self._update_count % 10 == 0:
            self.get_logger().debug(
                f'OctoMap updated {self._update_count} times')

    def _insert_scan(
        self, pts: np.ndarray, origin: np.ndarray
    ) -> None:
        """Insert a scan into the map backend, filtering near-vertical rays."""
        # Remove nearly-vertical rays (elevation angle > 60°) to suppress
        # floor/ceiling starburst artefacts in the 2D projection.
        # The 3D map keeps all points; only the occupancy logic filters.
        diff = pts - origin.astype(np.float32)
        dists = np.linalg.norm(diff, axis=1)
        valid = dists > 0.1
        elevation = np.where(
            valid,
            np.abs(diff[:, 2]) / np.maximum(dists, 1e-6),
            1.0)
        # sin(elevation) < sin(60°) ≈ 0.866  →  keep mostly horizontal rays
        horiz_mask = elevation < 0.866
        pts_3d = pts                    # full cloud for 3D map
        pts_2d = pts[horiz_mask]        # horizontal-only for 2D grid raycasting

        # Cap raycasting to body-range filter (20 m) — longer rays only add
        # free-space starburst spokes without adding useful wall information.
        RAY_MAX = min(self._max_range, 20.0)

        if _HAS_OCTOMAP:
            try:
                self._octree.insertPointCloud(
                    pointcloud=pts_3d.astype(np.float64),
                    origin=origin.astype(np.float64),
                    maxrange=RAY_MAX,
                    lazy_eval=True,
                )
                self._octree.updateInnerOccupancy()
            except Exception as exc:
                self.get_logger().warn(f'OctoMap insert failed: {exc}')
        else:
            # Insert full cloud for 3D, but use filtered set for ray-casting
            # (the numpy grid does raycasting so floor/ceiling suppression matters)
            self._octree.insert_point_cloud(pts_2d.astype(np.float64), origin)


    # ── Publish callback ──────────────────────────────────────────────────

    def _publish_callback(self) -> None:
        now = self.get_clock().now().to_msg()

        if _HAS_OCTOMAP:
            occupied_pts = self._get_occupied_octomap()
            free_pts = self._get_free_octomap()
        else:
            occupied_pts = self._octree.get_occupied_centers()
            free_pts = self._octree.get_free_centers()

        if len(occupied_pts) == 0 and len(free_pts) == 0:
            return

        self._publish_marker_array(occupied_pts, free_pts, now)
        if len(occupied_pts) > 0:
            self._publish_occupied_cells(occupied_pts, now)
            self._publish_2d_grid(now)

    # ── OctoMap helpers ───────────────────────────────────────────────────

    def _get_occupied_octomap(self) -> np.ndarray:
        """Extract occupied leaf centers from octomap-python OcTree."""
        try:
            occupied = []
            for it in self._octree.begin_leafs():
                if self._octree.isNodeOccupied(it):
                    c = it.getCoordinate()
                    occupied.append([c[0], c[1], c[2]])
            return np.array(occupied, dtype=np.float32) if occupied \
                   else np.zeros((0, 3), dtype=np.float32)
        except Exception:
            return np.zeros((0, 3), dtype=np.float32)

    def _get_free_octomap(self) -> np.ndarray:
        """Extract free (non-occupied) leaf centers from octomap-python OcTree."""
        try:
            free = []
            for it in self._octree.begin_leafs():
                if not self._octree.isNodeOccupied(it):
                    c = it.getCoordinate()
                    free.append([c[0], c[1], c[2]])
            return np.array(free, dtype=np.float32) if free \
                   else np.zeros((0, 3), dtype=np.float32)
        except Exception:
            return np.zeros((0, 3), dtype=np.float32)

    # ── Publishing helpers ────────────────────────────────────────────────

    def _publish_marker_array(
        self, pts: np.ndarray, free_pts: np.ndarray, stamp: RosTime
    ) -> None:
        """Publish occupied + free voxels as a DELETE + ADD MarkerArray for RViz."""
        ma = MarkerArray()

        # Delete previous markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = self._map_frame
        delete_all.header.stamp = stamp
        ma.markers.append(delete_all)

        # ── Occupied voxels (height-coloured) ──────────────────────────────
        if len(pts) > 0:
            z_min = float(pts[:, 2].min())
            z_max = float(pts[:, 2].max())
            z_range = max(z_max - z_min, 0.01)

            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'octomap'
            m.id = 1
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.scale.x = self._resolution
            m.scale.y = self._resolution
            m.scale.z = self._resolution
            m.pose.orientation.w = 1.0

            for pt in pts:
                m.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
                t = (float(pt[2]) - z_min) / z_range
                r = min(1.0, 2.0 * t)
                g = 1.0 - abs(2.0 * t - 1.0)
                b = max(0.0, 1.0 - 2.0 * t)
                m.colors.append(ColorRGBA(r=r, g=g, b=b, a=0.8))

            ma.markers.append(m)

        # ── Free voxels (cyan, capped at 10 000 for performance) ───────────
        if len(free_pts) > 0:
            sample = free_pts[:10000]
            mf = Marker()
            mf.header.frame_id = self._map_frame
            mf.header.stamp = stamp
            mf.ns = 'free_voxels'
            mf.id = 2
            mf.type = Marker.CUBE_LIST
            mf.action = Marker.ADD
            mf.scale.x = self._resolution
            mf.scale.y = self._resolution
            mf.scale.z = self._resolution
            mf.pose.orientation.w = 1.0
            mf.color = ColorRGBA(r=0.0, g=0.8, b=0.8, a=0.3)

            for pt in sample:
                mf.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))

            ma.markers.append(mf)

        self._marker_pub.publish(ma)

    def _publish_occupied_cells(
        self, pts: np.ndarray, stamp: RosTime
    ) -> None:
        msg = _xyz_to_pc2(pts, self._map_frame, stamp)
        self._cells_pub.publish(msg)

    def _publish_2d_grid(self, stamp: RosTime) -> None:
        """Publish 2-D occupancy grid slice at the drone's current altitude."""
        with self._lock:
            sz = self._sensor_z

        # Use sensor height if known; otherwise fall back to static params
        if sz is not None:
            z_min = sz - self._z_half
            z_max = sz + self._z_half
        else:
            z_min, z_max = self._z_min, self._z_max

        if _HAS_OCTOMAP:
            grid_2d, origin_xy = self._slice_octomap_2d(z_min, z_max)
        else:
            grid_2d, origin_xy = self._octree.get_2d_slice(z_min, z_max)

        if grid_2d is None or grid_2d.size == 0:
            return

        # ── Dilate occupied cells to fill gaps between sparse LiDAR hits ───────
        # At 0.1 m/voxel, 2 iterations ≈ 0.2 m wall thickness — fills the
        # typical 0.15 m gap between adjacent LiDAR scan rings.
        occ_mask = grid_2d == 100
        if occ_mask.any():
            if _HAS_SCIPY_ND:
                dilated = _scipy_dilate(occ_mask, iterations=2)
            else:
                # Simple 3x3 max-pool fallback
                from numpy.lib.stride_tricks import sliding_window_view
                pad = np.pad(occ_mask.astype(np.uint8), 1, constant_values=0)
                dilated = (
                    sliding_window_view(pad, (3, 3)).max(axis=(-2, -1)) > 0
                )
            grid_2d[dilated & (grid_2d != 100)] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = stamp
        msg.info.resolution = self._resolution
        msg.info.width = int(grid_2d.shape[1])
        msg.info.height = int(grid_2d.shape[0])
        msg.info.origin.position.x = float(origin_xy[0])
        msg.info.origin.position.y = float(origin_xy[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_2d.flatten().tolist()
        self._grid_pub.publish(msg)

    def _slice_octomap_2d(
        self, z_min: float, z_max: float,
    ) -> Tuple[Optional[np.ndarray], Tuple[float, float]]:
        """Project octomap-python tree to 2-D slice at given z band."""
        try:
            pts = self._get_occupied_octomap()
            if len(pts) == 0:
                return None, (0.0, 0.0)
            mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
            pts2 = pts[mask]
            if len(pts2) == 0:
                return None, (0.0, 0.0)

            res = self._resolution
            ix = np.floor(pts2[:, 0] / res).astype(int)
            iy = np.floor(pts2[:, 1] / res).astype(int)
            ix_min, ix_max = ix.min(), ix.max()
            iy_min, iy_max = iy.min(), iy.max()
            W, H = int(ix_max - ix_min + 1), int(iy_max - iy_min + 1)
            grid = np.full((H, W), -1, dtype=np.int8)
            grid[iy - iy_min, ix - ix_min] = 100
            return grid, (ix_min * res, iy_min * res)
        except Exception as exc:
            self.get_logger().warn(f'2-D slice failed: {exc}')
            return None, (0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OctomapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
