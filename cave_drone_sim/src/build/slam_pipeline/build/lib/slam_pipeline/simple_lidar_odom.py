#!/usr/bin/env python3
"""
simple_lidar_odom.py
====================
Self-contained ICP-based LiDAR odometry node for cave drone SLAM.

No external SLAM packages required.  Uses Open3D when available;
falls back to a pure-scipy / numpy implementation otherwise.

Subscriptions
-------------
  /drone/lidar/points  (sensor_msgs/PointCloud2)
  /drone/imu           (sensor_msgs/Imu)

Publications
------------
  /slam/odom           (nav_msgs/Odometry)
  /slam/map_cloud      (sensor_msgs/PointCloud2)
  /slam/path           (nav_msgs/Path)

TF Broadcasts
-------------
  map  → odom      (SLAM correction transform)
  odom → base_link (current odometry pose)
"""

from __future__ import annotations

import math
import struct
import threading
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    TransformStamped,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
try:
    from px4_msgs.msg import VehicleLocalPosition as _PX4Pos
    _HAS_PX4 = True
except ImportError:
    _HAS_PX4 = False

# ---------------------------------------------------------------------------
# Optional Open3D import with graceful fallback
# ---------------------------------------------------------------------------
try:
    import open3d as o3d  # type: ignore
    _HAS_O3D = True
except ImportError:
    _HAS_O3D = False

try:
    from scipy.spatial import cKDTree as KDTree  # type: ignore
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def ros_time_to_sec(stamp: RosTime) -> float:
    """Convert a ROS stamp to floating-point seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_ros_time(t: float) -> RosTime:
    """Convert floating-point seconds to a ROS Time message."""
    msg = RosTime()
    msg.sec = int(t)
    msg.nanosec = int((t - msg.sec) * 1e9)
    return msg


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → quaternion [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def compose_transforms(
    t1: np.ndarray, R1: np.ndarray,
    t2: np.ndarray, R2: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compose two SE(3) transforms: T_result = T1 ∘ T2."""
    R_out = R1 @ R2
    t_out = R1 @ t2 + t1
    return t_out, R_out


def invert_transform(t: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Invert an SE(3) transform."""
    R_inv = R.T
    t_inv = -R_inv @ t
    return t_inv, R_inv


# ---------------------------------------------------------------------------
# Point cloud I/O
# ---------------------------------------------------------------------------

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N, 3) float32 XYZ array from a PointCloud2 message."""
    fmt_map: dict[int, str] = {
        PointField.FLOAT32: 'f',
        PointField.FLOAT64: 'd',
        PointField.INT8: 'b', PointField.UINT8: 'B',
        PointField.INT16: 'h', PointField.UINT16: 'H',
        PointField.INT32: 'i', PointField.UINT32: 'I',
    }
    field_map = {f.name: (f.offset, fmt_map.get(f.datatype, 'f'))
                 for f in msg.fields}
    ox, fx = field_map['x']
    oy, fy = field_map['y']
    oz, fz = field_map['z']

    pts = []
    for i in range(msg.width * msg.height):
        base = i * msg.point_step
        raw = bytes(msg.data)
        x = struct.unpack_from(fx, raw, base + ox)[0]
        y = struct.unpack_from(fy, raw, base + oy)[0]
        z = struct.unpack_from(fz, raw, base + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            pts.append((x, y, z))
    return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)


def pointcloud2_to_xyz_fast(msg: PointCloud2) -> np.ndarray:
    """
    Fast (N, 3) extraction using numpy structured arrays.
    Falls back to the safe loop-based parser on error.
    """
    try:
        dtype_list = []
        for f in msg.fields:
            if f.datatype == PointField.FLOAT32:
                np_type = np.float32
            elif f.datatype == PointField.FLOAT64:
                np_type = np.float64
            elif f.datatype == PointField.UINT8:
                np_type = np.uint8
            else:
                np_type = np.float32
            dtype_list.append((f.name, np_type, f.offset))

        # Build structured dtype from field offsets
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
        mask = np.isfinite(xyz).all(axis=1)
        return xyz[mask]
    except Exception:
        return pointcloud2_to_xyz(msg)


def xyz_to_pointcloud2(
    points: np.ndarray,
    frame_id: str,
    stamp: RosTime,
) -> PointCloud2:
    """Convert (N, 3) float32 array to a PointCloud2 message."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 12  # 3 × float32
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = bytearray(points.astype(np.float32).tobytes())
    return msg


# ---------------------------------------------------------------------------
# ICP implementations
# ---------------------------------------------------------------------------

def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel grid downsampling using numpy."""
    if len(points) == 0:
        return points
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return points[unique_idx]


def estimate_normals_kdtree(
    points: np.ndarray, k: int = 20
) -> np.ndarray:
    """Estimate point normals via PCA on k-NN neighbourhoods."""
    if not _HAS_SCIPY:
        # Dummy normals pointing up — ICP will degrade to point-to-point
        return np.tile([0.0, 0.0, 1.0], (len(points), 1)).astype(np.float32)

    tree = KDTree(points)
    normals = np.zeros_like(points)
    for i, p in enumerate(points):
        _, idx = tree.query(p, k=min(k, len(points)))
        nbrs = points[idx]
        cov = np.cov((nbrs - nbrs.mean(axis=0)).T)
        if cov.ndim < 2:
            normals[i] = [0, 0, 1]
            continue
        _, vecs = np.linalg.eigh(cov)
        normals[i] = vecs[:, 0]  # smallest eigenvalue → normal
    return normals.astype(np.float32)


def icp_point_to_plane_numpy(
    source: np.ndarray,
    target: np.ndarray,
    target_normals: np.ndarray,
    init_T: np.ndarray,
    max_dist: float = 1.0,
    max_iter: int = 30,
    tol: float = 1e-4,
) -> Tuple[np.ndarray, float]:
    """
    Point-to-plane ICP (numpy + scipy fallback).

    Parameters
    ----------
    source, target : (N, 3) arrays
    target_normals : (N, 3) normal array for *target*
    init_T         : 4×4 initial transform
    max_dist       : max correspondence distance (m)
    max_iter       : iteration limit
    tol            : convergence tolerance on translation delta

    Returns
    -------
    T : 4×4 refined transform (source → target frame)
    fitness : fraction of inlier correspondences
    """
    if not _HAS_SCIPY:
        # Degenerate to identity-delta if no scipy
        return init_T, 0.0

    T = init_T.copy()
    tree = KDTree(target)

    for _ in range(max_iter):
        # Transform source
        src_h = np.hstack([source, np.ones((len(source), 1))])
        src_t = (T @ src_h.T).T[:, :3]

        dists, idx = tree.query(src_t, k=1)
        mask = dists < max_dist
        if mask.sum() < 6:
            break

        s = src_t[mask]
        d = target[idx[mask]]
        n = target_normals[idx[mask]]

        # Build linear system A x = b for point-to-plane
        # x = [α, β, γ, tx, ty, tz]
        A = np.zeros((mask.sum(), 6))
        A[:, 0] = (n[:, 2] * s[:, 1] - n[:, 1] * s[:, 2])
        A[:, 1] = (n[:, 0] * s[:, 2] - n[:, 2] * s[:, 0])
        A[:, 2] = (n[:, 1] * s[:, 0] - n[:, 0] * s[:, 1])
        A[:, 3] = n[:, 0]
        A[:, 4] = n[:, 1]
        A[:, 5] = n[:, 2]
        b = np.sum(n * (d - s), axis=1)

        try:
            x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            break

        alpha, beta, gamma = x[:3]
        tx, ty, tz = x[3:]

        # Small-angle rotation approximation
        dR = np.array([
            [1,      -gamma,  beta],
            [gamma,   1,     -alpha],
            [-beta,   alpha,  1],
        ])
        dT = np.eye(4)
        dT[:3, :3] = dR
        dT[:3, 3] = [tx, ty, tz]
        T = dT @ T

        if np.linalg.norm([tx, ty, tz]) < tol:
            break

    fitness = float(mask.sum()) / len(source) if len(source) > 0 else 0.0
    return T, fitness


def icp_open3d(
    source: np.ndarray,
    target: np.ndarray,
    init_T: np.ndarray,
    max_dist: float = 1.0,
    max_iter: int = 30,
) -> Tuple[np.ndarray, float]:
    """Point-to-plane ICP using Open3D."""
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))

    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))
    tgt_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    result = o3d.pipelines.registration.registration_icp(
        src_pcd,
        tgt_pcd,
        max_dist,
        init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )
    return np.array(result.transformation), result.fitness


# ---------------------------------------------------------------------------
# IMU integrator
# ---------------------------------------------------------------------------

class ImuIntegrator:
    """
    Simple IMU pre-integration for ICP initial guess.

    Integrates angular velocity and linear acceleration (minus gravity)
    to produce a delta-pose prediction between consecutive LiDAR scans.
    """

    GRAVITY = np.array([0.0, 0.0, -9.81], dtype=np.float64)

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_t: Optional[float] = None
        # Running pose in odom frame
        self._vel = np.zeros(3, dtype=np.float64)
        self._pos = np.zeros(3, dtype=np.float64)
        self._R = np.eye(3, dtype=np.float64)
        # Delta since last reset
        self._d_pos = np.zeros(3, dtype=np.float64)
        self._d_R = np.eye(3, dtype=np.float64)

    def update(self, msg: Imu) -> None:
        t = ros_time_to_sec(msg.header.stamp)
        with self._lock:
            if self._last_t is None:
                self._last_t = t
                return
            dt = t - self._last_t
            if dt <= 0 or dt > 1.0:
                self._last_t = t
                return
            self._last_t = t

            # Angular velocity integration
            omega = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ], dtype=np.float64)
            angle = np.linalg.norm(omega) * dt
            if angle > 1e-8:
                axis = omega / np.linalg.norm(omega)
                # Rodrigues rotation formula
                K = np.array([
                    [0,        -axis[2],  axis[1]],
                    [axis[2],   0,       -axis[0]],
                    [-axis[1],  axis[0],  0],
                ], dtype=np.float64)
                dR = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
                self._d_R = self._d_R @ dR
                self._R = self._R @ dR

            # Linear acceleration integration (world frame)
            a_body = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ], dtype=np.float64)
            a_world = self._R @ a_body + self.GRAVITY
            self._vel += a_world * dt
            dp = self._vel * dt + 0.5 * a_world * dt * dt
            self._pos += dp
            self._d_pos += dp

    def get_delta_and_reset(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return delta (d_pos, d_R) since last call and reset accumulators."""
        with self._lock:
            d_pos = self._d_pos.copy()
            d_R = self._d_R.copy()
            self._d_pos = np.zeros(3, dtype=np.float64)
            self._d_R = np.eye(3, dtype=np.float64)
        return d_pos, d_R


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class SimpleLidarOdom(Node):
    """
    Self-contained ICP-based LiDAR odometry ROS 2 node.

    Maintains a sliding-window local map and performs scan-to-map ICP
    registration to estimate the robot's pose.  An IMU integrator
    provides the initial guess between scans.
    """

    def __init__(self) -> None:
        super().__init__('simple_lidar_odom')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('icp_max_distance', 1.0)
        self.declare_parameter('keyframe_distance', 0.5)
        self.declare_parameter('keyframe_rotation', 0.26)   # ~15 deg
        self.declare_parameter('local_map_size', 50)
        self.declare_parameter('icp_max_iter', 30)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self._voxel_size = self.get_parameter('voxel_size').value
        self._icp_max_dist = self.get_parameter('icp_max_distance').value
        self._kf_dist = self.get_parameter('keyframe_distance').value
        self._kf_rot = self.get_parameter('keyframe_rotation').value
        self._local_map_size = self.get_parameter('local_map_size').value
        self._icp_max_iter = self.get_parameter('icp_max_iter').value
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── State ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._imu = ImuIntegrator()

        # Current pose: map ← base_link
        self._pos = np.zeros(3, dtype=np.float64)
        self._R = np.eye(3, dtype=np.float64)

        # PX4 NED position (ground truth pose anchor)
        self._px4_pos: Optional[np.ndarray] = None   # [n, e, u] in map frame
        self._px4_hdg: float = 0.0
        self._px4_valid: bool = False

        # Last keyframe pose
        self._kf_pos = np.zeros(3, dtype=np.float64)
        self._kf_R = np.eye(3, dtype=np.float64)

        # Sliding-window local map: deque of (N,3) arrays in map frame
        # Used ONLY for ICP fallback — does NOT drive the published map cloud.
        self._keyframes: deque[np.ndarray] = deque(maxlen=self._local_map_size)
        self._local_map: Optional[np.ndarray] = None
        self._local_normals: Optional[np.ndarray] = None

        # Global accumulated map — NEVER drops frames, drives /slam/map_cloud.
        # Downsampled at GLOBAL_VOXEL (larger than scan voxel) to control memory.
        self._global_pts: Optional[np.ndarray] = None   # (N,3) float32 in map frame
        self._global_kf_count: int = 0
        self._GLOBAL_VOXEL: float = 0.3   # m — coarser than scan voxel for efficiency

        # Path
        self._path = Path()
        self._path.header.frame_id = self._map_frame

        self._initialized = False
        self._scan_count = 0

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/drone/lidar/points',
            self._lidar_callback, sensor_qos)
        self.create_subscription(
            Imu, '/drone/imu',
            self._imu_callback, sensor_qos)

        # PX4 position — the most accurate pose source available
        if _HAS_PX4:
            px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
            self.create_subscription(
                _PX4Pos, '/fmu/out/vehicle_local_position',
                self._px4_callback, px4_qos)
            self.get_logger().info('PX4 NED position subscribed — using as primary pose.')
        else:
            self.get_logger().warn(
                'px4_msgs not found — falling back to ICP+IMU pose.')

        # ── Publishers ────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/slam/odom', reliable_qos)
        self._map_pub = self.create_publisher(
            PointCloud2, '/slam/map_cloud', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1))
        # Per-keyframe scan in map frame — consumed by octomap_builder for raycasting.
        # Must NOT be the accumulated global cloud (that causes free-space starburst).
        self._scan_pub = self.create_publisher(
            PointCloud2, '/slam/current_scan', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=1))
        self._path_pub = self.create_publisher(Path, '/slam/path', reliable_qos)

        # ── TF broadcaster ────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'SimpleLidarOdom ready — '
            f'O3D={_HAS_O3D} scipy={_HAS_SCIPY} '
            f'voxel={self._voxel_size}m icp_dist={self._icp_max_dist}m')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _imu_callback(self, msg: Imu) -> None:
        self._imu.update(msg)

    def _px4_callback(self, msg) -> None:
        """Update pose from PX4 NED position — accurate GPS-based anchor."""
        if not (msg.xy_valid and msg.z_valid):
            return
        h = float(msg.heading) if math.isfinite(msg.heading) else self._px4_hdg
        # PX4 NED: x=North, y=East, z=Down  →  map frame z-up: (N, E, -D)
        self._px4_pos = np.array([msg.x, msg.y, -msg.z], dtype=np.float64)
        self._px4_hdg = h
        self._px4_valid = True

    def _px4_init_T(self) -> np.ndarray:
        """Build 4×4 map←body transform from current PX4 pose.

        Body frame: FRD (x=forward, y=right, z=down)  [PX4 / Gazebo x500]
        Map  frame: NED z-up (x=North, y=East, z=Up)

        Heading h: clockwise from North (standard PX4 compass bearing).
        R = R_z(h) with z-flip for Down→Up:
          FRD-x (forward) at h → (cos h,  sin h, 0)  in map
          FRD-y (right)   at h → (-sin h, cos h, 0)  in map
          FRD-z (down)         → (0, 0, -1)           in map
        Derived as: diag(1,1,-1) @ R_NED_yaw(h) where R_NED_yaw rotates by h CW.
        """
        h = self._px4_hdg
        c, s = math.cos(h), math.sin(h)
        # Columns = FRD body axes expressed in map frame
        R = np.array([
            [ c, -s, 0.0],
            [ s,  c, 0.0],
            [0.0, 0.0, -1.0],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3]  = self._px4_pos
        return T

    def _lidar_callback(self, msg: PointCloud2) -> None:
        t_start = time.monotonic()

        # Convert to numpy
        pts = pointcloud2_to_xyz_fast(msg)
        if len(pts) < 20:
            self.get_logger().warn('Too few points in scan, skipping.')
            return

        # Voxel downsample
        pts_ds = voxel_downsample(pts, self._voxel_size)
        if len(pts_ds) < 10:
            return

        stamp = msg.header.stamp

        with self._lock:
            if not self._initialized:
                self._initialize(pts_ds, stamp)
                return

            # ── Primary pose source: PX4 NED position ────────────────────────
            if self._px4_valid:
                # Use PX4 GPS/EKF position directly — far more accurate than ICP
                # in symmetric cave tunnels where ICP always diverges.
                T = self._px4_init_T()
                self._pos = T[:3, 3].copy()
                self._R   = T[:3, :3].copy()
                fitness   = 1.0   # treat as perfect match for keyframe logic

            else:
                # ── Fallback: ICP + IMU when PX4 unavailable ─────────────────
                d_pos, d_R = self._imu.get_delta_and_reset()
                imu_jump = float(np.linalg.norm(d_pos))
                if imu_jump > 2.0:
                    d_pos = np.zeros(3, dtype=np.float64)

                pred_R = self._R @ d_R
                pred_pos = self._R @ d_pos + self._pos
                init_T = np.eye(4)
                init_T[:3, :3] = pred_R
                init_T[:3, 3]  = pred_pos

                if self._local_map is not None and len(self._local_map) > 10:
                    T, fitness = self._run_icp(pts_ds, self._local_map,
                                               self._local_normals, init_T)
                else:
                    T, fitness = init_T, 0.0

                icp_jump = float(np.linalg.norm(T[:3, 3] - self._pos))
                if fitness >= 0.1 and icp_jump < 5.0:
                    self._pos = T[:3, 3].copy()
                    self._R   = T[:3, :3].copy()
                    U, _, Vt  = np.linalg.svd(self._R)
                    self._R   = (U @ Vt).astype(np.float64)
                elif imu_jump < 2.0:
                    self._pos = pred_pos
                    self._R   = pred_R

            # Publish

            self._publish_odom(stamp)
            self._publish_tf(stamp)

            # Keyframe check
            dp = np.linalg.norm(self._pos - self._kf_pos)
            dR = self._R @ self._kf_R.T
            angle = math.acos(
                max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0)))
            if dp > self._kf_dist or angle > self._kf_rot:
                self._add_keyframe(pts_ds)

        elapsed = (time.monotonic() - t_start) * 1000
        self._scan_count += 1
        if self._scan_count % 20 == 0:
            self.get_logger().info(
                f'Scan #{self._scan_count} processed in {elapsed:.1f}ms '
                f'fitness={fitness:.2f} pos={self._pos}')

    # ── Internal helpers ───────────────────────────────────────────────────

    def _initialize(self, pts: np.ndarray, stamp: RosTime) -> None:
        """Initialise from the first scan."""
        self._initialized = True
        self._add_keyframe(pts)
        self._imu.get_delta_and_reset()   # discard pre-init IMU data
        self._publish_odom(stamp)
        self._publish_tf(stamp)
        self.get_logger().info('SimpleLidarOdom initialised from first scan.')

    def _add_keyframe(self, pts_body: np.ndarray) -> None:
        """
        Transform scan to map frame, append to keyframe window,
        rebuild merged local map.
        """
        # Filter out max-range ghost returns (beams escaping the cave into open space).
        # Cave walls are always within ~15m; returns at 20-100m are noise/sky beams
        # that create the starburst artefact in the 2D/3D map.
        MAX_BODY_RANGE = 20.0   # metres from sensor; beyond this = discard
        ranges = np.linalg.norm(pts_body, axis=1)
        pts_body = pts_body[ranges < MAX_BODY_RANGE]
        if len(pts_body) == 0:
            return

        pts_map = (self._R @ pts_body.T).T + self._pos
        pts_map32 = pts_map.astype(np.float32)

        # Publish this keyframe's scan for octomap raycasting (not the accumulated cloud).
        self._scan_pub.publish(
            xyz_to_pointcloud2(pts_map32, self._map_frame,
                               self.get_clock().now().to_msg()))

        # ── Global accumulator (append-only — never loses old keyframes) ──
        if self._global_pts is None:
            self._global_pts = pts_map32
        else:
            self._global_pts = np.vstack([self._global_pts, pts_map32])
        self._global_kf_count += 1
        # Periodically downsample to control memory (every 20 new keyframes)
        if self._global_kf_count % 20 == 0:
            self._global_pts = voxel_downsample(
                self._global_pts, self._GLOBAL_VOXEL).astype(np.float32)

        # ── Local sliding window (ICP fallback only) ──────────────────────
        self._keyframes.append(pts_map32)

        self._kf_pos = self._pos.copy()
        self._kf_R = self._R.copy()

        # Merge & downsample local map
        merged = np.vstack(list(self._keyframes))
        self._local_map = voxel_downsample(merged, self._voxel_size)

        # Normal estimation
        if _HAS_O3D:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(
                self._local_map.astype(np.float64))
            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
            self._local_normals = np.asarray(pcd.normals).astype(np.float32)
        else:
            self._local_normals = estimate_normals_kdtree(self._local_map)

        # Publish map cloud
        self._publish_map_cloud()

    def _run_icp(
        self,
        source: np.ndarray,
        target: np.ndarray,
        target_normals: Optional[np.ndarray],
        init_T: np.ndarray,
    ) -> Tuple[np.ndarray, float]:
        """Dispatch to O3D or numpy ICP."""
        if _HAS_O3D:
            return icp_open3d(source, target, init_T,
                              self._icp_max_dist, self._icp_max_iter)
        else:
            if target_normals is None:
                target_normals = estimate_normals_kdtree(target)
            return icp_point_to_plane_numpy(
                source, target, target_normals, init_T,
                self._icp_max_dist, self._icp_max_iter)

    # ── Publishers ────────────────────────────────────────────────────────

    def _publish_odom(self, stamp: RosTime) -> None:
        q = rot_to_quat(self._R)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position = Point(
            x=float(self._pos[0]),
            y=float(self._pos[1]),
            z=float(self._pos[2]))
        msg.pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]),
            z=float(q[2]), w=float(q[3]))
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.001
        msg.pose.covariance[28] = 0.001
        msg.pose.covariance[35] = 0.001
        self._odom_pub.publish(msg)

        # Append to path
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._path.header.stamp = stamp
        self._path.poses.append(ps)
        if len(self._path.poses) > 5000:
            self._path.poses.pop(0)
        self._path_pub.publish(self._path)

    def _publish_tf(self, stamp: RosTime) -> None:
        q = rot_to_quat(self._R)

        # map → odom (SLAM correction — identity until loop closure)
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = stamp
        tf_map_odom.header.frame_id = self._map_frame
        tf_map_odom.child_frame_id = self._odom_frame
        tf_map_odom.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        tf_map_odom.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # odom → base_link (current odometry)
        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = stamp
        tf_odom_base.header.frame_id = self._odom_frame
        tf_odom_base.child_frame_id = self._base_frame
        tf_odom_base.transform.translation = Vector3(
            x=float(self._pos[0]),
            y=float(self._pos[1]),
            z=float(self._pos[2]))
        tf_odom_base.transform.rotation = Quaternion(
            x=float(q[0]), y=float(q[1]),
            z=float(q[2]), w=float(q[3]))

        self._tf_broadcaster.sendTransform([tf_map_odom, tf_odom_base])

    def _publish_map_cloud(self) -> None:
        """Publish the global accumulated map cloud (never loses old keyframes)."""
        if self._global_pts is None:
            return
        now = self.get_clock().now().to_msg()
        msg = xyz_to_pointcloud2(self._global_pts, self._map_frame, now)
        self._map_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleLidarOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
