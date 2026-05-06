#!/usr/bin/env python3
"""
degeneracy_detector.py
======================
Detects geometrically degenerate environments (e.g. featureless tunnels/
corridors) where LiDAR-based SLAM is likely to fail.

Degenerate geometry is characterised by point clouds that lie predominantly
in a plane or along a line — the eigenvalues of the point-cloud covariance
matrix reflect this.  A very small minimum eigenvalue indicates the scan
provides little constraint in at least one direction.

Subscriptions
-------------
  /drone/lidar/points     (sensor_msgs/PointCloud2)

Publications
------------
  /slam/degeneracy_warning  (std_msgs/Bool)   — True when degenerate
  /slam/degeneracy_score    (std_msgs/Float64) — 1.0=well-conditioned, 0.0=degenerate
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float64


# ---------------------------------------------------------------------------
# Minimal PointCloud2 → numpy helper (inline)
# ---------------------------------------------------------------------------

def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
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


# ---------------------------------------------------------------------------
# Degeneracy analysis
# ---------------------------------------------------------------------------

def compute_degeneracy_score(
    points: np.ndarray,
    voxel_size: float = 0.3,
) -> float:
    """
    Compute a degeneracy score in [0, 1].

    The score is based on the condition number of the 3-D point-cloud
    covariance matrix.  A well-spread cloud (rock faces, diverse walls)
    yields λ_min / λ_max close to 1.  A tunnel-like cloud where points
    concentrate in a plane has λ_min ≈ 0.

    Parameters
    ----------
    points     : (N, 3) float array
    voxel_size : downsampling grid size in metres

    Returns
    -------
    score : float in [0, 1]
        0 = fully degenerate (no geometric information in some axis)
        1 = well-conditioned (information in all directions)
    """
    if len(points) < 10:
        return 0.0

    # Voxel downsample for speed
    if voxel_size > 0:
        vox = np.floor(points / voxel_size).astype(np.int32)
        _, idx = np.unique(vox, axis=0, return_index=True)
        points = points[idx]

    if len(points) < 4:
        return 0.0

    # Covariance matrix
    centered = points - points.mean(axis=0)
    cov = (centered.T @ centered) / max(len(centered) - 1, 1)

    # Eigenvalues (always real for symmetric matrix)
    try:
        eigvals = np.linalg.eigvalsh(cov)  # sorted ascending
    except np.linalg.LinAlgError:
        return 0.0

    eigvals = np.abs(eigvals)
    lam_min = float(eigvals[0])
    lam_max = float(eigvals[2])

    if lam_max < 1e-9:
        return 0.0

    # Normalise: ratio of min to max eigenvalue
    raw_score = lam_min / lam_max

    # Clamp to [0, 1]
    return float(np.clip(raw_score, 0.0, 1.0))


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class DegeneracyDetector(Node):
    """
    Monitors LiDAR scans and emits degeneracy warnings.

    A sliding window of recent degeneracy scores is averaged to smooth
    out transient geometry changes.  When the smoothed score falls below
    the configured threshold a warning is published.

    Parameters
    ----------
    eigenvalue_threshold : float
        Minimum acceptable score.  Scans below this trigger a warning.
        Default: 0.01
    window_size : int
        Number of recent scans to average over.  Default: 5
    voxel_size : float
        Downsampling voxel size for covariance computation (metres).
        Default: 0.3
    """

    def __init__(self) -> None:
        super().__init__('degeneracy_detector')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('eigenvalue_threshold', 0.01)
        self.declare_parameter('window_size', 5)
        self.declare_parameter('voxel_size', 0.3)
        self.declare_parameter('min_points', 20)

        self._threshold = self.get_parameter('eigenvalue_threshold').value
        self._window_size = int(self.get_parameter('window_size').value)
        self._voxel_size = self.get_parameter('voxel_size').value
        self._min_pts = int(self.get_parameter('min_points').value)

        # ── State ─────────────────────────────────────────────────────────
        self._scores: deque[float] = deque(maxlen=self._window_size)
        self._lock = threading.Lock()
        self._last_warning = False
        self._scan_count = 0

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5)

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/drone/lidar/points',
            self._lidar_callback, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self._warn_pub = self.create_publisher(
            Bool, '/slam/degeneracy_warning', 10)
        self._score_pub = self.create_publisher(
            Float64, '/slam/degeneracy_score', 10)

        self.get_logger().info(
            f'DegeneracyDetector ready — '
            f'threshold={self._threshold} window={self._window_size}')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _lidar_callback(self, msg: PointCloud2) -> None:
        pts = _pc2_to_xyz(msg)
        if len(pts) < self._min_pts:
            return

        score = compute_degeneracy_score(pts, self._voxel_size)

        with self._lock:
            self._scores.append(score)
            smoothed = float(np.mean(self._scores))
            is_degenerate = smoothed < self._threshold
            self._scan_count += 1

        # Publish
        warn_msg = Bool()
        warn_msg.data = is_degenerate
        self._warn_pub.publish(warn_msg)

        score_msg = Float64()
        score_msg.data = smoothed
        self._score_pub.publish(score_msg)

        # Log state transitions
        if is_degenerate != self._last_warning:
            if is_degenerate:
                self.get_logger().warn(
                    f'[DEGENERACY] Degenerate geometry detected! '
                    f'score={smoothed:.4f} < threshold={self._threshold}. '
                    f'Recommend: reduce speed, use IMU-heavy prediction.')
            else:
                self.get_logger().info(
                    f'[DEGENERACY] Geometry recovered. score={smoothed:.4f}')
            self._last_warning = is_degenerate

        if self._scan_count % 50 == 0:
            self.get_logger().debug(
                f'Degeneracy score={smoothed:.4f} '
                f'degenerate={is_degenerate} n_scans={self._scan_count}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DegeneracyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
