#!/usr/bin/env python3
"""
map_saver.py
============
Utility node that saves maps to disk on request.

Services
--------
  /slam/save_map         — saves the current OctoMap (if available)
  /slam/save_pointcloud  — saves the accumulated point cloud as PCD/PLY

Publications
------------
  None (service-only node)

Subscriptions
-------------
  /slam/map_cloud      (sensor_msgs/PointCloud2) — current map cloud
  /slam/occupancy_grid (nav_msgs/OccupancyGrid)  — current 2-D grid (for .pgm/.yaml)
"""

from __future__ import annotations

import os
import struct
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# PointCloud2 → numpy
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
# PCD / PLY writers (pure-python, no dependency on pcl or open3d)
# ---------------------------------------------------------------------------

def save_as_pcd(points: np.ndarray, filepath: str) -> None:
    """Save (N, 3) float32 array to an ASCII PCD file."""
    n = len(points)
    with open(filepath, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {n}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {n}\n')
        f.write('DATA ascii\n')
        for pt in points:
            f.write(f'{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n')


def save_as_pcd_binary(points: np.ndarray, filepath: str) -> None:
    """Save (N, 3) float32 array to a binary PCD file (faster for large clouds)."""
    n = len(points)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())


def save_as_ply(points: np.ndarray, filepath: str) -> None:
    """Save (N, 3) float32 array to an ASCII PLY file."""
    n = len(points)
    with open(filepath, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {n}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('end_header\n')
        for pt in points:
            f.write(f'{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n')


def save_2d_grid_pgm(grid: OccupancyGrid, out_dir: str, base: str) -> None:
    """
    Save a nav_msgs/OccupancyGrid as a ROS-compatible .pgm + .yaml pair.
    Also saves a .png for easy viewing.
    """
    W = grid.info.width
    H = grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    data = np.array(grid.data, dtype=np.int8).reshape(H, W)

    # ── Morphological closing on occupied cells ───────────────────────────
    # Dilate walls 3 voxels to fill gaps between sparse LiDAR hits,
    # then erode 1 voxel to avoid bloating tunnels.
    occ_mask = data == 100
    if occ_mask.any():
        try:
            from scipy.ndimage import binary_dilation, binary_erosion
            occ_mask = binary_dilation(occ_mask, iterations=2)
            occ_mask = binary_erosion(occ_mask, iterations=1)
        except ImportError:
            # numpy fallback: simple 3x3 neighbourhood max
            import numpy.lib.stride_tricks as nst
            pad = np.pad(occ_mask.astype(np.uint8), 3, constant_values=0)
            for _ in range(3):
                occ_mask = nst.sliding_window_view(pad, (3, 3)).max(
                    axis=(-2, -1)) > 0
                pad = np.pad(occ_mask.astype(np.uint8), 1, constant_values=0)

    # Convert occupancy values for PGM: 100→0 (black), -1→128 (grey)
    # free (0) → 205 (light grey), but we skip free cells entirely now
    img = np.full((H, W), 128, dtype=np.uint8)  # all unknown
    img[occ_mask] = 0                             # occupied → black

    pgm_path  = os.path.join(out_dir, base + '.pgm')
    png_path  = os.path.join(out_dir, base + '.png')
    yaml_path = os.path.join(out_dir, base + '.yaml')

    # Write PGM (P5 binary greyscale)
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())

    # Write PNG (easier to open)
    try:
        import struct, zlib
        def _png(fn, arr):
            h, w = arr.shape
            raw = b''.join(b'\x00' + bytes(arr[r]) for r in range(h))
            def ck(n, d):
                c = struct.pack('>I', len(d)) + n + d
                return c + struct.pack('>I', zlib.crc32(c[4:]) & 0xffffffff)
            open(fn, 'wb').write(
                b'\x89PNG\r\n\x1a\n' +
                ck(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)) +
                ck(b'IDAT', zlib.compress(raw, 6)) +
                ck(b'IEND', b''))
        _png(png_path, img)
    except Exception:
        pass   # PNG is bonus — don't fail if something goes wrong

    # Write YAML metadata (compatible with map_server)
    with open(yaml_path, 'w') as f:
        f.write(f'image: {base}.pgm\n')
        f.write(f'resolution: {res}\n')
        f.write(f'origin: [{ox}, {oy}, 0.0]\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
        f.write('negate: 0\n')


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class MapSaver(Node):
    """
    Saves SLAM maps to disk on-demand via ROS 2 services.

    Listens to map topics passively and stores the latest message in memory.
    When a service is called, writes to the configured output directory.
    """

    def __init__(self) -> None:
        super().__init__('map_saver')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('output_dir', '/tmp/slam_maps')
        self.declare_parameter('cloud_format', 'pcd')   # 'pcd', 'pcd_binary', 'ply'
        self.declare_parameter('auto_save_interval', 0.0)  # 0 = disabled

        self._out_dir = self.get_parameter('output_dir').value
        self._cloud_fmt = self.get_parameter('cloud_format').value
        self._auto_interval = float(
            self.get_parameter('auto_save_interval').value)

        os.makedirs(self._out_dir, exist_ok=True)

        # ── State ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._latest_cloud: Optional[PointCloud2] = None
        self._latest_grid: Optional[OccupancyGrid] = None
        self._save_count = 0

        # ── QoS ───────────────────────────────────────────────────────────
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/slam/map_cloud',
            self._cloud_callback, map_qos)
        self.create_subscription(
            OccupancyGrid, '/slam/occupancy_grid',
            self._grid_callback, map_qos)

        # ── Services ──────────────────────────────────────────────────────
        self.create_service(
            Trigger, '/slam/save_map', self._save_map_srv)
        self.create_service(
            Trigger, '/slam/save_pointcloud', self._save_cloud_srv)

        # ── Auto-save timer ───────────────────────────────────────────────
        if self._auto_interval > 0:
            self.create_timer(self._auto_interval, self._auto_save)
            self.get_logger().info(
                f'Auto-save enabled every {self._auto_interval:.0f}s')

        self.get_logger().info(
            f'MapSaver ready — output_dir={self._out_dir} '
            f'format={self._cloud_fmt}')

    # ── Subscribers ───────────────────────────────────────────────────────

    def _cloud_callback(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_cloud = msg

    def _grid_callback(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._latest_grid = msg

    # ── Services ──────────────────────────────────────────────────────────

    def _save_cloud_srv(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Save the accumulated point cloud."""
        with self._lock:
            cloud_msg = self._latest_cloud

        if cloud_msg is None:
            response.success = False
            response.message = 'No point cloud received yet.'
            return response

        pts = _pc2_to_xyz(cloud_msg)
        if len(pts) == 0:
            response.success = False
            response.message = 'Point cloud is empty.'
            return response

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._save_count += 1

        try:
            if self._cloud_fmt == 'ply':
                ext = '.ply'
                path = os.path.join(self._out_dir,
                                    f'map_{timestamp}_{self._save_count}{ext}')
                save_as_ply(pts, path)
            elif self._cloud_fmt == 'pcd_binary':
                ext = '.pcd'
                path = os.path.join(self._out_dir,
                                    f'map_{timestamp}_{self._save_count}{ext}')
                save_as_pcd_binary(pts, path)
            else:
                ext = '.pcd'
                path = os.path.join(self._out_dir,
                                    f'map_{timestamp}_{self._save_count}{ext}')
                save_as_pcd(pts, path)

            msg = (f'Saved {len(pts)} points to {path}')
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        except Exception as exc:
            response.success = False
            response.message = f'Save failed: {exc}'
            self.get_logger().error(response.message)

        return response

    def _save_map_srv(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Save OctoMap (via occupancy grid + point cloud)."""
        with self._lock:
            grid_msg = self._latest_grid
            cloud_msg = self._latest_cloud

        if grid_msg is None and cloud_msg is None:
            response.success = False
            response.message = 'No map data received yet.'
            return response

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._save_count += 1
        saved = []

        try:
            if grid_msg is not None:
                base = f'grid_{timestamp}_{self._save_count}'
                save_2d_grid_pgm(grid_msg, self._out_dir, base)
                saved.append(f'{base}.pgm + {base}.yaml')

            if cloud_msg is not None:
                pts = _pc2_to_xyz(cloud_msg)
                if len(pts) > 0:
                    # Filter out diverged/corrupted points (ICP failure artefacts)
                    MAX_COORD = 500.0   # metres — any real cave fits in this
                    mask = np.all(np.abs(pts) < MAX_COORD, axis=1)
                    n_bad = int((~mask).sum())
                    pts_clean = pts[mask]
                    if n_bad > 0:
                        self.get_logger().warn(
                            f'Filtered {n_bad} corrupt points (|xyz|>{MAX_COORD}m) '
                            f'before saving — {len(pts_clean)} points remain.')
                    if len(pts_clean) > 0:
                        fname = f'cloud_{timestamp}_{self._save_count}.pcd'
                        path = os.path.join(self._out_dir, fname)
                        save_as_pcd_binary(pts_clean, path)
                        saved.append(f'{fname} ({len(pts_clean)} pts)')

            msg = 'Saved: ' + ', '.join(saved)
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        except Exception as exc:
            response.success = False
            response.message = f'Save failed: {exc}'
            self.get_logger().error(response.message)

        return response

    # ── Auto-save ─────────────────────────────────────────────────────────

    def _auto_save(self) -> None:
        """Periodically save the current map without a service call."""
        req = Trigger.Request()
        resp = Trigger.Response()
        self._save_map_srv(req, resp)
        if not resp.success:
            self.get_logger().warn(f'Auto-save failed: {resp.message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
