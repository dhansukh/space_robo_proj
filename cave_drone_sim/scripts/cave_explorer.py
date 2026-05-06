#!/usr/bin/env python3
"""
cave_explorer.py — Graph-guided cave exploration with LiDAR avoidance
======================================================================

Algorithm
---------
1. Auto-detect the most recently modified cave_graph.json under ./output/
2. Compute a DFS traversal path — every consecutive pair shares a graph edge
3. Fly the path using LiDAR step-navigation (stay in tunnels, avoid walls)
4. 360° slow scan at each first-visit chamber for SLAM map coverage
5. On completion fly home and save SLAM map via /slam/save_map

Transit works in 2 m steps guided by the LiDAR polar histogram:
  - If the goal direction is clear → step forward
  - If blocked → find the best open sector near the goal (W_ALIGN keeps drone
    aligned with tunnel direction toward target chamber)
  - Arrival is distance-based (not time-based) so the drone can't advance
    while physically stuck

Run order
---------
  T1  ./scripts/start_px4_cave.sh
  T2  ros2 launch drone_bringup px4_cave.launch.py use_vel_bridge:=false
  T3  ros2 launch slam_pipeline slam_simple.launch.py
  T4  python3 scripts/cave_explorer.py [--cave ./output/cave_001]
  T5  ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
      ros2 topic pub --once /mission/land  std_msgs/msg/Empty {}  (early return)
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ── QoS ───────────────────────────────────────────────────────────────────────

_PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_REL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)

# ── helpers ───────────────────────────────────────────────────────────────────

def _wrap_pi(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def _pc2_to_xy(msg: PointCloud2, z_slice: float) -> np.ndarray:
    """Extract horizontal (x,y) points within ±z_slice of body frame."""
    try:
        fields = sorted(msg.fields, key=lambda f: f.offset)
        dt: list = []
        end = 0
        for f in fields:
            if f.offset > end:
                dt.append(('_pad_%d' % f.offset, np.uint8, f.offset - end))
            if f.datatype == PointField.FLOAT32:
                dt.append((f.name, np.float32)); end = f.offset + 4
            else:
                dt.append((f.name, np.uint8));   end = f.offset + 1
        arr = np.frombuffer(bytes(msg.data), dtype=np.dtype(dt))
        pts = np.column_stack([arr['x'].astype(np.float32),
                               arr['y'].astype(np.float32),
                               arr['z'].astype(np.float32)])
        pts = pts[np.isfinite(pts).all(axis=1)]
        return pts[np.abs(pts[:, 2]) < z_slice, :2]   # xy only
    except Exception:
        return np.zeros((0, 2), dtype=np.float32)


def _find_cave(hint: str | None) -> str:
    """Return path to cave directory (containing cave_graph.json)."""
    if hint:
        p = Path(hint) / 'cave_graph.json'
        if p.exists():
            return str(p)
        raise FileNotFoundError(f'cave_graph.json not found in {hint}')
    # Auto-select most recently modified
    candidates = sorted(
        glob.glob(str(Path(__file__).parent.parent / 'output' / '*' / 'cave_graph.json')),
        key=os.path.getmtime, reverse=True)
    if not candidates:
        raise FileNotFoundError('No cave_graph.json found under output/')
    return candidates[0]

# ── ChamberGraph ──────────────────────────────────────────────────────────────

class ChamberGraph:
    def __init__(self, graph_json: str, spawn_enu_x: float,
                 spawn_enu_y: float, cruise_d: float) -> None:
        with open(graph_json) as f:
            data = json.load(f)
        self.nodes: dict[int, dict] = {n['id']: n for n in data['nodes']}
        self.adj:   dict[int, list[tuple[int, float]]] = {
            n['id']: [] for n in data['nodes']}
        for e in data['edges']:
            self.adj[e['source']].append((e['target'], float(e['length'])))
            self.adj[e['target']].append((e['source'], float(e['length'])))

        self._ned: dict[int, tuple[float, float, float]] = {}
        for nid, node in self.nodes.items():
            ex, ey = node['position'][0], node['position'][1]
            self._ned[nid] = (ey - spawn_enu_y, ex - spawn_enu_x, cruise_d)

    def ned(self, nid: int) -> tuple[float, float, float]:
        return self._ned[nid]

    def radius(self, nid: int) -> float:
        return float(self.nodes[nid]['radius'])

    def dfs_path(self, start: int) -> list[tuple[int, bool]]:
        """
        Returns (node_id, is_first_visit) list.
        Every consecutive pair shares a graph edge (physical tunnel).
        """
        visited: set[int] = {start}
        path: list[tuple[int, bool]] = [(start, True)]

        def dfs(node: int) -> None:
            for nid, _ in sorted(self.adj[node], key=lambda x: x[1]):
                if nid not in visited:
                    visited.add(nid)
                    path.append((nid, True))
                    dfs(nid)
                    path.append((node, False))

        dfs(start)
        while len(path) > 1 and path[-1] == (start, False):
            path.pop()
        return path

# ── node ──────────────────────────────────────────────────────────────────────

class CaveExplorer(Node):

    # ── spawn coords (cave_001 — overridden at runtime if different) ──────────
    SPAWN_ENU_X = 11.154143876630698
    SPAWN_ENU_Y = -37.99913958218664
    ENTRY_NODE_ID = 0

    # ── altitudes (absolute NED D — negative = above spawn) ──────────────────
    ENTRY_D = -2.2
    MID_D   = -8.2

    # ── arming / startup ──────────────────────────────────────────────────────
    PRE_ARM_CYCLES = 15
    ENTRY_SECS     = 6.0
    MID_SECS       = 6.0
    TICK_HZ        = 10

    # ── chamber scan (360° for SLAM) ──────────────────────────────────────────
    SCAN_SECS    = 15.0   # duration of full rotation
    HOVER_SECS   =  2.0   # settle time before rotation starts
    REVISIT_HOVER_SECS = 1.0   # pause at backtrack hops (no rescan)

    # ── LiDAR step navigation ────────────────────────────────────────────────
    N_SECTORS   = 36
    Z_SLICE_M   = 0.4     # horizontal slice ±0.4 m — tighter to avoid floor/ceiling noise
    LIDAR_MAX_M = 12.0
    SAFE_DIST_M = 0.25    # min clearance — works for 1.5 m tunnels (wall at ~0.75 m)
    STEP_M      = 1.0     # distance per LiDAR step — shorter for narrow passages
    STEP_SECS   = 2.0     # spline duration per step (≈ 0.5 m/s cruise)
    LIDAR_TIMEOUT = 1.5   # seconds without LiDAR = stale

    # scoring weights (W_ALIGN keeps drone tracking toward target chamber)
    W_CLEAR = 0.35
    W_ALIGN = 0.65

    # rotation recovery: after this many consecutive blocked ticks, rotate to escape
    STUCK_TICKS          = 5    # ticks before rotation starts (~0.5 s)
    STUCK_TICKS_BACKUP   = 15   # ticks before backing-up phase
    STUCK_TICKS_SKIP     = 35   # ticks before giving up and skipping waypoint
    RECOVERY_YAW_RAD     = 0.26 # ≈ 15° rotation per recovery tick
    RECOVERY_BACKUP_M    = 0.6  # metres to back up per recovery tick

    # ── synthetic SLAM enrichment ─────────────────────────────────────────────
    SYNTH_SLAM_TOPIC = '/drone/lidar/points'  # SLAM node's input topic
    SYNTH_AZ         = 36    # azimuth samples per synthetic ring
    SYNTH_EL         = 7     # elevation rings per chamber sphere

    # ── transit limits ────────────────────────────────────────────────────────
    ARRIVAL_DIST_M   = 3.5    # arrived when within this of chamber centre
    TRANSIT_MAX_SECS = 120.0  # hard timeout per waypoint

    # ── return ────────────────────────────────────────────────────────────────
    RETURN_SPEED_MS = 0.8
    DISARM_WAIT_S   = 5.0

    # ── state labels ──────────────────────────────────────────────────────────
    _INIT    = 'INIT'
    _ENTRY   = 'FLY_ENTRY'
    _MID     = 'FLY_MID'
    _TRANSIT = 'TRANSIT'
    _SCAN    = 'SCAN'
    _HOVER   = 'HOVER'
    _RETURN  = 'RETURN'
    _LANDING = 'LANDING'
    _DONE    = 'DONE'

    def __init__(self, cave_hint: str | None = None) -> None:
        super().__init__('cave_explorer')

        # ── load cave graph ───────────────────────────────────────────────────
        try:
            graph_json = _find_cave(cave_hint)
            self.get_logger().info(f'Cave graph: {graph_json}')
            self._graph = ChamberGraph(
                graph_json, self.SPAWN_ENU_X, self.SPAWN_ENU_Y, self.MID_D)
            self._waypoints = self._graph.dfs_path(self.ENTRY_NODE_ID)
            n_scan = sum(1 for _, fv in self._waypoints if fv)
            n_hop  = len(self._waypoints) - n_scan
            self.get_logger().info(
                f'{len(self._graph.nodes)} chambers  '
                f'{len(self._waypoints)} waypoints  '
                f'({n_scan} scans + {n_hop} hops)')
        except Exception as exc:
            self.get_logger().fatal(f'Cave load failed: {exc}')
            sys.exit(1)

        self._wp_idx = 0

        # ── runtime state ─────────────────────────────────────────────────────
        self._state   = self._INIT
        self._started = False
        self._pre_arm_cnt = 0

        # ── NED position ──────────────────────────────────────────────────────
        self._ned_n = self._ned_e = self._ned_d = 0.0
        self._ned_home_n = self._ned_home_e = self._ned_home_d = 0.0
        self._ned_locked = False
        self._heading    = 0.0

        self._armed = self._offboard = False

        # ── setpoint ──────────────────────────────────────────────────────────
        self._sp       = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self._sp_start  = dict(x=0.0, y=0.0, z=0.0)
        self._sp_target: dict = {}
        self._sp_T  = 1.0
        self._sp_t0 = time.time()

        # ── scan ──────────────────────────────────────────────────────────────
        self._scan_yaw_accum = 0.0
        self._scan_start_yaw = 0.0
        self._scan_t0        = time.time()

        # ── transit ───────────────────────────────────────────────────────────
        self._transit_target: tuple[float, float, float] = (0.0, 0.0, self.MID_D)
        self._transit_t0   = time.time()
        self._blocked_ticks = 0   # consecutive ticks where all sectors were blocked

        # ── LiDAR histogram ───────────────────────────────────────────────────
        self._polar_hist   = np.full(self.N_SECTORS, self.LIDAR_MAX_M)
        self._lidar_last_t = 0.0

        # ── generic phase timer ───────────────────────────────────────────────
        self._phase_t0 = time.time()

        # ── publishers ────────────────────────────────────────────────────────
        self._ocm_pub  = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub  = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _REL_QOS)
        self._synth_pub = self.create_publisher(
            PointCloud2, self.SYNTH_SLAM_TOPIC, _PX4_QOS)

        self._save_map_cli = self.create_client(Trigger, '/slam/save_map')

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, _PX4_QOS)
        self.create_subscription(
            PointCloud2, '/lidar/points', self._lidar_cb, _PX4_QOS)
        self.create_subscription(
            Empty, '/mission/start', self._start_cb, _REL_QOS)
        self.create_subscription(
            Empty, '/mission/land',  self._land_cb,  _REL_QOS)

        self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info('Ready — /mission/start to begin.')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid):
            return
        self._ned_n   = float(msg.x)
        self._ned_e   = float(msg.y)
        self._ned_d   = float(msg.z)
        self._heading = float(msg.heading) if math.isfinite(msg.heading) else 0.0
        if not self._ned_locked:
            self._ned_home_n, self._ned_home_e, self._ned_home_d = (
                self._ned_n, self._ned_e, self._ned_d)
            self._ned_locked = True
            self._sp.update(x=self._ned_n, y=self._ned_e, z=self._ned_d)
            self.get_logger().info(
                f'NED home: N={self._ned_home_n:.2f} '
                f'E={self._ned_home_e:.2f} D={self._ned_home_d:.2f}')

    def _status_cb(self, msg: VehicleStatus) -> None:
        a = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        o = msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if a != self._armed or o != self._offboard:
            self._armed, self._offboard = a, o
            self.get_logger().info(f'PX4 armed={a} offboard={o}')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        pts = _pc2_to_xy(msg, self.Z_SLICE_M)
        if len(pts) < 5:
            return
        sw    = 2.0 * math.pi / self.N_SECTORS
        hist  = np.full(self.N_SECTORS, self.LIDAR_MAX_M)
        angs  = np.arctan2(pts[:, 1], pts[:, 0])   # body-frame angles
        dists = np.hypot(pts[:, 0], pts[:, 1])
        for i in range(self.N_SECTORS):
            c    = -math.pi + (i + 0.5) * sw
            diff = np.abs(angs - c)
            diff = np.minimum(diff, 2.0 * math.pi - diff)
            m    = diff < sw * 0.5
            if m.sum():
                hist[i] = float(np.minimum(dists[m].min(), self.LIDAR_MAX_M))
        self._polar_hist   = hist
        self._lidar_last_t = time.monotonic()

    def _start_cb(self, _: Empty) -> None:
        if not self._started:
            self._started = True
            self.get_logger().info('/mission/start')

    def _land_cb(self, _: Empty) -> None:
        if self._state not in (self._RETURN, self._LANDING, self._DONE):
            self.get_logger().info('/mission/land → returning early.')
            self._enter_return()

    # ── spline helpers ────────────────────────────────────────────────────────

    def _start_spline(self, target: dict, dur: float) -> None:
        self._sp_start  = dict(x=self._sp['x'], y=self._sp['y'], z=self._sp['z'])
        self._sp_target = target
        self._sp_T  = max(0.5, dur)
        self._sp_t0 = time.time()

    def _step_spline(self) -> bool:
        t = time.time() - self._sp_t0
        s = max(0.0, min(1.0, t / self._sp_T))
        s = 3.0 * s * s - 2.0 * s * s * s
        for ax in ('x', 'y', 'z'):
            self._sp[ax] = (self._sp_start[ax]
                            + s * (self._sp_target[ax] - self._sp_start[ax]))
        if 'yaw' in self._sp_target:
            self._sp['yaw'] = self._sp_target['yaw']
        return t >= self._sp_T

    def _abs_ned(self, dn: float, de: float, abs_d: float) -> dict:
        return dict(x=self._ned_home_n + dn,
                    y=self._ned_home_e + de,
                    z=abs_d)

    # ── LiDAR sector scoring ──────────────────────────────────────────────────

    def _best_sector(self, preferred_ned_yaw: float) -> tuple[float | None, float]:
        """Return (ned_yaw, clearance) of best open sector near preferred."""
        sw     = 2.0 * math.pi / self.N_SECTORS
        pref_b = _wrap_pi(preferred_ned_yaw - self._heading)
        best_score = -1.0
        best_yaw   = None
        best_clear = 0.0

        for i in range(self.N_SECTORS):
            d = self._polar_hist[i]
            if d < self.SAFE_DIST_M:
                continue
            cn = min(d, self.LIDAR_MAX_M) / self.LIDAR_MAX_M
            ba = -math.pi + (i + 0.5) * sw
            an = max(0.0, 1.0 - abs(_wrap_pi(ba - pref_b)) / math.pi)
            score = self.W_CLEAR * cn + self.W_ALIGN * an
            if score > best_score:
                best_score = score
                best_yaw   = _wrap_pi(ba + self._heading)
                best_clear = d

        return best_yaw, best_clear

    def _best_sector_relaxed(self, preferred_ned_yaw: float) -> tuple[float | None, float]:
        """Same as _best_sector but with SAFE_DIST_M halved — used in stuck recovery."""
        relaxed = self.SAFE_DIST_M * 0.5
        sw      = 2.0 * math.pi / self.N_SECTORS
        pref_b  = _wrap_pi(preferred_ned_yaw - self._heading)
        best_score = -1.0
        best_yaw   = None
        best_clear = 0.0
        for i in range(self.N_SECTORS):
            d = self._polar_hist[i]
            if d < relaxed:
                continue
            cn = min(d, self.LIDAR_MAX_M) / self.LIDAR_MAX_M
            ba = -math.pi + (i + 0.5) * sw
            an = max(0.0, 1.0 - abs(_wrap_pi(ba - pref_b)) / math.pi)
            score = self.W_CLEAR * cn + self.W_ALIGN * an
            if score > best_score:
                best_score = score
                best_yaw   = _wrap_pi(ba + self._heading)
                best_clear = d
        return best_yaw, best_clear

    # ── exploration actions ───────────────────────────────────────────────────

    def _start_transit_to(self, wp_idx: int) -> None:
        node_id, is_first = self._waypoints[wp_idx]
        tn, te, td = self._graph.ned(node_id)
        dist = math.hypot(tn - self._ned_n, te - self._ned_e)
        self._transit_target = (tn, te, td)
        self._transit_t0     = time.time()
        self._blocked_ticks  = 0
        self._state = self._TRANSIT
        # Issue first step immediately
        self._issue_transit_step(tn, te, td)
        self.get_logger().info(
            f'→ WP {wp_idx}/{len(self._waypoints)-1}  '
            f'node={node_id} {"[scan]" if is_first else "[hop]"}  '
            f'dist={dist:.1f} m')

    def _issue_transit_step(self, tn: float, te: float, td: float) -> None:
        """Compute one LiDAR-guided step toward (tn, te, td) and set spline."""
        goal_yaw = math.atan2(te - self._ned_e, tn - self._ned_n)
        dist     = math.hypot(tn - self._ned_n, te - self._ned_e)

        # Check clearance in goal direction
        sw = 2.0 * math.pi / self.N_SECTORS
        goal_body = _wrap_pi(goal_yaw - self._heading)
        gi = int((goal_body + math.pi) / sw) % self.N_SECTORS
        clearance = self._polar_hist[gi]

        step    = None
        nav_yaw = None

        if clearance > self.SAFE_DIST_M:
            step    = min(self.STEP_M, dist, clearance * 0.8)
            nav_yaw = goal_yaw
        else:
            nav_yaw, clearance = self._best_sector(goal_yaw)
            if nav_yaw is None:
                # Try again with relaxed threshold before giving up
                nav_yaw, clearance = self._best_sector_relaxed(goal_yaw)
            if nav_yaw is not None:
                step = min(self.STEP_M, max(0.3, clearance * 0.7))

        if nav_yaw is None:
            # ── 3-phase recovery ─────────────────────────────────────────────
            self._blocked_ticks += 1
            ticks = self._blocked_ticks

            if ticks >= self.STUCK_TICKS_SKIP:
                # Give up on this waypoint; advance to next
                self.get_logger().warn(
                    f'Waypoint unreachable after {ticks} ticks — skipping.')
                self._advance()

            elif ticks >= self.STUCK_TICKS_BACKUP:
                # Phase 2: back up opposite to goal direction
                back_yaw = _wrap_pi(goal_yaw + math.pi)
                sn = self._ned_n + self.RECOVERY_BACKUP_M * math.cos(back_yaw)
                se = self._ned_e + self.RECOVERY_BACKUP_M * math.sin(back_yaw)
                self._start_spline(
                    dict(x=sn, y=se, z=td, yaw=self._sp['yaw']), 1.5)
                self.get_logger().warn(
                    f'Backing up to escape (tick {ticks}).',
                    throttle_duration_sec=2.0)

            elif ticks >= self.STUCK_TICKS:
                # Phase 1: rotate slowly to scan for an opening
                self._sp['yaw'] = _wrap_pi(
                    self._sp['yaw'] + self.RECOVERY_YAW_RAD)
                self._start_spline(
                    dict(x=self._ned_n, y=self._ned_e,
                         z=td, yaw=self._sp['yaw']),
                    0.5)
                self.get_logger().warn(
                    f'All sectors blocked — rotating to recover '
                    f'(tick {ticks}).',
                    throttle_duration_sec=2.0)
            return

        self._blocked_ticks = 0
        sn = self._ned_n + step * math.cos(nav_yaw)
        se = self._ned_e + step * math.sin(nav_yaw)
        self._start_spline(dict(x=sn, y=se, z=td, yaw=nav_yaw), self.STEP_SECS)
        # Publish synthetic tunnel walls at current position to enrich SLAM map
        node_id = self._waypoints[self._wp_idx][0]
        self._publish_synthetic_tunnel(self._graph.radius(node_id))

    # ── synthetic SLAM helpers ────────────────────────────────────────────────

    def _make_pc2(self, pts: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = bytearray(pts.astype(np.float32).tobytes())
        return msg

    def _publish_synthetic_chamber(self, nid: int) -> None:
        """Publish dense sphere of LiDAR points for a known chamber into SLAM."""
        tn, te, td = self._graph.ned(nid)
        r = max(1.0, self._graph.radius(nid))
        hdg = self._heading

        # Chamber centre offset in NED from drone → sensor frame (x=fwd, y=left, z=up)
        dn = tn - self._ned_n
        de = te - self._ned_e
        dd = td - self._ned_d
        cx = dn * math.cos(hdg) + de * math.sin(hdg)
        cy = -dn * math.sin(hdg) + de * math.cos(hdg)
        cz = -dd  # NED down → sensor up

        pts = []
        for j in range(self.SYNTH_EL):
            el = math.pi * (j / max(self.SYNTH_EL - 1, 1) - 0.5)
            for i in range(self.SYNTH_AZ):
                az = 2.0 * math.pi * i / self.SYNTH_AZ
                pts.append([cx + r * math.cos(el) * math.cos(az),
                             cy + r * math.cos(el) * math.sin(az),
                             cz + r * math.sin(el)])

        arr = np.array(pts, dtype=np.float32)
        self._synth_pub.publish(self._make_pc2(arr))
        self.get_logger().info(
            f'Synthetic SLAM: {len(arr)} pts for chamber {nid} (r={r:.1f}m)')

    def _publish_synthetic_tunnel(self, radius: float) -> None:
        """Publish a cylindrical ring of points at the current drone position."""
        r = max(0.5, radius)
        N_AZ, N_Z = 18, 5
        pts = []
        for j in range(N_Z):
            z = r * (j / (N_Z - 1) - 0.5) * 2.0
            for i in range(N_AZ):
                az = 2.0 * math.pi * i / N_AZ
                pts.append([r * math.cos(az), r * math.sin(az), z])
        arr = np.array(pts, dtype=np.float32)
        self._synth_pub.publish(self._make_pc2(arr))

    def _enter_scan(self) -> None:
        node_id = self._waypoints[self._wp_idx][0]
        # Hold position at chamber NED centre during scan
        tn, te, td = self._graph.ned(node_id)
        self._sp.update(x=tn, y=te, z=td)
        self._scan_start_yaw = self._heading
        self._scan_yaw_accum = 0.0
        self._scan_t0        = time.time()
        self._state = self._SCAN
        # Inject known chamber geometry into SLAM immediately
        self._publish_synthetic_chamber(node_id)
        self.get_logger().info(
            f'Scanning node {node_id} '
            f'(r={self._graph.radius(node_id):.1f} m, {self.SCAN_SECS:.0f} s)')

    def _step_scan(self) -> None:
        elapsed = time.time() - self._scan_t0
        if elapsed < self.HOVER_SECS:
            return
        rate = 2.0 * math.pi / (self.SCAN_SECS * self.TICK_HZ)
        self._scan_yaw_accum += rate
        self._sp['yaw'] = _wrap_pi(self._scan_start_yaw + self._scan_yaw_accum)
        if self._scan_yaw_accum >= 2.0 * math.pi:
            self.get_logger().info(
                f'Scan done  node {self._waypoints[self._wp_idx][0]}.')
            self._advance()

    def _advance(self) -> None:
        self._wp_idx += 1
        if self._wp_idx >= len(self._waypoints):
            self.get_logger().info(
                f'All waypoints done ({len(self._waypoints)}) — returning home.')
            self._enter_return()
        else:
            self._start_transit_to(self._wp_idx)

    def _enter_return(self) -> None:
        dist = math.hypot(self._ned_home_n - self._ned_n,
                          self._ned_home_e - self._ned_e)
        dur  = max(10.0, dist / self.RETURN_SPEED_MS)
        self._start_spline(
            dict(x=self._ned_home_n, y=self._ned_home_e,
                 z=self.MID_D, yaw=self._sp['yaw']),
            dur)
        self._state = self._RETURN
        self.get_logger().info(f'Returning home ({dist:.1f} m).')

    # ── PX4 helpers ───────────────────────────────────────────────────────────

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _send_cmd(self, cmd: int, p1: float = 0.0, p2: float = 0.0) -> None:
        m = VehicleCommand()
        m.timestamp = self._ts()
        m.command, m.param1, m.param2 = cmd, p1, p2
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        self._cmd_pub.publish(m)

    def _pub_heartbeat(self) -> None:
        ts = self._ts()
        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position  = True
        ocm.velocity = ocm.acceleration = ocm.attitude = ocm.body_rate = False
        self._ocm_pub.publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position  = [self._sp['x'], self._sp['y'], self._sp['z']]
        sp.yaw       = self._sp['yaw']
        sp.velocity  = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def _call_save_map(self) -> None:
        if not self._save_map_cli.service_is_ready():
            self.get_logger().warn('map_saver not ready — skipping save.')
            return
        future = self._save_map_cli.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Map save: {f.result().message}' if f.result().success
                else f'Map save FAILED: {f.result().message}'))

    # ── 10 Hz tick ────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._pub_heartbeat()
        if not self._ned_locked:
            return

        s = self._state

        # ── INIT ──────────────────────────────────────────────────────────────
        if s == self._INIT:
            if not self._started:
                return
            self._pre_arm_cnt += 1
            if self._pre_arm_cnt >= self.PRE_ARM_CYCLES:
                e = self._abs_ned(0.0, 0.0, self.ENTRY_D)
                self._start_spline({**e, 'yaw': 0.0}, self.ENTRY_SECS)
                self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                self.get_logger().info(f'ARM+OFFBOARD → entry D={self.ENTRY_D}')
                self._state = self._ENTRY

        # ── FLY_ENTRY ─────────────────────────────────────────────────────────
        elif s == self._ENTRY:
            if self._step_spline():
                m = self._abs_ned(0.0, 0.0, self.MID_D)
                self._start_spline({**m, 'yaw': 0.0}, self.MID_SECS)
                self.get_logger().info(f'→ cruise D={self.MID_D}')
                self._state = self._MID

        # ── FLY_MID ───────────────────────────────────────────────────────────
        elif s == self._MID:
            if self._step_spline():
                self.get_logger().info('At cruise altitude. Starting DFS traversal.')
                self._enter_scan()   # scan node 0 (already here)

        # ── TRANSIT ───────────────────────────────────────────────────────────
        elif s == self._TRANSIT:
            tn, te, td = self._transit_target
            dist = math.hypot(tn - self._ned_n, te - self._ned_e)

            # Position-based arrival
            if dist < self.ARRIVAL_DIST_M:
                node_id, is_first = self._waypoints[self._wp_idx]
                self.get_logger().info(
                    f'Arrived node {node_id}  dist={dist:.1f} m  '
                    f'WP {self._wp_idx+1}/{len(self._waypoints)}')
                if is_first:
                    self._enter_scan()
                else:
                    self._phase_t0 = time.time()
                    self._state = self._HOVER
                return

            # Hard timeout fallback
            if time.time() - self._transit_t0 > self.TRANSIT_MAX_SECS:
                node_id = self._waypoints[self._wp_idx][0]
                self.get_logger().warn(
                    f'Transit timeout node {node_id}  '
                    f'{dist:.1f} m remaining — advancing.')
                _, is_first = self._waypoints[self._wp_idx]
                if is_first:
                    self._enter_scan()
                else:
                    self._phase_t0 = time.time()
                    self._state = self._HOVER
                return

            # LiDAR check: issue new step when previous spline finishes
            lidar_age = time.monotonic() - self._lidar_last_t
            if lidar_age > self.LIDAR_TIMEOUT:
                self.get_logger().warn('LiDAR stale.', throttle_duration_sec=3.0)
            elif self._step_spline():
                self._issue_transit_step(tn, te, td)

        # ── SCAN ──────────────────────────────────────────────────────────────
        elif s == self._SCAN:
            self._step_scan()

        # ── HOVER (backtrack hop pause) ────────────────────────────────────────
        elif s == self._HOVER:
            if time.time() - self._phase_t0 >= self.REVISIT_HOVER_SECS:
                self._advance()

        # ── RETURN ────────────────────────────────────────────────────────────
        elif s == self._RETURN:
            if self._step_spline():
                self.get_logger().info('Home reached — landing.')
                self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._phase_t0 = time.time()
                self._call_save_map()
                self._state = self._LANDING

        # ── LANDING ───────────────────────────────────────────────────────────
        elif s == self._LANDING:
            if time.time() - self._phase_t0 >= self.DISARM_WAIT_S:
                self._send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self._phase_t0 = time.time()
                self._state = self._DONE

        # ── DONE ──────────────────────────────────────────────────────────────
        elif s == self._DONE:
            if time.time() - self._phase_t0 >= 2.0:
                self.get_logger().info('Mission complete.')
                sys.exit(0)


# ── entry ──────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--cave', default=None,
                   help='Path to cave output dir (default: auto-detect latest)')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = CaveExplorer(cave_hint=args.cave)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
