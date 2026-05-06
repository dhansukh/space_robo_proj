#!/usr/bin/env python3
"""
slam_mapper.py — BFS-ordered systematic cave SLAM mapper
=========================================================

Algorithm
---------
1. Load cave_graph.json → entrance node ENU (drone spawn reference)
2. Build BFS spanning-tree walk: every consecutive pair is a graph edge,
   but exploration order is nearest-first (better loop closures early)
3. Lock home NED from first valid VehicleLocalPosition reading (dynamic)
4. At each first-visit chamber: hover + 360° slow yaw scan + synthetic
   point cloud injection from known cave geometry
5. Backtrack hops (revisit nodes) pause briefly with no rescan
6. Return home and save SLAM map

Improvements over cave_explorer.py
-----------------------------------
- BFS spanning-tree walk: nearest chambers first → shorter initial drift,
  earlier loop closures for simple ICP backend
- Dynamic home from VehicleLocalPosition (no hardcoded spawn coordinates)
- Slower transit speed (TRANSIT_SPEED) → denser scan-to-scan overlap
- Smooth yaw interpolation toward target during transit
- Configurable cruise altitude via CRUISE_D

Usage
-----
  python3 scripts/slam_mapper.py [--cave ./output/cave_001]
  ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
  ros2 topic pub --once /mission/land  std_msgs/msg/Empty {}  # early return
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from collections import deque
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


# ── cave graph helpers ─────────────────────────────────────────────────────────

def _find_cave(hint: str | None) -> str:
    if hint:
        p = Path(hint) / 'cave_graph.json'
        if p.exists():
            return str(p)
        raise FileNotFoundError(f'cave_graph.json not found in {hint}')
    candidates = sorted(
        glob.glob(str(Path(__file__).parent.parent / 'output' / '*' / 'cave_graph.json')),
        key=os.path.getmtime, reverse=True)
    if not candidates:
        raise FileNotFoundError('No cave_graph.json found under output/')
    return candidates[0]


def _bfs_tree_dfs_walk(
    adj: dict[int, list[tuple[int, float]]], start: int
) -> list[tuple[int, bool]]:
    """
    Build a BFS spanning tree from *start*, then walk it with DFS.
    Consecutive entries are always graph-adjacent (connected by a tunnel edge).
    BFS spanning tree ensures we explore nearest chambers first.

    Returns list of (node_id, is_first_visit).
    """
    # Phase 1: BFS spanning tree (children sorted by edge length = nearest first)
    visited: set[int] = {start}
    children: dict[int, list[int]] = {start: []}
    queue: deque[int] = deque([start])
    while queue:
        node = queue.popleft()
        for neighbor, _ in sorted(adj[node], key=lambda x: x[1]):
            if neighbor not in visited:
                visited.add(neighbor)
                children[node].append(neighbor)
                children[neighbor] = []
                queue.append(neighbor)

    # Phase 2: DFS walk on the BFS tree (gives graph-adjacent path)
    result: list[tuple[int, bool]] = []

    def _dfs(n: int) -> None:
        result.append((n, True))
        for child in children.get(n, []):
            _dfs(child)
            result.append((n, False))  # backtrack to parent

    _dfs(start)
    # Remove trailing redundant backtracks to start
    while len(result) > 1 and result[-1] == (start, False):
        result.pop()
    return result


def _smooth(t: float, T: float) -> float:
    """Cubic ease-in-out: 0 → 1 over duration T."""
    s = max(0.0, min(1.0, t / T))
    return 3.0 * s * s - 2.0 * s * s * s


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── main node ──────────────────────────────────────────────────────────────────

class SlamMapper(Node):
    """
    Systematic SLAM mapper: loads the cave graph, plans a BFS-order traversal,
    and flies the drone through every chamber with a full 360° scan at each.
    """

    # Cruise altitude: NED Down (negative = above spawn)
    # Adjust for your cave height. Default 5 m cave → 2 m above spawn is safe.
    CRUISE_D: float = -2.0

    TAKEOFF_SECS: float = 6.0
    TRANSIT_SPEED: float = 0.4   # m/s — slow for dense LiDAR scan overlap
    SCAN_SECS: float = 20.0      # seconds for one full 360° chamber scan
    HOVER_SECS: float = 2.0      # settle time before rotation starts
    HOP_SECS: float = 1.5        # pause at backtrack hops (no rescan)
    DISARM_WAIT: float = 5.0

    # Synthetic point cloud: azimuth × elevation samples per chamber sphere
    SYNTH_AZ: int = 36
    SYNTH_EL: int = 9

    TICK_HZ: int = 10
    PRE_ARM_CYCLES: int = 15

    def __init__(self, cave_hint: str | None = None) -> None:
        super().__init__('slam_mapper')

        # ── load cave graph ───────────────────────────────────────────────────
        graph_json = _find_cave(cave_hint)
        self.get_logger().info(f'Cave graph: {graph_json}')
        with open(graph_json) as fh:
            data = json.load(fh)

        self._nodes: dict[int, dict] = {n['id']: n for n in data['nodes']}
        adj: dict[int, list[tuple[int, float]]] = {
            n['id']: [] for n in data['nodes']}
        for e in data['edges']:
            adj[e['source']].append((e['target'], float(e['length'])))
            adj[e['target']].append((e['source'], float(e['length'])))

        # Entrance = node with lowest z (planar caves: all z=0 → node 0)
        entrance = min(self._nodes, key=lambda n: self._nodes[n]['position'][2])
        # ENU position of entrance = Gazebo spawn reference for NED conversion
        self._entrance_enu_x = float(self._nodes[entrance]['position'][0])
        self._entrance_enu_y = float(self._nodes[entrance]['position'][1])

        self._path = _bfs_tree_dfs_walk(adj, entrance)
        n_scans = sum(1 for _, fv in self._path if fv)
        n_hops  = len(self._path) - n_scans
        self.get_logger().info(
            f'{len(self._nodes)} chambers  '
            f'path={len(self._path)} waypoints  '
            f'({n_scans} scans + {n_hops} backtrack hops)')

        # ── home NED: locked from first valid VehicleLocalPosition ───────────
        # Dynamic: no hardcoded spawn coordinates.
        self._home_n = 0.0
        self._home_e = 0.0
        self._home_locked = False

        # ── current NED position / heading ────────────────────────────────────
        self._ned_n = self._ned_e = self._ned_d = 0.0
        self._heading = 0.0

        # ── position setpoint ─────────────────────────────────────────────────
        self._sp: dict[str, float] = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)

        # ── spline state ──────────────────────────────────────────────────────
        self._sp_start:  dict[str, float] = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self._sp_target: dict[str, float] = {}
        self._sp_T  = 1.0
        self._sp_t0 = time.time()

        # ── scan state ────────────────────────────────────────────────────────
        self._scan_accum = 0.0
        self._scan_t0    = time.time()

        # ── phase / mission state ─────────────────────────────────────────────
        self._phase    = 'init'
        self._phase_t0 = time.time()
        self._wp_idx   = 0
        self._pre_arm  = 0
        self._started  = False

        # ── publishers ────────────────────────────────────────────────────────
        self._ocm_pub  = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub  = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _REL_QOS)
        self._synth_pub = self.create_publisher(
            PointCloud2, '/drone/lidar/points', _PX4_QOS)

        self._save_cli = self.create_client(Trigger, '/slam/save_map')

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, _PX4_QOS)
        self.create_subscription(
            Empty, '/mission/start',
            lambda _: self._on_start(), _REL_QOS)
        self.create_subscription(
            Empty, '/mission/land',
            lambda _: self._on_land(), _REL_QOS)

        self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info('Ready — publish /mission/start to begin.')

    # ── subscriptions callbacks ───────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid):
            return
        self._ned_n   = float(msg.x)
        self._ned_e   = float(msg.y)
        self._ned_d   = float(msg.z)
        self._heading = float(msg.heading) if math.isfinite(msg.heading) else 0.0

        if not self._home_locked:
            # Lock home from first valid reading — dynamic spawn support
            self._home_n = self._ned_n
            self._home_e = self._ned_e
            self._sp.update(x=self._home_n, y=self._home_e, z=self._ned_d)
            self._home_locked = True
            self.get_logger().info(
                f'Home NED locked: N={self._home_n:.3f} E={self._home_e:.3f}')

    def _status_cb(self, msg: VehicleStatus) -> None:
        armed    = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.get_logger().debug(f'armed={armed} offboard={offboard}')

    def _on_start(self) -> None:
        if not self._started:
            self._started = True
            self.get_logger().info('/mission/start received.')

    def _on_land(self) -> None:
        if self._phase not in ('return_home', 'landing', 'done'):
            self.get_logger().info('/mission/land → returning early.')
            self._enter_return()

    # ── coordinate conversion ─────────────────────────────────────────────────

    def _chamber_ned(self, nid: int) -> tuple[float, float]:
        """
        Convert a cave graph node (ENU) to absolute NED (N, E).

        Cave graph positions are Gazebo ENU (x=East, y=North).
        Spawn is at entrance node ENU. PX4 NED origin = spawn point.
          ned_north = home_n + (enu_north - entrance_enu_north)
                    = home_n + (enu_y    - entrance_enu_y)
          ned_east  = home_e + (enu_east  - entrance_enu_east)
                    = home_e + (enu_x    - entrance_enu_x)
        """
        enu_x = float(self._nodes[nid]['position'][0])
        enu_y = float(self._nodes[nid]['position'][1])
        ned_n = self._home_n + (enu_y - self._entrance_enu_y)
        ned_e = self._home_e + (enu_x - self._entrance_enu_x)
        return ned_n, ned_e

    # ── spline helpers ────────────────────────────────────────────────────────

    def _start_spline(self, target: dict[str, float], dur: float) -> None:
        self._sp_start  = dict(self._sp)  # snapshot current setpoint (incl. yaw)
        self._sp_target = target
        self._sp_T      = max(0.5, dur)
        self._sp_t0     = time.time()

    def _step_spline(self) -> bool:
        t = time.time() - self._sp_t0
        s = _smooth(t, self._sp_T)
        for ax in ('x', 'y', 'z'):
            self._sp[ax] = (self._sp_start[ax]
                            + s * (self._sp_target[ax] - self._sp_start[ax]))
        if 'yaw' in self._sp_target:
            dy = _wrap_pi(self._sp_target['yaw'] - self._sp_start['yaw'])
            self._sp['yaw'] = self._sp_start['yaw'] + s * dy
        return t >= self._sp_T

    # ── phase transitions ─────────────────────────────────────────────────────

    def _enter_transit(self) -> None:
        nid, is_first = self._path[self._wp_idx]
        cn, ce = self._chamber_ned(nid)
        dist = math.hypot(cn - self._ned_n, ce - self._ned_e)
        dur  = max(3.0, dist / max(self.TRANSIT_SPEED, 0.1))
        # Yaw toward target so LiDAR sweeps tunnel walls during transit
        goal_yaw = math.atan2(ce - self._ned_e, cn - self._ned_n)
        self._start_spline(
            dict(x=cn, y=ce, z=self.CRUISE_D, yaw=goal_yaw), dur)
        self._phase = 'transit'
        tag = 'SCAN' if is_first else 'HOP'
        self.get_logger().info(
            f'[{tag}] → node {nid}  dist={dist:.1f} m  dur={dur:.1f} s')

    def _enter_scan(self) -> None:
        nid, _ = self._path[self._wp_idx]
        cn, ce = self._chamber_ned(nid)
        # Hold position at chamber centre
        self._sp.update(x=cn, y=ce, z=self.CRUISE_D)
        self._scan_accum = 0.0
        self._scan_t0    = time.time()
        self._phase      = 'scan'
        # Inject known chamber geometry immediately to seed SLAM
        self._publish_synth_chamber(nid)
        r = self._nodes[nid]['radius']
        self.get_logger().info(
            f'Scanning node {nid} (r={r:.1f} m, {self.SCAN_SECS:.0f} s)')

    def _enter_return(self) -> None:
        dist = math.hypot(self._home_n - self._ned_n,
                          self._home_e - self._ned_e)
        dur  = max(8.0, dist / max(self.TRANSIT_SPEED, 0.1))
        goal_yaw = math.atan2(self._home_e - self._ned_e,
                               self._home_n - self._ned_n)
        self._start_spline(
            dict(x=self._home_n, y=self._home_e,
                 z=self.CRUISE_D, yaw=goal_yaw),
            dur)
        self._phase = 'return_home'
        self.get_logger().info(f'Returning home ({dist:.1f} m, {dur:.0f} s).')

    def _advance(self) -> None:
        self._wp_idx += 1
        if self._wp_idx >= len(self._path):
            self.get_logger().info('All chambers mapped — returning home.')
            self._enter_return()
        else:
            self._enter_transit()

    # ── synthetic point cloud injection ───────────────────────────────────────

    def _publish_synth_chamber(self, nid: int) -> None:
        """
        Publish a sphere of synthetic LiDAR points for the chamber at *nid*.
        Uses known cave geometry (chamber radius) to enrich the SLAM map
        even before the actual LiDAR scan completes the rotation.
        Points are expressed in the drone body frame (base_link).
        """
        cn, ce = self._chamber_ned(nid)
        r   = max(1.0, float(self._nodes[nid]['radius']))
        hdg = self._heading

        # Offset from drone to chamber centre in NED
        dn = cn - self._ned_n
        de = ce - self._ned_e
        dd = self.CRUISE_D - self._ned_d

        # Rotate to body frame (NED heading → body x=fwd, y=right, z=up)
        cx =  dn * math.cos(hdg) + de * math.sin(hdg)
        cy = -dn * math.sin(hdg) + de * math.cos(hdg)
        cz = -dd  # NED down → sensor up (z positive up in body frame)

        pts: list[list[float]] = []
        for j in range(self.SYNTH_EL):
            el = math.pi * (j / max(self.SYNTH_EL - 1, 1) - 0.5)
            cos_el = math.cos(el)
            sin_el = math.sin(el)
            for i in range(self.SYNTH_AZ):
                az = 2.0 * math.pi * i / self.SYNTH_AZ
                pts.append([
                    cx + r * cos_el * math.cos(az),
                    cy + r * cos_el * math.sin(az),
                    cz + r * sin_el,
                ])

        arr = np.array(pts, dtype=np.float32)
        self._synth_pub.publish(self._make_pc2(arr))
        self.get_logger().info(
            f'  Injected {len(arr)} synthetic pts for chamber {nid}.')

    def _make_pc2(self, pts: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.height     = 1
        msg.width      = len(pts)
        msg.is_dense   = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step   = 12 * len(pts)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = bytearray(pts.tobytes())
        return msg

    # ── PX4 interface ─────────────────────────────────────────────────────────

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _heartbeat(self) -> None:
        ts  = self._ts()
        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position  = True
        self._ocm_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position  = [self._sp['x'], self._sp['y'], self._sp['z']]
        sp.yaw       = self._sp['yaw']
        sp.velocity  = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def _cmd(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        m = VehicleCommand()
        m.timestamp          = self._ts()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system      = m.source_system = 1
        m.target_component   = m.source_component = 1
        m.from_external      = True
        self._cmd_pub.publish(m)

    def _save_map(self) -> None:
        if not self._save_cli.service_is_ready():
            self.get_logger().warn('map_saver service not ready — skipping save.')
            return
        fut = self._save_cli.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f'Map saved: {f.result().message}' if f.result().success
                else f'Map save FAILED: {f.result().message}'))

    # ── 10 Hz control tick ────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._heartbeat()

        if not self._home_locked:
            return  # wait for NED origin

        p = self._phase

        # ── INIT: count heartbeat cycles, then arm ────────────────────────────
        if p == 'init':
            if not self._started:
                return
            self._pre_arm += 1
            if self._pre_arm >= self.PRE_ARM_CYCLES:
                # Set OFFBOARD mode, then arm
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.05)
                self._cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                # Takeoff to cruise altitude above locked home position
                self._start_spline(
                    dict(x=self._home_n, y=self._home_e,
                         z=self.CRUISE_D, yaw=0.0),
                    self.TAKEOFF_SECS)
                self._phase = 'takeoff'
                self.get_logger().info(
                    f'ARM → takeoff to CRUISE_D={self.CRUISE_D:.1f} m '
                    f'from NED home N={self._home_n:.2f} E={self._home_e:.2f}')

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        elif p == 'takeoff':
            if self._step_spline():
                self.get_logger().info('Takeoff complete. Starting BFS traversal.')
                self._wp_idx = 0
                self._enter_scan()  # scan entrance chamber (already there)

        # ── TRANSIT: smooth spline toward next chamber ────────────────────────
        elif p == 'transit':
            if self._step_spline():
                nid, is_first = self._path[self._wp_idx]
                if is_first:
                    self._enter_scan()
                else:
                    self._phase    = 'hop'
                    self._phase_t0 = time.time()

        # ── HOP: brief pause at backtrack node (no rescan) ───────────────────
        elif p == 'hop':
            if time.time() - self._phase_t0 >= self.HOP_SECS:
                self._advance()

        # ── SCAN: 360° rotation at chamber centre ────────────────────────────
        elif p == 'scan':
            elapsed = time.time() - self._scan_t0
            if elapsed < self.HOVER_SECS:
                return  # settle before rotating

            rate = 2.0 * math.pi / (self.SCAN_SECS * self.TICK_HZ)  # rad/tick
            self._scan_accum  += rate
            self._sp['yaw']    = _wrap_pi(self._sp['yaw'] + rate)

            if self._scan_accum >= 2.0 * math.pi:
                nid, _ = self._path[self._wp_idx]
                self.get_logger().info(f'Scan complete: node {nid}.')
                self._advance()

        # ── RETURN HOME ───────────────────────────────────────────────────────
        elif p == 'return_home':
            if self._step_spline():
                self.get_logger().info('Home reached — landing.')
                self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._save_map()
                self._phase_t0 = time.time()
                self._phase    = 'landing'

        # ── LANDING: wait then force disarm ───────────────────────────────────
        elif p == 'landing':
            if time.time() - self._phase_t0 >= self.DISARM_WAIT:
                self._cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self._phase_t0 = time.time()
                self._phase    = 'done'

        # ── DONE ──────────────────────────────────────────────────────────────
        elif p == 'done':
            if time.time() - self._phase_t0 >= 2.0:
                self.get_logger().info('Mission complete.')
                sys.exit(0)


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description='BFS-ordered systematic cave SLAM mapper.')
    p.add_argument('--cave', default=None,
                   help='Path to cave output dir (default: auto-detect latest).')
    p.add_argument('--cruise-d', type=float, default=None,
                   help='Override CRUISE_D in metres NED-down (e.g. -2.5).')
    p.add_argument('--speed', type=float, default=None,
                   help='Override transit speed in m/s (default 0.4).')
    p.add_argument('--scan-secs', type=float, default=None,
                   help='Override 360° scan duration in seconds (default 20).')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = SlamMapper(cave_hint=args.cave)

    if args.cruise_d is not None:
        node.CRUISE_D = args.cruise_d
    if args.speed is not None:
        node.TRANSIT_SPEED = args.speed
    if args.scan_secs is not None:
        node.SCAN_SECS = args.scan_secs

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
