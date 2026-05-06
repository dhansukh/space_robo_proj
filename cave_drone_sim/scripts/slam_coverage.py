#!/usr/bin/env python3
"""
slam_coverage.py — Centerline-guided SLAM coverage mapper
==========================================================
Flies the drone along actual tunnel centerline splines extracted from
the cave generator, ensuring full edge coverage and clean SLAM maps.

Algorithm:
  1. Load cave_graph.json + centerlines.json
  2. Plan Chinese-Postman walk (visit every edge at least once)
  3. Sample centerline splines at ~1 m intervals as flight waypoints
  4. At each first-visit chamber: 360° slow scan + synthetic cloud
  5. Loop-closure revisit pass on cycle edges
  6. Return home → land → save SLAM map

Usage:
  python3 scripts/slam_coverage.py [--cave ./output/cave_flat]
  ros2 topic pub --once /mission/start std_msgs/msg/Empty {}
"""
from __future__ import annotations
import argparse, glob, json, math, os, signal, sys, time
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
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus,
)

_PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_REL_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=10)

def _wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def _smooth(t, T):
    s = max(0.0, min(1.0, t/T))
    return 3*s*s - 2*s*s*s

def _find_cave(hint):
    if hint:
        p = Path(hint)/'cave_graph.json'
        if p.exists(): return str(p.parent)
        raise FileNotFoundError(f'cave_graph.json not in {hint}')
    cands = sorted(glob.glob(str(Path(__file__).parent.parent/'output'/'*'/'cave_graph.json')),
                   key=os.path.getmtime, reverse=True)
    if not cands: raise FileNotFoundError('No cave_graph.json under output/')
    return str(Path(cands[0]).parent)

# ── Path planning ─────────────────────────────────────────────────────────────

def _chinese_postman_walk(nodes, adj, edges, start):
    """Visit every edge at least once. Returns [(node, is_first_visit), ...]."""
    # Find odd-degree nodes for Euler circuit feasibility
    tree_edges = set()
    visited_nodes = {start}
    children = {start: []}
    q = deque([start])
    while q:
        n = q.popleft()
        for nb, _ in sorted(adj[n], key=lambda x: x[1]):
            if nb not in visited_nodes:
                visited_nodes.add(nb)
                children[n] = children.get(n, []) + [nb]
                children[nb] = []
                tree_edges.add((min(n, nb), max(n, nb)))
                q.append(nb)

    # DFS walk on spanning tree, but also traverse non-tree edges
    result = []
    edge_visited = set()
    node_visited = set()

    def dfs(n):
        node_visited.add(n)
        result.append((n, True))
        # First visit tree children
        for child in children.get(n, []):
            ek = (min(n, child), max(n, child))
            edge_visited.add(ek)
            dfs(child)
            result.append((n, False))
        # Then visit non-tree edges from this node
        for nb, _ in sorted(adj[n], key=lambda x: x[1]):
            ek = (min(n, nb), max(n, nb))
            if ek not in edge_visited:
                edge_visited.add(ek)
                result.append((nb, nb not in node_visited))
                if nb not in node_visited:
                    node_visited.add(nb)
                result.append((n, False))

    dfs(start)
    while len(result) > 1 and result[-1] == (start, False):
        result.pop()
    return result

def _build_flight_plan(cave_dir):
    """Load cave data and build ordered flight waypoints (ENU)."""
    import logging
    _log = logging.getLogger('slam_coverage.plan')

    with open(os.path.join(cave_dir, 'cave_graph.json')) as f:
        gdata = json.load(f)

    cl_path = os.path.join(cave_dir, 'centerlines.json')
    centerlines = {}
    if os.path.exists(cl_path):
        with open(cl_path) as f:
            centerlines = json.load(f).get('edges', {})
        _log.info(f'Centerlines loaded: {len(centerlines)} edges')
    else:
        _log.warning(
            f'centerlines.json NOT FOUND in {cave_dir}. '
            'All segments will use straight-line fallback — '
            'regenerate cave with generate_cave.py to get centerlines.')

    nodes = {n['id']: n for n in gdata['nodes']}
    # Build adjacency set for quick edge lookup
    adj = {n['id']: [] for n in gdata['nodes']}
    edge_set = set()   # set of (min,max) node-id pairs that are real edges
    for e in gdata['edges']:
        adj[e['source']].append((e['target'], e['length']))
        adj[e['target']].append((e['source'], e['length']))
        edge_set.add((min(e['source'], e['target']),
                      max(e['source'], e['target'])))

    entrance = min(nodes, key=lambda n: nodes[n]['position'][2])
    enu_x = nodes[entrance]['position'][0]
    enu_y = nodes[entrance]['position'][1]

    walk = _chinese_postman_walk(nodes, adj, list(edge_set), entrance)

    def get_cl(u, v):
        """Return waypoints for edge u→v using centerline or straight-line fallback.
        Only called when u-v is a real graph edge."""
        for key in [f'{u}-{v}', f'{v}-{u}']:
            if key in centerlines:
                pts = centerlines[key]
                # Reverse if stored in opposite direction
                if key == f'{v}-{u}':
                    pts = list(reversed(pts))
                arr = np.array(pts)
                if len(arr) < 2:
                    break
                dists = np.cumsum(
                    np.r_[0, np.linalg.norm(np.diff(arr, axis=0), axis=1)])
                total = dists[-1]
                n_pts = max(2, int(total / 1.0))
                targets = np.linspace(0, total, n_pts)
                resampled = np.array(
                    [np.interp(targets, dists, arr[:, i]) for i in range(3)]).T
                return resampled.tolist()
        # Straight-line fallback (only for real edges, so path stays inside cave)
        p1, p2 = nodes[u]['position'], nodes[v]['position']
        dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        n_pts = max(2, int(dist / 1.0))
        return [[(1-t)*p1[0]+t*p2[0], (1-t)*p1[1]+t*p2[1], 0.0]
                for t in np.linspace(0, 1, n_pts).tolist()]

    # Convert walk into flight segments — every consecutive pair MUST be a real edge
    segments = []
    for i, (nid, is_first) in enumerate(walk):
        if i == 0:
            segments.append({'waypoints': [nodes[nid]['position']],
                             'node_id': nid, 'is_scan': True,
                             'edge': None})
            continue
        prev_nid = walk[i-1][0]
        ek = (min(prev_nid, nid), max(prev_nid, nid))
        if ek not in edge_set:
            # Walk generated a non-edge transition — this should not happen
            # but guard against it: fly directly to node centre (single wp)
            _log.warning(
                f'Walk step {prev_nid}→{nid} is not a graph edge — using 1-wp fallback')
            segments.append({'waypoints': [nodes[nid]['position']],
                             'node_id': nid, 'is_scan': is_first,
                             'edge': None})
        else:
            segments.append({'waypoints': get_cl(prev_nid, nid),
                             'node_id': nid, 'is_scan': is_first,
                             'edge': ek})

    return segments, nodes, entrance, enu_x, enu_y, walk

# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class SlamCoverage(Node):
    CRUISE_D = -8.0
    TAKEOFF_SECS = 6.0
    SPEED = 0.3
    SCAN_SECS = 20.0
    HOVER_SECS = 2.0
    HOP_SECS = 1.5
    DISARM_WAIT = 5.0
    SYNTH_AZ, SYNTH_EL = 36, 9
    TICK_HZ = 10
    PRE_ARM = 15
    # Stuck-detection timeouts
    WP_TIMEOUT           = 15.0  # seconds per waypoint before skipping it
    SEGMENT_TIMEOUT      = 90.0  # seconds per whole segment before giving up
    CONSEC_TIMEOUT_LIMIT = 3     # consecutive WP timeouts → stuck outside cave → return home

    def __init__(self, cave_dir, dry_run=False, max_segs=None, auto_save_mins=0.0):
        super().__init__('slam_coverage')
        self._segments, self._nodes, self._entrance, \
            self._enu_x, self._enu_y, self._walk = _build_flight_plan(cave_dir)

        # Truncate to first N segments for quick test runs
        if max_segs is not None and max_segs > 0:
            self._segments = self._segments[:max_segs]
            self.get_logger().info(f'[--max-segs] Capped to first {max_segs} segments.')

        n_scan = sum(1 for s in self._segments if s['is_scan'])
        n_edges = sum(1 for s in self._segments if s.get('edge') is not None)
        total_wps = sum(len(s['waypoints']) for s in self._segments)
        self.get_logger().info(
            f'{len(self._nodes)} chambers, {len(self._segments)} segments '
            f'({n_scan} scans, {n_edges} edge-transits, {total_wps} waypoints)')

        if dry_run:
            self.get_logger().info('DRY RUN — printing plan and exiting.')
            for i, seg in enumerate(self._segments):
                tag = 'SCAN' if seg['is_scan'] else 'HOP'
                edge = seg.get('edge') or 'fallback'
                self.get_logger().info(
                    f'  [{tag}] seg {i} node {seg["node_id"]} '
                    f'edge={edge} {len(seg["waypoints"])} wps')
            sys.exit(0)

        # State
        self._home_n = self._home_e = 0.0
        self._home_locked = False
        self._ned_n = self._ned_e = self._ned_d = self._heading = 0.0
        self._sp = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self._sp_start = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self._sp_target = {}
        self._sp_T = 1.0; self._sp_t0 = time.time()
        self._scan_accum = 0.0; self._scan_t0 = time.time()
        self._phase = 'init'; self._phase_t0 = time.time()
        self._seg_idx = 0; self._wp_idx = 0; self._pre_arm_cnt = 0
        self._started = False
        # Per-waypoint and per-segment stuck detection
        self._wp_issued_t     = time.time()
        self._seg_start_t     = time.time()
        self._consec_wp_timeouts = 0  # consecutive WP timeouts (reset on any success)
        # PX4 mode tracking for OFFBOARD recovery
        self._armed = False
        self._offboard = False
        self._offboard_req_t = 0.0

        # Publishers
        self._ocm_pub = self.create_publisher(OffboardControlMode,
            '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._traj_pub = self.create_publisher(TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub = self.create_publisher(VehicleCommand,
            '/fmu/in/vehicle_command', _REL_QOS)
        self._synth_pub = self.create_publisher(PointCloud2,
            '/debug/synth_cloud', _PX4_QOS)  # NOT /drone/lidar/points — that is SLAM's ICP input
        self._save_cli = self.create_client(Trigger, '/slam/save_map')

        # Subscriptions
        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position', self._pos_cb, _PX4_QOS)
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status', self._status_cb, _PX4_QOS)
        self.create_subscription(Empty, '/mission/start',
            lambda _: self._on_start(), _REL_QOS)
        self.create_subscription(Empty, '/mission/land',
            lambda _: self._on_land(), _REL_QOS)
        self.create_timer(1.0/self.TICK_HZ, self._tick)

        # Auto-save timer: periodically save map during long missions
        if auto_save_mins > 0:
            interval = auto_save_mins * 60.0
            self.create_timer(interval, self._save_map)
            self.get_logger().info(
                f'Auto-save every {auto_save_mins:.1f} min to /tmp/slam_maps/')

        self.get_logger().info('Ready — /mission/start to begin.')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _pos_cb(self, msg):
        if not (msg.xy_valid and msg.z_valid): return
        self._ned_n, self._ned_e, self._ned_d = float(msg.x), float(msg.y), float(msg.z)
        self._heading = float(msg.heading) if math.isfinite(msg.heading) else 0.0
        if not self._home_locked:
            self._home_n, self._home_e = self._ned_n, self._ned_e
            self._sp.update(x=self._home_n, y=self._home_e, z=self._ned_d)
            self._home_locked = True
            self.get_logger().info(f'Home NED locked: N={self._home_n:.3f} E={self._home_e:.3f}')

    def _status_cb(self, msg):
        armed    = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if armed != self._armed or offboard != self._offboard:
            self._armed, self._offboard = armed, offboard
            self.get_logger().info(f'PX4 armed={armed} offboard={offboard}')
        # Auto-recover OFFBOARD mode if we lose it mid-mission
        if (self._phase not in ('init', 'landing', 'done')
                and self._armed and not self._offboard):
            now = time.time()
            if now - self._offboard_req_t > 2.0:   # re-request at most every 2 s
                self.get_logger().warn(
                    'Lost OFFBOARD mode — re-requesting.', throttle_duration_sec=2.0)
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._offboard_req_t = now
    def _on_start(self):
        if not self._started:
            self._started = True
            self.get_logger().info('/mission/start received.')
    def _on_land(self):
        if self._phase not in ('return_home','landing','done'):
            self._enter_return()

    # ── Coordinate conversion ─────────────────────────────────────────────────
    def _enu_to_ned(self, enu_x, enu_y):
        """Cave ENU → absolute NED. PX4 NED origin = spawn = entrance node ENU."""
        return (self._home_n + (enu_y - self._enu_y),
                self._home_e + (enu_x - self._enu_x))

    # ── Spline helpers ────────────────────────────────────────────────────────
    def _start_spline(self, target, dur):
        self._sp_start = dict(self._sp)
        self._sp_target = target
        self._sp_T = max(0.5, dur)
        self._sp_t0 = time.time()

    def _step_spline(self):
        t = time.time() - self._sp_t0
        s = _smooth(t, self._sp_T)
        for ax in ('x','y','z'):
            self._sp[ax] = self._sp_start[ax] + s*(self._sp_target[ax]-self._sp_start[ax])
        if 'yaw' in self._sp_target:
            dy = _wrap(self._sp_target['yaw'] - self._sp_start['yaw'])
            self._sp['yaw'] = self._sp_start['yaw'] + s*dy
        return t >= self._sp_T

    # ── Phase transitions ─────────────────────────────────────────────────────
    def _enter_transit_segment(self):
        seg = self._segments[self._seg_idx]
        self._wp_idx = 0
        self._seg_start_t = time.time()
        nid = seg['node_id']
        tag = 'SCAN' if seg['is_scan'] else 'HOP'
        self.get_logger().info(
            f'[{tag}] seg {self._seg_idx}/{len(self._segments)-1} → node {nid} '
            f'({len(seg["waypoints"])} wps)')
        self._issue_next_wp()

    def _issue_next_wp(self):
        seg = self._segments[self._seg_idx]
        wps = seg['waypoints']

        if self._wp_idx >= len(wps):
            self._on_segment_done()
            return

        wp = wps[self._wp_idx]
        cn, ce = self._enu_to_ned(wp[0], wp[1])
        dist = math.hypot(cn - self._ned_n, ce - self._ned_e)

        # If already within arrival radius of this waypoint, skip ahead
        ARRIVAL = 1.5  # metres — larger than 1m waypoint spacing
        while dist < ARRIVAL and self._wp_idx < len(wps) - 1:
            self._wp_idx += 1
            wp = wps[self._wp_idx]
            cn, ce = self._enu_to_ned(wp[0], wp[1])
            dist = math.hypot(cn - self._ned_n, ce - self._ned_e)

        if self._wp_idx >= len(wps):
            self._on_segment_done()
            return

        dur = max(0.5, dist / max(self.SPEED, 0.05))
        goal_yaw = math.atan2(ce - self._ned_e, cn - self._ned_n)
        self._start_spline(dict(x=cn, y=ce, z=self.CRUISE_D, yaw=goal_yaw), dur)
        self._wp_issued_t = time.time()
        self._phase = 'transit'

    def _on_wp_arrived(self):
        """Call when a waypoint is successfully reached. Resets stuck counter."""
        self._consec_wp_timeouts = 0
        self._wp_idx += 1
        self._issue_next_wp()

    def _on_segment_done(self):
        seg = self._segments[self._seg_idx]
        if seg['is_scan']:
            self._enter_scan()
        else:
            self._phase = 'hop'
            self._phase_t0 = time.time()

    def _enter_scan(self):
        nid = self._segments[self._seg_idx]['node_id']
        cn, ce = self._enu_to_ned(
            self._nodes[nid]['position'][0], self._nodes[nid]['position'][1])
        self._sp.update(x=cn, y=ce, z=self.CRUISE_D)
        self._scan_accum = 0.0
        self._scan_t0 = time.time()
        self._phase = 'scan'
        self._publish_synth(nid)
        r = self._nodes[nid]['radius']
        self.get_logger().info(f'Scanning node {nid} (r={r:.1f}m, {self.SCAN_SECS:.0f}s)')

    def _enter_return(self):
        dist = math.hypot(self._home_n-self._ned_n, self._home_e-self._ned_e)
        dur = max(8.0, dist/max(self.SPEED, 0.05))
        yaw = math.atan2(self._home_e-self._ned_e, self._home_n-self._ned_n)
        self._start_spline(dict(x=self._home_n, y=self._home_e,
                                z=self.CRUISE_D, yaw=yaw), dur)
        self._phase = 'return_home'
        self.get_logger().info(f'Returning home ({dist:.1f}m, {dur:.0f}s)')

    def _advance(self):
        self._seg_idx += 1
        if self._seg_idx >= len(self._segments):
            self.get_logger().info('All segments complete — returning home.')
            self._enter_return()
        else:
            self._enter_transit_segment()

    # ── Synthetic point cloud ─────────────────────────────────────────────────
    def _publish_synth(self, nid):
        cn, ce = self._enu_to_ned(
            self._nodes[nid]['position'][0], self._nodes[nid]['position'][1])
        r = max(1.0, float(self._nodes[nid]['radius']))
        hdg = self._heading
        dn, de = cn-self._ned_n, ce-self._ned_e
        dd = self.CRUISE_D - self._ned_d
        cx = dn*math.cos(hdg) + de*math.sin(hdg)
        cy = -dn*math.sin(hdg) + de*math.cos(hdg)
        cz = -dd
        pts = []
        for j in range(self.SYNTH_EL):
            el = math.pi*(j/max(self.SYNTH_EL-1,1) - 0.5)
            for i in range(self.SYNTH_AZ):
                az = 2*math.pi*i/self.SYNTH_AZ
                pts.append([cx+r*math.cos(el)*math.cos(az),
                            cy+r*math.cos(el)*math.sin(az),
                            cz+r*math.sin(el)])
        arr = np.array(pts, dtype=np.float32)
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1; msg.width = len(arr)
        msg.is_dense = True; msg.is_bigendian = False
        msg.point_step = 12; msg.row_step = 12*len(arr)
        msg.fields = [
            PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1)]
        msg.data = bytearray(arr.tobytes())
        self._synth_pub.publish(msg)
        self.get_logger().info(f'  Synth: {len(arr)} pts for chamber {nid}')

    # ── PX4 interface ─────────────────────────────────────────────────────────
    def _ts(self): return self.get_clock().now().nanoseconds // 1000
    def _heartbeat(self):
        ts = self._ts()
        ocm = OffboardControlMode(); ocm.timestamp = ts; ocm.position = True
        self._ocm_pub.publish(ocm)
        sp = TrajectorySetpoint(); sp.timestamp = ts
        sp.position = [self._sp['x'], self._sp['y'], self._sp['z']]
        sp.yaw = self._sp['yaw']; sp.velocity = [float('nan')]*3
        self._traj_pub.publish(sp)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.timestamp = self._ts()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        self._cmd_pub.publish(m)

    def _save_map(self):
        if not self._save_cli.service_is_ready():
            self.get_logger().warn('map_saver not ready — skip.'); return
        fut = self._save_cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self.get_logger().info(
            f'Map: {f.result().message}' if f.result().success
            else f'Map FAILED: {f.result().message}'))

    # ── Main tick ─────────────────────────────────────────────────────────────
    def _tick(self):
        self._heartbeat()
        if not self._home_locked: return
        p = self._phase

        if p == 'init':
            if not self._started: return
            self._pre_arm_cnt += 1
            if self._pre_arm_cnt >= self.PRE_ARM:
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.05)
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                self._start_spline(dict(x=self._home_n, y=self._home_e,
                                        z=self.CRUISE_D, yaw=0.0), self.TAKEOFF_SECS)
                self._phase = 'takeoff'
                self.get_logger().info(f'ARM → takeoff D={self.CRUISE_D}')

        elif p == 'takeoff':
            if self._step_spline():
                self.get_logger().info('Takeoff done. Starting coverage traversal.')
                self._seg_idx = 0
                self._enter_scan()  # scan entrance (already there)

        elif p == 'transit':
            now = time.time()

            # ── Per-segment hard timeout ──────────────────────────────────────
            if now - self._seg_start_t > self.SEGMENT_TIMEOUT:
                nid = self._segments[self._seg_idx]['node_id']
                self.get_logger().warn(
                    f'Segment {self._seg_idx} timed out after {self.SEGMENT_TIMEOUT:.0f}s '
                    f'({self._wp_idx}/{len(self._segments[self._seg_idx]["waypoints"])} wps done) '
                    f'— skipping to node {nid}.')
                self._on_segment_done()  # accept wherever we are and scan/hop
                return

            # ── Per-waypoint stuck timeout ─────────────────────────────────────
            if now - self._wp_issued_t > self.WP_TIMEOUT:
                seg = self._segments[self._seg_idx]
                wps = seg['waypoints']
                self._consec_wp_timeouts += 1

                # Too many consecutive timeouts → drone is stuck outside cave
                if self._consec_wp_timeouts >= self.CONSEC_TIMEOUT_LIMIT:
                    self.get_logger().error(
                        f'{self._consec_wp_timeouts} consecutive WP timeouts on '
                        f'seg {self._seg_idx} — drone stuck outside cave. '
                        f'Aborting mission, returning home.')
                    self._consec_wp_timeouts = 0
                    self._enter_return()
                    return

                if self._wp_idx < len(wps) - 1:
                    self.get_logger().warn(
                        f'WP {self._wp_idx} timed out ({self.WP_TIMEOUT:.0f}s) '
                        f'[{self._consec_wp_timeouts}/{self.CONSEC_TIMEOUT_LIMIT}] — skipping.',
                        throttle_duration_sec=self.WP_TIMEOUT)
                    self._wp_idx += 1
                    self._issue_next_wp()
                else:
                    self.get_logger().warn(
                        f'Last WP of seg {self._seg_idx} timed out — accepting.')
                    self._consec_wp_timeouts = 0
                    self._on_segment_done()
                return

            if self._step_spline():
                self._on_wp_arrived()

        elif p == 'scan':
            elapsed = time.time() - self._scan_t0
            if elapsed < self.HOVER_SECS: return
            rate = 2*math.pi / (self.SCAN_SECS * self.TICK_HZ)
            self._scan_accum += rate
            self._sp['yaw'] = _wrap(self._sp['yaw'] + rate)
            if self._scan_accum >= 2*math.pi:
                nid = self._segments[self._seg_idx]['node_id']
                self.get_logger().info(f'Scan done: node {nid}')
                self._advance()

        elif p == 'hop':
            if time.time() - self._phase_t0 >= self.HOP_SECS:
                self._advance()

        elif p == 'return_home':
            if self._step_spline():
                self.get_logger().info('Home — landing.')
                self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._save_map()
                self._phase_t0 = time.time(); self._phase = 'landing'

        elif p == 'landing':
            if time.time() - self._phase_t0 >= self.DISARM_WAIT:
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self._phase_t0 = time.time(); self._phase = 'done'

        elif p == 'done':
            if time.time() - self._phase_t0 >= 2.0:
                self.get_logger().info('Mission complete.'); sys.exit(0)

# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='Centerline-guided SLAM coverage mapper.')
    p.add_argument('--cave',       default=None,  help='Cave output directory')
    p.add_argument('--cruise-d',   type=float,    default=None, help='Cruise altitude NED-D')
    p.add_argument('--speed',      type=float,    default=None, help='Transit speed m/s')
    p.add_argument('--scan-secs',  type=float,    default=None, help='360° scan duration')
    p.add_argument('--max-segs',   type=int,      default=None,
                   help='Stop after this many segments and return home (for quick tests)')
    p.add_argument('--auto-save',  type=float,    default=0.0,  metavar='MINS',
                   help='Auto-save SLAM map every MINS minutes (0 = disabled)')
    p.add_argument('--dry-run',    action='store_true', help='Print plan and exit')
    args, ros_args = p.parse_known_args()

    cave_dir = _find_cave(args.cave)
    rclpy.init(args=ros_args)
    node = SlamCoverage(cave_dir,
                        dry_run=args.dry_run,
                        max_segs=args.max_segs,
                        auto_save_mins=args.auto_save)
    if args.cruise_d  is not None: node.CRUISE_D  = args.cruise_d
    if args.speed     is not None: node.SPEED     = args.speed
    if args.scan_secs is not None: node.SCAN_SECS = args.scan_secs

    # ── Ctrl+C handler: save map then return home gracefully ─────────────────
    def _sigint(sig, frame):
        node.get_logger().info(
            'Ctrl+C received — saving map and returning home...')
        node._save_map()              # trigger /slam/save_map service
        node._enter_return()          # fly home
        # Give the drone 30 s to reach home before hard-exit
        deadline = time.time() + 30.0
        while node._phase not in ('done',) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('Shutdown complete.')
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
