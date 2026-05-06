#!/usr/bin/env python3
"""
square_flight.py
================
PX4 SITL offboard position demo — smooth spline, square path.

Fixes vs. original
-------------------
1. Home position is read from VehicleLocalPosition (not hardcoded to 0,0,0).
   All waypoints are *offsets* from that home point.
2. Force-arm (param2=21196) so SITL pre-flight checks don't block arming.
3. TAKEOFF_ALTITUDE raised to -2.0 m NED (= 2 m above ground).
4. VehicleLocalPosition subscription uses BEST_EFFORT QoS (PX4 requirement).
5. Arming is deferred until a valid home position lock is obtained.

Waypoint format (NED offsets from home)
----------------------------------------
  x : North offset in metres  (+ = forward)
  y : East  offset in metres  (+ = right)
  z : Down  offset in metres  (- = up, so -2 means 2 m above home)
  duration : seconds to spend reaching this waypoint

Usage
-----
    python3 scripts/square_flight.py
"""

from __future__ import annotations

import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
_RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── spline ────────────────────────────────────────────────────────────────────

def _spline(t: float, T: float) -> float:
    """Cubic ease-in/out: 0 → 1 over duration T."""
    s = max(0.0, min(1.0, t / T))
    return 3 * s * s - 2 * s * s * s


# ── node ──────────────────────────────────────────────────────────────────────

class SquareFlight(Node):
    """
    Arm → takeoff → square path → land → disarm.

    All NED setpoints are expressed as offsets from the home position
    captured just before arming, so the script works regardless of where
    the drone spawns in the world.
    """

    # ── tunables ──────────────────────────────────────────────────────────────
    TAKEOFF_ALT_NED = -2.0    # NED metres (negative = up); 2 m above home
    TAKEOFF_SECS    = 6.0     # seconds to climb to takeoff altitude
    DISARM_WAIT_S   = 5.0     # seconds to wait after NAV_LAND before disarming
    PRE_ARM_CYCLES  = 10      # heartbeat ticks before arming (at 10 Hz = 1 s)

    # Square path — NED offsets from home, all at cruise altitude
    WAYPOINTS = [
        {'x': 0.0, 'y': 0.0, 'z': -2.0, 'duration': 5.0},  # hover at altitude
        {'x': 3.0, 'y': 0.0, 'z': -2.0, 'duration': 6.0},  # forward
        {'x': 3.0, 'y': 3.0, 'z': -2.0, 'duration': 6.0},  # right
        {'x': 0.0, 'y': 3.0, 'z': -2.0, 'duration': 6.0},  # back
        {'x': 0.0, 'y': 0.0, 'z': -2.0, 'duration': 6.0},  # home XY
    ]

    def __init__(self) -> None:
        super().__init__('square_flight')

        # Home position in NED (locked from VehicleLocalPosition)
        self._home_n: float = 0.0
        self._home_e: float = 0.0
        self._home_d: float = 0.0
        self._home_locked: bool = False

        # Current NED setpoint (absolute, published every tick)
        self._sp: dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        # Spline interpolation state
        self._sp_start: dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._sp_target: dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._sp_T: float = 1.0
        self._sp_t0: float = time.time()

        # Mission state
        self._phase: str = 'init'
        self._phase_t0: float = time.time()
        self._pre_arm_count: int = 0
        self._wp_idx: int = 0

        # ── publishers ────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _RELIABLE_QOS)

        # ── subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, _PX4_QOS)

        # 10 Hz tick
        self.create_timer(0.1, self._tick)
        self.get_logger().info('SquareFlight ready — starting in 1 s after position lock.')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not self._home_locked and msg.xy_valid and msg.z_valid:
            self._home_n = float(msg.x)
            self._home_e = float(msg.y)
            self._home_d = float(msg.z)
            self._home_locked = True
            self.get_logger().info(
                f'Home locked: N={self._home_n:.2f}  '
                f'E={self._home_e:.2f}  D={self._home_d:.2f}'
            )
            # Initialise setpoint at current position so pre-arm setpoints
            # don't command the drone to move before takeoff.
            self._sp.update(x=self._home_n, y=self._home_e, z=self._home_d)

    def _status_cb(self, msg: VehicleStatus) -> None:
        pass  # status read via landed_state if needed

    # ── helpers ───────────────────────────────────────────────────────────────

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _pub_offboard(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp    = self._ts()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._ocm_pub.publish(msg)

    def _pub_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._ts()
        msg.position  = [self._sp['x'], self._sp['y'], self._sp['z']]
        msg.yaw       = self._sp['yaw']
        self._traj_pub.publish(msg)

    def _send_cmd(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = p1
        msg.param2           = p2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._cmd_pub.publish(msg)

    def _set_offboard_mode(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('SET_MODE OFFBOARD sent.')

    def _arm(self) -> None:
        # param2=21196 = force-arm (bypasses SITL pre-flight checks)
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        self.get_logger().info('ARM sent.')

    def _disarm(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        self.get_logger().info('DISARM sent.')

    def _land(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('NAV_LAND sent.')

    def _abs(self, wp: dict) -> dict:
        """Convert a home-relative waypoint to an absolute NED position."""
        return {
            'x': self._home_n + wp['x'],
            'y': self._home_e + wp['y'],
            'z': self._home_d + wp['z'],
        }

    def _start_spline(self, target_abs: dict, duration: float) -> None:
        self._sp_start  = {'x': self._sp['x'], 'y': self._sp['y'], 'z': self._sp['z']}
        self._sp_target = target_abs
        self._sp_T      = duration
        self._sp_t0     = time.time()

    def _step_spline(self) -> bool:
        """Advance spline; update self._sp. Returns True when done."""
        t = time.time() - self._sp_t0
        s = _spline(t, self._sp_T)
        for ax in ('x', 'y', 'z'):
            self._sp[ax] = self._sp_start[ax] + s * (self._sp_target[ax] - self._sp_start[ax])
        return t >= self._sp_T

    # ── main tick ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._pub_offboard()
        self._pub_setpoint()

        # ── wait for home lock ────────────────────────────────────────────
        if not self._home_locked:
            return

        # ── init: count heartbeat cycles, then arm ────────────────────────
        if self._phase == 'init':
            self._pre_arm_count += 1
            if self._pre_arm_count >= self.PRE_ARM_CYCLES:
                self.get_logger().info(
                    f'Arming — takeoff to {abs(self.TAKEOFF_ALT_NED):.1f} m '
                    f'above home (NED D={self._home_d + self.TAKEOFF_ALT_NED:.2f})'
                )
                self._set_offboard_mode()
                self._arm()
                self._start_spline(
                    self._abs({'x': 0.0, 'y': 0.0, 'z': self.TAKEOFF_ALT_NED}),
                    self.TAKEOFF_SECS,
                )
                self._phase = 'takeoff'

        # ── smooth takeoff ────────────────────────────────────────────────
        elif self._phase == 'takeoff':
            if self._step_spline():
                self.get_logger().info('Takeoff complete — starting square path.')
                self._wp_idx = 0
                self._begin_waypoint(self._wp_idx)
                self._phase = 'waypoints'

        # ── waypoint sequence ─────────────────────────────────────────────
        elif self._phase == 'waypoints':
            if self._step_spline():
                self._wp_idx += 1
                if self._wp_idx < len(self.WAYPOINTS):
                    self._begin_waypoint(self._wp_idx)
                else:
                    self.get_logger().info('All waypoints done — landing.')
                    self._land()
                    self._phase_t0 = time.time()
                    self._phase = 'landing'

        # ── wait after NAV_LAND, then disarm ─────────────────────────────
        elif self._phase == 'landing':
            if time.time() - self._phase_t0 >= self.DISARM_WAIT_S:
                self._disarm()
                self._phase = 'done'
                self._phase_t0 = time.time()

        # ── done ──────────────────────────────────────────────────────────
        elif self._phase == 'done':
            if time.time() - self._phase_t0 >= 2.0:
                self.get_logger().info('Mission complete.')
                sys.exit(0)

    def _begin_waypoint(self, idx: int) -> None:
        wp = self.WAYPOINTS[idx]
        target = self._abs(wp)
        self._start_spline(target, wp['duration'])
        self.get_logger().info(
            f'WP {idx + 1}/{len(self.WAYPOINTS)}: '
            f'N={target["x"]:.2f}  E={target["y"]:.2f}  D={target["z"]:.2f}  '
            f'({wp["duration"]:.0f} s)'
        )


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init(args=sys.argv)
    node = SquareFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
