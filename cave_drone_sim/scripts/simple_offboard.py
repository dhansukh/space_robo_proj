#!/usr/bin/env python3
"""
simple_offboard.py
==================
Minimal PX4 SITL offboard takeoff and hover — no SLAM, no mission manager.

Uses POSITION control mode (not velocity), so PX4 holds the exact NED
coordinate and never drifts or disarms due to a landed-state timeout.

Usage
-----
    # Terminal 1: start Gazebo + PX4
    ./scripts/start_px4_cave.sh

    # Terminal 2: start the ROS 2 bridge stack
    ros2 launch drone_bringup px4_cave.launch.py

    # Terminal 3: run this script
    python3 scripts/simple_offboard.py

    # Terminal 4: trigger takeoff
    ros2 topic pub --once /mission/start std_msgs/msg/Empty {}

    # Optional: move the drone with cmd_vel after it is hovering
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

Controls
--------
After arming, every /cmd_vel received increments the held NED position:
    linear.x  → north offset   (ENU forward  = NED north)
    linear.y  → east  offset   (ENU left     = NED east, sign flipped)
    linear.z  → altitude delta (ENU up        = NED down, sign flipped)
    angular.z → yaw rate (rad/s) integrated into held yaw

Ctrl-C to stop; PX4 will switch to Hold mode.
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ── tunables ──────────────────────────────────────────────────────────────────

TARGET_ALT_M   = 2.0    # climb to this altitude after arm (metres, ENU up)
PRE_ARM_CYCLES = 50     # publish this many heartbeats before arming (~2.5 s at 20 Hz)
TICK_HZ        = 20     # heartbeat rate (PX4 needs ≥ 2 Hz)
CMD_VEL_SCALE  = 0.05   # metres-per-tick gained per unit cmd_vel

# ── QoS presets ───────────────────────────────────────────────────────────────

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


class SimpleOffboard(Node):
    """
    Arm → OFFBOARD → climb to TARGET_ALT_M → hover.

    Position setpoints keep PX4 locked at a fixed NED coordinate so the
    drone cannot drift or trigger a landed-state auto-disarm.
    """

    def __init__(self) -> None:
        super().__init__('simple_offboard')

        # Flight state
        self._armed    = False
        self._offboard = False
        self._started  = False          # /mission/start received?
        self._cycle    = 0              # heartbeat counter before arming

        # NED position of the drone (from VehicleLocalPosition)
        self._cur_n = 0.0
        self._cur_e = 0.0
        self._cur_d = 0.0

        # Held NED setpoint (locked at arm time, then updated by cmd_vel)
        self._hold_n   = 0.0
        self._hold_e   = 0.0
        self._hold_d   = 0.0           # NED down; set negative (= ENU up) on arm
        self._hold_yaw = 0.0           # radians, NED heading

        # ── publishers ────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _RELIABLE_QOS)

        # ── subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, _PX4_QOS)
        self.create_subscription(
            Empty, '/mission/start', self._start_cb, _RELIABLE_QOS)
        self.create_subscription(
            Twist, '/cmd_vel', self._cmdvel_cb, _RELIABLE_QOS)

        # ── 20 Hz heartbeat ───────────────────────────────────────────────
        self.create_timer(1.0 / TICK_HZ, self._tick)

        self.get_logger().info(
            f'SimpleOffboard ready — waiting for /mission/start  '
            f'(target altitude: {TARGET_ALT_M} m)'
        )

    # ── subscription callbacks ────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._cur_n = float(msg.x)
        self._cur_e = float(msg.y)
        self._cur_d = float(msg.z)

    def _status_cb(self, msg: VehicleStatus) -> None:
        was_armed    = self._armed
        was_offboard = self._offboard
        self._armed    = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._offboard = msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self._armed != was_armed or self._offboard != was_offboard:
            self.get_logger().info(
                f'Status: armed={self._armed}  offboard={self._offboard}'
            )

    def _start_cb(self, _: Empty) -> None:
        if not self._started:
            self._started = True
            self.get_logger().info(
                '/mission/start received — will arm after '
                f'{PRE_ARM_CYCLES} heartbeats (~{PRE_ARM_CYCLES / TICK_HZ:.1f} s)'
            )

    def _cmdvel_cb(self, msg: Twist) -> None:
        """Shift the held NED setpoint by the scaled cmd_vel."""
        if not (self._armed and self._offboard):
            return
        # ENU → NED sign conventions
        self._hold_n   += msg.linear.x  * CMD_VEL_SCALE   # ENU x → NED north
        self._hold_e   += msg.linear.y  * CMD_VEL_SCALE    # ENU y → NED east  (same sign)
        self._hold_d   -= msg.linear.z  * CMD_VEL_SCALE    # ENU z up → NED down (flip)
        self._hold_yaw -= msg.angular.z * (1.0 / TICK_HZ)  # ENU CCW → NED CW  (flip)

    # ── 20 Hz heartbeat ───────────────────────────────────────────────────────

    def _tick(self) -> None:
        ts = self.get_clock().now().nanoseconds // 1000

        # 1. Publish OffboardControlMode (position control)
        ocm = OffboardControlMode()
        ocm.timestamp    = ts
        ocm.position     = True
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        self._ocm_pub.publish(ocm)

        # 2. Publish TrajectorySetpoint
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.velocity  = [float('nan'), float('nan'), float('nan')]
        sp.yaw       = self._hold_yaw

        if self._armed and self._offboard:
            # Holding a locked NED position
            sp.position = [self._hold_n, self._hold_e, self._hold_d]
        else:
            # Pre-arm: command current position (drone stays on ground)
            sp.position = [self._cur_n, self._cur_e, self._cur_d]

        self._sp_pub.publish(sp)

        # 3. Arm sequence: count cycles after /mission/start, then arm
        if self._started and not self._armed:
            self._cycle += 1
            if self._cycle == PRE_ARM_CYCLES:
                self._arm()

    # ── PX4 commands ─────────────────────────────────────────────────────────

    def _arm(self) -> None:
        """Lock the hover target, switch to OFFBOARD mode, then ARM."""
        # Lock hover target: same XY, climb TARGET_ALT_M above current position
        self._hold_n = self._cur_n
        self._hold_e = self._cur_e
        self._hold_d = self._cur_d - TARGET_ALT_M   # NED: subtract = go up
        self.get_logger().info(
            f'Hover target locked: N={self._hold_n:.2f}  '
            f'E={self._hold_e:.2f}  D={self._hold_d:.2f}  '
            f'(= {TARGET_ALT_M} m above takeoff point)'
        )
        self._send_offboard_mode()
        self._send_arm_cmd()

    def _send_offboard_mode(self) -> None:
        cmd = VehicleCommand()
        cmd.timestamp        = self.get_clock().now().nanoseconds // 1000
        cmd.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1           = 1.0   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2           = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.target_system    = 1
        cmd.target_component = 1
        cmd.source_system    = 1
        cmd.source_component = 1
        cmd.from_external    = True
        self._cmd_pub.publish(cmd)
        self.get_logger().info('SET_MODE OFFBOARD sent.')

    def _send_arm_cmd(self) -> None:
        cmd = VehicleCommand()
        cmd.timestamp        = self.get_clock().now().nanoseconds // 1000
        cmd.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1           = 1.0       # arm
        cmd.param2           = 21196.0   # force-arm (SITL safety bypass)
        cmd.target_system    = 1
        cmd.target_component = 1
        cmd.source_system    = 1
        cmd.source_component = 1
        cmd.from_external    = True
        self._cmd_pub.publish(cmd)
        self.get_logger().info('ARM sent.')


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init(args=sys.argv)
    node = SimpleOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
