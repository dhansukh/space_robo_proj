"""
px4_vel_bridge.py
=================
Bridges /cmd_vel (Twist) to PX4 SITL offboard control.

Also relays /fmu/out/vehicle_local_position → /slam/odom so that
mission_manager has a pose immediately on arm, breaking the circular
dependency: mission_manager needs /slam/odom to send cmd_vel, but
SLAM needs the drone to be moving first.  Once real SLAM starts
publishing its own /slam/odom the relay becomes a no-op (whoever
publishes last wins — the real SLAM takes over).

Sequence on /mission/start:
  1. DO_SET_MODE → OFFBOARD   (must be before arm in SITL)
  2. ARM (force-arm)
  3. Bootstrap: command z=+0.5 m/s for up to 4 s if no cmd_vel arrives
     (guards against rare race where mission_manager hasn't received
     the first /slam/odom yet)
  4. Normal: cmd_vel → TrajectorySetpoint at 20 Hz
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

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

_PX4_CUSTOM_MAIN_OFFBOARD = 6   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
_BOOTSTRAP_VZ   = 0.5           # m/s upward during bootstrap
_BOOTSTRAP_SECS = 4.0           # seconds of bootstrap before giving up


class PX4VelBridge(Node):
    """Converts /cmd_vel Twist to PX4 offboard TrajectorySetpoints."""

    def __init__(self) -> None:
        super().__init__('px4_vel_bridge')

        self.declare_parameter('max_vxy', 1.5)
        self.declare_parameter('max_vz',  1.0)
        self.declare_parameter('max_yaw_rate', 1.0)

        self._max_vxy = self.get_parameter('max_vxy').value
        self._max_vz  = self.get_parameter('max_vz').value
        self._max_yr  = self.get_parameter('max_yaw_rate').value

        self._armed   = False
        self._offboard = False

        # cmd_vel state
        self._vx = self._vy = self._vz = self._yaw_rate = 0.0
        self._last_cmdvel_t: float = 0.0   # monotonic time of last cmd_vel

        # bootstrap state: active for _BOOTSTRAP_SECS after /mission/start
        self._bootstrap_start: float = 0.0
        self._bootstrapping = False

        # ── publishers ──────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _RELIABLE_QOS)
        self._odom_pub = self.create_publisher(
            Odometry, '/slam/odom', _RELIABLE_QOS)

        # ── subscriptions ───────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_cb, _RELIABLE_QOS)
        self.create_subscription(Empty, '/mission/start', self._start_cb, _RELIABLE_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, _PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, _PX4_QOS)

        # 20 Hz heartbeat (PX4 requires OffboardControlMode at ≥2 Hz)
        self.create_timer(0.05, self._heartbeat)

        self.get_logger().info(
            'PX4VelBridge ready — publish /mission/start to arm + offboard.')

    # ── subscriptions ────────────────────────────────────────────────────

    def _cmdvel_cb(self, msg: Twist) -> None:
        self._vx = _clamp(msg.linear.x,  self._max_vxy)
        self._vy = _clamp(msg.linear.y,  self._max_vxy)
        self._vz = _clamp(msg.linear.z,  self._max_vz)
        self._yaw_rate = _clamp(msg.angular.z, self._max_yr)
        self._last_cmdvel_t = time.monotonic()

    def _start_cb(self, _: Empty) -> None:
        self.get_logger().info('PX4VelBridge: /mission/start → offboard + arm.')
        # Order matters: set mode first, then arm
        self._send_offboard_mode()
        self._send_arm()
        self._bootstrapping = True
        self._bootstrap_start = time.monotonic()

    def _status_cb(self, msg: VehicleStatus) -> None:
        armed_now    = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        offboard_now = (msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        if armed_now != self._armed or offboard_now != self._offboard:
            self._armed    = armed_now
            self._offboard = offboard_now
            self.get_logger().info(
                f'PX4VelBridge: armed={self._armed}  offboard={self._offboard}')

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        """Relay PX4 NED local position as ROS ENU /slam/odom fallback.

        This gives mission_manager an immediate pose on arm so it can
        send cmd_vel for takeoff without waiting for SLAM to initialise.
        Real SLAM will overwrite /slam/odom once its ICP converges.
        """
        if not (msg.xy_valid and msg.z_valid):
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id  = 'base_link'

        # NED → ENU:  ros_x = px4_y, ros_y = px4_x, ros_z = -px4_z
        odom.pose.pose.position.x = float(msg.y)
        odom.pose.pose.position.y = float(msg.x)
        odom.pose.pose.position.z = -float(msg.z)

        # Velocity NED → ENU
        odom.twist.twist.linear.x = float(msg.vy)
        odom.twist.twist.linear.y = float(msg.vx)
        odom.twist.twist.linear.z = -float(msg.vz)

        odom.pose.pose.orientation.w = 1.0  # no heading tracking from local pos alone

        self._odom_pub.publish(odom)

    # ── 20 Hz heartbeat ──────────────────────────────────────────────────

    def _heartbeat(self) -> None:
        ts = self._now_us()
        now = time.monotonic()

        # OffboardControlMode must be published continuously
        ocm = OffboardControlMode()
        ocm.timestamp  = ts
        ocm.velocity   = True
        ocm.position   = False
        ocm.acceleration = False
        ocm.attitude   = False
        ocm.body_rate  = False
        self._ocm_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position  = [float('nan'), float('nan'), float('nan')]
        sp.yaw       = float('nan')

        if self._armed and self._offboard:
            cmd_vel_age = now - self._last_cmdvel_t
            bootstrap_active = (
                self._bootstrapping
                and (now - self._bootstrap_start) < _BOOTSTRAP_SECS
                and cmd_vel_age > 0.5   # no recent cmd_vel
            )

            if bootstrap_active:
                # No cmd_vel yet — nudge upward so drone lifts before disarm timer fires
                # PX4 TrajectorySetpoint uses NED: [North, East, Down]
                # ENU up (+0.5) → NED down (-0.5)
                sp.velocity  = [0.0, 0.0, -_BOOTSTRAP_VZ]
                sp.yawspeed  = 0.0
            else:
                self._bootstrapping = False
                # ENU cmd_vel → NED setpoint:
                #   NED North = ENU y  (self._vy)
                #   NED East  = ENU x  (self._vx)
                #   NED Down  = -ENU z (-self._vz)
                sp.velocity  = [self._vy, self._vx, -self._vz]
                sp.yawspeed  = -self._yaw_rate  # ENU CCW = NED CW (opposite sign)
        else:
            sp.velocity  = [0.0, 0.0, 0.0]
            sp.yawspeed  = 0.0

        self._sp_pub.publish(sp)

    # ── PX4 commands ─────────────────────────────────────────────────────

    def _send_arm(self) -> None:
        cmd = VehicleCommand()
        cmd.timestamp          = self._now_us()
        cmd.command            = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1             = 1.0       # arm
        cmd.param2             = 21196.0   # force-arm (SITL bypass)
        cmd.target_system      = 1
        cmd.target_component   = 1
        cmd.source_system      = 1
        cmd.source_component   = 1
        cmd.from_external      = True
        self._cmd_pub.publish(cmd)
        self.get_logger().info('PX4VelBridge: ARM sent.')

    def _send_offboard_mode(self) -> None:
        cmd = VehicleCommand()
        cmd.timestamp          = self._now_us()
        cmd.command            = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1             = 1.0                              # custom mode flag
        cmd.param2             = float(_PX4_CUSTOM_MAIN_OFFBOARD) # offboard
        cmd.target_system      = 1
        cmd.target_component   = 1
        cmd.source_system      = 1
        cmd.source_component   = 1
        cmd.from_external      = True
        self._cmd_pub.publish(cmd)
        self.get_logger().info('PX4VelBridge: SET_MODE OFFBOARD sent.')

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PX4VelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
