#!/usr/bin/env python3
"""
drone_controller.py
===================
ROS 2 node that provides basic velocity-based control for the cave exploration
drone simulated in Gazebo Harmonic.

Responsibilities
----------------
* Subscribes to ``/cmd_vel`` (geometry_msgs/Twist) for desired velocities.
* Publishes ``/drone/odom`` (nav_msgs/Odometry) from Gazebo ground-truth pose.
* Broadcasts the ``odom`` → ``base_link`` TF transform.
* Runs a simple PID controller that converts Twist commands into Gazebo
  ``ApplyLinkWrench`` service calls (or topic-based wrench in Harmonic).

Parameters (declared via ``drone_params.yaml``)
------------------------------------------------
max_linear_velocity : float  — clamp for linear x/y/z commands (m/s)
max_angular_velocity: float  — clamp for angular z command (rad/s)
mass                : float  — drone body mass (kg), used for hover thrust
hover_thrust        : float  — nominal upward force to counteract gravity (N)
pid_linear.kp/ki/kd : float  — PID gains for linear velocity control
pid_angular.kp/ki/kd: float  — PID gains for yaw-rate control
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Wrench, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ros_gz_interfaces — used for ApplyLinkWrench (Gazebo Harmonic)
try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import EntityFactory
except ImportError:
    pass  # optional at import time; node degrades gracefully


class PIDController:
    """Simple single-axis PID controller with anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -100.0, output_max: float = 100.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_call: bool = True

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output given an *error* and time delta *dt* (seconds).

        Returns the clamped control output.
        """
        if dt <= 0.0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup (clamping)
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative (backward difference, skip first call)
        if self._first_call:
            d_term = 0.0
            self._first_call = False
        else:
            d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        output = p_term + i_term + d_term

        # Clamp and back-calculate to prevent integrator windup
        clamped = max(self.output_min, min(self.output_max, output))
        if clamped != output:
            self._integral -= error * dt  # undo last integration step

        return clamped


class DroneController(Node):
    """
    ROS 2 node for cave-drone velocity control and odometry publishing.

    The node converts incoming Twist commands to body-frame forces/torques
    applied via the Gazebo ``ApplyLinkWrench`` interface, and publishes
    odometry from the Gazebo ground-truth pose topic.
    """

    def __init__(self) -> None:
        super().__init__('drone_controller')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter('max_linear_velocity', 2.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('mass', 1.5)
        self.declare_parameter('hover_thrust', 14.715)
        self.declare_parameter('pid_linear.kp', 5.0)
        self.declare_parameter('pid_linear.ki', 0.1)
        self.declare_parameter('pid_linear.kd', 2.0)
        self.declare_parameter('pid_angular.kp', 3.0)
        self.declare_parameter('pid_angular.ki', 0.05)
        self.declare_parameter('pid_angular.kd', 1.0)

        # ------------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------------
        self._max_lin_vel: float = self.get_parameter(
            'max_linear_velocity').get_parameter_value().double_value
        self._max_ang_vel: float = self.get_parameter(
            'max_angular_velocity').get_parameter_value().double_value
        self._mass: float = self.get_parameter(
            'mass').get_parameter_value().double_value
        self._hover_thrust: float = self.get_parameter(
            'hover_thrust').get_parameter_value().double_value

        lin_kp = self.get_parameter('pid_linear.kp').get_parameter_value().double_value
        lin_ki = self.get_parameter('pid_linear.ki').get_parameter_value().double_value
        lin_kd = self.get_parameter('pid_linear.kd').get_parameter_value().double_value
        ang_kp = self.get_parameter('pid_angular.kp').get_parameter_value().double_value
        ang_ki = self.get_parameter('pid_angular.ki').get_parameter_value().double_value
        ang_kd = self.get_parameter('pid_angular.kd').get_parameter_value().double_value

        # ------------------------------------------------------------------
        # PID controllers  (one per axis: x, y, z linear; yaw angular)
        # ------------------------------------------------------------------
        force_limit = self._mass * 50.0  # generous force ceiling
        self._pid_x = PIDController(lin_kp, lin_ki, lin_kd,
                                    -force_limit, force_limit)
        self._pid_y = PIDController(lin_kp, lin_ki, lin_kd,
                                    -force_limit, force_limit)
        self._pid_z = PIDController(lin_kp, lin_ki, lin_kd,
                                    -force_limit, force_limit)
        self._pid_yaw = PIDController(ang_kp, ang_ki, ang_kd,
                                      -self._mass * 10.0, self._mass * 10.0)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._cmd_vel: Optional[Twist] = None
        self._current_vel_x: float = 0.0
        self._current_vel_y: float = 0.0
        self._current_vel_z: float = 0.0
        self._current_yaw_rate: float = 0.0
        self._last_control_time: Optional[float] = None

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            reliable_qos,
        )

        # Ground-truth odometry from Gazebo via ros_gz_bridge
        self._gz_odom_sub = self.create_subscription(
            Odometry,
            '/drone/gz/odom',
            self._gz_odom_callback,
            sensor_qos,
        )

        # IMU for velocity estimation feedback (optional)
        self._imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self._imu_callback,
            sensor_qos,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._odom_pub = self.create_publisher(
            Odometry, '/drone/odom', reliable_qos)

        # Wrench command published to Gazebo's ApplyLinkWrench topic
        self._wrench_pub = self.create_publisher(
            Wrench,
            '/drone/wrench_cmd',
            reliable_qos,
        )

        # ------------------------------------------------------------------
        # TF broadcaster
        # ------------------------------------------------------------------
        self._tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Control loop timer (50 Hz)
        # ------------------------------------------------------------------
        self._control_timer = self.create_timer(0.02, self._control_loop)

        self.get_logger().info(
            f'DroneController started | '
            f'max_lin={self._max_lin_vel} m/s  '
            f'hover_thrust={self._hover_thrust} N'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Cache the latest velocity command, clamping to safe limits."""
        self._cmd_vel = msg

    def _gz_odom_callback(self, msg: Odometry) -> None:
        """Re-publish Gazebo ground-truth odometry as /drone/odom and TF."""
        # Stamp and frame names
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Update current velocity estimate from odometry twist
        self._current_vel_x = msg.twist.twist.linear.x
        self._current_vel_y = msg.twist.twist.linear.y
        self._current_vel_z = msg.twist.twist.linear.z
        self._current_yaw_rate = msg.twist.twist.angular.z

        self._odom_pub.publish(msg)
        self._broadcast_odom_tf(msg)

    def _imu_callback(self, _msg: Imu) -> None:
        """IMU callback reserved for future use (e.g. attitude estimation)."""
        pass

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------

    def _broadcast_odom_tf(self, odom: Odometry) -> None:
        """Broadcast odom → base_link transform from odometry message."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation

        self._tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """
        50 Hz control loop.

        Converts the desired Twist into body-frame forces and torques via
        independent PID controllers for vx, vy, vz, and yaw-rate.
        The resulting Wrench is published for Gazebo's ApplyLinkWrench.
        """
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._last_control_time is None:
            self._last_control_time = now
            return

        dt = now - self._last_control_time
        self._last_control_time = now

        if self._cmd_vel is None:
            # No command yet — just apply hover thrust
            self._publish_wrench(0.0, 0.0, self._hover_thrust, 0.0, 0.0, 0.0)
            return

        cmd = self._cmd_vel

        # Clamp desired velocities
        des_vx = max(-self._max_lin_vel,
                     min(self._max_lin_vel, cmd.linear.x))
        des_vy = max(-self._max_lin_vel,
                     min(self._max_lin_vel, cmd.linear.y))
        des_vz = max(-self._max_lin_vel,
                     min(self._max_lin_vel, cmd.linear.z))
        des_yaw = max(-self._max_ang_vel,
                      min(self._max_ang_vel, cmd.angular.z))

        # PID errors
        err_vx = des_vx - self._current_vel_x
        err_vy = des_vy - self._current_vel_y
        err_vz = des_vz - self._current_vel_z
        err_yaw = des_yaw - self._current_yaw_rate

        # Compute force/torque outputs
        fx = self._pid_x.compute(err_vx, dt)
        fy = self._pid_y.compute(err_vy, dt)
        # Z force = hover + PID correction
        fz = self._hover_thrust + self._pid_z.compute(err_vz, dt)
        tz = self._pid_yaw.compute(err_yaw, dt)

        self._publish_wrench(fx, fy, fz, 0.0, 0.0, tz)

    def _publish_wrench(
        self,
        fx: float, fy: float, fz: float,
        tx: float, ty: float, tz: float,
    ) -> None:
        """Publish a body-frame Wrench message."""
        wrench = Wrench()
        wrench.force = Vector3(x=fx, y=fy, z=fz)
        wrench.torque = Vector3(x=tx, y=ty, z=tz)
        self._wrench_pub.publish(wrench)


def main(args=None) -> None:
    """Entry point for the drone_controller node."""
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
