#!/usr/bin/env python3
"""
tf_broadcaster.py
=================
ROS 2 node that publishes static TF transforms for all sensor frames
on the cave exploration drone.

Static transforms published
---------------------------
base_link → lidar_link
    translation : (0.0, 0.0, 0.1)   — LiDAR on top of body
    rotation    : identity

base_link → depth_camera_link
    translation : (0.15, 0.0, -0.02) — forward-facing, slightly below centre
    rotation    : pitch = -10° (−0.1745329 rad, nose-down)

base_link → imu_link
    translation : (0.0, 0.0, 0.0)   — at centre of mass
    rotation    : identity

Usage
-----
Run as a standalone node or include via a launch file:

    ros2 run drone_bringup tf_broadcaster

All transforms are published once at startup (static) and re-latched so
late-joining subscribers receive them immediately.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


def _make_transform(
    parent_frame: str,
    child_frame: str,
    tx: float,
    ty: float,
    tz: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    stamp=None,
) -> TransformStamped:
    """
    Build a :class:`geometry_msgs.msg.TransformStamped` from Euler angles.

    Parameters
    ----------
    parent_frame : str
        The parent TF frame id.
    child_frame : str
        The child TF frame id.
    tx, ty, tz : float
        Translation in metres.
    roll, pitch, yaw : float
        Rotation in radians (ZYX convention).
    stamp : rclpy.time.Time or None
        Timestamp; if ``None`` use epoch (valid for static transforms).

    Returns
    -------
    TransformStamped
    """
    ts = TransformStamped()

    if stamp is not None:
        ts.header.stamp = stamp.to_msg()
    # else: leave at zero — static TF broadcaster sets its own stamp

    ts.header.frame_id = parent_frame
    ts.child_frame_id = child_frame

    ts.transform.translation.x = tx
    ts.transform.translation.y = ty
    ts.transform.translation.z = tz

    # Euler → quaternion (ZYX / extrinsic XYZ convention)
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    ts.transform.rotation.w = cr * cp * cy + sr * sp * sy
    ts.transform.rotation.x = sr * cp * cy - cr * sp * sy
    ts.transform.rotation.y = cr * sp * cy + sr * cp * sy
    ts.transform.rotation.z = cr * cp * sy - sr * sp * cy

    return ts


class DroneTFBroadcaster(Node):
    """
    Publishes static TF frames for all drone sensor links.

    This node should be started once alongside ``robot_state_publisher``
    so that all sensor frames are immediately resolvable in TF.
    """

    #: Camera pitch-down angle in radians (−10°)
    CAMERA_PITCH_RAD: float = -math.radians(10.0)

    def __init__(self) -> None:
        super().__init__('drone_tf_broadcaster')

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self.get_logger().info(
            'Static TF transforms published: '
            'base_link → lidar_link, depth_camera_link, imu_link'
        )

    def _publish_static_transforms(self) -> None:
        """Compute and broadcast all static sensor transforms."""
        transforms = [
            self._lidar_transform(),
            self._depth_camera_transform(),
            self._imu_transform(),
        ]
        self._static_broadcaster.sendTransform(transforms)

    def _lidar_transform(self) -> TransformStamped:
        """
        Return the static transform from ``base_link`` to ``lidar_link``.

        The LiDAR is mounted 0.1 m above the body centre, centred in X/Y.
        """
        return _make_transform(
            parent_frame='base_link',
            child_frame='lidar_link',
            tx=0.0,
            ty=0.0,
            tz=0.1,
        )

    def _depth_camera_transform(self) -> TransformStamped:
        """
        Return the static transform from ``base_link`` to ``depth_camera_link``.

        The depth camera is mounted:
          * 0.15 m forward of body centre
          * 0.02 m below body centre
          * pitched 10° nose-down (−0.1745 rad about Y-axis)
        """
        return _make_transform(
            parent_frame='base_link',
            child_frame='depth_camera_link',
            tx=0.15,
            ty=0.0,
            tz=-0.02,
            pitch=self.CAMERA_PITCH_RAD,
        )

    def _imu_transform(self) -> TransformStamped:
        """
        Return the static transform from ``base_link`` to ``imu_link``.

        The IMU is co-located with the centre of mass — zero offset.
        """
        return _make_transform(
            parent_frame='base_link',
            child_frame='imu_link',
            tx=0.0,
            ty=0.0,
            tz=0.0,
        )


def main(args=None) -> None:
    """Entry point for the tf_broadcaster node."""
    rclpy.init(args=args)
    node = DroneTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
