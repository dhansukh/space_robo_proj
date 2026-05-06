#!/usr/bin/env python3
"""
depth_colorizer.py
==================
Converts /depth_camera/depth (32FC1, metres) to /depth_camera/depth_colorized
(mono8, 0-255) for display in RViz's Image plugin.

Pixel mapping:  0.1 m → 0,  10.0 m → 255.  NaN / Inf → 0 (black).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


NEAR = 0.1
FAR  = 10.0


class DepthColorizer(Node):
    def __init__(self) -> None:
        super().__init__('depth_colorizer')
        self._pub = self.create_publisher(Image, '/depth_camera/depth_colorized', 10)
        self._sub = self.create_subscription(
            Image, '/depth_camera/depth', self._cb, 10)

    def _cb(self, msg: Image) -> None:
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        # clamp and normalize to uint8
        valid = np.isfinite(depth)
        out = np.zeros_like(depth, dtype=np.uint8)
        out[valid] = np.clip(
            (depth[valid] - NEAR) / (FAR - NEAR) * 255.0, 0, 255
        ).astype(np.uint8)

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width  = msg.width
        out_msg.encoding = 'mono8'
        out_msg.is_bigendian = 0
        out_msg.step = msg.width
        out_msg.data = out.tobytes()
        self._pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
