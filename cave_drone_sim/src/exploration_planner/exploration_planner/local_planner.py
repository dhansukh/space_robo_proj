"""
local_planner.py
================
Local obstacle avoidance using Artificial Potential Fields (APF).

The local planner tracks a global path (published by path_planner) and
reactively adjusts velocity using LiDAR-derived repulsive forces.

Subscriptions
-------------
/drone/lidar/points   (sensor_msgs/PointCloud2)  — raw LiDAR
/planning/path        (nav_msgs/Path)             — global path
/slam/odom            (nav_msgs/Odometry)         — current pose

Publications
------------
/cmd_vel              (geometry_msgs/Twist)       — velocity command
/local_planner/status (std_msgs/String)           — planner state string
"""

from __future__ import annotations

import threading
from enum import Enum
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2


# ---------------------------------------------------------------------------
# Local planner state
# ---------------------------------------------------------------------------

class LocalPlannerState(Enum):
    IDLE            = 'idle'
    FOLLOWING       = 'following'
    AVOIDING        = 'avoiding'
    STUCK           = 'stuck'
    EMERGENCY_STOP  = 'emergency_stop'


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class LocalPlannerNode(Node):
    """
    APF-based local planner for 3-D drone navigation.

    The planner maintains an index into the global path (``_wp_idx``) and
    advances it whenever the drone comes within ``waypoint_reach_tolerance``
    of the current target waypoint.

    Force model
    -----------
    F_att = k_att * (target - pos)          (capped to max_speed)
    F_rep = k_rep * (1/d - 1/d0)^2 * unit  for each obstacle within d0
    v_cmd = clip(F_att + ΣF_rep, max_speed)
    """

    def __init__(self) -> None:
        super().__init__('local_planner')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_influence_distance', 2.0)
        self.declare_parameter('emergency_stop_distance', 0.3)
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('k_repulsive', 0.5)
        self.declare_parameter('path_deviation_threshold', 2.0)
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('waypoint_reach_tolerance', 0.4)
        self.declare_parameter('control_rate', 20.0)

        def _dp(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._max_speed: float = _dp('max_speed')
        self._max_ang: float = _dp('max_angular_speed')
        self._d0: float = _dp('obstacle_influence_distance')
        self._e_stop_dist: float = _dp('emergency_stop_distance')
        self._k_att: float = _dp('k_attractive')
        self._k_rep: float = _dp('k_repulsive')
        self._dev_thresh: float = _dp('path_deviation_threshold')
        self._lookahead: float = _dp('lookahead_distance')
        self._wp_tol: float = _dp('waypoint_reach_tolerance')
        rate: float = _dp('control_rate')

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._lock = threading.RLock()
        self._pose: Optional[np.ndarray] = None          # (x, y, z)
        self._orientation: Optional[np.ndarray] = None   # quaternion (x,y,z,w)
        self._path: List[np.ndarray] = []                # ordered waypoints
        self._wp_idx: int = 0                            # current target index
        self._lidar_pts: Optional[np.ndarray] = None     # shape (N, 3)
        self._state: LocalPlannerState = LocalPlannerState.IDLE
        self._stuck_counter: int = 0
        self._replan_requested: bool = False

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self._lidar_sub = self.create_subscription(
            PointCloud2, '/drone/lidar/points',
            self._lidar_cb, sensor_qos,
        )
        self._path_sub = self.create_subscription(
            Path, '/planning/path',
            self._path_cb, reliable_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/slam/odom',
            self._odom_cb, sensor_qos,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self._status_pub = self.create_publisher(
            String, '/local_planner/status', reliable_qos
        )

        # ------------------------------------------------------------------
        # Control loop timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'LocalPlanner ready — max_speed={self._max_speed} m/s, '
            f'emergency_stop={self._e_stop_dist} m'
        )

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _lidar_cb(self, msg: PointCloud2) -> None:
        try:
            pts = np.array(
                list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)),
                dtype=np.float64,
            )
            with self._lock:
                self._lidar_pts = pts[:, :3] if pts.shape[0] > 0 else None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'LocalPlanner: lidar_cb error: {exc}')

    def _path_cb(self, msg: Path) -> None:
        waypoints = []
        for ps in msg.poses:
            waypoints.append(np.array([
                ps.pose.position.x,
                ps.pose.position.y,
                ps.pose.position.z,
            ], dtype=np.float64))
        with self._lock:
            self._path = waypoints
            self._wp_idx = 0
            self._stuck_counter = 0
            if waypoints:
                self._state = LocalPlannerState.FOLLOWING
                self.get_logger().info(
                    f'LocalPlanner: received path with {len(waypoints)} waypoints.'
                )

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            self._pose = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            self._orientation = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

    # ======================================================================
    # Control loop
    # ======================================================================

    def _control_loop(self) -> None:
        """Main APF control cycle — runs at control_rate Hz."""
        with self._lock:
            pose = self._pose
            lidar = self._lidar_pts
            path = self._path
            wp_idx = self._wp_idx
            orientation = self._orientation

        if pose is None:
            self._publish_status(LocalPlannerState.IDLE.value)
            return

        # ------------------------------------------------------------------
        # Safety: emergency stop check
        # ------------------------------------------------------------------
        if lidar is not None and len(lidar) > 0:
            # Transform LiDAR points to world frame
            world_pts = self._lidar_to_world(lidar, pose, orientation)
            min_dist = float(np.min(np.linalg.norm(world_pts - pose, axis=1)))

            if min_dist < self._e_stop_dist:
                self._publish_cmd(np.zeros(3), 0.0)
                self._set_state(LocalPlannerState.EMERGENCY_STOP)
                self.get_logger().warn(
                    f'LocalPlanner: EMERGENCY STOP — obstacle at {min_dist:.2f}m'
                )
                return

        # ------------------------------------------------------------------
        # No path → idle
        # ------------------------------------------------------------------
        if not path or wp_idx >= len(path):
            self._publish_cmd(np.zeros(3), 0.0)
            self._set_state(LocalPlannerState.IDLE)
            return

        target = path[wp_idx]

        # ------------------------------------------------------------------
        # Advance waypoint index if close enough
        # ------------------------------------------------------------------
        dist_to_wp = float(np.linalg.norm(target - pose))
        if dist_to_wp < self._wp_tol:
            with self._lock:
                self._wp_idx = min(self._wp_idx + 1, len(path) - 1)
            wp_idx = self._wp_idx
            if wp_idx >= len(path):
                self._publish_cmd(np.zeros(3), 0.0)
                self._set_state(LocalPlannerState.IDLE)
                return
            target = path[wp_idx]

        # ------------------------------------------------------------------
        # Path deviation check
        # ------------------------------------------------------------------
        path_pts = np.array(path)
        dists_to_path = np.linalg.norm(path_pts - pose, axis=1)
        min_path_dist = float(np.min(dists_to_path))
        if min_path_dist > self._dev_thresh:
            with self._lock:
                self._replan_requested = True
            self.get_logger().warn(
                f'LocalPlanner: path deviation {min_path_dist:.2f}m > threshold — replan needed.'
            )

        # ------------------------------------------------------------------
        # APF computation
        # ------------------------------------------------------------------
        f_att = self._attractive_force(pose, target)
        f_rep = np.zeros(3, dtype=np.float64)
        avoiding = False

        if lidar is not None and len(lidar) > 0:
            world_pts = self._lidar_to_world(lidar, pose, orientation)
            f_rep, avoiding = self._repulsive_force(pose, world_pts)

        velocity = f_att + f_rep

        # Forward collision check along velocity vector
        v_norm = float(np.linalg.norm(velocity))
        if v_norm > 1e-6 and lidar is not None and len(lidar) > 0:
            v_dir = velocity / v_norm
            look_pt = pose + v_dir * self._lookahead
            fwd_dist = float(np.min(np.linalg.norm(world_pts - look_pt, axis=1)))
            if fwd_dist < self._e_stop_dist * 2:
                # Slow down proportionally
                scale = max(0.0, fwd_dist / (self._e_stop_dist * 2))
                velocity *= scale

        # Cap to max speed
        velocity = self._clip_vector(velocity, self._max_speed)

        # Yaw: point toward target
        yaw_cmd = self._compute_yaw(pose, target, orientation)

        self._publish_cmd(velocity, yaw_cmd)

        if avoiding:
            self._set_state(LocalPlannerState.AVOIDING)
        else:
            self._set_state(LocalPlannerState.FOLLOWING)

    # ======================================================================
    # APF helpers
    # ======================================================================

    def _attractive_force(self, pos: np.ndarray, target: np.ndarray) -> np.ndarray:
        """F_att = k_att * (target - pos), capped at max_speed."""
        delta = target - pos
        f = self._k_att * delta
        return self._clip_vector(f, self._max_speed)

    def _repulsive_force(
        self, pos: np.ndarray, obstacle_pts: np.ndarray
    ) -> tuple[np.ndarray, bool]:
        """
        Σ F_rep for all obstacle points within d0.

        Returns the total repulsive force and a boolean indicating whether
        any obstacles were within the influence distance.
        """
        f_total = np.zeros(3, dtype=np.float64)
        any_close = False

        deltas = pos - obstacle_pts            # vectors pointing away (N, 3)
        dists = np.linalg.norm(deltas, axis=1) # shape (N,)

        mask = dists < self._d0
        if not np.any(mask):
            return f_total, False

        any_close = True
        close_deltas = deltas[mask]            # (M, 3)
        close_dists = dists[mask]              # (M,)

        # Avoid division by zero
        safe_dists = np.maximum(close_dists, 1e-4)

        # F_rep = k_rep * (1/d - 1/d0)^2 * unit_away
        coeff = self._k_rep * (1.0 / safe_dists - 1.0 / self._d0) ** 2
        units = close_deltas / safe_dists[:, None]
        f_total = np.sum(coeff[:, None] * units, axis=0)

        # Cap repulsive magnitude
        mag = float(np.linalg.norm(f_total))
        if mag > self._max_speed * 2:
            f_total = f_total / mag * self._max_speed * 2

        return f_total, any_close

    # ======================================================================
    # Utility helpers
    # ======================================================================

    @staticmethod
    def _lidar_to_world(
        lidar_pts: np.ndarray,
        pose: np.ndarray,
        orientation: Optional[np.ndarray],
    ) -> np.ndarray:
        """
        Transform LiDAR points from sensor frame to world frame.
        Assumes LiDAR is rigidly mounted at the drone body origin.
        Uses the drone's quaternion orientation for rotation.
        """
        if orientation is None:
            return lidar_pts + pose

        # Quaternion → rotation matrix (passive convention)
        x, y, z, w = orientation
        R = np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y)],
        ], dtype=np.float64)

        rotated = lidar_pts @ R.T
        return rotated + pose

    @staticmethod
    def _clip_vector(v: np.ndarray, max_mag: float) -> np.ndarray:
        mag = float(np.linalg.norm(v))
        if mag > max_mag:
            return v / mag * max_mag
        return v

    def _compute_yaw(
        self,
        pos: np.ndarray,
        target: np.ndarray,
        orientation: Optional[np.ndarray],
    ) -> float:
        """Return yaw rate command to face toward target."""
        delta = target - pos
        desired_yaw = float(np.arctan2(delta[1], delta[0]))

        if orientation is None:
            return 0.0

        # Extract current yaw from quaternion
        x, y, z, w = orientation
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        current_yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        yaw_error = desired_yaw - current_yaw
        # Wrap to [-pi, pi]
        yaw_error = float(np.arctan2(np.sin(yaw_error), np.cos(yaw_error)))

        # Proportional yaw control
        yaw_rate = np.clip(yaw_error * 1.0, -self._max_ang, self._max_ang)
        return float(yaw_rate)

    # ======================================================================
    # Publishers
    # ======================================================================

    def _publish_cmd(self, velocity: np.ndarray, yaw_rate: float) -> None:
        msg = Twist()
        msg.linear.x = float(velocity[0])
        msg.linear.y = float(velocity[1])
        msg.linear.z = float(velocity[2])
        msg.angular.z = float(yaw_rate)
        self._cmd_pub.publish(msg)

    def _set_state(self, state: LocalPlannerState) -> None:
        self._state = state
        self._publish_status(state.value)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    @property
    def replan_requested(self) -> bool:
        """True if the local planner has detected path deviation and needs replan."""
        with self._lock:
            val = self._replan_requested
            self._replan_requested = False  # reset after reading
            return val


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
