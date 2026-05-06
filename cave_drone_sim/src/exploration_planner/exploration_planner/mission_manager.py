"""
mission_manager.py
==================
Mission-level state machine that coordinates the exploration drone.

States
------
IDLE          → waiting for mission start command
TAKEOFF       → ascending to exploration altitude
EXPLORING     → frontier-based exploration active
NAVIGATING    → goal-directed A→B navigation
RETURNING     → returning to home position
LANDING       → descending and landing
EMERGENCY     → SLAM failure or imminent collision recovery

Subscriptions
-------------
/slam/odom                (nav_msgs/Odometry)
/slam/degeneracy_score    (std_msgs/Float64)
/exploration/status       (std_msgs/String)
/local_planner/status     (std_msgs/String)
/mission/start            (std_msgs/Empty)
/mission/goal             (geometry_msgs/PoseStamped)
/drone/battery            (std_msgs/Float64)

Publications
------------
/mission/state  (std_msgs/String)
/mission/info   (std_msgs/String)
/exploration/goal (geometry_msgs/PoseStamped)  — forwarded from exploration or mission
/cmd_vel        (geometry_msgs/Twist)          — direct velocity during takeoff/landing
"""

from __future__ import annotations

import threading
import time
from enum import Enum, auto
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Empty


# ---------------------------------------------------------------------------
# State enum
# ---------------------------------------------------------------------------

class MissionState(Enum):
    IDLE        = 'IDLE'
    TAKEOFF     = 'TAKEOFF'
    EXPLORING   = 'EXPLORING'
    NAVIGATING  = 'NAVIGATING'
    RETURNING   = 'RETURNING'
    LANDING     = 'LANDING'
    EMERGENCY   = 'EMERGENCY'


# ---------------------------------------------------------------------------
# Mission manager node
# ---------------------------------------------------------------------------

class MissionManagerNode(Node):
    """
    Finite-state machine coordinating the cave-drone mission.

    The manager does not do planning itself — it delegates to
    frontier_explorer (exploration) and path_planner / local_planner
    (navigation).  It monitors health signals and orchestrates transitions.
    """

    # Allowed transitions: {source: [targets]}
    _TRANSITIONS = {
        MissionState.IDLE:      [MissionState.TAKEOFF],
        MissionState.TAKEOFF:   [MissionState.EXPLORING, MissionState.EMERGENCY],
        MissionState.EXPLORING: [MissionState.EXPLORING, MissionState.NAVIGATING,
                                  MissionState.RETURNING, MissionState.EMERGENCY],
        MissionState.NAVIGATING:[MissionState.EXPLORING, MissionState.RETURNING,
                                  MissionState.EMERGENCY],
        MissionState.RETURNING: [MissionState.LANDING, MissionState.EMERGENCY],
        MissionState.LANDING:   [],
        MissionState.EMERGENCY: [MissionState.RETURNING],
    }

    def __init__(self) -> None:
        super().__init__('mission_manager')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('exploration_altitude', 2.0)
        self.declare_parameter('takeoff_speed', 0.5)
        self.declare_parameter('landing_speed', 0.3)
        self.declare_parameter('battery_drain_rate', 0.001)
        self.declare_parameter('battery_return_threshold', 30.0)
        self.declare_parameter('battery_land_threshold', 10.0)
        self.declare_parameter('home_position', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_reach_tolerance', 0.5)
        self.declare_parameter('slam_degeneracy_threshold', 0.1)
        self.declare_parameter('home_reach_tolerance', 0.5)
        self.declare_parameter('state_publish_rate', 5.0)
        # Allow starting directly in NAVIGATING mode (for navigate.launch.py)
        self.declare_parameter('initial_state', 'IDLE')

        def _dp(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        def _sp(n: str) -> str:
            return self.get_parameter(n).get_parameter_value().string_value

        self._explore_alt: float = _dp('exploration_altitude')
        self._takeoff_speed: float = _dp('takeoff_speed')
        self._landing_speed: float = _dp('landing_speed')
        self._battery_drain: float = _dp('battery_drain_rate')
        self._battery_return_thresh: float = _dp('battery_return_threshold')
        self._battery_land_thresh: float = _dp('battery_land_threshold')
        home_raw = (
            self.get_parameter('home_position').get_parameter_value().double_array_value
        )
        self._home: np.ndarray = np.array(list(home_raw), dtype=np.float64)
        self._goal_tol: float = _dp('goal_reach_tolerance')
        self._degeneracy_thresh: float = _dp('slam_degeneracy_threshold')
        self._home_tol: float = _dp('home_reach_tolerance')
        rate: float = _dp('state_publish_rate')
        initial_state_str: str = _sp('initial_state')

        # ------------------------------------------------------------------
        # Mission state
        # ------------------------------------------------------------------
        self._lock = threading.RLock()
        self._state: MissionState = MissionState[initial_state_str]
        self._pose: Optional[np.ndarray] = None
        self._battery: float = 100.0
        self._battery_simulated: bool = True          # use sim if no /drone/battery
        self._last_battery_tick: float = time.monotonic()
        self._degeneracy_score: float = 1.0
        self._exploration_status: str = 'waiting_for_slam'
        self._local_status: str = 'idle'
        self._mission_goal: Optional[np.ndarray] = None  # explicit A→B goal
        self._return_after_goal: bool = False
        self._emergency_recovery_attempts: int = 0
        self._no_frontier_ticks: int = 0   # consecutive ticks with no_frontiers

        self.get_logger().info(
            f'MissionManager: starting in state {self._state.value}'
        )

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
        self._odom_sub = self.create_subscription(
            Odometry, '/slam/odom', self._odom_cb, sensor_qos
        )
        self._degeneracy_sub = self.create_subscription(
            Float64, '/slam/degeneracy_score', self._degeneracy_cb, reliable_qos
        )
        self._exploration_status_sub = self.create_subscription(
            String, '/exploration/status', self._exploration_status_cb, reliable_qos
        )
        self._local_status_sub = self.create_subscription(
            String, '/local_planner/status', self._local_status_cb, reliable_qos
        )
        self._start_sub = self.create_subscription(
            Empty, '/mission/start', self._start_cb, reliable_qos
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, '/mission/goal', self._goal_cb, reliable_qos
        )
        self._battery_sub = self.create_subscription(
            Float64, '/drone/battery', self._battery_cb, reliable_qos
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._state_pub = self.create_publisher(
            String, '/mission/state', reliable_qos
        )
        self._info_pub = self.create_publisher(
            String, '/mission/info', reliable_qos
        )
        self._cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', reliable_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, '/exploration/goal', reliable_qos
        )

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / rate, self._state_machine_tick)

        self.get_logger().info('MissionManager ready.')

    # ======================================================================
    # Subscription callbacks
    # ======================================================================

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        with self._lock:
            self._pose = np.array([pos.x, pos.y, pos.z], dtype=np.float64)

    def _degeneracy_cb(self, msg: Float64) -> None:
        with self._lock:
            self._degeneracy_score = float(msg.data)

    def _exploration_status_cb(self, msg: String) -> None:
        with self._lock:
            self._exploration_status = msg.data

    def _local_status_cb(self, msg: String) -> None:
        with self._lock:
            self._local_status = msg.data

    def _start_cb(self, _msg: Empty) -> None:
        with self._lock:
            if self._state == MissionState.IDLE:
                self._transition(MissionState.TAKEOFF)
            else:
                self.get_logger().warn(
                    f'MissionManager: /mission/start received in state '
                    f'{self._state.value} — ignored.'
                )

    def _goal_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._mission_goal = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ], dtype=np.float64)
            # If already exploring, switch to navigating immediately
            if self._state == MissionState.EXPLORING:
                self._transition(MissionState.NAVIGATING)
            self.get_logger().info(
                f'MissionManager: mission goal set to {self._mission_goal}'
            )

    def _battery_cb(self, msg: Float64) -> None:
        with self._lock:
            self._battery = float(msg.data)
            self._battery_simulated = False  # real battery data received

    # ======================================================================
    # State machine tick
    # ======================================================================

    def _state_machine_tick(self) -> None:
        """Called at state_publish_rate Hz; drives all state transitions."""
        with self._lock:
            pose = self._pose
            battery = self._battery
            state = self._state
            degeneracy = self._degeneracy_score
            local_st = self._local_status
            expl_st = self._exploration_status
            goal = self._mission_goal

        # Battery simulation (if no real topic received)
        self._maybe_drain_battery()

        # ----------------------------------------------------------------
        # Global safety checks (any state → EMERGENCY)
        # ----------------------------------------------------------------
        if state not in (MissionState.IDLE, MissionState.LANDING, MissionState.EMERGENCY):
            if degeneracy < self._degeneracy_thresh:
                self.get_logger().error(
                    f'MissionManager: SLAM degeneracy={degeneracy:.3f} < '
                    f'threshold {self._degeneracy_thresh} — EMERGENCY'
                )
                self._transition(MissionState.EMERGENCY)
                return

            if local_st == 'emergency_stop':
                self.get_logger().error(
                    'MissionManager: local_planner EMERGENCY_STOP — EMERGENCY'
                )
                self._transition(MissionState.EMERGENCY)
                return

        # ----------------------------------------------------------------
        # Battery critical → force land
        # ----------------------------------------------------------------
        with self._lock:
            bat = self._battery
        if bat < self._battery_land_thresh and state not in (
            MissionState.IDLE, MissionState.LANDING
        ):
            self.get_logger().error(
                f'MissionManager: battery critically low ({bat:.1f}%) — forcing LANDING'
            )
            self._transition(MissionState.LANDING)
            return

        # ----------------------------------------------------------------
        # Per-state logic
        # ----------------------------------------------------------------
        if state == MissionState.IDLE:
            self._handle_idle()
        elif state == MissionState.TAKEOFF:
            self._handle_takeoff(pose)
        elif state == MissionState.EXPLORING:
            self._handle_exploring(expl_st, bat, goal)
        elif state == MissionState.NAVIGATING:
            self._handle_navigating(pose, expl_st)
        elif state == MissionState.RETURNING:
            self._handle_returning(pose)
        elif state == MissionState.LANDING:
            self._handle_landing(pose)
        elif state == MissionState.EMERGENCY:
            self._handle_emergency()

        # Publish state
        self._publish_state()

    # ======================================================================
    # Per-state handlers
    # ======================================================================

    def _handle_idle(self) -> None:
        self._publish_info('Mission idle — send /mission/start to begin.')

    def _handle_takeoff(self, pose: Optional[np.ndarray]) -> None:
        if pose is None:
            self._publish_info('TAKEOFF: waiting for odometry...')
            return

        alt = float(pose[2])
        target_alt = self._explore_alt

        if alt < target_alt - 0.1:
            # Command upward velocity
            cmd = Twist()
            cmd.linear.z = self._takeoff_speed
            self._cmd_pub.publish(cmd)
            self._publish_info(f'TAKEOFF: ascending... {alt:.2f}/{target_alt:.2f} m')
        else:
            # Altitude reached
            cmd = Twist()  # zero velocity
            self._cmd_pub.publish(cmd)
            self.get_logger().info(
                f'MissionManager: takeoff complete at {alt:.2f}m → EXPLORING'
            )
            self._transition(MissionState.EXPLORING)

    def _handle_exploring(
        self,
        expl_st: str,
        battery: float,
        goal: Optional[np.ndarray],
    ) -> None:
        # Explicit goal arrived → switch to navigating
        if goal is not None:
            self._transition(MissionState.NAVIGATING)
            return

        # Battery low → return
        if battery < self._battery_return_thresh:
            self.get_logger().info(
                f'MissionManager: battery {battery:.1f}% < return threshold — RETURNING'
            )
            self._transition(MissionState.RETURNING)
            return

        # No more frontiers → require 15 consecutive ticks before giving up
        # (gives the map time to build during initial takeoff)
        if expl_st == 'no_frontiers':
            self._no_frontier_ticks += 1
            self._publish_info(
                f'EXPLORING — no_frontiers tick {self._no_frontier_ticks}/15'
            )
            if self._no_frontier_ticks >= 15:
                self.get_logger().info(
                    'MissionManager: no frontiers for 15 consecutive ticks — RETURNING'
                )
                self._transition(MissionState.RETURNING)
            return
        else:
            self._no_frontier_ticks = 0

        self._publish_info(f'EXPLORING — battery={battery:.1f}%  frontier={expl_st}')

    def _handle_navigating(
        self,
        pose: Optional[np.ndarray],
        expl_st: str,
    ) -> None:
        with self._lock:
            goal = self._mission_goal
            return_after = self._return_after_goal

        if goal is None or pose is None:
            self._publish_info('NAVIGATING: awaiting pose/goal data...')
            return

        dist = float(np.linalg.norm(goal - pose))
        self._publish_info(f'NAVIGATING → goal dist={dist:.2f}m')

        # Publish goal for path_planner / local_planner
        self._publish_goal(goal)

        if dist < self._goal_tol:
            self.get_logger().info(
                f'MissionManager: goal reached (dist={dist:.2f}m)'
            )
            with self._lock:
                self._mission_goal = None

            if return_after:
                self._transition(MissionState.RETURNING)
            else:
                self._transition(MissionState.EXPLORING)

    def _handle_returning(self, pose: Optional[np.ndarray]) -> None:
        if pose is None:
            self._publish_info('RETURNING: waiting for odometry...')
            return

        dist_home = float(np.linalg.norm(pose - self._home))
        self._publish_info(f'RETURNING — dist_home={dist_home:.2f}m')

        # Publish home as goal
        self._publish_goal(self._home)

        if dist_home < self._home_tol:
            self.get_logger().info('MissionManager: home reached → LANDING')
            self._transition(MissionState.LANDING)

    def _handle_landing(self, pose: Optional[np.ndarray]) -> None:
        if pose is None:
            return

        alt = float(pose[2])
        if alt > 0.1:
            cmd = Twist()
            cmd.linear.z = -self._landing_speed
            self._cmd_pub.publish(cmd)
            self._publish_info(f'LANDING: descending... {alt:.2f}m')
        else:
            # Landed
            cmd = Twist()
            self._cmd_pub.publish(cmd)
            self._publish_info('LANDING: touchdown complete.')
            self.get_logger().info('MissionManager: landing complete.')
            # No further transitions from LANDING

    def _handle_emergency(self) -> None:
        # Stop all motion immediately
        cmd = Twist()
        self._cmd_pub.publish(cmd)

        with self._lock:
            attempts = self._emergency_recovery_attempts
            score = self._degeneracy_score
            local_st = self._local_status

        self._publish_info(
            f'EMERGENCY — degeneracy={score:.3f}  local={local_st}  '
            f'recovery_attempts={attempts}'
        )

        # Recovery: if SLAM recovered and no collision, try returning
        if score >= self._degeneracy_thresh and local_st != 'emergency_stop':
            with self._lock:
                self._emergency_recovery_attempts += 1
            self.get_logger().info(
                'MissionManager: EMERGENCY recovery — transitioning to RETURNING'
            )
            self._transition(MissionState.RETURNING)

    # ======================================================================
    # Helpers
    # ======================================================================

    def _transition(self, target: MissionState) -> None:
        """Attempt a state transition, logging at INFO level."""
        with self._lock:
            current = self._state
            allowed = self._TRANSITIONS.get(current, [])

        if target not in allowed:
            self.get_logger().warn(
                f'MissionManager: invalid transition {current.value} → {target.value} — ignored.'
            )
            return

        self.get_logger().info(
            f'MissionManager: {current.value} → {target.value}'
        )
        with self._lock:
            self._state = target

    def _maybe_drain_battery(self) -> None:
        """Simulate battery drain when flying (no real battery topic)."""
        with self._lock:
            if not self._battery_simulated:
                return
            state = self._state

        now = time.monotonic()
        elapsed = now - self._last_battery_tick
        self._last_battery_tick = now

        flying_states = {
            MissionState.TAKEOFF,
            MissionState.EXPLORING,
            MissionState.NAVIGATING,
            MissionState.RETURNING,
            MissionState.LANDING,
        }
        if state in flying_states:
            with self._lock:
                self._battery = max(0.0, self._battery - self._battery_drain * elapsed * 100)

    def _publish_state(self) -> None:
        with self._lock:
            state = self._state
        msg = String()
        msg.data = state.value
        self._state_pub.publish(msg)

    def _publish_info(self, info: str) -> None:
        msg = String()
        msg.data = info
        self._info_pub.publish(msg)

    def _publish_goal(self, position: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
