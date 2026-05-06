#!/usr/bin/env python3
"""
monitor_status.py
==================
Real-time status monitor for the cave drone simulation.

Subscribes to key topics and prints a formatted dashboard every second.

Dashboard columns
-----------------
  Mission state      – current state machine state from /mission/state
  Battery %          – charge level from /drone/battery
  SLAM degeneracy    – degeneracy score from /slam/degeneracy_score
  Frontier count     – number of active frontiers from /exploration/frontiers
  Drone position     – XYZ position from /slam/odom
  Path length        – cumulative distance explored (m)

Usage
-----
  # Run while a simulation is active:
  python3 scripts/monitor_status.py

  # Show raw topic values without the formatted dashboard:
  python3 scripts/monitor_status.py --raw

  # Change update rate:
  python3 scripts/monitor_status.py --rate 2.0

  # Show this help:
  python3 scripts/monitor_status.py --help
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from typing import Optional

# ---------------------------------------------------------------------------
# Graceful import of rclpy – show helpful message if not found
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
except ImportError:
    print(
        "ERROR: rclpy not found.\n"
        "  Source your ROS 2 workspace:  source /opt/ros/humble/setup.bash\n"
        "  Then source the build:        source install/setup.bash",
        file=sys.stderr,
    )
    sys.exit(1)

# ROS message types – imported lazily so the script still parses without ROS
try:
    from std_msgs.msg import String, Float32, Int32
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from visualization_msgs.msg import MarkerArray
except ImportError as exc:
    print(f"WARNING: Some message types not available: {exc}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Terminal helpers
# ---------------------------------------------------------------------------

RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
WHITE  = "\033[37m"
DIM    = "\033[2m"

CLEAR_SCREEN = "\033[H\033[J"


def colored(text: str, color: str) -> str:
    return f"{color}{text}{RESET}"


def battery_color(pct: float) -> str:
    if pct > 50:
        return GREEN
    elif pct > 20:
        return YELLOW
    return RED


def degeneracy_color(score: float) -> str:
    """Low score = good geometry; high score = degenerate environment."""
    if score < 0.5:
        return GREEN
    elif score < 0.8:
        return YELLOW
    return RED


# ---------------------------------------------------------------------------
# Monitor node
# ---------------------------------------------------------------------------

class DroneStatusMonitor(Node):
    """ROS 2 node that subscribes to simulation topics and holds latest values."""

    def __init__(self, rate_hz: float = 1.0) -> None:
        super().__init__('drone_status_monitor')
        self.rate_hz = rate_hz

        # --- State ---
        self.mission_state: str = "UNKNOWN"
        self.battery_pct: float = -1.0        # –1 = no data
        self.degeneracy_score: float = -1.0
        self.frontier_count: int = -1
        self.position: Optional[tuple] = None  # (x, y, z)
        self.path_length_m: float = 0.0
        self._last_pos: Optional[tuple] = None

        # --- QoS ---
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscriptions ---
        self.create_subscription(
            String, '/mission/state',
            self._cb_mission_state, reliable_qos,
        )
        self.create_subscription(
            BatteryState, '/drone/battery',
            self._cb_battery, best_effort_qos,
        )
        self.create_subscription(
            Float32, '/slam/degeneracy_score',
            self._cb_degeneracy, best_effort_qos,
        )
        self.create_subscription(
            MarkerArray, '/exploration/frontiers',
            self._cb_frontiers, best_effort_qos,
        )
        self.create_subscription(
            Odometry, '/slam/odom',
            self._cb_odom, best_effort_qos,
        )
        # Also accept Int32 frontier count directly
        self.create_subscription(
            Int32, '/exploration/frontier_count',
            self._cb_frontier_count, best_effort_qos,
        )

        self.get_logger().info('DroneStatusMonitor ready.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_mission_state(self, msg: String) -> None:
        self.mission_state = msg.data

    def _cb_battery(self, msg: BatteryState) -> None:
        self.battery_pct = msg.percentage * 100.0

    def _cb_degeneracy(self, msg: Float32) -> None:
        self.degeneracy_score = msg.data

    def _cb_frontiers(self, msg: MarkerArray) -> None:
        self.frontier_count = len(msg.markers)

    def _cb_frontier_count(self, msg: Int32) -> None:
        self.frontier_count = msg.data

    def _cb_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)
        self.position = pos

        if self._last_pos is not None:
            dx = pos[0] - self._last_pos[0]
            dy = pos[1] - self._last_pos[1]
            dz = pos[2] - self._last_pos[2]
            self.path_length_m += math.sqrt(dx*dx + dy*dy + dz*dz)

        self._last_pos = pos


# ---------------------------------------------------------------------------
# Dashboard renderer
# ---------------------------------------------------------------------------

def _bar(value: float, total: float, width: int = 20, color: str = GREEN) -> str:
    """Render a simple ASCII progress bar."""
    filled = int(min(max(value / total, 0.0), 1.0) * width)
    bar = "█" * filled + "░" * (width - filled)
    return f"{color}{bar}{RESET}"


def render_dashboard(node: DroneStatusMonitor, elapsed_s: float) -> str:
    lines = []

    w = 62  # dashboard width

    lines.append(colored("═" * w, CYAN))
    lines.append(colored(
        f"{'  Cave Drone Simulation Monitor':^{w}}", BOLD + CYAN
    ))
    lines.append(colored("═" * w, CYAN))

    # Uptime
    mins, secs = divmod(int(elapsed_s), 60)
    hrs,  mins = divmod(mins, 60)
    lines.append(f"  {colored('Uptime', DIM)}  {hrs:02d}:{mins:02d}:{secs:02d}   "
                 f"{colored('(Ctrl-C to exit)', DIM)}")
    lines.append(colored("─" * w, DIM))

    # Mission state
    state_color = {
        'IDLE': WHITE, 'EXPLORING': GREEN, 'NAVIGATING': CYAN,
        'RETURNING': YELLOW, 'EMERGENCY': RED, 'COMPLETE': GREEN + BOLD,
    }.get(node.mission_state.upper(), WHITE)
    lines.append(f"  {colored('Mission State', BOLD)}   {colored(node.mission_state, state_color)}")

    # Battery
    if node.battery_pct >= 0:
        bc = battery_color(node.battery_pct)
        bar = _bar(node.battery_pct, 100, width=20, color=bc)
        lines.append(
            f"  {colored('Battery', BOLD)}        {bar}  {colored(f'{node.battery_pct:.1f}%', bc)}"
        )
    else:
        lines.append(f"  {colored('Battery', BOLD)}        {colored('No data', DIM)}")

    # SLAM degeneracy
    if node.degeneracy_score >= 0:
        dc = degeneracy_color(node.degeneracy_score)
        deg_bar = _bar(node.degeneracy_score, 1.0, width=20, color=dc)
        deg_label = "good" if node.degeneracy_score < 0.5 else ("marginal" if node.degeneracy_score < 0.8 else "DEGENERATE")
        lines.append(
            f"  {colored('SLAM Degeneracy', BOLD)} {deg_bar}  {colored(f'{node.degeneracy_score:.3f} ({deg_label})', dc)}"
        )
    else:
        lines.append(f"  {colored('SLAM Degeneracy', BOLD)} {colored('No data', DIM)}")

    # Frontier count
    if node.frontier_count >= 0:
        fc_color = GREEN if node.frontier_count > 0 else YELLOW
        lines.append(
            f"  {colored('Frontiers', BOLD)}      {colored(str(node.frontier_count), fc_color)} active"
        )
    else:
        lines.append(f"  {colored('Frontiers', BOLD)}      {colored('No data', DIM)}")

    lines.append(colored("─" * w, DIM))

    # Drone position
    if node.position is not None:
        x, y, z = node.position
        lines.append(
            f"  {colored('Position', BOLD)}  "
            f"X={colored(f'{x:+8.2f}', WHITE)}  "
            f"Y={colored(f'{y:+8.2f}', WHITE)}  "
            f"Z={colored(f'{z:+8.2f}', WHITE)}  m"
        )
    else:
        lines.append(f"  {colored('Position', BOLD)}  {colored('No data', DIM)}")

    # Path length
    lines.append(
        f"  {colored('Path explored', BOLD)}  {colored(f'{node.path_length_m:.1f} m', CYAN)}"
    )

    lines.append(colored("═" * w, CYAN))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Display loop (runs in main thread, ROS spins in background)
# ---------------------------------------------------------------------------

def display_loop(node: DroneStatusMonitor, rate_hz: float, raw: bool) -> None:
    start = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - start
            if raw:
                # Print raw values, one per line, CSV-like
                pos_str = (
                    f"{node.position[0]:.3f},{node.position[1]:.3f},{node.position[2]:.3f}"
                    if node.position else "nan,nan,nan"
                )
                print(
                    f"{elapsed:.1f},"
                    f"{node.mission_state},"
                    f"{node.battery_pct:.1f},"
                    f"{node.degeneracy_score:.3f},"
                    f"{node.frontier_count},"
                    f"{pos_str},"
                    f"{node.path_length_m:.1f}"
                )
                sys.stdout.flush()
            else:
                print(CLEAR_SCREEN + render_dashboard(node, elapsed))
                sys.stdout.flush()

            time.sleep(1.0 / rate_hz)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='monitor_status.py',
        description='Real-time dashboard for the cave drone simulation.',
    )
    p.add_argument(
        '--rate', '-r',
        type=float, default=1.0, metavar='HZ',
        help='Dashboard refresh rate in Hz (default: 1.0)',
    )
    p.add_argument(
        '--raw',
        action='store_true',
        help='Print CSV rows instead of the formatted dashboard',
    )
    return p


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = DroneStatusMonitor(rate_hz=args.rate)

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if args.raw:
        print("elapsed_s,mission_state,battery_pct,degeneracy_score,"
              "frontier_count,pos_x,pos_y,pos_z,path_length_m")

    try:
        display_loop(node, rate_hz=args.rate, raw=args.raw)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
