#!/bin/bash
# =============================================================================
# Cave Drone Teleop Helper
# =============================================================================
# Launches teleop_twist_keyboard remapped to /cmd_vel for manual drone control.
# Use during SLAM testing (slam_test.launch.py) to fly the drone manually.
#
# Usage:
#   bash scripts/teleop_helper.sh
#
# Controls (teleop_twist_keyboard defaults):
#   Moving:
#     u  i  o        ↖ ↑ ↗
#     j  k  l        ← ■ →
#     m  ,  .        ↙ ↓ ↘
#
#   Altitude (z):
#     t  – ascend
#     b  – descend
#
#   Speed:
#     q/z  – increase/decrease max speeds by 10%
#     w/x  – increase/decrease linear speed only
#     e/c  – increase/decrease angular speed only
#
#   k      – stop (all zeros)
#   CTRL-C – quit
# =============================================================================

set -euo pipefail

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║            Cave Drone Teleop Control                     ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  Moving around:                                          ║"
echo "║    u    i    o      ↖   ↑   ↗                           ║"
echo "║    j    k    l      ←  stop  →                          ║"
echo "║    m    ,    .      ↙   ↓   ↘                           ║"
echo "║                                                          ║"
echo "║  Altitude:   t (up)   b (down)                          ║"
echo "║                                                          ║"
echo "║  Speed:   q/z ±10%   w/x linear   e/c angular          ║"
echo "║                                                          ║"
echo "║  Publishing to: /cmd_vel                                 ║"
echo "║  Press CTRL-C to exit                                    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Check that ROS 2 is sourced
if ! command -v ros2 &>/dev/null; then
    echo "ERROR: ros2 not found. Source your ROS 2 installation first:" >&2
    echo "  source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

# Check the workspace is sourced (best-effort)
if ! ros2 pkg list 2>/dev/null | grep -q teleop_twist_keyboard; then
    echo "WARNING: teleop_twist_keyboard package not found in current environment." >&2
    echo "         Make sure ros-humble-teleop-twist-keyboard is installed:" >&2
    echo "           sudo apt-get install ros-humble-teleop-twist-keyboard" >&2
    echo "         or run:  bash install_deps.sh" >&2
    echo ""
fi

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel
