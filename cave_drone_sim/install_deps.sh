#!/bin/bash
# =============================================================================
# install_deps.sh
# =============================================================================
# Install all system and Python dependencies for cave_drone_sim.
# Run once after cloning the repository (or after adding new packages).
#
# Usage:
#   bash install_deps.sh
#
# What this script does:
#   1. Install Python packages from requirements.txt
#   2. Install ROS 2 Humble binary packages (Gazebo bridge, nav2, rviz2, etc.)
#   3. Print guidance for optional FAST-LIO2 / RTAB-Map setup
#   4. Build the colcon workspace
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# ---------------------------------------------------------------------------
# Colour helpers
# ---------------------------------------------------------------------------
BOLD="\033[1m"
GREEN="\033[32m"
YELLOW="\033[33m"
RED="\033[31m"
RESET="\033[0m"

log()   { echo -e "${BOLD}[install_deps]${RESET} $*"; }
ok()    { echo -e "${GREEN}✔${RESET} $*"; }
warn()  { echo -e "${YELLOW}⚠${RESET}  $*"; }
error() { echo -e "${RED}✘${RESET}  $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Detect ROS distro
# ---------------------------------------------------------------------------
ROS_DISTRO="${ROS_DISTRO:-humble}"
log "ROS distro: ${ROS_DISTRO}"

if ! command -v ros2 &>/dev/null; then
    warn "ros2 not found in PATH.  Source your ROS 2 installation first:"
    warn "  source /opt/ros/${ROS_DISTRO}/setup.bash"
fi

# ---------------------------------------------------------------------------
# 1. Python dependencies
# ---------------------------------------------------------------------------
log "=== Installing Python dependencies ==="

if ! command -v pip3 &>/dev/null; then
    error "pip3 not found. Install python3-pip first: sudo apt-get install python3-pip"
fi

pip3 install --upgrade pip
pip3 install -r requirements.txt
ok "Python dependencies installed."

# ---------------------------------------------------------------------------
# 2. ROS 2 binary packages
# ---------------------------------------------------------------------------
log "=== Installing ROS 2 dependencies ==="

sudo apt-get update -qq

sudo apt-get install -y \
    "ros-${ROS_DISTRO}-gazebo-ros-pkgs"        \
    "ros-${ROS_DISTRO}-ros-gz-bridge"           \
    "ros-${ROS_DISTRO}-ros-gz-sim"              \
    "ros-${ROS_DISTRO}-robot-state-publisher"   \
    "ros-${ROS_DISTRO}-tf2-ros"                 \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs"       \
    "ros-${ROS_DISTRO}-nav2-msgs"               \
    "ros-${ROS_DISTRO}-visualization-msgs"      \
    "ros-${ROS_DISTRO}-rviz2"                   \
    "ros-${ROS_DISTRO}-teleop-twist-keyboard"   \
    "ros-${ROS_DISTRO}-xacro"

ok "ROS 2 binary packages installed."

# ---------------------------------------------------------------------------
# 3. Optional backends
# ---------------------------------------------------------------------------
log "=== Optional SLAM backends ==="

echo ""
echo -e "${BOLD}FAST-LIO2 (slam_backend:=fastlio)${RESET}"
echo "  FAST-LIO2 must be built from source.  Follow the instructions at:"
echo "  https://github.com/hku-mars/FAST_LIO"
echo "  Then rebuild this workspace with colcon."
echo ""
echo -e "${BOLD}RTAB-Map (slam_backend:=rtabmap)${RESET}"
echo "  Install the binary package:"
echo "    sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros"
echo ""

# Auto-install rtabmap if requested
if [[ "${INSTALL_RTABMAP:-0}" == "1" ]]; then
    log "Installing RTAB-Map (INSTALL_RTABMAP=1)..."
    sudo apt-get install -y "ros-${ROS_DISTRO}-rtabmap-ros"
    ok "RTAB-Map installed."
fi

# ---------------------------------------------------------------------------
# 4. Make scripts executable
# ---------------------------------------------------------------------------
log "=== Setting script permissions ==="
chmod +x scripts/run_demo.sh
chmod +x scripts/teleop_helper.sh
chmod +x scripts/monitor_status.py
chmod +x scripts/generate_cave.py
ok "Script permissions set."

# ---------------------------------------------------------------------------
# 5. Build workspace
# ---------------------------------------------------------------------------
log "=== Building colcon workspace ==="

if ! command -v colcon &>/dev/null; then
    warn "colcon not found. Installing..."
    pip3 install colcon-common-extensions
fi

colcon build --symlink-install

ok "Workspace built successfully."

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo -e "${BOLD}${GREEN}=== Installation complete! ===${RESET}"
echo ""
echo "Next steps:"
echo "  1. Source the workspace:"
echo "       source install/setup.bash"
echo ""
echo "  2. Run the full demo:"
echo "       bash scripts/run_demo.sh --seed 42 --nodes 25"
echo ""
echo "  3. Or just test SLAM:"
echo "       ros2 launch cave_drone_sim slam_test.launch.py"
echo "       bash scripts/teleop_helper.sh   # in another terminal"
echo ""
