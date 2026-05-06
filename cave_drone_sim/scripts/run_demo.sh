#!/bin/bash
# =============================================================================
# Cave Drone Exploration Demo Runner
# =============================================================================
# Generates a cave, launches the full simulation, and starts the autonomous
# exploration mission automatically.
#
# Usage:
#   ./run_demo.sh [OPTIONS]
#
# Options:
#   --seed   SEED     Random seed for cave generation     (default: 42)
#   --nodes  N        Number of cave chambers              (default: 25)
#   --bounds BX BY BZ Cave bounding box in metres         (default: 80 80 25)
#   --slam   BACKEND  SLAM backend: simple|fastlio|rtabmap (default: simple)
#   --output DIR      Output directory for cave files      (default: /tmp/cave_demo)
#   --no-rviz         Disable RViz2
#   --help            Show this help message
#
# Example:
#   ./run_demo.sh --seed 42 --nodes 30 --slam rtabmap
#   ./run_demo.sh --seed 7  --bounds 60 60 20 --no-rviz
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
SEED=42
NODES=25
BOUNDS="80 80 25"
SLAM_BACKEND="simple"
OUTPUT_DIR="/tmp/cave_demo"
USE_RVIZ="true"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --seed)
            SEED="$2"; shift 2 ;;
        --nodes)
            NODES="$2"; shift 2 ;;
        --bounds)
            BOUNDS="$2 $3 $4"; shift 4 ;;
        --slam)
            SLAM_BACKEND="$2"
            if [[ "$SLAM_BACKEND" != "simple" && "$SLAM_BACKEND" != "fastlio" && "$SLAM_BACKEND" != "rtabmap" ]]; then
                echo "ERROR: --slam must be one of: simple, fastlio, rtabmap" >&2
                exit 1
            fi
            shift 2 ;;
        --output)
            OUTPUT_DIR="$2"; shift 2 ;;
        --no-rviz)
            USE_RVIZ="false"; shift ;;
        --help|-h)
            head -35 "${BASH_SOURCE[0]}" | tail -30
            exit 0 ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            echo "Run with --help for usage." >&2
            exit 1 ;;
    esac
done

WORLD_FILE="${OUTPUT_DIR}/cave_world.sdf"
GENERATE_SCRIPT="${SCRIPT_DIR}/generate_cave.py"

# ---------------------------------------------------------------------------
# Cleanup handler – kills background jobs on Ctrl+C
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "=== Shutting down Cave Drone Demo ==="
    # Kill all child processes in the process group
    jobs -p | xargs -r kill 2>/dev/null || true
    # Kill any ros2 launch processes we spawned
    if [[ -n "${LAUNCH_PID:-}" ]]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
    fi
    echo "=== Done. Goodbye! ==="
    exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { echo "ERROR: $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
log "=== Cave Drone Exploration Demo ==="
log "  Seed:         ${SEED}"
log "  Cave nodes:   ${NODES}"
log "  Cave bounds:  ${BOUNDS}"
log "  SLAM backend: ${SLAM_BACKEND}"
log "  Output dir:   ${OUTPUT_DIR}"
log "  RViz2:        ${USE_RVIZ}"
echo ""

# Check that generate_cave.py exists
if [[ ! -f "${GENERATE_SCRIPT}" ]]; then
    die "generate_cave.py not found at ${GENERATE_SCRIPT}"
fi

# Check that ROS 2 is sourced
if ! command -v ros2 &>/dev/null; then
    die "ros2 not found. Source your ROS 2 installation first:
  source /opt/ros/humble/setup.bash"
fi

# ---------------------------------------------------------------------------
# Step 1 – Generate the cave environment
# ---------------------------------------------------------------------------
log "=== Step 1/3: Generating cave environment ==="
mkdir -p "${OUTPUT_DIR}"

# Build bounds arguments (three separate values)
read -r BX BY BZ <<< "${BOUNDS}"

python3 "${GENERATE_SCRIPT}" \
    --seed   "${SEED}"   \
    --nodes  "${NODES}"  \
    --bounds "${BX}" "${BY}" "${BZ}" \
    --output "${OUTPUT_DIR}"

if [[ ! -f "${WORLD_FILE}" ]]; then
    die "Cave world file not created: ${WORLD_FILE}"
fi

log "Cave world file ready: ${WORLD_FILE}"
echo ""

# ---------------------------------------------------------------------------
# Step 2 – Launch full simulation
# ---------------------------------------------------------------------------
log "=== Step 2/3: Launching full simulation ==="
log "  (Gazebo at t=0s, SLAM at t=5s, Exploration at t=8s)"
log "  Press Ctrl+C to stop everything."
echo ""

ros2 launch cave_drone_sim full_demo.launch.py \
    world_file:="${WORLD_FILE}"        \
    cave_seed:="${SEED}"               \
    num_nodes:="${NODES}"              \
    slam_backend:="${SLAM_BACKEND}"    \
    use_rviz:="${USE_RVIZ}"            \
    &

LAUNCH_PID=$!
log "Launch PID: ${LAUNCH_PID}"

# ---------------------------------------------------------------------------
# Step 3 – Send mission start command after 10 seconds
# ---------------------------------------------------------------------------
log "=== Step 3/3: Waiting 10 s before sending /mission/start ==="
sleep 10

log "Sending mission start command to /mission/start ..."
ros2 topic pub --once /mission/start std_msgs/msg/String \
    "data: 'explore'" 2>/dev/null || {
    log "WARNING: Could not publish /mission/start (is the topic available?)"
    log "         You can start manually: ros2 topic pub --once /mission/start std_msgs/msg/String \"data: 'explore'\""
}

log "Mission start sent. Autonomous exploration underway."
log "Monitor status with:  python3 scripts/monitor_status.py"
echo ""

# ---------------------------------------------------------------------------
# Wait for the launch process to exit
# ---------------------------------------------------------------------------
wait "$LAUNCH_PID"
