#!/usr/bin/env bash
# ============================================================================
# start_px4_cave.sh
# ============================================================================
# Convenience script to launch PX4 SITL with the x500_cave drone model
# inside a generated cave world.
#
# This script:
#   1. Ensures the x500_cave model is symlinked into PX4's models dir
#   2. Ensures cave_world.sdf is symlinked into PX4's worlds dir
#   3. Sets GZ_SIM_RESOURCE_PATH so Gazebo can find the cave mesh models
#   4. Sets PX4_GZ_MODEL_POSE to spawn the drone at the cave entry point
#   5. Runs PX4 SITL with gz_x500_cave airframe
#
# Usage:
#   ./scripts/start_px4_cave.sh                            # default cave_001
#   ./scripts/start_px4_cave.sh /path/to/cave_world.sdf    # custom cave
#   ./scripts/start_px4_cave.sh /path/to/cave_world.sdf "1,2,-5,0,0,0"
#
# Prerequisites:
#   - PX4-Autopilot built at ~/PX4-Autopilot (or set PX4_DIR)
#   - Gazebo Harmonic installed
# ============================================================================

set -euo pipefail

# ---- Configuration ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"

# Default cave world: prefer cave_flat (actively used), fall back to cave_001
if [ -f "${PROJECT_DIR}/output/cave_flat/cave_world.sdf" ]; then
    DEFAULT_CAVE_WORLD="${PROJECT_DIR}/output/cave_flat/cave_world.sdf"
else
    DEFAULT_CAVE_WORLD="${PROJECT_DIR}/output/cave_001/cave_world.sdf"
fi
CAVE_WORLD="${1:-${DEFAULT_CAVE_WORLD}}"

# Default spawn pose: fallback if auto-detect fails
DEFAULT_SPAWN_POSE="0,0,5.0,0,0,0"

# Auto-detect spawn pose from cave_world.sdf (already has correct Z-offset)
CAVE_OUTPUT_DIR_TEMP="$(dirname "${1:-${DEFAULT_CAVE_WORLD}}")"
if [ -z "${2:-}" ] && [ -f "${1:-${DEFAULT_CAVE_WORLD}}" ]; then
    DETECTED_POSE=$(python3 -c "
import re, sys
try:
    with open('${1:-${DEFAULT_CAVE_WORLD}}') as f:
        sdf = f.read()
    # Find the drone_spawn_point model pose
    m = re.search(r'<model name=\"drone_spawn_point\">.*?<pose>(.*?)</pose>', sdf, re.DOTALL)
    if m:
        parts = m.group(1).split()
        # Convert from 'x y z roll pitch yaw' to 'x,y,z,roll,pitch,yaw'
        print(','.join(parts))
    else:
        sys.exit(1)
except Exception:
    sys.exit(1)
" 2>/dev/null) && DEFAULT_SPAWN_POSE="${DETECTED_POSE}" && \
    echo "Auto-detected spawn pose from cave_world.sdf: ${DEFAULT_SPAWN_POSE}"
fi

SPAWN_POSE="${2:-${DEFAULT_SPAWN_POSE}}"

# Directories
CAVE_MODELS_DIR="${PROJECT_DIR}/models"
CAVE_OUTPUT_DIR="$(dirname "${CAVE_WORLD}")"
PX4_MODELS_DIR="${PX4_DIR}/Tools/simulation/gz/models"
PX4_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"

# ---- Validate ----
if [ ! -f "${CAVE_WORLD}" ]; then
    echo "ERROR: Cave world SDF not found: ${CAVE_WORLD}"
    echo "       Run cave generation first: python3 scripts/generate_cave.py"
    exit 1
fi

if [ ! -d "${PX4_DIR}" ]; then
    echo "ERROR: PX4-Autopilot not found at: ${PX4_DIR}"
    echo "       Set PX4_DIR environment variable to your PX4 installation."
    exit 1
fi

# ---- Symlink x500_cave model into PX4 models directory ----
if [ ! -e "${PX4_MODELS_DIR}/x500_cave" ]; then
    echo "Creating model symlink: ${PX4_MODELS_DIR}/x500_cave"
    ln -sf "${CAVE_MODELS_DIR}/x500_cave" "${PX4_MODELS_DIR}/x500_cave"
fi

# ---- Symlink cave_world.sdf into PX4 worlds directory ----
# PX4 resolves worlds from PX4_GZ_WORLDS = Tools/simulation/gz/worlds/
if [ ! -e "${PX4_WORLDS_DIR}/cave_world.sdf" ] || \
   [ "$(readlink -f "${PX4_WORLDS_DIR}/cave_world.sdf")" != "$(readlink -f "${CAVE_WORLD}")" ]; then
    echo "Creating world symlink: ${PX4_WORLDS_DIR}/cave_world.sdf"
    ln -sf "$(readlink -f "${CAVE_WORLD}")" "${PX4_WORLDS_DIR}/cave_world.sdf"
fi

# ---- Set environment variables ----
# Add cave output dir (contains cave_model_001/) to Gazebo resource path
# so the <include><uri> in cave_world.sdf can resolve the cave mesh.
# Also add our custom models dir.
export GZ_SIM_RESOURCE_PATH="${CAVE_OUTPUT_DIR}:${CAVE_MODELS_DIR}:${GZ_SIM_RESOURCE_PATH:-}"

# Tell PX4 where to spawn the drone
export PX4_GZ_MODEL_POSE="${SPAWN_POSE}"

echo "============================================================"
echo "  PX4 SITL — Cave Drone Simulation"
echo "============================================================"
echo "  Cave world : ${CAVE_WORLD}"
echo "  Spawn pose : ${SPAWN_POSE}"
echo "  Model      : x500_cave"
echo "  PX4 dir    : ${PX4_DIR}"
echo "  GZ_SIM_RESOURCE_PATH includes:"
echo "    - ${CAVE_OUTPUT_DIR}"
echo "    - ${CAVE_MODELS_DIR}"
echo "============================================================"
echo ""

# ---- Launch PX4 SITL ----
cd "${PX4_DIR}"
make px4_sitl gz_x500_cave
