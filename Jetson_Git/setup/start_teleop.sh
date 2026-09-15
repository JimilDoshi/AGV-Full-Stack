#!/usr/bin/env bash
# =============================================================================
# AGV — Jetson keyboard teleop startup script
# Must be run in a real terminal (not piped) — uses raw keyboard input.
# =============================================================================

set -e

# --------------------------------------------------------------------------
# Configuration — edit these to match your rover
# --------------------------------------------------------------------------
LINEAR_SPEED=0.25       # m/s forward / reverse speed
ANGULAR_SPEED=0.8       # rad/s rotation speed
AGV_WS="${HOME}/agv_ws"
ROS_DISTRO="humble"
# --------------------------------------------------------------------------

echo "=== AGV Keyboard Teleop Startup ==="

# Guard: must be a real terminal
if [ ! -t 0 ]; then
  echo "ERROR: This script must be run in an interactive terminal."
  exit 1
fi

# Source ROS 2
echo "[1/2] Sourcing ROS 2 ${ROS_DISTRO}..."
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${AGV_WS}/install/setup.bash"

# Launch the teleop node
echo "[2/2] Starting keyboard_teleop..."
echo "      Controls: W/A/S/D = move  |  E = arm/disarm  |  Q = quit"
echo ""
ros2 run agv_keyboard keyboard_teleop --ros-args \
  -p linear_speed_mps:="${LINEAR_SPEED}" \
  -p angular_speed_rps:="${ANGULAR_SPEED}"
