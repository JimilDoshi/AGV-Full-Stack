#!/usr/bin/env bash
# =============================================================================
# AGV — Raspberry Pi CAN bridge startup script
# Run this script once per boot to bring up CAN and start the ROS 2 bridge.
# =============================================================================

set -e

# --------------------------------------------------------------------------
# Configuration — edit these to match your rover
# --------------------------------------------------------------------------
CAN_IFACE="can0"
CAN_BITRATE=500000
WHEEL_SEPARATION=0.60   # metres, left-wheel centre to right-wheel centre
MAX_WHEEL_SPEED=1.0     # m/s, maximum wheel speed
MODE=1                  # 1 = Jetson keyboard, 2 = Autoware autonomous
AGV_WS="${HOME}/agv_ws"
ROS_DISTRO="humble"
# --------------------------------------------------------------------------

echo "=== AGV CAN Bridge Startup ==="

# Bring up SocketCAN interface
echo "[1/3] Configuring ${CAN_IFACE} at ${CAN_BITRATE} bit/s..."
sudo ip link set "${CAN_IFACE}" down 2>/dev/null || true
sudo ip link set "${CAN_IFACE}" type can bitrate "${CAN_BITRATE}" restart-ms 100
sudo ip link set "${CAN_IFACE}" up
echo "      ${CAN_IFACE} is up."

# Source ROS 2
echo "[2/3] Sourcing ROS 2 ${ROS_DISTRO}..."
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${AGV_WS}/install/setup.bash"

# Launch the bridge node
echo "[3/3] Starting cmd_vel_can_bridge (mode=${MODE})..."
echo "      Press Ctrl+C to stop — will send 3 disabled frames before closing."
ros2 run agv_can_bridge cmd_vel_can_bridge --ros-args \
  -p can_interface:="${CAN_IFACE}" \
  -p wheel_separation_m:="${WHEEL_SEPARATION}" \
  -p max_wheel_speed_mps:="${MAX_WHEEL_SPEED}" \
  -p mode:="${MODE}"
