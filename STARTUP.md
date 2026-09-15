# AGV System Startup Guide

Complete step-by-step procedure to bring the rover online from cold boot.

---

## Overview — what runs where

| Device | What runs | Role |
|---|---|---|
| **ESP32** | `ESP32_UGV_Controller_from_Autoware` firmware | Real-time motor control, safety FSM |
| **Raspberry Pi** | `agv_can_bridge` ROS 2 node | Translates `/cmd_vel` → CAN frames |
| **Jetson** | `agv_keyboard` ROS 2 node | Publishes `/cmd_vel` from keyboard |

Start order: **ESP32 first → Pi second → Jetson last.**

---

## Step 1 — Flash the ESP32 (only needed after firmware update)

On your development machine, open the project in PlatformIO and upload:

```bash
cd "AGV Full Stack"
pio run --target upload
```

Or use the PlatformIO IDE **Upload** button. After upload the ESP32 boots automatically and prints its state over USB serial at 115200 baud.

To monitor:
```bash
pio device monitor --baud 115200
```

You should see lines like `[CAN] TWAI started at 500 kbit/s` and `State : DISARMED`.

---

## Step 2 — Raspberry Pi setup

SSH into the Pi or open a terminal on it.

### 2a — Bring up the CAN interface

Run this once per boot (or add it to `/etc/rc.local` to automate):

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

Verify it came up:

```bash
ip link show can0
# should show: UP RUNNING
```

### 2b — Build the ROS 2 workspace (only needed once or after code changes)

```bash
cd ~/agv_ws
colcon build --packages-select agv_can_bridge
```

### 2c — Source and run the bridge node

```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 run agv_can_bridge cmd_vel_can_bridge --ros-args \
  -p can_interface:=can0 \
  -p wheel_separation_m:=0.60 \
  -p max_wheel_speed_mps:=1.0 \
  -p mode:=1
```

> **Parameters to tune for your rover:**
> - `wheel_separation_m` — measure left-wheel centre to right-wheel centre in metres
> - `max_wheel_speed_mps` — maximum linear speed of one wheel in m/s
> - `mode` — `1` for Jetson keyboard, `2` for Autoware autonomous

The terminal will show: `CAN bridge on can0 at 500000 bit/s; waiting for /cmd_vel`

The node is now sending disabled CAN frames to the ESP32 at 50 Hz, keeping it alive.

---

## Step 3 — Jetson setup

Open a **new terminal** on the Jetson (must be a real terminal, not piped — the node uses raw keyboard input).

### 3a — Build the ROS 2 workspace (only needed once or after code changes)

```bash
cd ~/agv_ws
colcon build --packages-select agv_keyboard
```

### 3b — Source and run the keyboard teleop node

```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 run agv_keyboard keyboard_teleop --ros-args \
  -p linear_speed_mps:=0.25 \
  -p angular_speed_rps:=0.8
```

> **Parameters:**
> - `linear_speed_mps` — top forward/reverse speed in m/s (default 0.25)
> - `angular_speed_rps` — top rotation rate in rad/s (default 0.8)

---

## Step 4 — Arm and drive

Make sure the RC transmitter is on and **CH6 is set to the Jetson position** (middle).

In the Jetson terminal:

| Key | Action |
|---|---|
| `E` | **Toggle arm / disarm** — press once to arm |
| `W` | Drive forward |
| `S` | Drive reverse |
| `A` | Rotate left (tank turn) |
| `D` | Rotate right (tank turn) |
| `E` again | Disarm |
| `Q` | Disarm and quit |

After pressing `E` you will see `ARMED` in the terminal. The ESP32 will auto-arm over CAN once it sees zero-velocity commands for ~400 ms — this is normal, it is a safety hold.

Releasing a movement key immediately sends zero velocity, and the motors cut instantly.

---

## Shutdown procedure

1. Press `Q` in the Jetson terminal → node disarms and quits
2. `Ctrl+C` the Pi bridge node → it sends 3 disabled frames then closes CAN
3. Power off the rover

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| ESP32 shows `[CAN] TWAI start failed` | CAN transceiver not wired or unpowered | Check GPIO4/5 and transceiver VCC/GND |
| Pi node prints `CAN transmit failed` | `can0` not up | Re-run the `ip link set can0 up` commands |
| Rover does not move after arming | ESP32 zero-hold rearm waiting | Hold still for ~400 ms after arming, then move |
| Motors jerk at start | `RAMP_UP` too high | Already fixed in v2.3.0 — reflash the ESP32 |
| RC signal lost warning | FlySky receiver lost signal | Check transmitter battery and CH6 position |
| `/cmd_vel` not received on Pi | DDS domain mismatch | Ensure both Jetson and Pi are on the same `ROS_DOMAIN_ID` |

### Check ROS 2 topics are flowing

From any machine on the same network:
```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel        # should show ~50 Hz
ros2 topic echo /agv/arm_enable
```

---

## ROS_DOMAIN_ID (important for multi-machine setup)

Both the Jetson and Pi must use the same domain ID or their nodes won't see each other. Set this in `~/.bashrc` on **both** machines:

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

Use any number 0–101; just keep it the same on both devices.

---

## Quick-start scripts

See [`Raspberry Pi/setup/start_bridge.sh`](Raspberry%20Pi/setup/start_bridge.sh) and [`Jetson/setup/start_teleop.sh`](Jetson/setup/start_teleop.sh) for one-command startup on each device.
