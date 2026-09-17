# AGV System Startup Guide

Two operating modes are supported:

| Mode | Use case |
|---|---|
| **Manual / Keyboard** | Teleoperation from Jetson keyboard |
| **Autonomous** | Nav2 + SLAM — rover navigates to goals on its own |

---

## Hardware overview — what runs where

| Device | Keyboard mode | Autonomous mode |
|---|---|---|
| **ESP32** | Motor FSM firmware (always) | Motor FSM firmware (always) |
| **Raspberry Pi** | `agv_can_bridge` (mode=1) | `agv_sensors` + `agv_can_bridge` (mode=2) |
| **Jetson** | `agv_keyboard` teleop node | `agv_navigation` (SLAM + Nav2 + YOLO) + RViz2 |

Start order always: **ESP32 first → Pi second → Jetson last.**

---

## Before first run — fill in your rover dimensions

Three files need your physical measurements. Fill these in once before building:

### 1. `Raspberry Pi/agv_sensors/launch/sensors.launch.py`
```python
LIDAR_X  = 0.0   # FILL_IN_1 — LiDAR forward offset from rover centre (m)
LIDAR_Z  = 0.0   # FILL_IN_2 — LiDAR height above ground (m)
CAMERA_Z = 0.0   # FILL_IN_3 — Camera height above ground (m)
```

### 2. `Jetson/agv_navigation/urdf/agv.urdf.xacro`
```xml
<xacro:property name="ROVER_WIDTH"  value="0.0"/>  <!-- FILL_IN_1 — wheel to wheel (m) -->
<xacro:property name="ROVER_LENGTH" value="0.0"/>  <!-- FILL_IN_2 — front to back (m) -->
<xacro:property name="ROVER_HEIGHT" value="0.0"/>  <!-- FILL_IN_3 — ground to chassis top (m) -->
```

### 3. `Jetson/agv_navigation/config/costmap.yaml`
```yaml
robot_radius: 0.0       # FILL_IN — ROVER_WIDTH / 2  (m)
inflation_radius: 0.0   # FILL_IN — robot_radius + 0.10  (m)
```
Set both occurrences (global_costmap and local_costmap).

---

## One-time install

### On Raspberry Pi
```bash
sudo apt update
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-v4l2-camera \
  ros-humble-image-transport \
  ros-humble-rf2o-laser-odometry \
  ros-humble-robot-state-publisher

# Copy packages and build
cd ~/agv_ws
colcon build --packages-select agv_can_bridge agv_sensors
source install/setup.bash
```

### On Jetson
```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher \
  ros-humble-xacro

pip install "numpy<2" ultralytics opencv-python-headless
# numpy<2 is required — ultralytics/cv2 are compiled against NumPy 1.x and
# will crash with "numpy.core.multiarray failed to import" on NumPy 2.x.

# Copy packages and build
cd ~/agv_ws
colcon build --packages-select agv_keyboard agv_navigation
source install/setup.bash
```

### On your Jetson (RViz2 — for autonomous mode)
```bash
sudo apt install -y ros-humble-rviz2
```

### ROS_DOMAIN_ID — set on BOTH Pi and Jetson
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

---
# MODE A — KEYBOARD / MANUAL TELEOPERATION
---

## Step 1 — Flash ESP32 (only after firmware update)

On your dev Mac:
```bash
cd "AGV Full Stack"
pio run --target upload
pio device monitor --baud 115200
# Confirm: ESP32 UGV v2.2.0   State : DISARMED
```

## Step 2 — Pi: bring up CAN + start bridge (mode=1)

```bash
# Bring up CAN interface (once per boot)
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

# Start bridge
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 run agv_can_bridge cmd_vel_can_bridge --ros-args \
  -p can_interface:=can0 \
  -p wheel_separation_m:=0.60 \
  -p max_wheel_speed_mps:=1.0 \
  -p mode:=1
```

## Step 3 — Jetson: start keyboard teleop

Open a real terminal (not SSH piped):
```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 run agv_keyboard keyboard_teleop --ros-args \
  -p linear_speed_mps:=0.25 \
  -p angular_speed_rps:=0.8
```

## Step 4 — Arm and drive

Set RC CH6 to **middle position** (Jetson mode).

| Key | Action |
|---|---|
| `E` | Toggle arm / disarm |
| `W` | Forward |
| `S` | Reverse |
| `A` | Rotate left |
| `D` | Rotate right |
| `Q` | Disarm and quit |

## Shutdown (keyboard mode)
1. Press `Q` on Jetson → disarms and quits
2. `Ctrl+C` on Pi bridge → sends 3 disabled frames then closes
3. Power off rover

---

---
# MODE B — AUTONOMOUS NAVIGATION (Nav2 + SLAM)
---

## Step 1 — Flash ESP32 (only after firmware update)

Same as keyboard mode. Confirm serial monitor shows `State : DISARMED`.

## Step 2 — Pi: bring up CAN + start full sensor bringup (mode=2)

```bash
# Bring up CAN interface (once per boot)
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

# Start everything: sensors + CAN bridge in autonomous mode
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 launch agv_sensors bringup.launch.py
```

This starts:
- RPLiDAR A1 → `/scan`
- Logitech camera → `/camera/image_raw/compressed`
- rf2o laser odometry → `/odom`  *(replaces wheel encoders)*
- Static TF transforms
- CAN bridge in **mode=2** (autonomous)

Verify sensors are publishing (from any machine on same network):
```bash
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom          # ~10 Hz
ros2 topic hz /camera/image_raw/compressed  # ~30 Hz
```

## Step 3 — Jetson: start navigation stack

```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

ros2 launch agv_navigation navigation.launch.py
```

This starts (with automatic delays):
- `robot_state_publisher` — URDF + TF tree (immediate)
- `slam_toolbox` — live SLAM, builds map as rover drives (immediate)
- `Nav2` — path planning + obstacle avoidance (after 3 s)
- `YOLO safety node` — person detection → auto-disarm (after 5 s)

Wait until you see in the terminal:
```
[slam_toolbox]: Message Filter dropping message: frame 'laser' ...
```
This means SLAM is receiving scans. Nav2 active message appears ~3 s later.

## Step 4 — Set RC transmitter to autonomous mode

Set RC **CH6 to the top position** (autonomous mode).

The ESP32 serial monitor will show:
```
Mode: AUTO
```

## Step 5 — Arm the rover

On the Jetson terminal or a separate keyboard node terminal:
```bash
# Publish arm_enable=True once to start the arm sequence
ros2 topic pub --once /agv/arm_enable std_msgs/Bool "data: true"
```

Or open the keyboard node in a separate terminal (it can co-exist with Nav2):
```bash
ros2 run agv_keyboard keyboard_teleop
# Press E to arm
```

The ESP32 will auto-arm once it receives `enable=1` frames.

## Step 6 — Open RViz2 and send goals

On the Jetson, open a new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash
rviz2 -d ~/agv_ws/src/agv_navigation/config/rviz.rviz
```

In RViz2:
1. You will see the **map being built** as the rover's LiDAR sweeps the room
2. Use the **"2D Goal Pose"** tool (toolbar) to click a destination on the map
3. Nav2 plans a path and the rover drives autonomously
4. The YOLO safety node will cut power if a person is detected — press `E` to re-arm

## Shutdown (autonomous mode)
1. Send zero goal or `Ctrl+C` in the navigation terminal
2. `Ctrl+C` on Pi bringup → sends 3 disabled CAN frames then closes
3. Power off rover

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `[CAN] TWAI start failed` | Transceiver not wired | Check GPIO4/5, VCC/GND, CANH/CANL |
| `CAN transmit failed` | `can0` not up | Re-run `ip link set can0 up` |
| Rover doesn't arm in CAN mode | Zero-hold waiting | Hold still 400 ms after arming |
| `/scan` not on Jetson | RPLiDAR not started or domain ID mismatch | Check `ros2 topic hz /scan` on Pi first |
| Map not building | rf2o not running or bad TF | Check `ros2 topic hz /odom` — must be >0 |
| Nav2 not accepting goals | SLAM map not yet published | Wait 10 s after launch, retry |
| YOLO stops rover unexpectedly | Low confidence threshold | Raise `confidence_threshold` to 0.70 in navigation.launch.py |
| Camera not publishing | Wrong `/dev/video*` device | Run `ls /dev/video*` on Pi, update sensors.launch.py |
| Relays clicking while holding key | Keyboard node not rebuilt after KEY_TIMEOUT fix | Rebuild `agv_keyboard` and restart |
| Motors jerk | RAMP_UP too high in ESP32 firmware | Set `RAMP_UP = 3` and reflash |

### Useful diagnostic commands
```bash
# Check all running nodes
ros2 node list

# Check all active topics
ros2 topic list

# Verify TF tree is complete (map→odom→base_link→laser)
ros2 run tf2_tools view_frames

# Watch Nav2 state
ros2 topic echo /navigate_to_pose/_action/status

# Watch YOLO detections
ros2 topic echo /agv/yolo_detections

# Live costmap debug
ros2 topic hz /global_costmap/costmap
```

---

## File structure reference

```
AGV Full Stack/
├── STARTUP.md                          ← this file
├── platformio.ini
├── Esp32/
│   └── ESP32_UGV_Controller.../       ← flash to ESP32 via PlatformIO
├── Raspberry Pi/
│   ├── agv_can_bridge/                ← cmd_vel → CAN bridge
│   └── agv_sensors/                   ← RPLiDAR + camera + rf2o + TF
│       └── launch/
│           ├── sensors.launch.py      ← sensors only
│           └── bringup.launch.py      ← sensors + CAN bridge (autonomous)
└── Jetson/
    ├── agv_keyboard/                  ← keyboard teleop node
    └── agv_navigation/                ← SLAM + Nav2 + YOLO
        ├── urdf/agv.urdf.xacro        ← rover geometry
        ├── config/
        │   ├── slam_toolbox.yaml      ← SLAM params
        │   ├── nav2_params.yaml       ← Nav2 / DWB params
        │   ├── costmap.yaml           ← obstacle inflation
        │   └── rviz.rviz              ← RViz2 layout
        └── launch/
            └── navigation.launch.py  ← full Jetson launch
```
