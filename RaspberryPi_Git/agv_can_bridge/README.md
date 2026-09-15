# AGV CAN bridge

ROS 2 Humble node for sending `/cmd_vel` to the ESP32 built-in TWAI controller through a Raspberry Pi SocketCAN interface.

## ESP32 CAN protocol

- CAN bitrate: 500000 bit/s
- Standard frame ID: `0x101`
- Payload: left int16, right int16, enable, mode, sequence, `0xA5`
- Left/right int16 values are normalized from `-1000` to `1000`
- Mode `1` is Jetson; mode `2` is autonomous

## Install

```bash
sudo apt update
sudo apt install python3-can iproute2 ros-humble-rclpy ros-humble-geometry-msgs
cd ~/agv_ws
colcon build --packages-select agv_can_bridge
source install/setup.bash
```

Configure the CAN interface before starting the node. Substitute the real device name if the HAT exposes one other than `can0`:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

Run the bridge:

```bash
ros2 run agv_can_bridge cmd_vel_can_bridge --ros-args \
  -p can_interface:=can0 \
  -p wheel_separation_m:=0.60 \
  -p max_wheel_speed_mps:=1.0 \
  -p mode:=1
```

The wheel separation and maximum wheel speed are placeholders and must be measured for the rover. The node starts disabled and only enables motion after receiving a fresh nonzero `/cmd_vel` message. It sends disabled frames when `/cmd_vel` is stale or the node shuts down.
