# AGV keyboard teleoperation

ROS 2 Humble keyboard node for the Jetson. It publishes `/cmd_vel` and `/agv/arm_enable` for the Raspberry Pi CAN bridge.

Controls:

- `W`: forward
- `S`: reverse
- `A`: rotate left
- `D`: rotate right
- `E`: toggle arm/disarm
- `Q`: disarm and quit

Run from a terminal because the node uses raw keyboard input:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agv_keyboard keyboard_teleop --ros-args \
  -p linear_speed_mps:=0.25 \
  -p angular_speed_rps:=0.8
```

The node publishes commands at 50 Hz. A held key stays active for 300 ms after the last key-repeat event; releasing the key causes zero velocity on the next tick. Arming is separate from movement: press `E` to arm, then hold a movement key. Press `E` again or `Q` to disarm.
