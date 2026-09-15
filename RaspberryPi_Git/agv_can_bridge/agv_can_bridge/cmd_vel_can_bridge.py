#!/usr/bin/env python3
import math
import time
from typing import Tuple

import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

CAN_CMD_ID = 0x101
CAN_MAGIC = 0xA5
CAN_MODE_JETSON = 1
CAN_MODE_AUTONOMOUS = 2
COMMAND_HZ = 50.0
COMMAND_TIMEOUT_S = 0.15
COMMAND_SCALE = 1000.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def encode_int16(value: int) -> Tuple[int, int]:
    value = max(-32768, min(value, 32767))
    unsigned = value & 0xFFFF
    return (unsigned >> 8, unsigned & 0xFF)


def encode_command(left: float, right: float, enable: bool,
                   mode: int, sequence: int) -> bytes:
    left_bytes = encode_int16(round(clamp(left, -1.0, 1.0) * COMMAND_SCALE))
    right_bytes = encode_int16(round(clamp(right, -1.0, 1.0) * COMMAND_SCALE))
    return bytes((
        left_bytes[0], left_bytes[1],
        right_bytes[0], right_bytes[1],
        1 if enable else 0,
        mode & 0xFF,
        sequence & 0xFF,
        CAN_MAGIC,
    ))


class CmdVelCanBridge(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_can_bridge')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('wheel_separation_m', 0.60)
        self.declare_parameter('max_wheel_speed_mps', 1.0)
        self.declare_parameter('mode', CAN_MODE_JETSON)

        interface = str(self.get_parameter('can_interface').value)
        bitrate = int(self.get_parameter('can_bitrate').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation_m').value)
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed_mps').value)
        self.mode = int(self.get_parameter('mode').value)

        if self.wheel_separation <= 0.0 or self.max_wheel_speed <= 0.0:
            raise ValueError('wheel separation and maximum speed must be positive')
        if self.mode not in (CAN_MODE_JETSON, CAN_MODE_AUTONOMOUS):
            raise ValueError('mode must be 1 (Jetson) or 2 (autonomous)')

        self.bus = can.Bus(interface='socketcan', channel=interface)
        self.left = 0.0
        self.right = 0.0
        self.armed = False
        self.last_cmd_time = 0.0
        self.sequence = 0
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.arm_subscription = self.create_subscription(
            Bool, '/agv/arm_enable', self.arm_callback, 10)
        self.timer = self.create_timer(1.0 / COMMAND_HZ, self.send_command)
        self.get_logger().info(
            f'CAN bridge on {interface} at {bitrate} bit/s; waiting for /cmd_vel')

    def cmd_vel_callback(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.left = 0.0
            self.right = 0.0
            self.last_cmd_time = 0.0
            self.get_logger().error('Rejected non-finite /cmd_vel command')
            return

        half_track = self.wheel_separation / 2.0
        left_mps = linear - angular * half_track
        right_mps = linear + angular * half_track
        scale = max(self.max_wheel_speed,
                    abs(left_mps), abs(right_mps))
        self.left = clamp(left_mps / scale, -1.0, 1.0)
        self.right = clamp(right_mps / scale, -1.0, 1.0)
        self.last_cmd_time = time.monotonic()

    def arm_callback(self, message: Bool) -> None:
        self.armed = bool(message.data)
        if not self.armed:
            self.left = 0.0
            self.right = 0.0
            # Clear the timestamp so a stale velocity from before disarm cannot
            # fire immediately if the operator re-arms quickly (fast E→E).
            self.last_cmd_time = 0.0

    def send_command(self) -> None:
        fresh = (time.monotonic() - self.last_cmd_time) <= COMMAND_TIMEOUT_S
        # Enable is driven by the arm state alone — NOT by freshness.
        # Freshness only controls whether the last velocity is used or zeroed.
        # Tying enable to freshness would latch a kill on the ESP32 the moment
        # a single /cmd_vel message is delayed > COMMAND_TIMEOUT_S on the
        # network, forcing the operator to fully re-arm to recover.
        enable = self.armed
        left = self.left if fresh else 0.0
        right = self.right if fresh else 0.0
        payload = encode_command(left, right, enable, self.mode, self.sequence)
        message = can.Message(arbitration_id=CAN_CMD_ID,
                              is_extended_id=False, data=payload)
        try:
            self.bus.send(message, timeout=0.01)
        except can.CanError as error:
            self.get_logger().error(f'CAN transmit failed: {error}')
        self.sequence = (self.sequence + 1) & 0xFF

    def destroy_node(self) -> bool:
        try:
            for _ in range(3):
                self.left = 0.0
                self.right = 0.0
                self.armed = False
                self.send_command()
        finally:
            self.bus.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelCanBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError, can.CanError) as error:
        if node is not None:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
