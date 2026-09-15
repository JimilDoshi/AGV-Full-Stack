#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

PUBLISH_HZ = 50.0
# How long after the last key event a movement command stays active.
# The timer fires every 20 ms (50 Hz). OS key-repeat events are not
# guaranteed to arrive every tick, so a window that is too tight (e.g. 100 ms
# = 5 ticks) causes the active flag to expire between repeats and publishes a
# spurious zero-velocity tick while the key is still physically held — this
# is what produces jerky motion. 300 ms (15 ticks) gives reliable coverage
# across normal key-repeat rates without making the stop feel delayed, because
# the moment you physically release the key the next poll sees no key and
# publishes zero immediately.
KEY_TIMEOUT_S = 0.3


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('agv_keyboard_teleop')
        self.declare_parameter('linear_speed_mps', 0.25)
        self.declare_parameter('angular_speed_rps', 0.8)
        self.linear_speed = float(self.get_parameter('linear_speed_mps').value)
        self.angular_speed = float(self.get_parameter('angular_speed_rps').value)
        if self.linear_speed <= 0.0 or self.angular_speed <= 0.0:
            raise ValueError('speed parameters must be positive')

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_publisher = self.create_publisher(Bool, '/agv/arm_enable', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_command)
        self.armed = False
        self.last_key = ''
        self.last_key_time = 0.0
        self.get_logger().info('W/A/S/D: move, E: toggle arm, Q: quit')

    def read_key(self) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1).lower() if ready else ''

    def publish_command(self) -> None:
        key = self.read_key()
        now = time.monotonic()
        if key == 'e':
            self.armed = not self.armed
            self.get_logger().warn('ARMED' if self.armed else 'DISARMED')
            self.last_key = ''
        elif key == 'q':
            self.armed = False
            self.request_shutdown()
            return
        elif key in ('w', 'a', 's', 'd'):
            self.last_key = key
            self.last_key_time = now

        active = self.armed and (now - self.last_key_time <= KEY_TIMEOUT_S)
        command = Twist()
        if active:
            if self.last_key == 'w':
                command.linear.x = self.linear_speed
            elif self.last_key == 's':
                command.linear.x = -self.linear_speed
            elif self.last_key == 'a':
                command.angular.z = self.angular_speed
            elif self.last_key == 'd':
                command.angular.z = -self.angular_speed
        self.cmd_publisher.publish(command)

        arm_message = Bool()
        arm_message.data = self.armed
        self.arm_publisher.publish(arm_message)

    def request_shutdown(self) -> None:
        self.cmd_publisher.publish(Twist())
        arm_message = Bool()
        arm_message.data = False
        self.arm_publisher.publish(arm_message)
        rclpy.shutdown()


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError('keyboard teleop requires a terminal')

    terminal_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.armed = False
            node.publish_command()
            node.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
