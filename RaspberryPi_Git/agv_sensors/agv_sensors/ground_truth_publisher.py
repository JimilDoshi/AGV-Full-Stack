#!/usr/bin/env python3
"""
Dummy ground truth publisher for rf2o_laser_odometry.

rf2o (MAPIRlab build) subscribes to /base_pose_ground_truth before it
will start processing laser scans. On real hardware there is no simulator
providing this topic, so rf2o blocks forever with "waiting for laser scans".

This node publishes a single zero-pose Odometry message on
/base_pose_ground_truth with a latched (transient local) QoS so any
late-joining subscriber (rf2o) receives it immediately on startup.

It publishes once then keeps spinning so the latched message stays alive.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry


class GroundTruthPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_dummy')

        # Transient local (latched) so rf2o gets the message even if it
        # starts after this node.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pub = self.create_publisher(
            Odometry, '/base_pose_ground_truth', qos)

        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.pub.publish(msg)
        self.get_logger().info(
            'Published dummy /base_pose_ground_truth to unblock rf2o.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
