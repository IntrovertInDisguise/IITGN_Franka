#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np


class CleanFTNode(Node):

    def __init__(self):
        super().__init__('clean_ft_node')

        # Parameters
        self.sample_size = 1000
        self.sample_count = 0
        self.offset_computed = False

        # Storage for offset estimation
        self.force_samples = []
        self.torque_samples = []

        # Offset values
        self.force_offset = np.zeros(3)
        self.torque_offset = np.zeros(3)

        # Subscriber
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/NS_1/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame',
            self.wrench_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(
            WrenchStamped,
            '/clean_ft_values',
            10)

        self.get_logger().info("Clean FT node started. Collecting samples for offset estimation...")

    def wrench_callback(self, msg):

        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

        # Collect samples for offset calculation
        if not self.offset_computed:
            self.force_samples.append(force)
            self.torque_samples.append(torque)
            self.sample_count += 1

            if self.sample_count >= self.sample_size:
                self.force_offset = np.mean(self.force_samples, axis=0)
                self.torque_offset = np.mean(self.torque_samples, axis=0)
                self.offset_computed = True

                self.get_logger().info("Offset computed!")
                self.get_logger().info(f"Force offset: {self.force_offset}")
                self.get_logger().info(f"Torque offset: {self.torque_offset}")

            return

        # Remove offset
        clean_force = force - self.force_offset
        clean_torque = torque - self.torque_offset

        # Create new message
        clean_msg = WrenchStamped()
        clean_msg.header = msg.header
        clean_msg.wrench.force.x = clean_force[0]
        clean_msg.wrench.force.y = clean_force[1]
        clean_msg.wrench.force.z = clean_force[2]
        clean_msg.wrench.torque.x = clean_torque[0]
        clean_msg.wrench.torque.y = clean_torque[1]
        clean_msg.wrench.torque.z = clean_torque[2]

        # Publish cleaned data
        self.publisher.publish(clean_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CleanFTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
