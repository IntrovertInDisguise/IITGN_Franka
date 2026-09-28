import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

import numpy as np
from collections import deque

BUFFER_SIZE = 170
ENERGY_THRESHOLD = 1.0

RECORD_SAMPLES = 150
COOLDOWN_SAMPLES = 300

SEARCHING = 0
RECORDING = 1
COOLDOWN = 2
    
class EnergyGateNode(Node):

    def __init__(self):
        super().__init__('enery_gate_node')

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/clean_ft_values',
            self.sensor_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/tap_window',
            10
        )


        self.raw_buffer = deque(maxlen=BUFFER_SIZE)
        self.diff_buffer = deque(maxlen=BUFFER_SIZE)

        self.prev_force = 0.0

        self.state = SEARCHING
        self.record_counter = 0
        self.cooldown_counter = 0
        self.trigger_index = None

    def sensor_callback(self, msg):

        raw = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

        if self.prev_force is None:
            self.prev_force = raw
            return
        
        delta = raw - self.prev_force
        self.prev_force = raw

        self.raw_buffer.append(raw)
        self.diff_buffer.append(delta)

        E = delta[0]**2 + delta[1]**2 + delta[2]**2

        #E = self.energy(delta)
        self.get_logger().info(f"Energy: {E}")

        #---State Machine---

        if self.state == SEARCHING:

            if E > ENERGY_THRESHOLD and len(self.diff_buffer) == BUFFER_SIZE:

                self.trigger_index = len(self.raw_buffer) - 1
                self.record_counter = RECORD_SAMPLES
                self.state = RECORDING

                self.get_logger().info("Impact detected")

        elif self.state == RECORDING:

            self.record_counter -= 1
            if self.record_counter == 0:

                window = np.array(self.diff_buffer)
                self.publish_window(window)
                self.calculate_surface_normal()

                self.cooldown_counter = COOLDOWN_SAMPLES
                self.state = COOLDOWN

        elif self.state == COOLDOWN:

            self.cooldown_counter = 1
            if self.cooldown_counter == 0:
                self.state = SEARCHING

    def publish_window(self, window):
        msg = Float32MultiArray()
        msg.data = window.flatten().tolist()

        self.publisher.publish(msg)

        self.get_logger().info("Tap window published")

    def calculate_surface_normal(self):

        if self.trigger_index is None:
            return
        
        raw_force = self.raw_buffer[self.trigger_index][:3]

        norm = np.linalg.norm(raw_force)

        if norm == 0:
            return
        
        normal = raw_force/norm

        self.get_logger().info(
            f"Surface normal: {normal}"
        )

def main(args=None):

    rclpy.init(args=args)

    node = EnergyGateNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

