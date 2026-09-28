import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from collections import deque
import numpy as np


class PeakTorqueDetector(Node):

    def __init__(self):

        super().__init__('peak_torque_detector')

        # ======================
        # PARAMETERS
        # ======================

        self.force_threshold = 10.0
        self.torque_threshold = 0.01

        # smoothing window
        self.window_size = 10

        self.force_buffer = deque(maxlen=self.window_size)

        # ======================
        # STATE MACHINE
        # ======================

        self.state = "waiting"

        self.max_force = -np.inf
        self.prev_force = None

        self.torque_values = []

        # ======================
        # SUBSCRIBER
        # ======================

        self.sub = self.create_subscription(
            WrenchStamped,
            "/clean_ft_values",
            self.callback,
            10
        )

    # ====================================
    # SMOOTHING FUNCTION
    # ====================================

    def smooth_force(self, fz):

        self.force_buffer.append(fz)

        if len(self.force_buffer) < self.window_size:
            return fz

        return np.mean(self.force_buffer)

    # ====================================
    # CALLBACK
    # ====================================

    def callback(self, msg):

        fz = msg.wrench.force.z
        tz = msg.wrench.torque.z

        fz_s = self.smooth_force(fz)

        if self.prev_force is None:
            self.prev_force = fz_s
            return

        # ====================================
        # STATE MACHINE
        # ====================================

        if self.state == "waiting":

            if fz_s > self.force_threshold:

                self.state = "rising"
                self.max_force = fz_s

                self.torque_values = [tz]

        elif self.state == "rising":

            self.torque_values.append(tz)

            if fz_s > self.max_force:
                self.max_force = fz_s

            if fz_s < self.prev_force:
                # peak detected

                torque_change = max(self.torque_values) - min(self.torque_values)

                if torque_change > self.torque_threshold:
                    result = True
                else:
                    result = False

                self.get_logger().info(
                    f"Peak detected | Fz={self.max_force:.3f} | ΔTz={torque_change:.5f} | result={result}"
                )

                # RESET
                self.reset()

        self.prev_force = fz_s

    # ====================================
    # RESET DETECTOR
    # ====================================

    def reset(self):

        self.state = "waiting"
        self.max_force = -np.inf
        self.torque_values = []
        self.prev_force = None


def main(args=None):

    rclpy.init(args=args)

    node = PeakTorqueDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()