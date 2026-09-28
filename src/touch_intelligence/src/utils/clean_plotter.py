import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

import csv
import os
import time

import matplotlib.pyplot as plt
from collections import deque


class WrenchPlotter(Node):

    def __init__(self):
        super().__init__('clean_ft_values')

        # -------- Subscriber --------
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/NS_1/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame',
            self.callback,
            10
        )

        # -------- CSV setup --------
        filename = f"wrench_log_{int(time.time())}.csv"
        self.csv_path = os.path.join(os.getcwd(), filename)

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'time',
            'fx','fy','fz',
            'tx','ty','tz'
        ])

        self.get_logger().info(f"Logging to: {self.csv_path}")

        # -------- Plot setup --------
        plt.ion()

        self.fig, self.axes = plt.subplots(6, 1, figsize=(8, 10), sharex=True)

        self.fig.canvas.manager.set_window_title("Clean FT Values")

        self.max_points = 300
        self.times = deque(maxlen=self.max_points)

        self.fx = deque(maxlen=self.max_points)
        self.fy = deque(maxlen=self.max_points)
        self.fz = deque(maxlen=self.max_points)
        self.tx = deque(maxlen=self.max_points)
        self.ty = deque(maxlen=self.max_points)
        self.tz = deque(maxlen=self.max_points)

        self.start_time = time.time()

        labels = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
        self.lines = []

        for ax, label in zip(self.axes, labels):
            line, = ax.plot([], [], label=label)
            ax.set_title(label)
            ax.set_ylim(-20, 20)   # FIXED LIMIT
            ax.grid(True)
            self.lines.append(line)

        self.axes[-1].set_xlabel("Time (s)")

    # -------- Optional cleaning --------
    def clean_value(self, v):
        deadband = 0.001
        if abs(v) < deadband:
            return 0.0
        return v

    # -------- Callback --------
    def callback(self, msg):

        t = time.time() - self.start_time

        fx = self.clean_value(msg.wrench.force.x)
        fy = self.clean_value(msg.wrench.force.y)
        fz = self.clean_value(msg.wrench.force.z)

        tx = self.clean_value(msg.wrench.torque.x)
        ty = self.clean_value(msg.wrench.torque.y)
        tz = self.clean_value(msg.wrench.torque.z)

        # Store data
        self.times.append(t)

        self.fx.append(fx)
        self.fy.append(fy)
        self.fz.append(fz)
        self.tx.append(tx)
        self.ty.append(ty)
        self.tz.append(tz)

        # CSV logging
        self.csv_writer.writerow([t, fx, fy, fz, tx, ty, tz])

        data_list = [
            self.fx, self.fy, self.fz,
            self.tx, self.ty, self.tz
        ]

        # Update plots
        for line, data in zip(self.lines, data_list):
            line.set_data(self.times, data)

        for ax in self.axes:
            ax.set_xlim(
                max(0, t - 20),
                max(20, t)
            )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WrenchPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
