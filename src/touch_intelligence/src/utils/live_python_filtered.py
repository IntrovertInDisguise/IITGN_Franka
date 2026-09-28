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
        super().__init__('wrench_plotter')

        # Subscriber
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/filtered_clean_ft_values',
            self.callback,
            10
        )

        # ---- CSV setup ----
        filename = f"wrench_log_{int(time.time())}.csv"
        self.csv_path = os.path.join(os.getcwd(), filename)

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

        self.get_logger().info(f"Logging to: {self.csv_path}")

        # ---- Plot setup ----
        plt.ion()

        self.fig, self.axes = plt.subplots(3, 2, figsize=(10, 8))

        self.max_points = 300
        self.times = deque(maxlen=self.max_points)

        self.fx_data = deque(maxlen=self.max_points)
        self.fy_data = deque(maxlen=self.max_points)
        self.fz_data = deque(maxlen=self.max_points)

        self.tx_data = deque(maxlen=self.max_points)
        self.ty_data = deque(maxlen=self.max_points)
        self.tz_data = deque(maxlen=self.max_points)

        self.start_time = time.time()

        # ---- Plot lines with different colors ----
        self.line_fx, = self.axes[0,0].plot([], [], color='red')
        self.line_fy, = self.axes[0,1].plot([], [], color='green')
        self.line_fz, = self.axes[1,0].plot([], [], color='blue')

        self.line_tx, = self.axes[1,1].plot([], [], color='purple')
        self.line_ty, = self.axes[2,0].plot([], [], color='orange')
        self.line_tz, = self.axes[2,1].plot([], [], color='brown')

        # Titles
        self.axes[0,0].set_title("Force X (Fx)")
        self.axes[0,1].set_title("Force Y (Fy)")
        self.axes[1,0].set_title("Force Z (Fz)")

        self.axes[1,1].set_title("Torque X (Tx)")
        self.axes[2,0].set_title("Torque Y (Ty)")
        self.axes[2,1].set_title("Torque Z (Tz)")

        # Axis labels
        self.axes[0,0].set_ylabel("Force (N)")
        self.axes[1,0].set_ylabel("Force (N)")

        self.axes[1,1].set_ylabel("Torque (Nm)")
        self.axes[2,0].set_ylabel("Torque (Nm)")

        self.axes[2,0].set_xlabel("Time (s)")
        self.axes[2,1].set_xlabel("Time (s)")

        for row in self.axes:
            for ax in row:
                ax.grid(True)

    def callback(self, msg):

        t = time.time() - self.start_time

        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z

        # Store data
        self.times.append(t)

        self.fx_data.append(fx)
        self.fy_data.append(fy)
        self.fz_data.append(fz)

        self.tx_data.append(tx)
        self.ty_data.append(ty)
        self.tz_data.append(tz)

        # Write CSV
        self.csv_writer.writerow([t, fx, fy, fz, tx, ty, tz])

        # Update plots
        self.line_fx.set_data(self.times, self.fx_data)
        self.line_fy.set_data(self.times, self.fy_data)
        self.line_fz.set_data(self.times, self.fz_data)

        self.line_tx.set_data(self.times, self.tx_data)
        self.line_ty.set_data(self.times, self.ty_data)
        self.line_tz.set_data(self.times, self.tz_data)

        for row in self.axes:
            for ax in row:
                ax.relim()
                ax.autoscale_view()

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