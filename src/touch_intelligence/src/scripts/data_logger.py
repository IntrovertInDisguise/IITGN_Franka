import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState

import csv
import os


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger_node')

        # -------------------------------------------------
        # Create ONLY session_logs directory
        # -------------------------------------------------
        current_dir = os.path.dirname(os.path.abspath(__file__))
        logs_dir = os.path.abspath(os.path.join(current_dir, "../logs"))
        base_path = os.path.join(logs_dir, "session_logs")

        # Create session_logs if it doesn't exist
        if not os.path.exists(base_path):
            os.mkdir(base_path)

        # CSV file paths
        self.ft_file_path = os.path.join(base_path, "clean_ft_values.csv")
        self.pose_file_path = os.path.join(base_path, "current_pose.csv")
        self.joint_file_path = os.path.join(base_path, "joint_states.csv")

        # Initialize CSV files
        self.init_csv_files()

        # -------------------------------------------------
        # Subscriptions
        # -------------------------------------------------
        self.create_subscription(
            WrenchStamped,
            '/clean_ft_values',
            self.ft_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/NS_1/franka_robot_state_broadcaster/current_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            JointState,
            '/NS_1/franka/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("Data Logger Node Started.")


    # -------------------------------------------------
    # Initialize CSV headers
    # -------------------------------------------------
    def init_csv_files(self):

        with open(self.ft_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "sec", "nanosec",
                "fx", "fy", "fz",
                "tx", "ty", "tz"
            ])

        with open(self.pose_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "px", "py", "pz",
                "ox", "oy", "oz", "ow"
            ])

        with open(self.joint_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "sec", "nanosec",
                "positions",
                "velocities",
                "efforts"
            ])


    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------

    def ft_callback(self, msg: WrenchStamped):
        with open(self.ft_file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z
            ])


    def pose_callback(self, msg: PoseStamped):
        with open(self.pose_file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])


    def joint_callback(self, msg: JointState):
        with open(self.joint_file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                list(msg.position),
                list(msg.velocity),
                list(msg.effort)
            ])


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
