#!/usr/bin/env python3
"""
Franka KDL FK node (ROS2).

Publishes two poses:
  1) EE pose:   base_link -> tip_link
  2) TOOL pose: base_link -> tool_link  (e.g., fr_hand_tcp)

Subscribes:
  - joint_states_topic (sensor_msgs/JointState)
  - robot_description_topic (std_msgs/String, TRANSIENT_LOCAL)

Params:
  base_link, tip_link, tool_link,
  joint_states_topic, robot_description_topic,
  pose_topic, tool_pose_topic
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class FrankaKDLForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("franka_kdl_forward_kinematics_node")

        # ----------------------------
        # Parameters (Franka defaults)
        # ----------------------------
        self.declare_parameter("base_link", "fr3_link0")
        self.declare_parameter("tip_link", "fr3_link8")
        self.declare_parameter("tool_link", "fr3_hand_tcp")

        self.declare_parameter("joint_states_topic", "/NS_1/franka/joint_states")
        self.declare_parameter("robot_description_topic", "/NS_1/robot_description")

        self.declare_parameter("pose_topic", "/NS_1/end_effector_pose")
        self.declare_parameter("tool_pose_topic", "/NS_1/tool_end_effector_pose")

        self.base_link = self.get_parameter("base_link").value
        self.tip_link = self.get_parameter("tip_link").value
        self.tool_link = self.get_parameter("tool_link").value

        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.robot_description_topic = self.get_parameter("robot_description_topic").value

        self.pose_topic = self.get_parameter("pose_topic").value
        self.tool_pose_topic = self.get_parameter("tool_pose_topic").value

        # ----------------------------
        # State
        # ----------------------------
        self._kdl_ready = False
        self.n_joints = 0
        self.joint_names: list[str] = []

        self.chain_ee = None
        self.chain_tool = None
        self.fk_solver_ee = None
        self.fk_solver_tool = None

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tool_pose_pub = self.create_publisher(PoseStamped, self.tool_pose_topic, 10)

        # Joint states: use SensorData QoS
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        # /robot_description is latched / TRANSIENT_LOCAL
        robot_desc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._robot_desc_sub = self.create_subscription(
            String,
            self.robot_description_topic,
            self.robot_description_callback,
            robot_desc_qos,
        )

        self.get_logger().info(
            "Franka KDL FK node started.\n"
            f"  base_link={self.base_link}\n"
            f"  tip_link={self.tip_link}\n"
            f"  tool_link={self.tool_link}\n"
            f"  joint_states_topic={self.joint_states_topic}\n"
            f"  robot_description_topic={self.robot_description_topic}\n"
            f"  pose_topic={self.pose_topic}\n"
            f"  tool_pose_topic={self.tool_pose_topic}\n"
            "Waiting for robot_description to build KDL chains..."
        )

    def robot_description_callback(self, msg: String):
        if self._kdl_ready:
            return

        urdf_xml = msg.data.strip()
        if not urdf_xml or "<robot" not in urdf_xml:
            self.get_logger().error("Received robot_description but it doesn't look like URDF XML.")
            return

        try:
            robot = URDF.from_xml_string(urdf_xml)
            ok, tree = treeFromUrdfModel(robot)
            if not ok:
                self.get_logger().error("Failed to construct KDL tree from URDF.")
                return

            # EE chain
            chain_ee = tree.getChain(self.base_link, self.tip_link)
            n_ee = int(chain_ee.getNrOfJoints())
            if n_ee <= 0:
                self.get_logger().error(
                    f"EE KDL chain has 0 joints. Check base_link='{self.base_link}', tip_link='{self.tip_link}'."
                )
                return

            # Joint names from EE chain (exactly n_ee movable joints)
            names: list[str] = []
            for i in range(chain_ee.getNrOfSegments()):
                seg = chain_ee.getSegment(i)
                j = seg.getJoint()
                jt = j.getTypeName()
                if jt not in ("None", "Fixed"):
                    names.append(j.getName())
                    if len(names) == n_ee:
                        break

            if len(names) != n_ee:
                self.get_logger().error(
                    f"Joint-name extraction mismatch: KDL n_joints={n_ee}, extracted={len(names)}: {names}"
                )
                return

            # TOOL chain (same joints expected, just more fixed segments at the end)
            chain_tool = tree.getChain(self.base_link, self.tool_link)
            n_tool = int(chain_tool.getNrOfJoints())
            if n_tool <= 0:
                self.get_logger().error(
                    f"TOOL KDL chain has 0 joints. Check base_link='{self.base_link}', tool_link='{self.tool_link}'."
                )
                return

            # Usually n_tool == n_ee for franka TCP frames; if not, warn but continue
            if n_tool != n_ee:
                self.get_logger().warn(
                    f"TOOL chain joint count ({n_tool}) != EE chain joint count ({n_ee}). "
                    "This is unusual but not necessarily wrong."
                )

            self.chain_ee = chain_ee
            self.chain_tool = chain_tool
            self.n_joints = n_ee
            self.joint_names = names

            self.fk_solver_ee = kdl.ChainFkSolverPos_recursive(self.chain_ee)
            self.fk_solver_tool = kdl.ChainFkSolverPos_recursive(self.chain_tool)

            self._kdl_ready = True
            self.get_logger().info(
                f"KDL ready. n_joints={self.n_joints}\n"
                f"  Joint order: {self.joint_names}\n"
                f"  EE chain:   {self.base_link} -> {self.tip_link}\n"
                f"  TOOL chain: {self.base_link} -> {self.tool_link}"
            )

            self.destroy_subscription(self._robot_desc_sub)

        except Exception as e:
            self.get_logger().error(f"Exception creating KDL from URDF: {e}")

    @staticmethod
    def _frame_to_pose(frame: kdl.Frame, stamp, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(frame.p[0])
        msg.pose.position.y = float(frame.p[1])
        msg.pose.position.z = float(frame.p[2])
        qx, qy, qz, qw = frame.M.GetQuaternion()
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        return msg

    def joint_state_callback(self, msg: JointState):
        if not self._kdl_ready:
            return
        if not msg.name or not msg.position:
            return

        name_to_index = {n: i for i, n in enumerate(msg.name)}
        q = kdl.JntArray(self.n_joints)

        for i, jn in enumerate(self.joint_names):
            idx = name_to_index.get(jn, None)
            if idx is None:
                q[i] = 0.0
                if (self.get_clock().now().nanoseconds // 1_000_000_000) % 5 == 0:
                    self.get_logger().warn(f"Joint '{jn}' not found in JointState; defaulting to 0.0")
            else:
                q[i] = float(msg.position[idx])

        stamp = self.get_clock().now().to_msg()

        # EE FK
        ee_frame = kdl.Frame()
        if self.fk_solver_ee.JntToCart(q, ee_frame) < 0:
            self.get_logger().error("EE FK failed (JntToCart < 0)")
            return
        self.pose_pub.publish(self._frame_to_pose(ee_frame, stamp, self.base_link))

        # TOOL FK
        tool_frame = kdl.Frame()
        if self.fk_solver_tool.JntToCart(q, tool_frame) < 0:
            self.get_logger().error("TOOL FK failed (JntToCart < 0)")
            return
        self.tool_pose_pub.publish(self._frame_to_pose(tool_frame, stamp, self.base_link))


def main():
    rclpy.init()
    node = FrankaKDLForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
