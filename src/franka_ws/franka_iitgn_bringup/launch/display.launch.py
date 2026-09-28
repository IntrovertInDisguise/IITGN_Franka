# Copyright (c) 2023 Franka Robotics GmbH
# Licensed under the Apache License, Version 2.0

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_robot_state_publisher(context: LaunchContext, arm_id, load_gripper, ee_id, joint_states_topic):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    ee_id_str = context.perform_substitution(ee_id)
    joint_states_topic_str = context.perform_substitution(joint_states_topic)

    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        f"{arm_id_str}.urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            "hand": load_gripper_str,
            "ee_id": ee_id_str,
        },
    ).toprettyxml(indent="  ")

    # robot_state_publisher listens on "joint_states" by default.
    # We remap it to whatever the hardware publishes (often /joint_states or namespaced).
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[
                ("joint_states", joint_states_topic_str),
            ],
        )
    ]


def generate_launch_description():
    # Defaults you requested
    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    # IMPORTANT: set this to your real hardware joint states topic if it's namespaced
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    rviz_config = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_id",
                default_value="fr3",
                description="Franka arm type (default: fr3). Examples: fer, fr3, fp3, fr3v2",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="true",
                description="Load Franka hand if true (default: true).",
            ),
            DeclareLaunchArgument(
                "ee_id",
                default_value="franka_hand",
                description="End-effector ID (default: franka_hand). Examples: none, franka_hand, cobot_pump",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description=(
                    "Topic that provides REAL joint states from hardware. "
                    "If your driver publishes a namespaced topic, set it here."
                ),
            ),

            # TF publisher driven by hardware joint_states
            OpaqueFunction(
                function=spawn_robot_state_publisher,
                args=[arm_id, load_gripper, ee_id, joint_states_topic],
            ),

            # RViz visualization (digital twin)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
