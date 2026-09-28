import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# franka_bringup utils: load_yaml()
package_share = get_package_share_directory("franka_bringup")
utils_path = os.path.join(package_share, "..", "..", "lib", "franka_bringup", "utils")
sys.path.append(os.path.abspath(utils_path))
from franka_bringup.launch_utils import load_yaml  # noqa: E402


def _controller_manager_path(namespace: str) -> str:
    # controller_manager service name inside namespace
    if namespace and not namespace.startswith("/"):
        namespace = "/" + namespace
    return f"{namespace}/controller_manager" if namespace else "/controller_manager"


def _ns_norm(ns: str) -> str:
    ns = (ns or "").strip()
    if not ns:
        return ""
    return ns if ns.startswith("/") else "/" + ns


def _ns_join(ns: str, topic: str) -> str:
    ns = _ns_norm(ns)
    topic = topic if topic.startswith("/") else "/" + topic
    return f"{ns}{topic}" if ns else topic


def generate_robot_nodes(context):
    config_file = LaunchConfiguration("robot_config_file").perform(context)
    controller_name = LaunchConfiguration("controller_name").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)

    configs = load_yaml(config_file)
    nodes = []

    for _, cfg in configs.items():
        namespace = str(cfg["namespace"])
        use_fake_hardware = (
            LaunchConfiguration("use_fake_hardware").perform(context).lower() in ["true"]
        )

        # if use_fake_hardware is true launch the robot and controllers in gazebo simulation (humble, fortress)
        # set gazebo_effort t= true in franka_iitgn.config.yaml file if you want to use impedance control
        # for available controllers in gazebo simulation refer to gazebo_controllers.yaml
        if use_fake_hardware:
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_iitgn_bringup"),
                                "launch",
                                "franka_gazebo.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "arm_id": str(cfg["arm_id"]),
                        "load_gripper": str(cfg["load_gripper"]),
                        "gazebo_effort": str(cfg["gazebo_effort"]),
                        "namespace": str(namespace),
                        "controller_name": controller_name,
                    }.items(),
                )
            )

        else:
            # 1) Bring up robot + ros2_control_node using franka.launch.py
            #    IMPORTANT: pass controllers_yaml := controllers_file
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_bringup"),
                                "launch",
                                "franka.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "arm_id": str(cfg["arm_id"]),
                        "robot_type": str(cfg["robot_type"]),
                        "arm_prefix": str(cfg["arm_prefix"]),
                        "namespace": str(namespace),
                        "urdf_file": str(cfg["urdf_file"]),
                        "robot_ip": str(cfg["robot_ip"]),
                        "load_gripper": str(cfg["load_gripper"]),
                        "use_fake_hardware": str(cfg["use_fake_hardware"]),
                        "fake_sensor_commands": str(cfg["fake_sensor_commands"]),
                        "joint_state_rate": str(cfg["joint_state_rate"]),
                        "controllers_yaml": str(controllers_file),  # <<< THIS IS THE KEY FIX
                    }.items(),
                )
            )

            # 2) Spawn the controller you requested (controller_manager now knows its type)
            nodes.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        controller_name,
                        "--controller-manager",
                        _controller_manager_path(namespace),
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                )
            )

        # 3) KDL FK broadcaster (EE + tool pose) for this namespace
        nodes.append(
            Node(
                package="franka_kdl_broadcaster",
                executable="franka_kdl_fk_node",
                name="franka_kdl_fk_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "base_link": str(cfg.get("base_link", "fr3_link0")),
                        "tip_link": str(cfg.get("tip_link", "fr3_link8")),
                        "tool_tip_link": str(cfg.get("tool_tip_link", "fr_hand_tcp")),
                        "joint_states_topic": _ns_join(namespace, "franka/joint_states"),
                        "robot_description_topic": _ns_join(namespace, "robot_description"),
                        "pose_topic": _ns_join(namespace, "end_effector_pose"),
                        "tool_pose_topic": _ns_join(namespace, "tool_end_effector_pose"),
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_iitgn_bringup"),
                        "config",
                        "franka_iitgn.config.yaml",
                    ]
                ),
                description="Path to the robot configuration file to load",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_iitgn_bringup"),
                        "config",
                        "controllers.yaml",
                    ]
                ),
                description="Path to controllers.yaml (IITGN controllers config)",
            ),
            DeclareLaunchArgument(
                "controller_name",
                description="Name of the controller to spawn (required, no default)",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="If true, launch Gazebo instead of real robot",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
