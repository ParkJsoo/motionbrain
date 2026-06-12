from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("motionbrain_ros2_control_mock"))
    urdf_path = package_share / "urdf" / "motionbrain_mock_control.urdf"
    controllers_path = package_share / "config" / "controllers.yaml"

    autostart_controllers = LaunchConfiguration("autostart_controllers")
    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart_controllers",
                default_value="true",
                description="Load and activate mock ros2_control controllers.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="motionbrain_mock_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    str(controllers_path),
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "control",
                            "load_controller",
                            "--set-state",
                            "active",
                            "joint_state_broadcaster",
                        ],
                        output="screen",
                        condition=IfCondition(autostart_controllers),
                    ),
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "control",
                            "load_controller",
                            "--set-state",
                            "active",
                            "motionbrain_arm_controller",
                        ],
                        output="screen",
                        condition=IfCondition(autostart_controllers),
                    ),
                ],
            ),
        ],
    )
