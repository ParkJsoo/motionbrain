from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("motionbrain_hardware_interface"))
    description = (share / "urdf" / "motionbrain_m4_proposal.urdf").read_text()
    controllers = str(share / "config" / "m4_proposal_controllers.yaml")
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="motionbrain_m4_proposal_robot_state_publisher",
                parameters=[{"robot_description": description}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[{"robot_description": description}, controllers],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"]
                    ),
                    ExecuteProcess(
                        cmd=["ros2", "control", "load_controller", "--set-state", "active", "m4_proposal_controller"]
                    ),
                ],
            ),
        ]
    )
