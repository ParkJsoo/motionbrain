from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("motionbrain_description"))
    urdf_path = package_share / "urdf" / "motionbrain.urdf"
    rviz_path = package_share / "rviz" / "motionbrain.rviz"

    use_rviz = LaunchConfiguration("use_rviz")
    start_joint_state_bridge = LaunchConfiguration("start_joint_state_bridge")

    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz2 with the MotionBrain display config.",
            ),
            DeclareLaunchArgument(
                "start_joint_state_bridge",
                default_value="true",
                description="Start MotionStatus to JointState bridge.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_joint_state_node",
                name="motionbrain_joint_state_node",
                output="screen",
                condition=IfCondition(start_joint_state_bridge),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
