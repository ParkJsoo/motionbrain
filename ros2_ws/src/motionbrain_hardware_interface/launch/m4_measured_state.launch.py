from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("motionbrain_hardware_interface"))
    urdf_path = package_share / "urdf" / "motionbrain_m4_measured.urdf"
    controllers_path = package_share / "config" / "m4_measured_controllers.yaml"

    autostart_controllers = LaunchConfiguration("autostart_controllers")
    status_topic = LaunchConfiguration("status_topic")
    shoulder_feedback_calibration_enabled = LaunchConfiguration(
        "shoulder_feedback_calibration_enabled"
    )
    shoulder_sensor_zero_deg = LaunchConfiguration("shoulder_sensor_zero_deg")
    shoulder_direction_sign = LaunchConfiguration("shoulder_direction_sign")
    shoulder_ros_joint_zero_rad = LaunchConfiguration("shoulder_ros_joint_zero_rad")
    state_stale_timeout_sec = LaunchConfiguration("state_stale_timeout_sec")

    def launch_setup(context, *_args, **_kwargs):
        robot_description = urdf_path.read_text(encoding="utf-8")
        replacements = {
            "<param name=\"status_topic\">/motionbrain/status_typed</param>":
                f"<param name=\"status_topic\">{status_topic.perform(context)}</param>",
            "<param name=\"shoulder_feedback_calibration_enabled\">false</param>":
                (
                    "<param name=\"shoulder_feedback_calibration_enabled\">"
                    f"{shoulder_feedback_calibration_enabled.perform(context)}</param>"
                ),
            "<param name=\"shoulder_sensor_zero_deg\">0.0</param>":
                (
                    "<param name=\"shoulder_sensor_zero_deg\">"
                    f"{shoulder_sensor_zero_deg.perform(context)}</param>"
                ),
            "<param name=\"shoulder_direction_sign\">1</param>":
                (
                    "<param name=\"shoulder_direction_sign\">"
                    f"{shoulder_direction_sign.perform(context)}</param>"
                ),
            "<param name=\"shoulder_ros_joint_zero_rad\">0.0</param>":
                (
                    "<param name=\"shoulder_ros_joint_zero_rad\">"
                    f"{shoulder_ros_joint_zero_rad.perform(context)}</param>"
                ),
            "<param name=\"state_stale_timeout_sec\">2.0</param>":
                (
                    "<param name=\"state_stale_timeout_sec\">"
                    f"{state_stale_timeout_sec.perform(context)}</param>"
                ),
        }
        for old, new in replacements.items():
            robot_description = robot_description.replace(old, new)

        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="motionbrain_m4_measured_robot_state_publisher",
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
                ],
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart_controllers",
                default_value="true",
                description="Load and activate read-only M4 state controllers.",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/motionbrain/status_typed",
                description="Typed MotionStatus source for read-only M4 state.",
            ),
            DeclareLaunchArgument(
                "shoulder_feedback_calibration_enabled",
                default_value="false",
                description="Enable only after supervised M4 sensor zero/sign calibration.",
            ),
            DeclareLaunchArgument(
                "shoulder_sensor_zero_deg",
                default_value="0.0",
                description="M4 sensor-space angle corresponding to ROS shoulder zero.",
            ),
            DeclareLaunchArgument(
                "shoulder_direction_sign",
                default_value="1",
                description="M4 sensor-to-ROS direction sign; must be -1 or 1.",
            ),
            DeclareLaunchArgument(
                "shoulder_ros_joint_zero_rad",
                default_value="0.0",
                description="ROS shoulder joint value at shoulder_sensor_zero_deg.",
            ),
            DeclareLaunchArgument(
                "state_stale_timeout_sec",
                default_value="2.0",
                description=(
                    "Maximum age for cached M4 feedback before state becomes unavailable; "
                    "keep above the expected status bridge poll interval."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ],
    )
