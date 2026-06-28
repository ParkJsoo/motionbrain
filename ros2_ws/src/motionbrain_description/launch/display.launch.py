from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("motionbrain_description"))
    urdf_path = package_share / "urdf" / "motionbrain.urdf"
    rviz_path = package_share / "rviz" / "motionbrain.rviz"

    use_rviz = LaunchConfiguration("use_rviz")
    start_joint_state_bridge = LaunchConfiguration("start_joint_state_bridge")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    estimated_joint_states_topic = LaunchConfiguration("estimated_joint_states_topic")
    joint_states_output = LaunchConfiguration("joint_states_output")
    shoulder_feedback_calibration_enabled = LaunchConfiguration(
        "shoulder_feedback_calibration_enabled"
    )
    shoulder_sensor_zero_deg = LaunchConfiguration("shoulder_sensor_zero_deg")
    shoulder_direction_sign = LaunchConfiguration("shoulder_direction_sign")
    shoulder_ros_joint_zero_rad = LaunchConfiguration("shoulder_ros_joint_zero_rad")

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
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="JointState topic used by the display stack.",
            ),
            DeclareLaunchArgument(
                "estimated_joint_states_topic",
                default_value="/motionbrain/estimated_joint_states",
                description="Explicit status-derived estimated JointState topic.",
            ),
            DeclareLaunchArgument(
                "joint_states_output",
                default_value="estimated",
                description="Selected output for joint_states_topic: estimated, measured, or none.",
            ),
            DeclareLaunchArgument(
                "shoulder_feedback_calibration_enabled",
                default_value="false",
                description="Use calibrated M4 AS5600 feedback for shoulder_pitch_joint.",
            ),
            DeclareLaunchArgument(
                "shoulder_sensor_zero_deg",
                default_value="0.0",
                description="Explicit M4 sensor-space angle corresponding to the ROS joint zero.",
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", joint_states_topic)],
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_joint_state_node",
                name="motionbrain_joint_state_node",
                output="screen",
                condition=IfCondition(start_joint_state_bridge),
                parameters=[
                    {
                        "joint_states_topic": joint_states_topic,
                        "estimated_joint_states_topic": estimated_joint_states_topic,
                        "joint_states_output": joint_states_output,
                        "shoulder_feedback_calibration_enabled": ParameterValue(
                            shoulder_feedback_calibration_enabled,
                            value_type=bool,
                        ),
                        "shoulder_sensor_zero_deg": ParameterValue(
                            shoulder_sensor_zero_deg,
                            value_type=float,
                        ),
                        "shoulder_direction_sign": ParameterValue(
                            shoulder_direction_sign,
                            value_type=int,
                        ),
                        "shoulder_ros_joint_zero_rad": ParameterValue(
                            shoulder_ros_joint_zero_rad,
                            value_type=float,
                        ),
                    }
                ],
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
