from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    motion_host = LaunchConfiguration("motion_host")
    camera_url = LaunchConfiguration("camera_url")
    detect_color = LaunchConfiguration("detect_color")
    poll_interval = LaunchConfiguration("poll_interval")
    http_timeout = LaunchConfiguration("http_timeout")
    events_limit = LaunchConfiguration("events_limit")
    enable_kinematics = LaunchConfiguration("enable_kinematics")
    enable_control_guard = LaunchConfiguration("enable_control_guard")
    enable_mission_supervisor = LaunchConfiguration("enable_mission_supervisor")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_host",
                default_value="motionbrain.local",
                description="ESP32 motion controller hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "camera_url",
                default_value="http://motionbrain-cam.local",
                description="ESP32-CAM base URL.",
            ),
            DeclareLaunchArgument(
                "detect_color",
                default_value="red",
                description="Target color for ESP32-CAM frame detection.",
            ),
            DeclareLaunchArgument(
                "poll_interval",
                default_value="1.0",
                description="Bridge polling interval in seconds.",
            ),
            DeclareLaunchArgument(
                "http_timeout",
                default_value="4.0",
                description="HTTP timeout in seconds.",
            ),
            DeclareLaunchArgument(
                "events_limit",
                default_value="8",
                description="Number of recent ESP32 events to publish per poll.",
            ),
            DeclareLaunchArgument(
                "enable_kinematics",
                default_value="true",
                description="Publish FK end-effector pose and kinematics diagnostics.",
            ),
            DeclareLaunchArgument(
                "enable_control_guard",
                default_value="true",
                description="Publish C++ control readiness guard from typed status and camera detection.",
            ),
            DeclareLaunchArgument(
                "enable_mission_supervisor",
                default_value="true",
                description="Publish lightweight mission state for detect-align-confirm-act demos.",
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_status_node",
                name="motionbrain_status_node",
                output="screen",
                parameters=[
                    {
                        "motion_host": motion_host,
                        "camera_url": camera_url,
                        "detect_color": detect_color,
                        "poll_interval": ParameterValue(poll_interval, value_type=float),
                        "http_timeout": ParameterValue(http_timeout, value_type=float),
                        "events_limit": ParameterValue(events_limit, value_type=int),
                    }
                ],
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_joint_state_node",
                name="motionbrain_joint_state_node",
                output="screen",
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_kinematics_node",
                name="motionbrain_kinematics_node",
                output="screen",
                condition=IfCondition(enable_kinematics),
            ),
            Node(
                package="motionbrain_control",
                executable="motionbrain_control_guard_node",
                name="motionbrain_control_guard_node",
                output="screen",
                condition=IfCondition(enable_control_guard),
            ),
            Node(
                package="motionbrain_mission",
                executable="motionbrain_mission_supervisor",
                name="motionbrain_mission_supervisor",
                output="screen",
                condition=IfCondition(enable_mission_supervisor),
            ),
        ]
    )
