from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fake_host = LaunchConfiguration("fake_host")
    fake_port = LaunchConfiguration("fake_port")
    scenario = LaunchConfiguration("scenario")
    delay_sec = LaunchConfiguration("delay_sec")
    poll_interval = LaunchConfiguration("poll_interval")
    http_timeout = LaunchConfiguration("http_timeout")
    events_limit = LaunchConfiguration("events_limit")

    fake_base_url = ParameterValue(
        ["http://", fake_host, ":", fake_port],
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_host",
                default_value="127.0.0.1",
                description="Bind address used by the fake MotionBrain endpoint.",
            ),
            DeclareLaunchArgument(
                "fake_port",
                default_value="8767",
                description="Port used by the fake MotionBrain endpoint.",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="ready",
                description=(
                    "Fault scenario: ready, controller_fault, malformed_status, "
                    "policy_mismatch, stale_detection, stale_shoulder, timeout_status."
                ),
            ),
            DeclareLaunchArgument(
                "delay_sec",
                default_value="3.0",
                description="Delay used by timeout_status fault injection.",
            ),
            DeclareLaunchArgument(
                "poll_interval",
                default_value="0.5",
                description="Status bridge poll interval in seconds.",
            ),
            DeclareLaunchArgument(
                "http_timeout",
                default_value="0.4",
                description="Status bridge HTTP timeout in seconds.",
            ),
            DeclareLaunchArgument(
                "events_limit",
                default_value="1",
                description="Number of fake events fetched per bridge poll.",
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_fake_endpoint",
                name="motionbrain_fake_endpoint",
                output="screen",
                arguments=[
                    "--host",
                    fake_host,
                    "--port",
                    fake_port,
                    "--scenario",
                    scenario,
                    "--delay-sec",
                    delay_sec,
                    "--quiet",
                ],
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_status_node",
                name="motionbrain_status_node",
                output="screen",
                parameters=[
                    {
                        "motion_host": fake_host,
                        "motion_port": ParameterValue(fake_port, value_type=int),
                        "perception_url": fake_base_url,
                        "camera_url": "",
                        "poll_interval": ParameterValue(poll_interval, value_type=float),
                        "http_timeout": ParameterValue(http_timeout, value_type=float),
                        "events_limit": ParameterValue(events_limit, value_type=int),
                        "http_token": "",
                    }
                ],
            ),
        ]
    )
