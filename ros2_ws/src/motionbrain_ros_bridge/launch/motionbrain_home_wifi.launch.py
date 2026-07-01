from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    motion_host = LaunchConfiguration("motion_host")
    camera_url = LaunchConfiguration("camera_url")
    perception_url = LaunchConfiguration("perception_url")
    detect_color = LaunchConfiguration("detect_color")
    poll_interval = LaunchConfiguration("poll_interval")
    http_timeout = LaunchConfiguration("http_timeout")
    events_limit = LaunchConfiguration("events_limit")
    status_autostart = LaunchConfiguration("status_autostart")
    enable_joint_state_bridge = LaunchConfiguration("enable_joint_state_bridge")
    joint_state_autostart = LaunchConfiguration("joint_state_autostart")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    estimated_joint_states_topic = LaunchConfiguration("estimated_joint_states_topic")
    kinematics_joint_states_topic = LaunchConfiguration("kinematics_joint_states_topic")
    joint_states_output = LaunchConfiguration("joint_states_output")
    shoulder_feedback_calibration_enabled = LaunchConfiguration(
        "shoulder_feedback_calibration_enabled"
    )
    shoulder_sensor_zero_deg = LaunchConfiguration("shoulder_sensor_zero_deg")
    shoulder_direction_sign = LaunchConfiguration("shoulder_direction_sign")
    shoulder_ros_joint_zero_rad = LaunchConfiguration("shoulder_ros_joint_zero_rad")
    enable_kinematics = LaunchConfiguration("enable_kinematics")
    kinematics_autostart = LaunchConfiguration("kinematics_autostart")
    enable_control_guard = LaunchConfiguration("enable_control_guard")
    control_guard_autostart = LaunchConfiguration("control_guard_autostart")
    control_guard_require_armed = LaunchConfiguration("control_guard_require_armed")
    control_guard_require_detection = LaunchConfiguration("control_guard_require_detection")
    enable_mission_supervisor = LaunchConfiguration("enable_mission_supervisor")
    mission_supervisor_autostart = LaunchConfiguration("mission_supervisor_autostart")

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
                "perception_url",
                default_value="",
                description="Optional Pi perception service base URL. When set, /camera/detection comes from /api/detection instead of direct camera polling.",
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
                "status_autostart",
                default_value="true",
                description="Automatically configure and activate the status bridge lifecycle node.",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_bridge",
                default_value="true",
                description="Start bridge-derived JointState outputs.",
            ),
            DeclareLaunchArgument(
                "joint_state_autostart",
                default_value="true",
                description="Automatically configure and activate the joint-state lifecycle node.",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="Selected /joint_states output topic. Use joint_states_output to choose its owner.",
            ),
            DeclareLaunchArgument(
                "estimated_joint_states_topic",
                default_value="/motionbrain/estimated_joint_states",
                description="Explicit status-derived estimated JointState topic.",
            ),
            DeclareLaunchArgument(
                "kinematics_joint_states_topic",
                default_value="/joint_states",
                description=(
                    "JointState input consumed by the FK/kinematics node. Set to "
                    "/motionbrain/estimated_joint_states when /joint_states is reserved "
                    "for measured M4-only state."
                ),
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
            DeclareLaunchArgument(
                "enable_kinematics",
                default_value="true",
                description="Publish FK end-effector pose and kinematics diagnostics.",
            ),
            DeclareLaunchArgument(
                "kinematics_autostart",
                default_value="true",
                description="Automatically configure and activate the kinematics lifecycle node.",
            ),
            DeclareLaunchArgument(
                "enable_control_guard",
                default_value="true",
                description="Publish C++ control readiness guard from typed status and camera detection.",
            ),
            DeclareLaunchArgument(
                "control_guard_autostart",
                default_value="true",
                description="Automatically configure and activate the control guard lifecycle node.",
            ),
            DeclareLaunchArgument(
                "control_guard_require_armed",
                default_value="false",
                description="Require controller armed state before the control guard reports ready.",
            ),
            DeclareLaunchArgument(
                "control_guard_require_detection",
                default_value="false",
                description="Require fresh positive camera detection before the control guard reports ready.",
            ),
            DeclareLaunchArgument(
                "enable_mission_supervisor",
                default_value="true",
                description="Publish lightweight mission state for detect-align-confirm-act demos.",
            ),
            DeclareLaunchArgument(
                "mission_supervisor_autostart",
                default_value="true",
                description="Automatically configure and activate the mission supervisor lifecycle node.",
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
                        "perception_url": perception_url,
                        "detect_color": detect_color,
                        "poll_interval": ParameterValue(poll_interval, value_type=float),
                        "http_timeout": ParameterValue(http_timeout, value_type=float),
                        "events_limit": ParameterValue(events_limit, value_type=int),
                        "autostart": ParameterValue(status_autostart, value_type=bool),
                    }
                ],
            ),
            Node(
                package="motionbrain_ros_bridge",
                executable="motionbrain_joint_state_node",
                name="motionbrain_joint_state_node",
                output="screen",
                condition=IfCondition(enable_joint_state_bridge),
                parameters=[
                    {
                        "joint_states_topic": joint_states_topic,
                        "estimated_joint_states_topic": estimated_joint_states_topic,
                        "joint_states_output": joint_states_output,
                        "autostart": ParameterValue(joint_state_autostart, value_type=bool),
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
                package="motionbrain_ros_bridge",
                executable="motionbrain_kinematics_node",
                name="motionbrain_kinematics_node",
                output="screen",
                condition=IfCondition(enable_kinematics),
                parameters=[
                    {
                        "joint_states_topic": kinematics_joint_states_topic,
                        "autostart": ParameterValue(kinematics_autostart, value_type=bool),
                    }
                ],
            ),
            Node(
                package="motionbrain_control",
                executable="motionbrain_control_guard_node",
                name="motionbrain_control_guard_node",
                output="screen",
                condition=IfCondition(enable_control_guard),
                parameters=[
                    {
                        "autostart": ParameterValue(control_guard_autostart, value_type=bool),
                        "require_armed": ParameterValue(
                            control_guard_require_armed,
                            value_type=bool,
                        ),
                        "require_detection": ParameterValue(
                            control_guard_require_detection,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="motionbrain_mission",
                executable="motionbrain_mission_supervisor",
                name="motionbrain_mission_supervisor",
                output="screen",
                condition=IfCondition(enable_mission_supervisor),
                parameters=[
                    {
                        "autostart": ParameterValue(
                            mission_supervisor_autostart,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
