import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC = REPO_ROOT / "ros2_ws" / "src"

EXPECTED_PACKAGES = {
    "motionbrain_msgs",
    "motionbrain_control",
    "motionbrain_hardware_interface",
    "motionbrain_mission",
    "motionbrain_ros_bridge",
    "motionbrain_description",
    "motionbrain_ros2_control_mock",
}

EXPECTED_RUNTIME_TOPICS = {
    "/motionbrain/status_typed",
    "/motionbrain/routine",
    "/motionbrain/routine_typed",
    "/motionbrain/lifecycle_typed",
    "/motionbrain/diagnostics",
    "/camera/detection_typed",
    "/motionbrain/estimated_joint_states",
    "/joint_states",
    "/motionbrain/end_effector_pose",
    "/motionbrain/kinematics_typed",
    "/motionbrain/control_guard_typed",
    "/motionbrain/mission_state_typed",
}

EXPECTED_MESSAGE_FILES = {
    "CameraDetection.msg",
    "ControlGuard.msg",
    "KinematicsState.msg",
    "LightCommand.msg",
    "LightResult.msg",
    "MissionCommand.msg",
    "MissionState.msg",
    "NodeLifecycleStatus.msg",
    "MotionEvent.msg",
    "MotionStatus.msg",
    "RoutineCommand.msg",
    "RoutineResult.msg",
    "RoutineStatus.msg",
}

EXPECTED_SERVICE_FILES = {
    "GuardedRoutineCommand.srv",
}

EXPECTED_ACTION_FILES = {
    "GuardedRoutine.action",
}

EXPECTED_PACKAGE_TEST_FILES = {
    "motionbrain_control/test/test_control_guard_logic.cpp",
    "motionbrain_control/test/test_control_guard_lifecycle.py",
    "motionbrain_hardware_interface/test/test_m4_measured_state_launch.py",
    "motionbrain_hardware_interface/test/test_load_motionbrain_hardware_interface.cpp",
    "motionbrain_mission/test/test_mission_flow.py",
    "motionbrain_mission/test/test_mission_supervisor_lifecycle.py",
    "motionbrain_ros_bridge/test/test_fake_endpoint_bridge_integration.py",
    "motionbrain_ros_bridge/test/test_fake_motionbrain_endpoint.py",
    "motionbrain_ros_bridge/test/test_payload_utils.py",
}


def package_xml(package_name):
    return ET.parse(ROS2_SRC / package_name / "package.xml").getroot()


def dependency_names(root, tags):
    return {
        element.text.strip()
        for tag in tags
        for element in root.findall(tag)
        if element.text and element.text.strip()
    }


class Ros2WorkspaceContractTest(unittest.TestCase):
    def test_workspace_package_inventory_is_explicit(self):
        packages = {
            path.name
            for path in ROS2_SRC.iterdir()
            if path.is_dir() and (path / "package.xml").exists()
        }

        self.assertEqual(EXPECTED_PACKAGES, packages)

    def test_bridge_launch_starts_portfolio_nodes(self):
        launch_text = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "launch"
            / "motionbrain_home_wifi.launch.py"
        ).read_text()

        required_fragments = [
            'executable="motionbrain_status_node"',
            'executable="motionbrain_joint_state_node"',
            'executable="motionbrain_kinematics_node"',
            'executable="motionbrain_control_guard_node"',
            'executable="motionbrain_mission_supervisor"',
            '"enable_kinematics"',
            '"kinematics_autostart"',
            '"enable_control_guard"',
            '"control_guard_autostart"',
            '"enable_mission_supervisor"',
            '"mission_supervisor_autostart"',
            '"enable_joint_state_bridge"',
            '"joint_states_topic"',
            '"estimated_joint_states_topic"',
            '"kinematics_joint_states_topic"',
            '"joint_states_output"',
            '"shoulder_feedback_calibration_enabled"',
            '"shoulder_sensor_zero_deg"',
            '"shoulder_direction_sign"',
            '"shoulder_ros_joint_zero_rad"',
            '"perception_url"',
            '"status_autostart"',
            '"joint_state_autostart"',
            '"autostart"',
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, launch_text)

    def test_joint_state_launch_exposes_calibration_and_single_owner_controls(self):
        bridge_launch = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "launch"
            / "motionbrain_home_wifi.launch.py"
        ).read_text()
        display_launch = (
            ROS2_SRC
            / "motionbrain_description"
            / "launch"
            / "display.launch.py"
        ).read_text()
        start_script = (REPO_ROOT / "tools" / "raspi" / "start_ros_bridge.sh").read_text()
        env_example = (
            REPO_ROOT
            / "deploy"
            / "systemd"
            / "motionbrain-ros-bridge.env.example"
        ).read_text()

        required_fragments = [
            "enable_joint_state_bridge",
            "joint_state_autostart",
            "kinematics_autostart",
            "joint_states_topic",
            "estimated_joint_states_topic",
            "kinematics_joint_states_topic",
            "joint_states_output",
            "shoulder_feedback_calibration_enabled",
            "shoulder_sensor_zero_deg",
            "shoulder_direction_sign",
            "shoulder_ros_joint_zero_rad",
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, bridge_launch)

        display_fragments = [
            "joint_states_topic",
            "estimated_joint_states_topic",
            "joint_states_output",
            "shoulder_feedback_calibration_enabled",
            "shoulder_sensor_zero_deg",
            "shoulder_direction_sign",
            "shoulder_ros_joint_zero_rad",
        ]
        for fragment in display_fragments:
            with self.subTest(display_fragment=fragment):
                self.assertIn(fragment, display_launch)
        self.assertIn(
            'remappings=[("joint_states", joint_states_topic)]',
            display_launch,
        )

        required_env = [
            "MOTIONBRAIN_ENABLE_JOINT_STATE_BRIDGE",
            "MOTIONBRAIN_JOINT_STATE_AUTOSTART",
            "MOTIONBRAIN_KINEMATICS_AUTOSTART",
            "MOTIONBRAIN_CONTROL_GUARD_AUTOSTART",
            "MOTIONBRAIN_MISSION_SUPERVISOR_AUTOSTART",
            "MOTIONBRAIN_JOINT_STATES_TOPIC",
            "MOTIONBRAIN_ESTIMATED_JOINT_STATES_TOPIC",
            "MOTIONBRAIN_KINEMATICS_JOINT_STATES_TOPIC",
            "MOTIONBRAIN_JOINT_STATES_OUTPUT",
            "MOTIONBRAIN_SHOULDER_FEEDBACK_CALIBRATION_ENABLED",
            "MOTIONBRAIN_SHOULDER_SENSOR_ZERO_DEG",
            "MOTIONBRAIN_SHOULDER_DIRECTION_SIGN",
            "MOTIONBRAIN_SHOULDER_ROS_JOINT_ZERO_RAD",
        ]
        for fragment in required_env:
            with self.subTest(env_fragment=fragment):
                self.assertIn(fragment, start_script)
                self.assertIn(fragment, env_example)

    def test_m4_measured_ros2_control_is_state_only(self):
        measured_urdf = (
            ROS2_SRC
            / "motionbrain_hardware_interface"
            / "urdf"
            / "motionbrain_m4_measured.urdf"
        ).read_text()
        measured_launch = (
            ROS2_SRC
            / "motionbrain_hardware_interface"
            / "launch"
            / "m4_measured_state.launch.py"
        ).read_text()
        measured_controllers = (
            ROS2_SRC
            / "motionbrain_hardware_interface"
            / "config"
            / "m4_measured_controllers.yaml"
        ).read_text()

        self.assertIn("<param name=\"transport_mode\">m4_state</param>", measured_urdf)
        self.assertIn("<param name=\"status_topic\">/motionbrain/status_typed</param>", measured_urdf)
        self.assertIn("<param name=\"feedback_source\">m4_as5600</param>", measured_urdf)
        self.assertIn("shoulder_feedback_calibration_enabled", measured_urdf)
        self.assertIn("<param name=\"state_stale_timeout_sec\">2.0</param>", measured_urdf)
        self.assertIn("<joint name=\"shoulder_pitch_joint\">", measured_urdf)
        self.assertNotIn("<command_interface", measured_urdf)
        self.assertIn("<state_interface name=\"position\"", measured_urdf)
        self.assertIn("<state_interface name=\"velocity\"", measured_urdf)

        self.assertIn("m4_measured_controllers.yaml", measured_launch)
        self.assertIn("shoulder_sensor_zero_deg", measured_launch)
        self.assertIn('default_value="2.0"', measured_launch)
        self.assertIn("expected status bridge poll interval", measured_launch)
        self.assertIn("joint_state_broadcaster", measured_launch)
        self.assertNotIn("joint_trajectory_controller", measured_launch)
        self.assertNotIn("motionbrain_arm_controller", measured_launch)

        self.assertIn("joint_state_broadcaster", measured_controllers)
        self.assertNotIn("joint_trajectory_controller", measured_controllers)
        self.assertNotIn("motionbrain_arm_controller", measured_controllers)

    def test_package_dependencies_cover_launch_edges(self):
        bridge_deps = dependency_names(
            package_xml("motionbrain_ros_bridge"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("motionbrain_msgs", bridge_deps)
        self.assertIn("motionbrain_mission", bridge_deps)
        self.assertIn("diagnostic_msgs", bridge_deps)
        self.assertIn("launch_ros", bridge_deps)

        mission_deps = dependency_names(
            package_xml("motionbrain_mission"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("motionbrain_msgs", mission_deps)
        self.assertIn("rclpy", mission_deps)
        self.assertIn("std_msgs", mission_deps)

        control_deps = dependency_names(
            package_xml("motionbrain_control"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("motionbrain_msgs", control_deps)
        self.assertIn("rclcpp", control_deps)
        self.assertIn("rclcpp_lifecycle", control_deps)
        self.assertIn("std_msgs", control_deps)

        description_deps = dependency_names(
            package_xml("motionbrain_description"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("motionbrain_ros_bridge", description_deps)
        self.assertIn("robot_state_publisher", description_deps)

        mock_control_deps = dependency_names(
            package_xml("motionbrain_ros2_control_mock"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("controller_manager", mock_control_deps)
        self.assertIn("joint_state_broadcaster", mock_control_deps)
        self.assertIn("joint_trajectory_controller", mock_control_deps)
        self.assertIn("ros2_control", mock_control_deps)

        hardware_test_deps = dependency_names(
            package_xml("motionbrain_hardware_interface"),
            {"test_depend"},
        )
        self.assertIn("ament_cmake_pytest", hardware_test_deps)
        self.assertIn("controller_manager_msgs", hardware_test_deps)
        self.assertIn("rclpy", hardware_test_deps)
        self.assertIn("sensor_msgs", hardware_test_deps)
        self.assertIn("ros2_control_test_assets", mock_control_deps)
        self.assertIn("ros2_controllers", mock_control_deps)
        self.assertIn("ros2controlcli", mock_control_deps)

        hardware_interface_deps = dependency_names(
            package_xml("motionbrain_hardware_interface"),
            {"depend", "exec_depend", "build_depend"},
        )
        self.assertIn("hardware_interface", hardware_interface_deps)
        self.assertIn("pluginlib", hardware_interface_deps)
        self.assertIn("rclcpp", hardware_interface_deps)
        self.assertIn("rclcpp_lifecycle", hardware_interface_deps)
        self.assertIn("controller_manager", hardware_interface_deps)
        self.assertIn("joint_trajectory_controller", hardware_interface_deps)

    def test_motionbrain_message_inventory_is_explicit(self):
        message_files = {
            path.name
            for path in (ROS2_SRC / "motionbrain_msgs" / "msg").iterdir()
            if path.suffix == ".msg"
        }
        self.assertEqual(EXPECTED_MESSAGE_FILES, message_files)

        cmake_text = (ROS2_SRC / "motionbrain_msgs" / "CMakeLists.txt").read_text()
        for message_file in EXPECTED_MESSAGE_FILES:
            with self.subTest(message_file=message_file):
                self.assertIn(f'"msg/{message_file}"', cmake_text)

        lifecycle_text = (
            ROS2_SRC
            / "motionbrain_msgs"
            / "msg"
            / "NodeLifecycleStatus.msg"
        ).read_text()
        expected_lifecycle_fields = [
            "uint8 PRIMARY_STATE_ACTIVE=3",
            "uint8 TRANSITION_STATE_ERRORPROCESSING=15",
            "string node_name",
            "uint8 state_id",
            "string state_label",
            "bool active",
            "bool error",
            "string detail",
            "string raw_json",
        ]
        for field in expected_lifecycle_fields:
            with self.subTest(field=field):
                self.assertIn(field, lifecycle_text)

        motion_status_text = (
            ROS2_SRC
            / "motionbrain_msgs"
            / "msg"
            / "MotionStatus.msg"
        ).read_text()
        expected_shoulder_fields = [
            "bool shoulder_feedback_available",
            "bool shoulder_sensor_connected",
            "bool shoulder_sensor_fresh",
            "bool shoulder_sensor_ready",
            "bool shoulder_magnet_detected",
            "bool shoulder_control_active",
            "bool shoulder_correction_active",
            "bool shoulder_manual_guard_blocked",
            "uint32 shoulder_correction_attempts",
            "uint32 shoulder_max_correction_attempts",
            "uint32 shoulder_sensor_age_ms",
            "uint32 shoulder_agc",
            "uint32 shoulder_magnitude",
            "float32 shoulder_raw_angle_deg",
            "float32 shoulder_angle_deg",
            "float32 shoulder_mount_offset_deg",
            "float32 shoulder_target_deg",
            "float32 shoulder_error_deg",
            "float32 shoulder_target_tolerance_deg",
            "string shoulder_stop_reason",
        ]
        for field in expected_shoulder_fields:
            with self.subTest(field=field):
                self.assertIn(field, motion_status_text)

        routine_status_text = (
            ROS2_SRC
            / "motionbrain_msgs"
            / "msg"
            / "RoutineStatus.msg"
        ).read_text()
        expected_routine_feedback_fields = [
            "string feedback_selected_target",
            "bool feedback_ready",
            "bool physical_routine_execution_allowed",
            "string feedback_block_reason",
            "bool base_yaw_feedback_installed",
            "bool base_yaw_feedback_available",
            "bool base_yaw_feedback_connected",
            "bool base_yaw_feedback_fresh",
            "bool base_yaw_feedback_referenced",
            "bool base_yaw_feedback_faulted",
            "bool base_yaw_feedback_hardware_ready",
            "bool base_yaw_feedback_signal_active",
            "uint32 base_yaw_feedback_pin",
            "bool base_yaw_feedback_active_low",
            "uint32 base_yaw_feedback_age_ms",
            "uint32 base_yaw_feedback_last_update_ms",
            "float32 base_yaw_feedback_position_deg",
            "float32 base_yaw_feedback_velocity_dps",
            "string base_yaw_feedback_stop_reason",
            "string base_yaw_feedback_fault",
        ]
        for field in expected_routine_feedback_fields:
            with self.subTest(field=field):
                self.assertIn(field, routine_status_text)

    def test_motionbrain_service_inventory_is_explicit(self):
        service_files = {
            path.name
            for path in (ROS2_SRC / "motionbrain_msgs" / "srv").iterdir()
            if path.suffix == ".srv"
        }
        self.assertEqual(EXPECTED_SERVICE_FILES, service_files)

        cmake_text = (ROS2_SRC / "motionbrain_msgs" / "CMakeLists.txt").read_text()
        for service_file in EXPECTED_SERVICE_FILES:
            with self.subTest(service_file=service_file):
                self.assertIn(f'"srv/{service_file}"', cmake_text)

        service_text = (
            ROS2_SRC
            / "motionbrain_msgs"
            / "srv"
            / "GuardedRoutineCommand.srv"
        ).read_text()
        expected_fields = [
            "string action",
            "string routine_name",
            "string confirm_code",
            "string raw_json",
            "bool success",
            "bool forwarded",
        ]
        for field in expected_fields:
            with self.subTest(field=field):
                self.assertIn(field, service_text)

    def test_motionbrain_action_inventory_is_explicit(self):
        action_files = {
            path.name
            for path in (ROS2_SRC / "motionbrain_msgs" / "action").iterdir()
            if path.suffix == ".action"
        }
        self.assertEqual(EXPECTED_ACTION_FILES, action_files)

        cmake_text = (ROS2_SRC / "motionbrain_msgs" / "CMakeLists.txt").read_text()
        package_text = (ROS2_SRC / "motionbrain_msgs" / "package.xml").read_text()
        for action_file in EXPECTED_ACTION_FILES:
            with self.subTest(action_file=action_file):
                self.assertIn(f'"action/{action_file}"', cmake_text)

        self.assertIn("find_package(action_msgs REQUIRED)", cmake_text)
        self.assertIn("DEPENDENCIES action_msgs builtin_interfaces", cmake_text)
        self.assertIn("<depend>action_msgs</depend>", package_text)

        action_text = (
            ROS2_SRC
            / "motionbrain_msgs"
            / "action"
            / "GuardedRoutine.action"
        ).read_text()
        expected_fields = [
            "string action",
            "string routine_name",
            "string confirm_code",
            "bool success",
            "bool forwarded",
            "uint32 current_step",
            "uint32 total_steps",
        ]
        for field in expected_fields:
            with self.subTest(field=field):
                self.assertIn(field, action_text)

    def test_health_check_covers_runtime_topics(self):
        script_text = (REPO_ROOT / "tools" / "raspi" / "check_ros_bridge_health.sh").read_text()
        required_block = re.search(r"required_topics=\(\n(?P<body>.*?)\n\)", script_text, re.S)
        self.assertIsNotNone(required_block)

        topics = set(re.findall(r'"([^"]+)"', required_block.group("body")))
        self.assertEqual(EXPECTED_RUNTIME_TOPICS, topics)

        for topic in EXPECTED_RUNTIME_TOPICS:
            with self.subTest(topic=topic):
                self.assertIn(f"OK topic: ${{topic}}", script_text)

        self.assertIn("OK routine diagnostics sample", script_text)
        self.assertIn("OK routine typed feedback readiness sample", script_text)
        self.assertIn(
            'EXPECTED_FEEDBACK_SELECTED_TARGET="${EXPECTED_FEEDBACK_SELECTED_TARGET:-base_yaw_reference}"',
            script_text,
        )
        self.assertIn("feedback_selected_target: ${EXPECTED_FEEDBACK_SELECTED_TARGET}", script_text)
        self.assertIn("feedback_ready: ${EXPECTED_FEEDBACK_READY}", script_text)
        self.assertIn(
            "physical_routine_execution_allowed: ${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}",
            script_text,
        )
        self.assertIn("base_yaw_feedback_fault: ${EXPECTED_BASE_YAW_FEEDBACK_FAULT}", script_text)
        self.assertIn("EXPECTED_BASE_YAW_FEEDBACK_FAULT", script_text)
        self.assertIn("base_yaw_feedback_hardware_ready", script_text)
        self.assertIn("base_yaw_feedback_signal_active", script_text)
        self.assertIn("base_yaw_feedback_pin: ${EXPECTED_BASE_YAW_FEEDBACK_PIN}", script_text)
        self.assertIn("base_yaw_feedback_active_low", script_text)
        self.assertIn("expected_lifecycle_nodes=(", script_text)
        self.assertIn("motionbrain_status_node", script_text)
        self.assertIn("motionbrain_joint_state_node", script_text)
        self.assertIn("motionbrain_kinematics_node", script_text)
        self.assertIn("motionbrain_control_guard_node", script_text)
        self.assertIn("motionbrain_mission_supervisor", script_text)
        self.assertIn("OK lifecycle active samples", script_text)
        self.assertIn("ros2 lifecycle get", script_text)
        self.assertIn("OK lifecycle get active", script_text)
        self.assertIn("check_topic_publisher_count", script_text)
        self.assertIn("ros2 topic info --verbose", script_text)
        self.assertIn("Publisher count:", script_text)
        self.assertIn("EXPECTED_JOINT_STATES_PUBLISHERS", script_text)
        self.assertIn("EXPECTED_ESTIMATED_JOINT_STATES_PUBLISHERS", script_text)
        self.assertIn('echo "OK ${label} publisher count', script_text)
        self.assertIn('"/joint_states"', script_text)
        self.assertIn('"/motionbrain/estimated_joint_states"', script_text)
        self.assertIn("OK diagnostics sample", script_text)
        self.assertIn("check_diagnostic_max_level", script_text)
        self.assertIn("diagnostic_level_number", script_text)
        self.assertIn("run_diagnostics_checks", script_text)
        self.assertIn("FAIL diagnostics did not reach expected levels before timeout", script_text)
        self.assertIn("OK diagnostic level", script_text)
        self.assertIn("EXPECTED_CONTROLLER_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("EXPECTED_ROUTINE_EXECUTOR_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("EXPECTED_TELEOP_SENSOR_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("EXPECTED_CAMERA_PERCEPTION_DIAGNOSTIC_MAX_LEVEL", script_text)
        self.assertIn("motionbrain/controller", script_text)
        self.assertIn("motionbrain/shoulder_feedback", script_text)
        self.assertIn("motionbrain/routine_executor", script_text)
        self.assertIn("motionbrain/feedback", script_text)
        self.assertIn("motionbrain/teleop_sensor", script_text)
        self.assertIn("motionbrain/camera_perception", script_text)
        self.assertIn("base_yaw_fault", script_text)
        self.assertIn("/motionbrain/routine_command", script_text)
        self.assertIn("motionbrain_msgs/srv/GuardedRoutineCommand", script_text)
        self.assertIn("success[:=][[:space:]]*(true|True)", script_text)
        self.assertIn("OK routine command service status sample", script_text)
        self.assertIn("CHECK_ROUTINE_RUN_REJECTION", script_text)
        self.assertIn("routine_execute_disabled_by_bridge_policy", script_text)
        self.assertIn("OK routine command service run rejection sample", script_text)
        self.assertIn("/motionbrain/guarded_routine", script_text)
        self.assertIn("motionbrain_msgs/action/GuardedRoutine", script_text)
        self.assertIn("OK guarded routine action status sample", script_text)
        self.assertIn("OK guarded routine action run rejection sample", script_text)

    def test_status_bridge_publishes_read_only_routine_diagnostics(self):
        bridge_text = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "motionbrain_ros_bridge"
            / "motionbrain_status_node.py"
        ).read_text()
        evidence_text = (REPO_ROOT / "tools" / "raspi" / "capture_ros2_evidence.sh").read_text()
        base_yaw_evidence_text = (
            REPO_ROOT / "tools" / "raspi" / "capture_base_yaw_reference_evidence.sh"
        ).read_text()

        self.assertIn('self.create_publisher(String, "/motionbrain/routine", 10)', bridge_text)
        self.assertIn(
            'self.create_publisher(RoutineStatus, "/motionbrain/routine_typed", 10)',
            bridge_text,
        )
        self.assertIn('fetch_json(f"{self.motion_base_url}/routine", timeout)', bridge_text)
        self.assertIn("self.publish_routine_typed(routine)", bridge_text)
        self.assertIn('capture_topic "/motionbrain/routine"', evidence_text)
        self.assertIn('capture_topic "/motionbrain/routine_typed"', evidence_text)
        self.assertIn("capture_feedback_readiness", evidence_text)
        self.assertIn("Read-only feedback readiness capture", evidence_text)
        self.assertIn("physical_routine_execution_allowed: ${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}", evidence_text)
        self.assertIn("base_yaw_feedback_fault: ${EXPECTED_BASE_YAW_FEEDBACK_FAULT}", evidence_text)
        self.assertIn("EXPECTED_BASE_YAW_FEEDBACK_FAULT", evidence_text)
        self.assertIn("base_yaw_feedback_hardware_ready", evidence_text)
        self.assertIn("base_yaw_feedback_signal_active", evidence_text)
        self.assertIn("base_yaw_feedback_pin: ${EXPECTED_BASE_YAW_FEEDBACK_PIN}", evidence_text)
        self.assertIn("base_yaw_feedback_active_low", evidence_text)
        self.assertIn('capture_topic "/motionbrain/lifecycle_typed"', evidence_text)
        self.assertIn('capture_topic "/motionbrain/lifecycle"', evidence_text)
        self.assertIn('capture_topic "/motionbrain/diagnostics"', evidence_text)
        self.assertIn("ESP32 status", base_yaw_evidence_text)
        self.assertIn("ESP32 routine", base_yaw_evidence_text)
        self.assertIn("Dashboard status", base_yaw_evidence_text)
        self.assertIn("Validate ${label}", base_yaw_evidence_text)
        self.assertIn("ROS2 bridge health with base yaw expectations", base_yaw_evidence_text)
        self.assertIn("Full ROS2 read-only evidence with base yaw expectations", base_yaw_evidence_text)
        self.assertIn("EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY", base_yaw_evidence_text)
        self.assertIn("EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE", base_yaw_evidence_text)
        self.assertIn("EXPECTED_BASE_YAW_FEEDBACK_REFERENCED", base_yaw_evidence_text)

    def test_status_bridge_publishes_standard_diagnostics(self):
        bridge_text = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "motionbrain_ros_bridge"
            / "motionbrain_status_node.py"
        ).read_text()

        expected_fragments = [
            "from diagnostic_msgs.msg import DiagnosticArray",
            "from diagnostic_msgs.msg import DiagnosticStatus",
            '"/motionbrain/diagnostics"',
            "publish_diagnostics(status_payload, routine_payload, detection_payload)",
            "motionbrain/controller",
            "motionbrain/shoulder_feedback",
            "motionbrain/routine_executor",
            "motionbrain/feedback",
            "motionbrain/teleop_sensor",
            "motionbrain/camera_perception",
            "queue_apply_allowed",
            "routine executor disabled by policy",
            "feedback not ready for physical routines",
            "feedback_ready",
            "base_yaw_fault",
            "base_yaw_hardware_ready",
            "base_yaw_signal_active",
            "base_yaw_pin",
            "base_yaw_active_low",
            "M4 shoulder feedback ready",
            "shoulder_feedback_available",
            "shoulder_sensor_ready",
            "shoulder_angle_deg",
            "shoulder_correction_active",
            "shoulder_correction_attempts",
            "M4 shoulder target missed",
            "target_tolerance_deg",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, bridge_text)

    def test_bridge_package_installs_fake_fault_injection_endpoint(self):
        package_dir = ROS2_SRC / "motionbrain_ros_bridge"
        setup_text = (package_dir / "setup.py").read_text()
        launch_text = (
            package_dir
            / "launch"
            / "motionbrain_fake_fault_injection.launch.py"
        ).read_text()
        fake_endpoint_text = (
            package_dir
            / "motionbrain_ros_bridge"
            / "fake_motionbrain_endpoint.py"
        ).read_text()

        expected_fragments = [
            "motionbrain_fake_endpoint",
            "motionbrain_ros_bridge.fake_motionbrain_endpoint:main",
        ]
        for fragment in expected_fragments:
            with self.subTest(setup_fragment=fragment):
                self.assertIn(fragment, setup_text)

        expected_scenarios = [
            '"ready"',
            '"controller_fault"',
            '"malformed_status"',
            '"policy_mismatch"',
            '"stale_detection"',
            '"stale_shoulder"',
            '"timeout_status"',
        ]
        for fragment in expected_scenarios:
            with self.subTest(scenario=fragment):
                self.assertIn(fragment, fake_endpoint_text)

        self.assertIn("fake_endpoint_read_only", fake_endpoint_text)
        self.assertIn("never forwards physical routine commands", fake_endpoint_text)
        self.assertIn('executable="motionbrain_fake_endpoint"', launch_text)
        self.assertIn('executable="motionbrain_status_node"', launch_text)
        self.assertIn('"scenario"', launch_text)
        self.assertIn('"status_autostart"', launch_text)
        self.assertIn('"perception_url"', launch_text)
        self.assertIn('"http_token": ""', launch_text)
        self.assertIn('"autostart"', launch_text)

    def test_portfolio_nodes_publish_lifecycle_status(self):
        bridge_dir = ROS2_SRC / "motionbrain_ros_bridge" / "motionbrain_ros_bridge"
        status_text = (bridge_dir / "motionbrain_status_node.py").read_text()
        joint_state_text = (bridge_dir / "motionbrain_joint_state_node.py").read_text()
        kinematics_text = (bridge_dir / "motionbrain_kinematics_node.py").read_text()
        mission_text = (
            ROS2_SRC
            / "motionbrain_mission"
            / "motionbrain_mission"
            / "mission_supervisor_node.py"
        ).read_text()
        guard_text = (
            ROS2_SRC
            / "motionbrain_control"
            / "src"
            / "control_guard_node.cpp"
        ).read_text()

        status_lifecycle_fragments = [
            "from rclpy.lifecycle import LifecycleNode",
            "from rclpy.lifecycle import TransitionCallbackReturn",
            "class MotionBrainStatusNode(LifecycleNode)",
            "def on_configure(",
            "def on_activate(",
            "def on_deactivate(",
            "def on_cleanup(",
            "def on_shutdown(",
            "self.trigger_configure()",
            "self.trigger_activate()",
            "if not self._polling_active:",
            "return TransitionCallbackReturn.SUCCESS",
        ]
        for fragment in status_lifecycle_fragments:
            with self.subTest(status_lifecycle_fragment=fragment):
                self.assertIn(fragment, status_text)

        joint_state_lifecycle_fragments = [
            "from rclpy.lifecycle import LifecycleNode",
            "from rclpy.lifecycle import TransitionCallbackReturn",
            "class MotionBrainJointStateNode(LifecycleNode)",
            "def on_configure(",
            "def on_activate(",
            "def on_deactivate(",
            "def on_cleanup(",
            "def on_shutdown(",
            "self.trigger_configure()",
            "self.trigger_activate()",
            "if self.estimated_publisher is None:",
            "return TransitionCallbackReturn.SUCCESS",
        ]
        for fragment in joint_state_lifecycle_fragments:
            with self.subTest(joint_state_lifecycle_fragment=fragment):
                self.assertIn(fragment, joint_state_text)

        kinematics_lifecycle_fragments = [
            "from rclpy.lifecycle import LifecycleNode",
            "from rclpy.lifecycle import TransitionCallbackReturn",
            "class MotionBrainKinematicsNode(LifecycleNode)",
            "def on_configure(",
            "def on_activate(",
            "def on_deactivate(",
            "def on_cleanup(",
            "def on_shutdown(",
            "self.trigger_configure()",
            "self.trigger_activate()",
            "if not self._processing_active:",
            "return TransitionCallbackReturn.SUCCESS",
        ]
        for fragment in kinematics_lifecycle_fragments:
            with self.subTest(kinematics_lifecycle_fragment=fragment):
                self.assertIn(fragment, kinematics_text)

        for text in [status_text, joint_state_text, kinematics_text]:
            with self.subTest(node="python_bridge_node"):
                self.assertIn("LifecycleStatusPublisher", text)
                self.assertIn("mark_active", text)

        self.assertIn("NodeLifecycleStatus", mission_text)
        self.assertIn('"/motionbrain/lifecycle_typed"', mission_text)
        self.assertIn('"/motionbrain/lifecycle"', mission_text)
        self.assertIn("PRIMARY_STATE_ACTIVE", mission_text)
        mission_lifecycle_fragments = [
            "from rclpy.lifecycle import LifecycleNode",
            "from rclpy.lifecycle import TransitionCallbackReturn",
            "class MotionBrainMissionSupervisor(LifecycleNode)",
            "def on_configure(",
            "def on_activate(",
            "def on_deactivate(",
            "def on_cleanup(",
            "def on_shutdown(",
            "self.trigger_configure()",
            "self.trigger_activate()",
            "if not self._processing_active:",
            "return TransitionCallbackReturn.SUCCESS",
        ]
        for fragment in mission_lifecycle_fragments:
            with self.subTest(mission_lifecycle_fragment=fragment):
                self.assertIn(fragment, mission_text)

        guard_lifecycle_fragments = [
            "rclcpp_lifecycle/lifecycle_node.hpp",
            "class MotionBrainControlGuardNode : public rclcpp_lifecycle::LifecycleNode",
            "CallbackReturn on_configure(",
            "CallbackReturn on_activate(",
            "CallbackReturn on_deactivate(",
            "CallbackReturn on_cleanup(",
            "CallbackReturn on_shutdown(",
            "configure();",
            "activate();",
            "if (!processing_active_",
            "rclcpp_lifecycle::LifecycleNode::on_activate",
            "rclcpp_lifecycle::LifecycleNode::on_deactivate",
            '"/motionbrain/lifecycle_typed"',
            '"/motionbrain/lifecycle"',
            "PRIMARY_STATE_ACTIVE",
        ]
        for fragment in guard_lifecycle_fragments:
            with self.subTest(guard_lifecycle_fragment=fragment):
                self.assertIn(fragment, guard_text)

    def test_status_bridge_exposes_non_motion_routine_command_boundary(self):
        bridge_text = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "motionbrain_ros_bridge"
            / "motionbrain_status_node.py"
        ).read_text()
        evidence_text = (REPO_ROOT / "tools" / "raspi" / "capture_ros2_evidence.sh").read_text()

        expected_fragments = [
            "from motionbrain_msgs.action import GuardedRoutine",
            "from rclpy.action import ActionServer",
            '"/motionbrain/routine_cmd"',
            '"/motionbrain/routine_cmd_typed"',
            'self.create_publisher(String, "/motionbrain/routine_result", 10)',
            '"/motionbrain/routine_result_typed"',
            "self.create_service(",
            "GuardedRoutineCommand",
            '"/motionbrain/routine_command"',
            "ActionServer(",
            '"/motionbrain/guarded_routine"',
            "execute_routine_goal",
            "execute_routine_action",
            "routine_execute_disabled_by_bridge_policy",
            "ROS2 routine bridge forwards only status, dry_run, and abort",
            "post_motionbrain(self.motion_base_url, path, timeout, token)",
            'fetch_json(f"{self.motion_base_url}/routine", timeout)',
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, bridge_text)

        expected_evidence_fragments = [
            "CAPTURE_ROUTINE_COMMAND_BOUNDARY",
            "Routine command status result",
            "Routine command run rejection result",
            "CAPTURE_ROUTINE_SERVICE_BOUNDARY",
            "Routine command service status result",
            "Routine command service run rejection result",
            "motionbrain_msgs/srv/GuardedRoutineCommand",
            "CAPTURE_ROUTINE_ACTION_BOUNDARY",
            "Guarded routine action status result",
            "Guarded routine action run rejection result",
            "motionbrain_msgs/action/GuardedRoutine",
            "success=True",
            "forwarded=True",
            "success: true",
            "action: status",
            "forwarded: true",
            "success=False",
            "forwarded=False",
            "success: false",
            "{action: status}",
            "{action: run, routine_name: inspect, confirm_code: confirm-inspect}",
            "routine_execute_disabled_by_bridge_policy",
            "forwarded: false",
        ]
        for fragment in expected_evidence_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, evidence_text)

    def test_evidence_helper_has_read_only_rosbag_capture_option(self):
        evidence_text = (REPO_ROOT / "tools" / "raspi" / "capture_ros2_evidence.sh").read_text()
        rosbag_block = re.search(
            r"capture_rosbag\(\) \{\n(?P<body>.*?)\n\}",
            evidence_text,
            re.S,
        )
        self.assertIsNotNone(rosbag_block)
        rosbag_body = rosbag_block.group("body")

        required_fragments = [
            'CAPTURE_ROSBAG="${CAPTURE_ROSBAG:-0}"',
            'ROSBAG_DURATION_SECONDS="${ROSBAG_DURATION_SECONDS:-10}"',
            'ROSBAG_OUTPUT="${MOTIONBRAIN_ROSBAG_OUTPUT:-/tmp/motionbrain_ros2_bag_${STAMP}}"',
            'section "ROS2 bag read-only capture"',
            "timeout --signal=SIGINT",
            "ros2 bag record",
            "metadata.yaml",
            'echo "ROS2 bag: ${ROSBAG_OUTPUT}"',
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, evidence_text)
        self.assertIn("JointState topic publisher ownership", evidence_text)
        self.assertIn("Estimated JointState topic publisher ownership", evidence_text)
        self.assertIn("EXPECTED_JOINT_STATES_PUBLISHERS", evidence_text)
        self.assertIn("EXPECTED_ESTIMATED_JOINT_STATES_PUBLISHERS", evidence_text)

        read_only_topics = {
            "/motionbrain/status_typed",
            "/motionbrain/routine_typed",
            "/motionbrain/diagnostics",
            "/motionbrain/events_typed",
            "/camera/detection_typed",
            "/joint_states",
            "/motionbrain/end_effector_pose",
            "/motionbrain/kinematics_typed",
            "/motionbrain/control_guard_typed",
            "/motionbrain/mission_state_typed",
        }
        for topic in read_only_topics:
            with self.subTest(topic=topic):
                self.assertIn(f'"{topic}"', rosbag_body)

        self.assertNotIn("/motionbrain/light_cmd", rosbag_body)
        self.assertNotIn("/motionbrain/routine_cmd", rosbag_body)
        self.assertNotIn("CAPTURE_MISSION_BOUNDARY", rosbag_body)
        self.assertNotIn("CAPTURE_ROUTINE_COMMAND_BOUNDARY", rosbag_body)

    def test_mission_config_matches_supervisor_topic_contract(self):
        config_text = (
            ROS2_SRC
            / "motionbrain_mission"
            / "config"
            / "mission_home_wifi.yaml"
        ).read_text()

        expected_pairs = {
            "control_guard_topic": "/motionbrain/control_guard_typed",
            "control_guard_json_topic": "/motionbrain/control_guard",
            "detection_topic": "/camera/detection_typed",
            "status_topic": "/motionbrain/status_typed",
            "mission_cmd_topic": "/motionbrain/mission_cmd_typed",
            "mission_cmd_json_topic": "/motionbrain/mission_cmd",
            "mission_state_topic": "/motionbrain/mission_state_typed",
            "mission_state_json_topic": "/motionbrain/mission_state",
            "light_cmd_topic": "/motionbrain/light_cmd_typed",
            "act_action": "toggle",
        }
        for key, value in expected_pairs.items():
            with self.subTest(key=key):
                self.assertRegex(config_text, rf"\b{key}:\s+{re.escape(value)}\b")

    def test_ros2_workspace_has_real_package_level_tests(self):
        for relative_path in EXPECTED_PACKAGE_TEST_FILES:
            with self.subTest(path=relative_path):
                self.assertTrue((ROS2_SRC / relative_path).exists())

        control_cmake = (ROS2_SRC / "motionbrain_control" / "CMakeLists.txt").read_text()
        self.assertIn("ament_add_gtest(test_control_guard_logic", control_cmake)

        hardware_cmake = (
            ROS2_SRC / "motionbrain_hardware_interface" / "CMakeLists.txt"
        ).read_text()
        self.assertIn("ament_add_pytest_test(", hardware_cmake)
        self.assertIn("test_m4_measured_state_launch", hardware_cmake)

        workflow_text = (REPO_ROOT / ".github" / "workflows" / "ros2.yml").read_text()
        self.assertNotIn("No package-level colcon test suites were produced yet", workflow_text)
        self.assertIn("colcon test-result --verbose", workflow_text)

    def test_ros2_control_mock_demo_is_optional_and_non_physical(self):
        package_dir = ROS2_SRC / "motionbrain_ros2_control_mock"
        self.assertTrue((package_dir / "package.xml").exists())
        self.assertTrue((package_dir / "launch" / "mock_control.launch.py").exists())
        self.assertTrue((package_dir / "config" / "controllers.yaml").exists())
        self.assertTrue((package_dir / "urdf" / "motionbrain_mock_control.urdf").exists())

        urdf_text = (package_dir / "urdf" / "motionbrain_mock_control.urdf").read_text()
        config_text = (package_dir / "config" / "controllers.yaml").read_text()
        launch_text = (package_dir / "launch" / "mock_control.launch.py").read_text()

        expected_joints = [
            "base_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "gripper_joint",
        ]
        for joint in expected_joints:
            with self.subTest(joint=joint):
                self.assertIn(f'joint name="{joint}"', urdf_text)
                self.assertIn(f"- {joint}", config_text)

        self.assertIn("<ros2_control", urdf_text)
        self.assertIn("mock_components/GenericSystem", urdf_text)
        self.assertIn("joint_trajectory_controller/JointTrajectoryController", config_text)
        self.assertIn("joint_state_broadcaster/JointStateBroadcaster", config_text)
        self.assertIn('package="controller_manager"', launch_text)
        self.assertIn('executable="ros2_control_node"', launch_text)
        self.assertIn('name="controller_manager"', launch_text)
        self.assertIn("motionbrain_arm_controller", launch_text)

    def test_ros2_control_mock_evidence_helper_is_mock_only(self):
        helper_text = (
            REPO_ROOT / "tools" / "raspi" / "capture_ros2_control_mock_evidence.sh"
        ).read_text()

        expected_fragments = [
            'MOCK_ROS_DOMAIN_ID="${MOCK_ROS_DOMAIN_ID:-42}"',
            "CAPTURE_MOCK_TRAJECTORY",
            "MOTIONBRAIN_ROS2_CONTROL_OVERLAY_PREFIX",
            "motionbrain_ros2_control_mock",
            "ros2_control_test_assets",
            "ros2controlcli",
            "ros2 launch motionbrain_ros2_control_mock mock_control.launch.py",
            "ros2 control list_controllers",
            "ros2 control list_hardware_interfaces",
            "timeout \"${SAMPLE_TIMEOUT_SECONDS}\" ros2 topic echo /joint_states --once",
            "ros-jazzy-ros2-control-test-assets",
            "ros-jazzy-ros2controlcli",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, helper_text)

    def test_ros2_control_hardware_evidence_helper_is_dry_run_only(self):
        helper_text = (
            REPO_ROOT / "tools" / "raspi" / "capture_ros2_control_hardware_evidence.sh"
        ).read_text()

        expected_fragments = [
            'HARDWARE_ROS_DOMAIN_ID="${HARDWARE_ROS_DOMAIN_ID:-43}"',
            "Physical actuation: disabled",
            "transport_mode is dry_run",
            "motionbrain_hardware_interface",
            "ros2 launch motionbrain_hardware_interface hardware_interface.launch.py",
            "ros2 control list_controllers",
            "ros2 control list_hardware_interfaces",
            "/motionbrain_arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            "ros2 topic echo /joint_states --once",
            "Stop hardware-interface launch",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, helper_text)

    def test_ros2_control_hardware_interface_scaffold_is_safe_and_read_only_capable(self):
        package_dir = ROS2_SRC / "motionbrain_hardware_interface"
        self.assertTrue((package_dir / "package.xml").exists())
        self.assertTrue((package_dir / "CMakeLists.txt").exists())
        self.assertTrue((package_dir / "README.md").exists())
        self.assertTrue((package_dir / "motionbrain_hardware_interface.xml").exists())
        self.assertTrue((package_dir / "launch" / "hardware_interface.launch.py").exists())
        self.assertTrue((package_dir / "config" / "controllers.yaml").exists())
        self.assertTrue((package_dir / "urdf" / "motionbrain_hardware_interface.urdf").exists())

        header_text = (
            package_dir
            / "include"
            / "motionbrain_hardware_interface"
            / "motionbrain_hardware_interface.hpp"
        ).read_text()
        source_text = (
            package_dir
            / "src"
            / "motionbrain_hardware_interface.cpp"
        ).read_text()
        plugin_text = (package_dir / "motionbrain_hardware_interface.xml").read_text()
        urdf_text = (
            package_dir
            / "urdf"
            / "motionbrain_hardware_interface.urdf"
        ).read_text()
        config_text = (package_dir / "config" / "controllers.yaml").read_text()
        launch_text = (
            package_dir
            / "launch"
            / "hardware_interface.launch.py"
        ).read_text()
        cmake_text = (package_dir / "CMakeLists.txt").read_text()

        expected_fragments = [
            "hardware_interface::SystemInterface",
            "MotionBrainHardwareInterface",
            "on_init",
            "export_state_interfaces",
            "export_command_interfaces",
            "on_activate",
            "on_cleanup",
            "on_shutdown",
            "on_error",
            "read(",
            "write(",
            "command_timeout_sec_",
            "last_command_change_time_",
            "max_state_step_rad_",
            "status_subscription_",
            "shoulder_feedback_calibration_enabled_",
            "state_stale_timeout_sec_",
            "handle_motion_status",
            "Physical actuation remains behind the firmware SafetyGate",
            "PLUGINLIB_EXPORT_CLASS",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, header_text + source_text)

        self.assertIn("pluginlib_export_plugin_description_file", cmake_text)
        self.assertIn("motionbrain_hardware_interface/MotionBrainHardwareInterface", plugin_text)
        self.assertIn("open-loop dry-run mode", plugin_text)
        self.assertIn("read-only M4 measured-state mode", plugin_text)
        self.assertIn("<ros2_control", urdf_text)
        self.assertIn("motionbrain_hardware_interface/MotionBrainHardwareInterface", urdf_text)
        self.assertIn('<param name="transport_mode">dry_run</param>', urdf_text)
        self.assertIn("joint_trajectory_controller/JointTrajectoryController", config_text)
        self.assertIn("interpolate_from_desired_state: true", config_text)
        self.assertIn('package="controller_manager"', launch_text)
        self.assertIn('get_package_share_directory("motionbrain_hardware_interface")', launch_text)
        self.assertIn("pluginlib::ClassLoader", (
            package_dir / "test" / "test_load_motionbrain_hardware_interface.cpp"
        ).read_text())


if __name__ == "__main__":
    unittest.main()
