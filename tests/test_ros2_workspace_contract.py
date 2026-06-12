import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC = REPO_ROOT / "ros2_ws" / "src"

EXPECTED_PACKAGES = {
    "motionbrain_msgs",
    "motionbrain_control",
    "motionbrain_mission",
    "motionbrain_ros_bridge",
    "motionbrain_description",
    "motionbrain_ros2_control_mock",
}

EXPECTED_RUNTIME_TOPICS = {
    "/motionbrain/status_typed",
    "/motionbrain/routine",
    "/motionbrain/routine_typed",
    "/motionbrain/diagnostics",
    "/camera/detection_typed",
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
    "motionbrain_mission/test/test_mission_flow.py",
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
            '"enable_control_guard"',
            '"enable_mission_supervisor"',
            '"perception_url"',
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, launch_text)

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
        self.assertIn("ros2_control_test_assets", mock_control_deps)
        self.assertIn("ros2_controllers", mock_control_deps)
        self.assertIn("ros2controlcli", mock_control_deps)

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
        self.assertIn("OK routine typed diagnostics sample", script_text)
        self.assertIn("OK diagnostics sample", script_text)
        self.assertIn("motionbrain/controller", script_text)
        self.assertIn("motionbrain/routine_executor", script_text)
        self.assertIn("/motionbrain/routine_command", script_text)
        self.assertIn("motionbrain_msgs/srv/GuardedRoutineCommand", script_text)
        self.assertIn("success[:=][[:space:]]*(true|True)", script_text)
        self.assertIn("OK routine command service status sample", script_text)
        self.assertIn("/motionbrain/guarded_routine", script_text)
        self.assertIn("motionbrain_msgs/action/GuardedRoutine", script_text)
        self.assertIn("OK guarded routine action status sample", script_text)

    def test_status_bridge_publishes_read_only_routine_diagnostics(self):
        bridge_text = (
            ROS2_SRC
            / "motionbrain_ros_bridge"
            / "motionbrain_ros_bridge"
            / "motionbrain_status_node.py"
        ).read_text()
        evidence_text = (REPO_ROOT / "tools" / "raspi" / "capture_ros2_evidence.sh").read_text()

        self.assertIn('self.create_publisher(String, "/motionbrain/routine", 10)', bridge_text)
        self.assertIn(
            'self.create_publisher(RoutineStatus, "/motionbrain/routine_typed", 10)',
            bridge_text,
        )
        self.assertIn('fetch_json(f"{self.motion_base_url}/routine", timeout)', bridge_text)
        self.assertIn("self.publish_routine_typed(routine)", bridge_text)
        self.assertIn('capture_topic "/motionbrain/routine"', evidence_text)
        self.assertIn('capture_topic "/motionbrain/routine_typed"', evidence_text)
        self.assertIn('capture_topic "/motionbrain/diagnostics"', evidence_text)

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
            "motionbrain/routine_executor",
            "motionbrain/teleop_sensor",
            "motionbrain/camera_perception",
            "queue_apply_allowed",
            "routine executor disabled by policy",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, bridge_text)

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
        readme_text = (package_dir / "README.md").read_text()

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
        self.assertIn("does not connect to the ESP32 controller", readme_text)
        self.assertIn("capture_ros2_control_mock_evidence.sh", readme_text)
        self.assertIn("It is not a", readme_text)
        self.assertIn("physical hardware interface", readme_text)

    def test_ros2_control_mock_evidence_helper_is_mock_only(self):
        helper_text = (
            REPO_ROOT / "tools" / "raspi" / "capture_ros2_control_mock_evidence.sh"
        ).read_text()
        bringup_text = (REPO_ROOT / "docs" / "RASPBERRY_PI_ROS2_BRINGUP.md").read_text()

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

        self.assertIn("capture_ros2_control_mock_evidence.sh", bringup_text)
        self.assertIn("separate `ROS_DOMAIN_ID`", bringup_text)


if __name__ == "__main__":
    unittest.main()
