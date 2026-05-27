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
}

EXPECTED_RUNTIME_TOPICS = {
    "/motionbrain/status_typed",
    "/camera/detection_typed",
    "/joint_states",
    "/motionbrain/end_effector_pose",
    "/motionbrain/kinematics",
    "/motionbrain/control_guard",
    "/motionbrain/mission_state",
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

    def test_health_check_covers_runtime_topics(self):
        script_text = (REPO_ROOT / "tools" / "raspi" / "check_ros_bridge_health.sh").read_text()
        required_block = re.search(r"required_topics=\(\n(?P<body>.*?)\n\)", script_text, re.S)
        self.assertIsNotNone(required_block)

        topics = set(re.findall(r'"([^"]+)"', required_block.group("body")))
        self.assertEqual(EXPECTED_RUNTIME_TOPICS, topics)

        for topic in EXPECTED_RUNTIME_TOPICS:
            with self.subTest(topic=topic):
                self.assertIn(f"OK topic: ${{topic}}", script_text)

    def test_mission_config_matches_supervisor_topic_contract(self):
        config_text = (
            ROS2_SRC
            / "motionbrain_mission"
            / "config"
            / "mission_home_wifi.yaml"
        ).read_text()

        expected_pairs = {
            "control_guard_topic": "/motionbrain/control_guard",
            "detection_topic": "/camera/detection_typed",
            "status_topic": "/motionbrain/status_typed",
            "mission_cmd_topic": "/motionbrain/mission_cmd",
            "mission_state_topic": "/motionbrain/mission_state",
            "light_cmd_topic": "/motionbrain/light_cmd_typed",
            "act_action": "toggle",
        }
        for key, value in expected_pairs.items():
            with self.subTest(key=key):
                self.assertRegex(config_text, rf"\b{key}:\s+{re.escape(value)}\b")


if __name__ == "__main__":
    unittest.main()
