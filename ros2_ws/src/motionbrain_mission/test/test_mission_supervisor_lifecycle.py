from __future__ import annotations

import time
import unittest

try:
    import rclpy
    from motionbrain_mission.mission_supervisor_node import MotionBrainMissionSupervisor
    from motionbrain_msgs.msg import CameraDetection
    from motionbrain_msgs.msg import ControlGuard
    from motionbrain_msgs.msg import LightCommand
    from motionbrain_msgs.msg import MissionCommand
    from motionbrain_msgs.msg import MissionState
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.parameter import Parameter
except ImportError as exc:  # pragma: no cover - host-only fallback
    ROS_IMPORT_ERROR = exc
    ROS_AVAILABLE = False
else:
    ROS_IMPORT_ERROR = None
    ROS_AVAILABLE = True


@unittest.skipUnless(
    ROS_AVAILABLE,
    f"ROS2 Python runtime is unavailable: {ROS_IMPORT_ERROR}",
)
class MissionSupervisorLifecycleIntegrationTest(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init(args=None)

    def tearDown(self) -> None:
        rclpy.shutdown()

    def spin_nodes(self, nodes: list, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)

    def wait_for_subscription(self, publisher_node, topic: str, nodes: list) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.spin_nodes(nodes, 0.05)
            if publisher_node.count_subscribers(topic) > 0:
                return
        self.fail(f"missing subscriber for {topic}")

    def publish_guard(self, publisher) -> None:
        message = ControlGuard()
        message.ready = True
        message.reason = "ready"
        message.suggested_action = "none"
        message.status_fresh = True
        message.detection_fresh = True
        publisher.publish(message)

    def publish_detection(self, publisher) -> None:
        message = CameraDetection()
        message.available = True
        message.detected = True
        message.alignment = "CENTER"
        message.command_suggestion = "hold"
        message.area_ratio = 0.08
        publisher.publish(message)

    def publish_command(self, publisher, command: str) -> None:
        message = MissionCommand()
        message.command = command
        publisher.publish(message)

    def wait_for_initial_state(self, nodes: list, messages: list[MissionState]) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.spin_nodes(nodes, 0.05)
            if messages:
                return
        self.fail("mission state was not published")

    def publish_until_light_command(
        self,
        guard_pub,
        detection_pub,
        command_pub,
        nodes: list,
        messages: list[LightCommand],
    ) -> None:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.publish_guard(guard_pub)
            self.spin_nodes(nodes, 0.05)
            self.publish_command(command_pub, "start")
            self.spin_nodes(nodes, 0.05)
            self.publish_detection(detection_pub)
            self.spin_nodes(nodes, 0.05)
            self.publish_command(command_pub, "confirm")
            self.spin_nodes(nodes, 0.1)
            if messages:
                return
        self.fail("light command was not published")

    def test_inactive_mission_supervisor_does_not_publish_until_activated(self) -> None:
        mission_node = MotionBrainMissionSupervisor(autostart=False)
        mission_node.set_parameters(
            [
                Parameter(
                    "control_guard_topic",
                    Parameter.Type.STRING,
                    "/test/mission_control_guard_typed",
                ),
                Parameter(
                    "control_guard_json_topic",
                    Parameter.Type.STRING,
                    "/test/mission_control_guard",
                ),
                Parameter(
                    "detection_topic",
                    Parameter.Type.STRING,
                    "/test/mission_detection_typed",
                ),
                Parameter(
                    "mission_cmd_topic",
                    Parameter.Type.STRING,
                    "/test/mission_cmd_typed",
                ),
                Parameter(
                    "mission_cmd_json_topic",
                    Parameter.Type.STRING,
                    "/test/mission_cmd",
                ),
                Parameter(
                    "mission_state_topic",
                    Parameter.Type.STRING,
                    "/test/mission_state_typed",
                ),
                Parameter(
                    "mission_state_json_topic",
                    Parameter.Type.STRING,
                    "/test/mission_state",
                ),
                Parameter(
                    "light_cmd_topic",
                    Parameter.Type.STRING,
                    "/test/light_cmd_typed",
                ),
                Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 20.0),
            ]
        )
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            mission_node.trigger_configure(),
        )

        io_node = rclpy.create_node("mission_supervisor_lifecycle_io")
        guard_pub = io_node.create_publisher(
            ControlGuard,
            "/test/mission_control_guard_typed",
            10,
        )
        detection_pub = io_node.create_publisher(
            CameraDetection,
            "/test/mission_detection_typed",
            10,
        )
        command_pub = io_node.create_publisher(
            MissionCommand,
            "/test/mission_cmd_typed",
            10,
        )
        state_messages: list[MissionState] = []
        light_messages: list[LightCommand] = []
        io_node.create_subscription(
            MissionState,
            "/test/mission_state_typed",
            state_messages.append,
            10,
        )
        io_node.create_subscription(
            LightCommand,
            "/test/light_cmd_typed",
            light_messages.append,
            10,
        )

        nodes = [mission_node, io_node]
        self.wait_for_subscription(io_node, "/test/mission_control_guard_typed", nodes)
        self.wait_for_subscription(io_node, "/test/mission_detection_typed", nodes)
        self.wait_for_subscription(io_node, "/test/mission_cmd_typed", nodes)

        self.publish_guard(guard_pub)
        self.publish_detection(detection_pub)
        self.publish_command(command_pub, "start")
        self.publish_command(command_pub, "confirm")
        mission_node.publish_state()
        self.spin_nodes(nodes, 0.4)
        self.assertEqual([], state_messages)
        self.assertEqual([], light_messages)

        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            mission_node.trigger_activate(),
        )
        self.wait_for_initial_state(nodes, state_messages)
        self.publish_until_light_command(
            guard_pub,
            detection_pub,
            command_pub,
            nodes,
            light_messages,
        )
        self.assertEqual("toggle", light_messages[-1].action)

        state_messages.clear()
        light_messages.clear()
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            mission_node.trigger_deactivate(),
        )
        self.spin_nodes(nodes, 0.2)
        state_messages.clear()
        light_messages.clear()
        self.publish_guard(guard_pub)
        self.publish_detection(detection_pub)
        self.publish_command(command_pub, "start")
        self.publish_command(command_pub, "confirm")
        mission_node.publish_state()
        self.spin_nodes(nodes, 0.4)
        self.assertEqual([], state_messages)
        self.assertEqual([], light_messages)

        io_node.destroy_node()
        mission_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
