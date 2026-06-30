from __future__ import annotations

import time
import unittest

from motionbrain_ros_bridge.fake_motionbrain_endpoint import make_server

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from diagnostic_msgs.msg import DiagnosticStatus
    from motionbrain_msgs.msg import CameraDetection
    from motionbrain_msgs.msg import MotionStatus
    from motionbrain_msgs.msg import RoutineStatus
    from motionbrain_ros_bridge.motionbrain_status_node import MotionBrainStatusNode
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.parameter import Parameter
except ImportError as exc:  # pragma: no cover - host-only fallback
    ROS_IMPORT_ERROR = exc
    ROS_AVAILABLE = False
else:
    ROS_IMPORT_ERROR = None
    ROS_AVAILABLE = True


class FakeEndpointServer:
    def __init__(self, scenario: str) -> None:
        self.server = make_server(
            "127.0.0.1",
            0,
            scenario=scenario,
            delay_sec=0.2,
            quiet=True,
        )

    def __enter__(self) -> "FakeEndpointServer":
        import threading

        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    @property
    def host(self) -> str:
        return str(self.server.server_address[0])

    @property
    def port(self) -> int:
        return int(self.server.server_address[1])

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"


@unittest.skipUnless(
    ROS_AVAILABLE,
    f"ROS2 Python runtime is unavailable: {ROS_IMPORT_ERROR}",
)
class FakeEndpointBridgeIntegrationTest(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init(args=None)

    def tearDown(self) -> None:
        rclpy.shutdown()

    def wait_for_subscriber_discovery(
        self,
        bridge,
        collector,
        expected_topics: list[str],
    ) -> None:
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(bridge, timeout_sec=0.0)
            rclpy.spin_once(collector, timeout_sec=0.05)
            if all(bridge.count_subscribers(topic) > 0 for topic in expected_topics):
                return

    def wait_for_bridge_messages(
        self,
        scenario: str,
        bridge,
        collector,
        messages: dict[str, list],
        timeout_sec: float = 3.0,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(bridge, timeout_sec=0.0)
            rclpy.spin_once(collector, timeout_sec=0.05)
            if messages["routine"] and messages["diagnostics"]:
                if scenario in {"malformed_status", "timeout_status"} and messages["detection"]:
                    return
                if messages["status"] and messages["detection"]:
                    return

    def set_fake_endpoint_parameters(
        self,
        bridge,
        server: FakeEndpointServer,
    ) -> None:
        bridge.set_parameters(
            [
                Parameter("motion_host", Parameter.Type.STRING, server.host),
                Parameter("motion_port", Parameter.Type.INTEGER, server.port),
                Parameter("perception_url", Parameter.Type.STRING, server.base_url),
                Parameter("camera_url", Parameter.Type.STRING, ""),
                Parameter("http_timeout", Parameter.Type.DOUBLE, 0.1),
                Parameter("events_limit", Parameter.Type.INTEGER, 1),
            ]
        )

    def run_bridge_poll(self, scenario: str) -> dict[str, list]:
        with FakeEndpointServer(scenario) as server:
            bridge = MotionBrainStatusNode(autostart=False)
            self.set_fake_endpoint_parameters(bridge, server)
            self.assertEqual(TransitionCallbackReturn.SUCCESS, bridge.trigger_configure())

            collector = rclpy.create_node(f"fake_endpoint_bridge_{scenario}_collector")
            messages = {
                "status": [],
                "routine": [],
                "detection": [],
                "diagnostics": [],
            }
            collector.create_subscription(
                MotionStatus,
                "/motionbrain/status_typed",
                messages["status"].append,
                10,
            )
            collector.create_subscription(
                RoutineStatus,
                "/motionbrain/routine_typed",
                messages["routine"].append,
                10,
            )
            collector.create_subscription(
                CameraDetection,
                "/camera/detection_typed",
                messages["detection"].append,
                10,
            )
            collector.create_subscription(
                DiagnosticArray,
                "/motionbrain/diagnostics",
                messages["diagnostics"].append,
                10,
            )

            expected_topics = [
                "/motionbrain/status_typed",
                "/motionbrain/routine_typed",
                "/camera/detection_typed",
                "/motionbrain/diagnostics",
            ]
            self.wait_for_subscriber_discovery(bridge, collector, expected_topics)

            self.assertEqual(TransitionCallbackReturn.SUCCESS, bridge.trigger_activate())
            bridge.poll_once()
            self.wait_for_bridge_messages(scenario, bridge, collector, messages)

            collector.destroy_node()
            bridge.destroy_node()
            return messages

    def diagnostic_by_name(self, diagnostics: DiagnosticArray, name: str) -> DiagnosticStatus:
        for status in diagnostics.status:
            if status.name == name:
                return status
        self.fail(f"missing diagnostic status {name}")

    def diagnostic_value(self, diagnostic: DiagnosticStatus, key: str) -> str:
        for value in diagnostic.values:
            if value.key == key:
                return value.value
        self.fail(f"missing diagnostic value {key} in {diagnostic.name}")

    def test_ready_fake_endpoint_publishes_typed_bridge_outputs(self) -> None:
        messages = self.run_bridge_poll("ready")

        self.assertEqual("IDLE", messages["status"][-1].state)
        self.assertTrue(messages["status"][-1].shoulder_sensor_ready)
        self.assertTrue(messages["routine"][-1].dry_run_only)
        self.assertFalse(messages["routine"][-1].physical_routine_execution_allowed)
        self.assertTrue(messages["detection"][-1].available)
        self.assertTrue(messages["detection"][-1].detected)

        diagnostics = messages["diagnostics"][-1]
        self.assertEqual(
            DiagnosticStatus.OK,
            self.diagnostic_by_name(diagnostics, "motionbrain/controller").level,
        )
        self.assertEqual(
            DiagnosticStatus.OK,
            self.diagnostic_by_name(diagnostics, "motionbrain/shoulder_feedback").level,
        )

    def test_stale_shoulder_fault_is_visible_in_status_and_diagnostics(self) -> None:
        messages = self.run_bridge_poll("stale_shoulder")

        self.assertFalse(messages["status"][-1].shoulder_sensor_fresh)
        self.assertFalse(messages["status"][-1].shoulder_sensor_ready)
        shoulder = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/shoulder_feedback",
        )
        self.assertEqual(DiagnosticStatus.ERROR, shoulder.level)
        self.assertEqual("M4 shoulder sensor not ready", shoulder.message)

    def test_controller_fault_is_visible_in_status_and_diagnostics(self) -> None:
        messages = self.run_bridge_poll("controller_fault")

        self.assertEqual("FAULT", messages["status"][-1].state)
        self.assertTrue(messages["status"][-1].faulted)
        self.assertFalse(messages["status"][-1].moving)
        controller = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/controller",
        )
        self.assertEqual(DiagnosticStatus.ERROR, controller.level)
        self.assertEqual("controller fault latched", controller.message)
        self.assertEqual(
            "FAKE_CONTROLLER_FAULT",
            self.diagnostic_value(controller, "fault_reason"),
        )
        self.assertEqual(
            "clear_fault",
            self.diagnostic_value(controller, "recovery_action"),
        )

    def test_policy_mismatch_fault_is_visible_in_routine_and_diagnostics(self) -> None:
        messages = self.run_bridge_poll("policy_mismatch")

        self.assertTrue(messages["routine"][-1].physical_routine_execution_allowed)
        self.assertFalse(messages["routine"][-1].feedback_ready)
        feedback = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/feedback",
        )
        self.assertEqual(DiagnosticStatus.ERROR, feedback.level)
        self.assertEqual("feedback policy mismatch", feedback.message)

    def test_stale_detection_fault_is_visible_in_detection_and_diagnostics(self) -> None:
        messages = self.run_bridge_poll("stale_detection")

        self.assertFalse(messages["detection"][-1].available)
        self.assertFalse(messages["detection"][-1].detected)
        self.assertEqual("LOST", messages["detection"][-1].alignment)
        self.assertEqual(
            "fault injection: stale detection",
            messages["detection"][-1].reason,
        )
        camera = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/camera_perception",
        )
        self.assertEqual(DiagnosticStatus.WARN, camera.level)
        self.assertEqual("camera detection unavailable", camera.message)
        self.assertEqual(
            "fault injection: stale detection",
            self.diagnostic_value(camera, "reason"),
        )

    def test_malformed_status_keeps_routine_and_diagnostics_available(self) -> None:
        messages = self.run_bridge_poll("malformed_status")

        self.assertEqual([], messages["status"])
        self.assertTrue(messages["routine"][-1].dry_run_only)
        controller = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/controller",
        )
        self.assertEqual(DiagnosticStatus.ERROR, controller.level)
        self.assertEqual("status poll unavailable", controller.message)

    def test_timeout_status_keeps_routine_detection_and_reports_controller_unavailable(
        self,
    ) -> None:
        messages = self.run_bridge_poll("timeout_status")

        self.assertEqual([], messages["status"])
        self.assertTrue(messages["routine"][-1].dry_run_only)
        self.assertTrue(messages["detection"][-1].available)
        controller = self.diagnostic_by_name(
            messages["diagnostics"][-1],
            "motionbrain/controller",
        )
        self.assertEqual(DiagnosticStatus.ERROR, controller.level)
        self.assertEqual("status poll unavailable", controller.message)

    def test_inactive_lifecycle_bridge_does_not_publish_poll_outputs(self) -> None:
        with FakeEndpointServer("ready") as server:
            bridge = MotionBrainStatusNode(autostart=False)
            self.set_fake_endpoint_parameters(bridge, server)
            self.assertEqual(TransitionCallbackReturn.SUCCESS, bridge.trigger_configure())

            collector = rclpy.create_node("fake_endpoint_bridge_inactive_collector")
            messages = {
                "status": [],
                "routine": [],
                "detection": [],
                "diagnostics": [],
            }
            collector.create_subscription(
                MotionStatus,
                "/motionbrain/status_typed",
                messages["status"].append,
                10,
            )
            collector.create_subscription(
                RoutineStatus,
                "/motionbrain/routine_typed",
                messages["routine"].append,
                10,
            )
            collector.create_subscription(
                CameraDetection,
                "/camera/detection_typed",
                messages["detection"].append,
                10,
            )
            collector.create_subscription(
                DiagnosticArray,
                "/motionbrain/diagnostics",
                messages["diagnostics"].append,
                10,
            )

            expected_topics = [
                "/motionbrain/status_typed",
                "/motionbrain/routine_typed",
                "/camera/detection_typed",
                "/motionbrain/diagnostics",
            ]
            self.wait_for_subscriber_discovery(bridge, collector, expected_topics)

            bridge.poll_once()
            inactive_deadline = time.monotonic() + 0.5
            while time.monotonic() < inactive_deadline:
                rclpy.spin_once(bridge, timeout_sec=0.0)
                rclpy.spin_once(collector, timeout_sec=0.05)

            self.assertEqual([], messages["status"])
            self.assertEqual([], messages["routine"])
            self.assertEqual([], messages["detection"])
            self.assertEqual([], messages["diagnostics"])

            self.assertEqual(TransitionCallbackReturn.SUCCESS, bridge.trigger_activate())
            bridge.poll_once()
            self.wait_for_bridge_messages("ready", bridge, collector, messages)
            self.assertTrue(messages["status"])
            self.assertTrue(messages["routine"])
            self.assertTrue(messages["detection"])
            self.assertTrue(messages["diagnostics"])

            collector.destroy_node()
            bridge.destroy_node()


if __name__ == "__main__":
    unittest.main()
