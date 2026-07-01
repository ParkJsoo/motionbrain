from __future__ import annotations

import math
import time
import unittest

try:
    import rclpy
    from motionbrain_msgs.msg import MotionStatus
    from motionbrain_ros_bridge.motionbrain_joint_state_node import MotionBrainJointStateNode
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
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
class JointStateLifecycleIntegrationTest(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init(args=None)

    def tearDown(self) -> None:
        rclpy.shutdown()

    def spin_pair(self, joint_state_node, collector, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(joint_state_node, timeout_sec=0.0)
            rclpy.spin_once(collector, timeout_sec=0.02)

    def wait_for_messages(
        self,
        joint_state_node,
        collector,
        messages: dict[str, list[JointState]],
        timeout_sec: float = 1.5,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(joint_state_node, timeout_sec=0.0)
            rclpy.spin_once(collector, timeout_sec=0.05)
            if messages["estimated"] and messages["selected"]:
                return

    def wait_for_publisher_count(
        self,
        observer,
        topic: str,
        expected_count: int,
        nodes: list,
        timeout_sec: float = 2.0,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
            if observer.count_publishers(topic) == expected_count:
                return
            time.sleep(0.01)
        self.fail(
            f"expected {expected_count} publishers for {topic}, "
            f"got {observer.count_publishers(topic)}"
        )

    def test_inactive_joint_state_bridge_does_not_publish_until_activated(self) -> None:
        joint_state_node = MotionBrainJointStateNode(autostart=False)
        joint_state_node.set_parameters(
            [
                Parameter("source_topic", Parameter.Type.STRING, "/test/status_typed"),
                Parameter("joint_states_topic", Parameter.Type.STRING, "/test/joint_states"),
                Parameter(
                    "estimated_joint_states_topic",
                    Parameter.Type.STRING,
                    "/test/estimated_joint_states",
                ),
                Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 20.0),
            ]
        )
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            joint_state_node.trigger_configure(),
        )

        collector = rclpy.create_node("joint_state_lifecycle_collector")
        messages: dict[str, list[JointState]] = {
            "estimated": [],
            "selected": [],
        }
        collector.create_subscription(
            JointState,
            "/test/estimated_joint_states",
            messages["estimated"].append,
            10,
        )
        collector.create_subscription(
            JointState,
            "/test/joint_states",
            messages["selected"].append,
            10,
        )

        self.spin_pair(joint_state_node, collector, 0.3)
        joint_state_node.publish_joint_states()
        self.spin_pair(joint_state_node, collector, 0.3)
        self.assertEqual([], messages["estimated"])
        self.assertEqual([], messages["selected"])

        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            joint_state_node.trigger_activate(),
        )
        self.wait_for_messages(joint_state_node, collector, messages)
        self.assertTrue(messages["estimated"])
        self.assertTrue(messages["selected"])
        self.assertEqual(
            [
                "base_yaw_joint",
                "shoulder_pitch_joint",
                "elbow_pitch_joint",
                "wrist_pitch_joint",
                "gripper_joint",
            ],
            list(messages["estimated"][-1].name),
        )

        messages["estimated"].clear()
        messages["selected"].clear()
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            joint_state_node.trigger_deactivate(),
        )
        joint_state_node.publish_joint_states()
        self.spin_pair(joint_state_node, collector, 0.3)
        self.assertEqual([], messages["estimated"])
        self.assertEqual([], messages["selected"])

        collector.destroy_node()
        joint_state_node.destroy_node()

    def test_joint_state_output_modes_keep_selected_topic_single_owner(self) -> None:
        cases = [
            ("estimated", "/test/joint_states_estimated", 1, 1),
            ("measured", "/test/joint_states_measured", 1, 1),
            ("none", "/test/joint_states_none", 1, 0),
        ]
        for output_mode, selected_topic, estimated_count, selected_count in cases:
            with self.subTest(output_mode=output_mode):
                joint_state_node = MotionBrainJointStateNode(autostart=False)
                joint_state_node.set_parameters(
                    [
                        Parameter(
                            "source_topic",
                            Parameter.Type.STRING,
                            f"/test/status_typed_{output_mode}",
                        ),
                        Parameter(
                            "joint_states_topic",
                            Parameter.Type.STRING,
                            selected_topic,
                        ),
                        Parameter(
                            "estimated_joint_states_topic",
                            Parameter.Type.STRING,
                            f"/test/estimated_joint_states_{output_mode}",
                        ),
                        Parameter(
                            "joint_states_output",
                            Parameter.Type.STRING,
                            output_mode,
                        ),
                    ]
                )
                self.assertEqual(
                    TransitionCallbackReturn.SUCCESS,
                    joint_state_node.trigger_configure(),
                )
                observer = rclpy.create_node(f"joint_state_owner_observer_{output_mode}")
                nodes = [joint_state_node, observer]
                self.wait_for_publisher_count(
                    observer,
                    f"/test/estimated_joint_states_{output_mode}",
                    estimated_count,
                    nodes,
                )
                self.wait_for_publisher_count(
                    observer,
                    selected_topic,
                    selected_count,
                    nodes,
                )

                observer.destroy_node()
                joint_state_node.destroy_node()

    def test_estimated_output_reuses_publisher_when_topics_match(self) -> None:
        topic = "/test/shared_joint_states"
        joint_state_node = MotionBrainJointStateNode(autostart=False)
        joint_state_node.set_parameters(
            [
                Parameter("joint_states_topic", Parameter.Type.STRING, topic),
                Parameter("estimated_joint_states_topic", Parameter.Type.STRING, topic),
                Parameter("joint_states_output", Parameter.Type.STRING, "estimated"),
            ]
        )
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            joint_state_node.trigger_configure(),
        )
        observer = rclpy.create_node("joint_state_shared_owner_observer")
        nodes = [joint_state_node, observer]
        self.wait_for_publisher_count(observer, topic, 1, nodes)

        observer.destroy_node()
        joint_state_node.destroy_node()

    def test_measured_output_rejects_shared_estimated_topic(self) -> None:
        topic = "/test/conflicting_joint_states"
        joint_state_node = MotionBrainJointStateNode(autostart=False)
        joint_state_node.set_parameters(
            [
                Parameter("joint_states_topic", Parameter.Type.STRING, topic),
                Parameter("estimated_joint_states_topic", Parameter.Type.STRING, topic),
                Parameter("joint_states_output", Parameter.Type.STRING, "measured"),
            ]
        )
        self.assertEqual(
            TransitionCallbackReturn.FAILURE,
            joint_state_node.trigger_configure(),
        )

        joint_state_node.destroy_node()

    def test_estimated_output_keeps_finite_fallback_when_m4_is_uncalibrated(self) -> None:
        joint_state_node = MotionBrainJointStateNode(autostart=False)
        status = MotionStatus()
        status.base_angle_deg = 10.0
        status.tilt_angle_deg = 30.0
        status.pan_angle_deg = -5.0
        status.wrist_angle_deg = 2.0
        status.gripper_angle_deg = 1.0
        status.shoulder_feedback_available = True
        status.shoulder_sensor_connected = True
        status.shoulder_sensor_fresh = True
        status.shoulder_sensor_ready = True
        status.shoulder_angle_deg = 244.0

        joint_state_node.handle_status(status)

        self.assertTrue(all(math.isfinite(value) for value in joint_state_node.estimated_positions))
        self.assertAlmostEqual(math.radians(30.0), joint_state_node.estimated_positions[1])
        self.assertTrue(math.isnan(joint_state_node.measured_shoulder_position))
        joint_state_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
