from __future__ import annotations

import time
import unittest

try:
    import rclpy
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


if __name__ == "__main__":
    unittest.main()
