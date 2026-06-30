from __future__ import annotations

import time
import unittest

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from motionbrain_msgs.msg import KinematicsState
    from motionbrain_ros_bridge.motionbrain_kinematics_node import MotionBrainKinematicsNode
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
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
class KinematicsLifecycleIntegrationTest(unittest.TestCase):
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

    def publish_joint_state(self, publisher) -> None:
        message = JointState()
        message.name = [
            "base_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "gripper_joint",
        ]
        message.position = [0.0, 0.1, -0.1, 0.0, 0.0]
        publisher.publish(message)

    def publish_until_outputs(
        self,
        publisher,
        nodes: list,
        messages: dict[str, list],
        timeout_sec: float = 2.0,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.publish_joint_state(publisher)
            self.spin_nodes(nodes, 0.05)
            if messages["pose"] and messages["typed"] and messages["json"]:
                return

    def test_inactive_kinematics_bridge_does_not_publish_until_activated(self) -> None:
        kinematics_node = MotionBrainKinematicsNode(autostart=False)
        kinematics_node.set_parameters(
            [
                Parameter(
                    "joint_states_topic",
                    Parameter.Type.STRING,
                    "/test/kinematics_joint_states",
                ),
                Parameter(
                    "pose_topic",
                    Parameter.Type.STRING,
                    "/test/end_effector_pose",
                ),
                Parameter(
                    "kinematics_topic",
                    Parameter.Type.STRING,
                    "/test/kinematics",
                ),
                Parameter(
                    "kinematics_typed_topic",
                    Parameter.Type.STRING,
                    "/test/kinematics_typed",
                ),
            ]
        )
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            kinematics_node.trigger_configure(),
        )

        io_node = rclpy.create_node("kinematics_lifecycle_io")
        publisher = io_node.create_publisher(JointState, "/test/kinematics_joint_states", 10)
        messages: dict[str, list] = {
            "pose": [],
            "typed": [],
            "json": [],
        }
        io_node.create_subscription(
            PoseStamped,
            "/test/end_effector_pose",
            messages["pose"].append,
            10,
        )
        io_node.create_subscription(
            KinematicsState,
            "/test/kinematics_typed",
            messages["typed"].append,
            10,
        )
        io_node.create_subscription(
            String,
            "/test/kinematics",
            messages["json"].append,
            10,
        )

        nodes = [kinematics_node, io_node]
        self.wait_for_subscription(
            io_node,
            "/test/kinematics_joint_states",
            nodes,
        )

        self.publish_joint_state(publisher)
        self.spin_nodes(nodes, 0.4)
        self.assertEqual([], messages["pose"])
        self.assertEqual([], messages["typed"])
        self.assertEqual([], messages["json"])

        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            kinematics_node.trigger_activate(),
        )
        self.publish_until_outputs(publisher, nodes, messages)
        self.assertTrue(messages["pose"])
        self.assertTrue(messages["typed"])
        self.assertTrue(messages["json"])
        self.assertGreater(messages["typed"][-1].x_m, 0.0)

        messages["pose"].clear()
        messages["typed"].clear()
        messages["json"].clear()
        self.assertEqual(
            TransitionCallbackReturn.SUCCESS,
            kinematics_node.trigger_deactivate(),
        )
        self.publish_joint_state(publisher)
        self.spin_nodes(nodes, 0.4)
        self.assertEqual([], messages["pose"])
        self.assertEqual([], messages["typed"])
        self.assertEqual([], messages["json"])

        io_node.destroy_node()
        kinematics_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
