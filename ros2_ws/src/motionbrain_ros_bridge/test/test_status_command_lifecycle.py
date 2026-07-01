from __future__ import annotations

import unittest
from unittest.mock import patch

try:
    import rclpy
    from motionbrain_msgs.action import GuardedRoutine
    from motionbrain_msgs.srv import GuardedRoutineCommand
    from motionbrain_ros_bridge.motionbrain_status_node import MotionBrainStatusNode
    from rclpy.lifecycle import TransitionCallbackReturn
except ImportError as exc:  # pragma: no cover - host-only fallback
    ROS_IMPORT_ERROR = exc
    ROS_AVAILABLE = False
else:
    ROS_IMPORT_ERROR = None
    ROS_AVAILABLE = True


class FakeGoalHandle:
    def __init__(self) -> None:
        self.request = GuardedRoutine.Goal()
        self.feedback = []
        self.aborted = False
        self.succeeded = False

    def publish_feedback(self, feedback) -> None:
        self.feedback.append(feedback)

    def abort(self) -> None:
        self.aborted = True

    def succeed(self) -> None:
        self.succeeded = True


@unittest.skipUnless(
    ROS_AVAILABLE,
    f"ROS2 Python runtime is unavailable: {ROS_IMPORT_ERROR}",
)
class StatusCommandLifecycleTest(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init(args=None)

    def tearDown(self) -> None:
        rclpy.shutdown()

    def make_configured_inactive_bridge(self) -> MotionBrainStatusNode:
        bridge = MotionBrainStatusNode(autostart=False)
        self.assertEqual(TransitionCallbackReturn.SUCCESS, bridge.trigger_configure())
        return bridge

    def test_inactive_service_returns_not_forwarded_without_http(self) -> None:
        bridge = self.make_configured_inactive_bridge()
        request = GuardedRoutineCommand.Request()
        request.action = "status"
        response = GuardedRoutineCommand.Response()

        with patch(
            "motionbrain_ros_bridge.motionbrain_status_node.fetch_json",
        ) as fetch_json, patch(
            "motionbrain_ros_bridge.motionbrain_status_node.post_motionbrain",
        ) as post_motionbrain:
            result = bridge.handle_routine_service(request, response)

        self.assertFalse(result.success)
        self.assertFalse(result.forwarded)
        self.assertEqual("bridge_inactive", result.error)
        self.assertEqual("bridge_inactive", result.result)
        fetch_json.assert_not_called()
        post_motionbrain.assert_not_called()
        bridge.destroy_node()

    def test_inactive_action_returns_not_forwarded_without_http(self) -> None:
        bridge = self.make_configured_inactive_bridge()
        goal_handle = FakeGoalHandle()
        goal_handle.request.action = "dry_run"
        goal_handle.request.routine_name = "inspect"

        with patch(
            "motionbrain_ros_bridge.motionbrain_status_node.fetch_json",
        ) as fetch_json, patch(
            "motionbrain_ros_bridge.motionbrain_status_node.post_motionbrain",
        ) as post_motionbrain:
            result = bridge.execute_routine_goal(goal_handle)

        self.assertFalse(result.success)
        self.assertFalse(result.forwarded)
        self.assertEqual("bridge_inactive", result.error)
        self.assertTrue(goal_handle.aborted)
        self.assertFalse(goal_handle.succeeded)
        self.assertEqual("rejected", goal_handle.feedback[-1].state)
        fetch_json.assert_not_called()
        post_motionbrain.assert_not_called()
        bridge.destroy_node()

    def test_inactive_light_command_does_not_post_http(self) -> None:
        bridge = self.make_configured_inactive_bridge()

        with patch(
            "motionbrain_ros_bridge.motionbrain_status_node.post_motionbrain",
        ) as post_motionbrain:
            bridge.handle_light_action("toggle", "toggle")

        post_motionbrain.assert_not_called()
        bridge.destroy_node()


if __name__ == "__main__":
    unittest.main()
