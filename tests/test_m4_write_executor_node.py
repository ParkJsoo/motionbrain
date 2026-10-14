"""Exercise the real ROS adapter with a fake Node; no ROS daemon or HTTP server."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tests.test_m4_write_executor_core import proposal
from tests.test_m4_write_contract import armed_status


NODE_SOURCE = (Path(__file__).resolve().parents[1] /
               "ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/m4_write_executor_node.py")


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.services = {}
        self.logger = Mock()

    def declare_parameter(self, name, value):
        self.parameters[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, *args):
        pass

    def create_service(self, service_type, name, callback):
        self.services[name] = callback

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "fake-stamp"))

    def destroy_node(self):
        return True


def load_adapter():
    modules = {name: ModuleType(name) for name in (
        "rclpy", "rclpy.node", "motionbrain_msgs", "motionbrain_msgs.msg", "motionbrain_msgs.srv")}
    modules["rclpy.node"].Node = FakeNode
    modules["motionbrain_msgs.msg"].M4WriteProposal = object
    modules["motionbrain_msgs.srv"].M4WriteConfirm = object
    spec = importlib.util.spec_from_file_location("m4_node_adapter_under_test", NODE_SOURCE)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class M4WriteExecutorNodeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        env = patch.dict(os.environ, {"XDG_STATE_HOME": self.temp.name})
        env.start()
        self.addCleanup(env.stop)
        FakeNode.overrides = {}
        self.adapter = load_adapter()

    def node(self):
        node = self.adapter.M4WriteExecutorNode()
        self.addCleanup(node.destroy_node)
        return node

    @staticmethod
    def message():
        request = proposal()
        return SimpleNamespace(
            command_id=request["commandId"], joint=request["joint"],
            target_position_rad=request["targetPositionRad"], timeout_ms=request["timeoutMs"],
            forwarded=False, operator_confirmation_required=True,
            created_at=SimpleNamespace(sec=123, nanosec=456),
            schema_version="motionbrain.m4_write_proposal.v1")

    def test_default_is_durable_and_restart_does_not_reactivate_pending(self):
        node = self.node()
        node.handle_proposal(self.message())
        expected = Path(self.temp.name) / "motionbrain/m4_commands.sqlite3"
        self.assertTrue(expected.is_file())
        node.destroy_node()
        restarted = self.node()
        restarted.handle_proposal(self.message())
        with patch.object(self.adapter, "request_json", side_effect=AssertionError("unexpected HTTP")):
            response = restarted.handle_confirm(SimpleNamespace(command_id="proposal-1"), SimpleNamespace())
        self.assertEqual("canceled", json.loads(response.raw_json)["executionState"])

    def test_result_service_preserves_source_metadata_and_is_read_only(self):
        node = self.node()
        node.handle_proposal(self.message())
        with patch.object(self.adapter, "request_json", side_effect=AssertionError("unexpected HTTP")):
            callback = node.services["/motionbrain/m4_write_result"]
            result = callback(SimpleNamespace(command_id="proposal-1"), SimpleNamespace())
        metadata = json.loads(result.raw_json)["proposalMetadata"]
        self.assertEqual({"sec": 123, "nanosec": 456}, metadata["createdAt"])
        self.assertEqual("ros", metadata["createdClockDomain"])
        self.assertEqual("not_sent", json.loads(result.raw_json)["executionState"])

    def test_missing_result_is_unknown_not_proof_of_non_execution(self):
        node = self.node()
        with patch.object(self.adapter, "request_json", side_effect=AssertionError("unexpected HTTP")):
            result = node.handle_result(SimpleNamespace(command_id="absent"), SimpleNamespace())
        self.assertEqual("result_not_found", result.reason)
        self.assertEqual("outcome_unknown", json.loads(result.raw_json)["executionState"])
        self.assertIsNone(json.loads(result.raw_json)["dispatchAttempted"])

    def test_lost_post_response_exposes_unknown_and_replay_never_posts(self):
        node = self.node()
        node.handle_proposal(self.message())
        calls = []
        def fake_http(url, **kwargs):
            calls.append((url, kwargs))
            if url.endswith("/status"):
                return armed_status()
            raise TimeoutError("response lost after device receipt")
        with patch.object(self.adapter, "request_json", side_effect=fake_http):
            result = node.handle_confirm(SimpleNamespace(command_id="proposal-1"), SimpleNamespace())
            node.handle_proposal(self.message())
            replay = node.handle_confirm(SimpleNamespace(command_id="proposal-1"), SimpleNamespace())
            queried = node.handle_result(SimpleNamespace(command_id="proposal-1"), SimpleNamespace())
        self.assertEqual(2, len(calls))  # One status GET and one POST attempt.
        self.assertEqual("POST", calls[1][1]["method"])
        for response in (result, replay, queried):
            self.assertFalse(response.success)
            self.assertTrue(response.forwarded)
            self.assertEqual("outcome_unknown", json.loads(response.raw_json)["executionState"])

    def test_physical_node_rejects_ephemeral_journal(self):
        for value in (":memory:", "", " ", "relative.sqlite3"):
            with self.subTest(path=value):
                FakeNode.overrides = {"command_journal_path": value}
                with self.assertRaisesRegex(ValueError, "persistent command journal"):
                    self.adapter.M4WriteExecutorNode()

    def test_empty_or_relative_state_home_uses_home_not_working_directory(self):
        for value in ("", "relative"):
            with self.subTest(state_home=value), patch.dict(os.environ, {"XDG_STATE_HOME": value}), \
                    patch.object(self.adapter.Path, "home", return_value=Path(self.temp.name)):
                node = self.node()
                self.assertEqual(
                    str(Path(self.temp.name) / ".local/state/motionbrain/m4_commands.sqlite3"),
                    node.get_parameter("command_journal_path").value)
                node.destroy_node()

    def test_ros_entity_failure_releases_journal_lock(self):
        with patch.object(FakeNode, "create_service", side_effect=RuntimeError("ROS entity failure")):
            with self.assertRaisesRegex(RuntimeError, "ROS entity failure"):
                self.adapter.M4WriteExecutorNode()
        # Another initialization can acquire the exact same journal immediately.
        self.node()

    def test_main_shuts_down_context_when_node_initialization_fails(self):
        fake_ros = SimpleNamespace(init=Mock(), spin=Mock(), shutdown=Mock())
        with patch.object(self.adapter, "rclpy", fake_ros), \
                patch.object(self.adapter, "M4WriteExecutorNode", side_effect=RuntimeError("init failure")):
            with self.assertRaisesRegex(RuntimeError, "init failure"):
                self.adapter.main()
        fake_ros.spin.assert_not_called()
        fake_ros.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
