from __future__ import annotations

import json
import os
from pathlib import Path
import urllib.request

import rclpy
from motionbrain_msgs.msg import M4WriteProposal
from motionbrain_msgs.srv import M4WriteConfirm
from rclpy.node import Node

from motionbrain_ros_bridge.m4_write_executor_core import M4ContractError
from motionbrain_ros_bridge.m4_write_executor_core import M4WriteExecutorCore


def request_json(url: str, *, timeout: float, token: str = "", method: str = "GET") -> dict:
    headers = {"Accept": "application/json", "X-MotionBrain": "1"}
    if token:
        headers["X-MotionBrain-Token"] = token
    request = urllib.request.Request(url, headers=headers, method=method)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("response_not_object")
    return payload


class M4WriteExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_m4_write_executor")
        self.declare_parameter("controller_url", "http://motionbrain.local")
        self.declare_parameter("http_token", os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""))
        self.declare_parameter("http_timeout_sec", 3.0)
        self.declare_parameter("proposal_ttl_sec", 20.0)
        state_home = Path(os.environ.get("XDG_STATE_HOME", ""))
        if not state_home.is_absolute():
            state_home = Path.home() / ".local" / "state"
        self.declare_parameter("command_journal_path", str(state_home / "motionbrain" / "m4_commands.sqlite3"))
        self.declare_parameter("command_journal_max_records", 10000)
        journal_path = str(self.get_parameter("command_journal_path").value)
        if not journal_path.strip() or not Path(journal_path).expanduser().is_absolute():
            raise ValueError("physical executor requires a persistent command journal at an absolute path")
        self.core = M4WriteExecutorCore(
            float(self.get_parameter("proposal_ttl_sec").value),
            journal_path=journal_path,
            max_records=int(self.get_parameter("command_journal_max_records").value),
        )
        try:
            self.create_subscription(
                M4WriteProposal, "/motionbrain/m4_write_proposal", self.handle_proposal, 10
            )
            self.create_service(M4WriteConfirm, "/motionbrain/m4_write_confirm", self.handle_confirm)
            # Same wire shape, separate read-only endpoint: never fetches or POSTs.
            self.create_service(M4WriteConfirm, "/motionbrain/m4_write_result", self.handle_result)
        except Exception:
            self.core.close()
            super().destroy_node()
            raise

    def handle_proposal(self, message: M4WriteProposal) -> None:
        try:
            result = self.core.accept_proposal(
                {
                    "commandId": message.command_id,
                    "joint": message.joint,
                    "targetPositionRad": message.target_position_rad,
                    "timeoutMs": message.timeout_ms,
                    "forwarded": message.forwarded,
                    "operatorConfirmationRequired": message.operator_confirmation_required,
                    "createdAt": {"sec": message.created_at.sec, "nanosec": message.created_at.nanosec},
                    "createdClockDomain": "ros",
                    "schemaVersion": message.schema_version,
                }
            )
            self.get_logger().info(
                f"M4 proposal {message.command_id}: {result['executionState']} ({result['reason']})"
            )
        except (M4ContractError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected M4 proposal: {exc}")

    def handle_confirm(self, request: M4WriteConfirm.Request, response: M4WriteConfirm.Response):
        base = str(self.get_parameter("controller_url").value).rstrip("/")
        token = str(self.get_parameter("http_token").value)
        timeout = float(self.get_parameter("http_timeout_sec").value)
        result = self.core.confirm(
            request.command_id,
            lambda: request_json(f"{base}/status", timeout=timeout),
            lambda path: request_json(f"{base}{path}", timeout=timeout, token=token, method="POST"),
        )
        return self._fill_response(result, response)

    def handle_result(self, request: M4WriteConfirm.Request, response: M4WriteConfirm.Response):
        result = self.core.get_result(request.command_id)
        if result is None:
            # Absence from this host journal does not prove physical non-execution.
            result = {
                "success": False, "forwarded": False, "commandId": request.command_id,
                "reason": "result_not_found", "message": "result_not_found",
                "executionState": "outcome_unknown", "dispatchAttempted": None,
                "acknowledged": False,
            }
            result["rawJson"] = json.dumps(result, separators=(",", ":"), sort_keys=True)
        return self._fill_response(result, response)

    def _fill_response(self, result: dict, response: M4WriteConfirm.Response):
        response.stamp = self.get_clock().now().to_msg()
        response.success = result["success"]
        response.forwarded = result["forwarded"]
        response.command_id = result["commandId"]
        response.reason = result["reason"]
        response.message = result["message"]
        response.raw_json = result["rawJson"]
        return response

    def destroy_node(self):
        self.core.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = M4WriteExecutorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
