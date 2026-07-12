from __future__ import annotations

import json
import urllib.request

import rclpy
from motionbrain_msgs.msg import M4WriteProposal
from motionbrain_msgs.srv import M4WriteConfirm
from rclpy.node import Node

from motionbrain_ros_bridge.m4_write_executor_core import M4ContractError
from motionbrain_ros_bridge.m4_write_executor_core import M4WriteExecutorCore


def request_json(url: str, *, timeout: float, token: str = "", method: str = "GET") -> dict:
    headers = {"Accept": "application/json"}
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
        self.declare_parameter("http_token", "")
        self.declare_parameter("http_timeout_sec", 3.0)
        self.declare_parameter("proposal_ttl_sec", 20.0)
        self.core = M4WriteExecutorCore(float(self.get_parameter("proposal_ttl_sec").value))
        self.create_subscription(
            M4WriteProposal, "/motionbrain/m4_write_proposal", self.handle_proposal, 10
        )
        self.create_service(M4WriteConfirm, "/motionbrain/m4_write_confirm", self.handle_confirm)

    def handle_proposal(self, message: M4WriteProposal) -> None:
        try:
            self.core.accept_proposal(
                {
                    "commandId": message.command_id,
                    "joint": message.joint,
                    "targetPositionRad": message.target_position_rad,
                    "timeoutMs": message.timeout_ms,
                    "forwarded": message.forwarded,
                    "operatorConfirmationRequired": message.operator_confirmation_required,
                }
            )
            self.get_logger().info(f"M4 proposal pending confirmation: {message.command_id}")
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
        response.stamp = self.get_clock().now().to_msg()
        response.success = result["success"]
        response.forwarded = result["forwarded"]
        response.command_id = result["commandId"]
        response.reason = result["reason"]
        response.message = result["message"]
        response.raw_json = result["rawJson"]
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M4WriteExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
