import json
from typing import Any

from motionbrain_msgs.msg import NodeLifecycleStatus
from rclpy.node import Node
from std_msgs.msg import String


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


class LifecycleStatusPublisher:
    def __init__(
        self,
        node: Node,
        *,
        node_name: str | None = None,
        detail: str = "node starting",
        typed_topic: str = "/motionbrain/lifecycle_typed",
        json_topic: str = "/motionbrain/lifecycle",
        heartbeat_period_sec: float = 5.0,
    ) -> None:
        self.node = node
        self.node_name = node_name or node.get_name()
        self.detail = detail
        self.state_id = NodeLifecycleStatus.TRANSITION_STATE_CONFIGURING
        self.state_label = "configuring"
        self.active = False
        self.error = False
        self.typed_pub = node.create_publisher(NodeLifecycleStatus, typed_topic, 10)
        self.json_pub = node.create_publisher(String, json_topic, 10)
        self.timer = node.create_timer(max(float(heartbeat_period_sec), 1.0), self.publish)
        self.publish()

    def mark_active(self, detail: str = "node active") -> None:
        self.state_id = NodeLifecycleStatus.PRIMARY_STATE_ACTIVE
        self.state_label = "active"
        self.active = True
        self.error = False
        self.detail = detail
        self.publish()

    def mark_inactive(self, detail: str = "node inactive") -> None:
        self.state_id = NodeLifecycleStatus.PRIMARY_STATE_INACTIVE
        self.state_label = "inactive"
        self.active = False
        self.error = False
        self.detail = detail
        self.publish()

    def mark_error(self, detail: str) -> None:
        self.state_id = NodeLifecycleStatus.TRANSITION_STATE_ERRORPROCESSING
        self.state_label = "errorprocessing"
        self.active = False
        self.error = True
        self.detail = detail
        self.publish()

    def publish(self) -> None:
        now = self.node.get_clock().now().to_msg()
        payload = {
            "nodeName": self.node_name,
            "stateId": int(self.state_id),
            "stateLabel": self.state_label,
            "active": bool(self.active),
            "error": bool(self.error),
            "detail": self.detail,
        }
        raw_json = compact_json(payload)

        message = NodeLifecycleStatus()
        message.stamp = now
        message.node_name = self.node_name
        message.state_id = int(self.state_id)
        message.state_label = self.state_label
        message.active = bool(self.active)
        message.error = bool(self.error)
        message.detail = self.detail
        message.raw_json = raw_json
        self.typed_pub.publish(message)

        json_message = String()
        json_message.data = raw_json
        self.json_pub.publish(json_message)
