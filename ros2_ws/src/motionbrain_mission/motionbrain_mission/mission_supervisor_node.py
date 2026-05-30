import json

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import ControlGuard
from motionbrain_msgs.msg import LightCommand
from motionbrain_msgs.msg import MissionCommand
from motionbrain_msgs.msg import MissionState as MissionStateMsg
from motionbrain_msgs.msg import MotionStatus
from rclpy.node import Node
from std_msgs.msg import String

from motionbrain_mission.mission_flow import MissionConfig, MissionDecision, MissionFlow


class MotionBrainMissionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_mission_supervisor")

        self.control_guard_topic = self.declare_parameter(
            "control_guard_topic", "/motionbrain/control_guard_typed"
        ).value
        self.control_guard_json_topic = self.declare_parameter(
            "control_guard_json_topic", "/motionbrain/control_guard"
        ).value
        self.detection_topic = self.declare_parameter(
            "detection_topic", "/camera/detection_typed"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/motionbrain/status_typed"
        ).value
        self.mission_cmd_topic = self.declare_parameter(
            "mission_cmd_topic", "/motionbrain/mission_cmd_typed"
        ).value
        self.mission_cmd_json_topic = self.declare_parameter(
            "mission_cmd_json_topic", "/motionbrain/mission_cmd"
        ).value
        self.mission_state_topic = self.declare_parameter(
            "mission_state_topic", "/motionbrain/mission_state_typed"
        ).value
        self.mission_state_json_topic = self.declare_parameter(
            "mission_state_json_topic", "/motionbrain/mission_state"
        ).value
        self.light_cmd_topic = self.declare_parameter(
            "light_cmd_topic", "/motionbrain/light_cmd_typed"
        ).value
        require_center_alignment = bool(
            self.declare_parameter("require_center_alignment", True).value
        )
        require_guard_ready = bool(self.declare_parameter("require_guard_ready", True).value)
        act_action = str(self.declare_parameter("act_action", "toggle").value)
        publish_rate_hz = max(float(self.declare_parameter("publish_rate_hz", 2.0).value), 0.1)

        self.flow = MissionFlow(
            MissionConfig(
                require_center_alignment=require_center_alignment,
                require_guard_ready=require_guard_ready,
                act_action=act_action,
            )
        )
        self.latest_status = None
        self.latest_decision = self.flow.evaluate()

        self.state_pub = self.create_publisher(MissionStateMsg, self.mission_state_topic, 10)
        self.state_json_pub = self.create_publisher(String, self.mission_state_json_topic, 10)
        self.light_cmd_pub = self.create_publisher(LightCommand, self.light_cmd_topic, 10)

        self.create_subscription(ControlGuard, self.control_guard_topic, self.on_guard, 10)
        self.create_subscription(String, self.control_guard_json_topic, self.on_guard_json, 10)
        self.create_subscription(CameraDetection, self.detection_topic, self.on_detection, 10)
        self.create_subscription(MotionStatus, self.status_topic, self.on_status, 10)
        self.create_subscription(MissionCommand, self.mission_cmd_topic, self.on_command, 10)
        self.create_subscription(String, self.mission_cmd_json_topic, self.on_command_json, 10)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            f"Mission supervisor ready: {self.detection_topic} -> "
            f"{self.mission_state_topic} -> confirm -> {self.light_cmd_topic}"
        )

    def on_guard(self, msg: ControlGuard) -> None:
        self.latest_decision = self.flow.update_guard(
            ready=msg.ready,
            reason=msg.reason,
            suggested_action=msg.suggested_action,
            status_fresh=msg.status_fresh,
            detection_fresh=msg.detection_fresh,
        )

    def on_guard_json(self, msg: String) -> None:
        self.latest_decision = self.flow.update_guard_json(msg.data)

    def on_detection(self, msg: CameraDetection) -> None:
        self.latest_decision = self.flow.update_detection(
            available=msg.available,
            detected=msg.detected,
            alignment=msg.alignment,
            command_suggestion=msg.command_suggestion,
            area_ratio=float(msg.area_ratio),
        )

    def on_status(self, msg: MotionStatus) -> None:
        self.latest_status = msg

    def on_command(self, msg: MissionCommand) -> None:
        decision = self.flow.handle_command(msg.command or msg.raw_json)
        self.handle_decision(decision)

    def on_command_json(self, msg: String) -> None:
        decision = self.flow.handle_command(msg.data)
        self.handle_decision(decision)

    def handle_decision(self, decision: MissionDecision) -> None:
        self.latest_decision = decision
        if decision.act_request:
            self.publish_light_command(decision)
        self.publish_state()

    def publish_light_command(self, decision: MissionDecision) -> None:
        cmd = LightCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.action = decision.act_request or ""
        cmd.raw_json = json.dumps(
            {
                "source": "motionbrain_mission_supervisor",
                "missionState": decision.state.value,
                "reason": decision.reason,
            },
            separators=(",", ":"),
        )
        self.light_cmd_pub.publish(cmd)
        self.get_logger().info(f"Published operator-confirmed light command: {cmd.action}")

    def publish_state(self) -> None:
        payload = self.flow.to_dict(self.latest_decision)
        raw_json = json.dumps(payload, separators=(",", ":"))

        typed_msg = MissionStateMsg()
        typed_msg.stamp = self.get_clock().now().to_msg()
        typed_msg.state = str(payload["state"])
        typed_msg.reason = str(payload["reason"])
        typed_msg.next_step = str(payload["nextStep"])
        typed_msg.suggested_action = str(payload["suggestedAction"])
        typed_msg.guard_ready = bool(payload["guardReady"])
        typed_msg.guard_reason = str(payload["guardReason"])
        typed_msg.status_fresh = bool(payload["statusFresh"])
        typed_msg.detection_fresh = bool(payload["detectionFresh"])
        typed_msg.target_detected = bool(payload["targetDetected"])
        typed_msg.alignment = str(payload["alignment"])
        typed_msg.area_ratio = float(payload["areaRatio"])
        typed_msg.raw_json = raw_json
        self.state_pub.publish(typed_msg)

        json_msg = String()
        json_msg.data = raw_json
        self.state_json_pub.publish(json_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionBrainMissionSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
