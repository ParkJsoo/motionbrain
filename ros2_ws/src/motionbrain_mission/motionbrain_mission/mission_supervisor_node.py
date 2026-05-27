import json

import rclpy
from motionbrain_msgs.msg import CameraDetection, LightCommand, MotionStatus
from rclpy.node import Node
from std_msgs.msg import String

from motionbrain_mission.mission_flow import MissionConfig, MissionDecision, MissionFlow


class MotionBrainMissionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_mission_supervisor")

        self.control_guard_topic = self.declare_parameter(
            "control_guard_topic", "/motionbrain/control_guard"
        ).value
        self.detection_topic = self.declare_parameter(
            "detection_topic", "/camera/detection_typed"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/motionbrain/status_typed"
        ).value
        self.mission_cmd_topic = self.declare_parameter(
            "mission_cmd_topic", "/motionbrain/mission_cmd"
        ).value
        self.mission_state_topic = self.declare_parameter(
            "mission_state_topic", "/motionbrain/mission_state"
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

        self.state_pub = self.create_publisher(String, self.mission_state_topic, 10)
        self.light_cmd_pub = self.create_publisher(LightCommand, self.light_cmd_topic, 10)

        self.create_subscription(String, self.control_guard_topic, self.on_guard, 10)
        self.create_subscription(CameraDetection, self.detection_topic, self.on_detection, 10)
        self.create_subscription(MotionStatus, self.status_topic, self.on_status, 10)
        self.create_subscription(String, self.mission_cmd_topic, self.on_command, 10)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            f"Mission supervisor ready: {self.detection_topic} -> "
            f"{self.mission_state_topic} -> confirm -> {self.light_cmd_topic}"
        )

    def on_guard(self, msg: String) -> None:
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

    def on_command(self, msg: String) -> None:
        decision = self.flow.handle_command(msg.data)
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
        msg = String()
        msg.data = self.flow.to_json(self.latest_decision)
        self.state_pub.publish(msg)


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
