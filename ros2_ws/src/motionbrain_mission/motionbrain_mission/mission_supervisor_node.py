import json
from typing import Any

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import ControlGuard
from motionbrain_msgs.msg import LightCommand
from motionbrain_msgs.msg import MissionCommand
from motionbrain_msgs.msg import MissionState as MissionStateMsg
from motionbrain_msgs.msg import MotionStatus
from motionbrain_msgs.msg import NodeLifecycleStatus
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from motionbrain_mission.mission_flow import MissionConfig, MissionDecision, MissionFlow


class MotionBrainMissionSupervisor(LifecycleNode):
    def __init__(self, autostart: bool | None = None) -> None:
        super().__init__("motionbrain_mission_supervisor")

        self.declare_parameter("control_guard_topic", "/motionbrain/control_guard_typed")
        self.declare_parameter("control_guard_json_topic", "/motionbrain/control_guard")
        self.declare_parameter("detection_topic", "/camera/detection_typed")
        self.declare_parameter("status_topic", "/motionbrain/status_typed")
        self.declare_parameter("mission_cmd_topic", "/motionbrain/mission_cmd_typed")
        self.declare_parameter("mission_cmd_json_topic", "/motionbrain/mission_cmd")
        self.declare_parameter("mission_state_topic", "/motionbrain/mission_state_typed")
        self.declare_parameter("mission_state_json_topic", "/motionbrain/mission_state")
        self.declare_parameter("light_cmd_topic", "/motionbrain/light_cmd_typed")
        self.declare_parameter("require_center_alignment", True)
        self.declare_parameter("require_guard_ready", True)
        self.declare_parameter("act_action", "toggle")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("autostart", True if autostart is None else bool(autostart))

        self.control_guard_topic = "/motionbrain/control_guard_typed"
        self.control_guard_json_topic = "/motionbrain/control_guard"
        self.detection_topic = "/camera/detection_typed"
        self.status_topic = "/motionbrain/status_typed"
        self.mission_cmd_topic = "/motionbrain/mission_cmd_typed"
        self.mission_cmd_json_topic = "/motionbrain/mission_cmd"
        self.mission_state_topic = "/motionbrain/mission_state_typed"
        self.mission_state_json_topic = "/motionbrain/mission_state"
        self.light_cmd_topic = "/motionbrain/light_cmd_typed"
        self.require_center_alignment = True
        self.require_guard_ready = True
        self.act_action = "toggle"
        self.publish_rate_hz = 2.0
        self.flow = MissionFlow()
        self.latest_status = None
        self.latest_decision = self.flow.evaluate()

        self._configured = False
        self._processing_active = False
        self.state_pub = None
        self.state_json_pub = None
        self.light_cmd_pub = None
        self.timer = None
        self.control_guard_sub = None
        self.control_guard_json_sub = None
        self.detection_sub = None
        self.status_sub = None
        self.mission_cmd_sub = None
        self.mission_cmd_json_sub = None

        self.lifecycle_pub = self.create_publisher(
            NodeLifecycleStatus, "/motionbrain/lifecycle_typed", 10
        )
        self.lifecycle_json_pub = self.create_publisher(String, "/motionbrain/lifecycle", 10)
        self.lifecycle_timer = self.create_timer(5.0, self.publish_lifecycle_status)
        self.lifecycle_state_id = NodeLifecycleStatus.PRIMARY_STATE_UNCONFIGURED
        self.lifecycle_state_label = "unconfigured"
        self.lifecycle_active = False
        self.lifecycle_error = False
        self.lifecycle_detail = "unconfigured mission supervisor"
        self.publish_lifecycle_status()

        if bool(self.get_parameter("autostart").value):
            self.trigger_configure()
            self.trigger_activate()

    def on_configure(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            self._read_configuration()
            self._reset_flow()
            self._create_configured_entities()
            self.set_lifecycle_state(
                NodeLifecycleStatus.PRIMARY_STATE_INACTIVE,
                "inactive",
                False,
                False,
                f"configured mission supervisor for {self.detection_topic}; "
                "waiting for activation",
            )
            self.get_logger().info(
                f"MotionBrain mission supervisor configured: {self.detection_topic} -> "
                f"{self.mission_state_topic}; waiting for lifecycle activation"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.set_lifecycle_state(
                NodeLifecycleStatus.TRANSITION_STATE_ERRORPROCESSING,
                "errorprocessing",
                False,
                True,
                f"configure failed: {exc}",
            )
            self.get_logger().error(f"MotionBrain mission supervisor configure failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            if not self._configured:
                self._read_configuration()
                self._reset_flow()
                self._create_configured_entities()
            self._create_publish_timer()
            self._processing_active = True
            self.set_lifecycle_state(
                NodeLifecycleStatus.PRIMARY_STATE_ACTIVE,
                "active",
                True,
                False,
                f"supervising {self.detection_topic} and publishing "
                f"{self.mission_state_topic}",
            )
            self.publish_state()
            self._log_active()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.set_lifecycle_state(
                NodeLifecycleStatus.TRANSITION_STATE_ERRORPROCESSING,
                "errorprocessing",
                False,
                True,
                f"activate failed: {exc}",
            )
            self.get_logger().error(f"MotionBrain mission supervisor activate failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self.set_lifecycle_state(
            NodeLifecycleStatus.PRIMARY_STATE_INACTIVE,
            "inactive",
            False,
            False,
            f"mission state publishing stopped for {self.mission_state_topic}",
        )
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self._destroy_configured_entities()
        self._reset_flow()
        self.set_lifecycle_state(
            NodeLifecycleStatus.PRIMARY_STATE_INACTIVE,
            "inactive",
            False,
            False,
            "unconfigured mission supervisor",
        )
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self.set_lifecycle_state(
            NodeLifecycleStatus.PRIMARY_STATE_INACTIVE,
            "inactive",
            False,
            False,
            "mission supervisor shutdown requested",
        )
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self.set_lifecycle_state(
            NodeLifecycleStatus.TRANSITION_STATE_ERRORPROCESSING,
            "errorprocessing",
            False,
            True,
            "mission supervisor lifecycle error",
        )
        return TransitionCallbackReturn.SUCCESS

    def _read_configuration(self) -> None:
        self.control_guard_topic = str(self.get_parameter("control_guard_topic").value)
        self.control_guard_json_topic = str(
            self.get_parameter("control_guard_json_topic").value
        )
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.mission_cmd_topic = str(self.get_parameter("mission_cmd_topic").value)
        self.mission_cmd_json_topic = str(self.get_parameter("mission_cmd_json_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.mission_state_json_topic = str(
            self.get_parameter("mission_state_json_topic").value
        )
        self.light_cmd_topic = str(self.get_parameter("light_cmd_topic").value)
        self.require_center_alignment = bool(
            self.get_parameter("require_center_alignment").value
        )
        self.require_guard_ready = bool(self.get_parameter("require_guard_ready").value)
        self.act_action = str(self.get_parameter("act_action").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)

    def _reset_flow(self) -> None:
        self.flow = MissionFlow(
            MissionConfig(
                require_center_alignment=self.require_center_alignment,
                require_guard_ready=self.require_guard_ready,
                act_action=self.act_action,
            )
        )
        self.latest_status = None
        self.latest_decision = self.flow.evaluate()

    def _create_configured_entities(self) -> None:
        if self._configured:
            return

        self.state_pub = self.create_publisher(MissionStateMsg, self.mission_state_topic, 10)
        self.state_json_pub = self.create_publisher(String, self.mission_state_json_topic, 10)
        self.light_cmd_pub = self.create_publisher(LightCommand, self.light_cmd_topic, 10)
        self.control_guard_sub = self.create_subscription(
            ControlGuard, self.control_guard_topic, self.on_guard, 10
        )
        self.control_guard_json_sub = self.create_subscription(
            String, self.control_guard_json_topic, self.on_guard_json, 10
        )
        self.detection_sub = self.create_subscription(
            CameraDetection, self.detection_topic, self.on_detection, 10
        )
        self.status_sub = self.create_subscription(
            MotionStatus, self.status_topic, self.on_status, 10
        )
        self.mission_cmd_sub = self.create_subscription(
            MissionCommand, self.mission_cmd_topic, self.on_command, 10
        )
        self.mission_cmd_json_sub = self.create_subscription(
            String, self.mission_cmd_json_topic, self.on_command_json, 10
        )
        self._configured = True

    def _destroy_configured_entities(self) -> None:
        if self.state_pub is not None:
            self.destroy_publisher(self.state_pub)
            self.state_pub = None
        if self.state_json_pub is not None:
            self.destroy_publisher(self.state_json_pub)
            self.state_json_pub = None
        if self.light_cmd_pub is not None:
            self.destroy_publisher(self.light_cmd_pub)
            self.light_cmd_pub = None
        for subscription_name in [
            "control_guard_sub",
            "control_guard_json_sub",
            "detection_sub",
            "status_sub",
            "mission_cmd_sub",
            "mission_cmd_json_sub",
        ]:
            subscription = getattr(self, subscription_name)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, subscription_name, None)
        self._configured = False

    def _create_publish_timer(self) -> None:
        self._destroy_publish_timer()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_state)

    def _destroy_publish_timer(self) -> None:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

    def _log_active(self) -> None:
        self.get_logger().info(
            f"Mission supervisor ready: {self.detection_topic} -> "
            f"{self.mission_state_topic} -> confirm -> {self.light_cmd_topic}"
        )

    def on_guard(self, msg: ControlGuard) -> None:
        if not self._processing_active:
            return
        self.latest_decision = self.flow.update_guard(
            ready=msg.ready,
            reason=msg.reason,
            suggested_action=msg.suggested_action,
            status_fresh=msg.status_fresh,
            detection_fresh=msg.detection_fresh,
        )

    def on_guard_json(self, msg: String) -> None:
        if not self._processing_active:
            return
        self.latest_decision = self.flow.update_guard_json(msg.data)

    def on_detection(self, msg: CameraDetection) -> None:
        if not self._processing_active:
            return
        self.latest_decision = self.flow.update_detection(
            available=msg.available,
            detected=msg.detected,
            alignment=msg.alignment,
            command_suggestion=msg.command_suggestion,
            area_ratio=float(msg.area_ratio),
        )

    def on_status(self, msg: MotionStatus) -> None:
        if not self._processing_active:
            return
        self.latest_status = msg

    def on_command(self, msg: MissionCommand) -> None:
        if not self._processing_active:
            return
        decision = self.flow.handle_command(msg.command or msg.raw_json)
        self.handle_decision(decision)

    def on_command_json(self, msg: String) -> None:
        if not self._processing_active:
            return
        decision = self.flow.handle_command(msg.data)
        self.handle_decision(decision)

    def handle_decision(self, decision: MissionDecision) -> None:
        self.latest_decision = decision
        if decision.act_request:
            self.publish_light_command(decision)
        self.publish_state()

    def publish_light_command(self, decision: MissionDecision) -> None:
        if not self._processing_active or self.light_cmd_pub is None:
            return

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
        if not self._processing_active or self.state_pub is None or self.state_json_pub is None:
            return

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

    def set_lifecycle_state(
        self,
        state_id: int,
        state_label: str,
        active: bool,
        error: bool,
        detail: str,
    ) -> None:
        self.lifecycle_state_id = state_id
        self.lifecycle_state_label = state_label
        self.lifecycle_active = active
        self.lifecycle_error = error
        self.lifecycle_detail = detail
        self.publish_lifecycle_status()

    def publish_lifecycle_status(self) -> None:
        payload = {
            "nodeName": self.get_name(),
            "stateId": int(self.lifecycle_state_id),
            "stateLabel": self.lifecycle_state_label,
            "active": bool(self.lifecycle_active),
            "error": bool(self.lifecycle_error),
            "detail": self.lifecycle_detail,
        }
        raw_json = json.dumps(payload, separators=(",", ":"), sort_keys=True)

        message = NodeLifecycleStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.node_name = self.get_name()
        message.state_id = int(self.lifecycle_state_id)
        message.state_label = self.lifecycle_state_label
        message.active = bool(payload["active"])
        message.error = bool(payload["error"])
        message.detail = self.lifecycle_detail
        message.raw_json = raw_json
        self.lifecycle_pub.publish(message)

        json_message = String()
        json_message.data = raw_json
        self.lifecycle_json_pub.publish(json_message)


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
