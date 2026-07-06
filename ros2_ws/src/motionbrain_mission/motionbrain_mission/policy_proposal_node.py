import json
from typing import Any

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import ControlGuard
from motionbrain_msgs.msg import MissionState
from motionbrain_msgs.msg import MotionStatus
from motionbrain_msgs.msg import NodeLifecycleStatus
from motionbrain_msgs.msg import PolicyProposal as PolicyProposalMsg
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from motionbrain_mission.policy_proposal import PolicyConfig
from motionbrain_mission.policy_proposal import PolicyDetectionSnapshot
from motionbrain_mission.policy_proposal import PolicyGuardSnapshot
from motionbrain_mission.policy_proposal import PolicyStatusSnapshot
from motionbrain_mission.policy_proposal import propose_policy_action


class MotionBrainPolicyProposalNode(LifecycleNode):
    def __init__(self, autostart: bool | None = None) -> None:
        super().__init__("motionbrain_policy_proposal_node")

        self.declare_parameter("status_topic", "/motionbrain/status_typed")
        self.declare_parameter("detection_topic", "/camera/detection_typed")
        self.declare_parameter("control_guard_topic", "/motionbrain/control_guard_typed")
        self.declare_parameter("mission_state_topic", "/motionbrain/mission_state_typed")
        self.declare_parameter("instruction_topic", "/motionbrain/policy_instruction")
        self.declare_parameter("proposal_topic", "/motionbrain/policy_proposal_typed")
        self.declare_parameter("proposal_json_topic", "/motionbrain/policy_proposal")
        self.declare_parameter("target_label", "cup")
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("allow_motion_candidates", True)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("autostart", True if autostart is None else bool(autostart))

        self.status_topic = "/motionbrain/status_typed"
        self.detection_topic = "/camera/detection_typed"
        self.control_guard_topic = "/motionbrain/control_guard_typed"
        self.mission_state_topic = "/motionbrain/mission_state_typed"
        self.instruction_topic = "/motionbrain/policy_instruction"
        self.proposal_topic = "/motionbrain/policy_proposal_typed"
        self.proposal_json_topic = "/motionbrain/policy_proposal"
        self.publish_rate_hz = 2.0
        self.policy_config = PolicyConfig()

        self.latest_status: MotionStatus | None = None
        self.latest_detection: CameraDetection | None = None
        self.latest_guard: ControlGuard | None = None
        self.latest_mission_state: MissionState | None = None
        self.latest_instruction = ""

        self._configured = False
        self._processing_active = False
        self.timer = None
        self.proposal_pub = None
        self.proposal_json_pub = None
        self.status_sub = None
        self.detection_sub = None
        self.control_guard_sub = None
        self.mission_state_sub = None
        self.instruction_sub = None

        self.lifecycle_pub = self.create_publisher(
            NodeLifecycleStatus, "/motionbrain/lifecycle_typed", 10
        )
        self.lifecycle_json_pub = self.create_publisher(String, "/motionbrain/lifecycle", 10)
        self.lifecycle_timer = self.create_timer(5.0, self.publish_lifecycle_status)
        self.lifecycle_state_id = NodeLifecycleStatus.PRIMARY_STATE_UNCONFIGURED
        self.lifecycle_state_label = "unconfigured"
        self.lifecycle_active = False
        self.lifecycle_error = False
        self.lifecycle_detail = "unconfigured policy proposal node"
        self.publish_lifecycle_status()

        if bool(self.get_parameter("autostart").value):
            self.trigger_configure()
            self.trigger_activate()

    def on_configure(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            self._read_configuration()
            self._create_configured_entities()
            self.set_lifecycle_state(
                NodeLifecycleStatus.PRIMARY_STATE_INACTIVE,
                "inactive",
                False,
                False,
                f"configured policy proposal node for {self.proposal_topic}; "
                "waiting for activation",
            )
            self.get_logger().info(
                f"MotionBrain policy proposal node configured: {self.status_topic}, "
                f"{self.detection_topic}, {self.control_guard_topic} -> {self.proposal_topic}"
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
            self.get_logger().error(f"MotionBrain policy proposal configure failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            if not self._configured:
                self._read_configuration()
                self._create_configured_entities()
            self._create_publish_timer()
            self._processing_active = True
            self.set_lifecycle_state(
                NodeLifecycleStatus.PRIMARY_STATE_ACTIVE,
                "active",
                True,
                False,
                f"publishing bounded policy proposals on {self.proposal_topic}",
            )
            self.publish_proposal()
            self.get_logger().info(
                f"MotionBrain policy proposal node active: {self.proposal_topic}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.set_lifecycle_state(
                NodeLifecycleStatus.TRANSITION_STATE_ERRORPROCESSING,
                "errorprocessing",
                False,
                True,
                f"activate failed: {exc}",
            )
            self.get_logger().error(f"MotionBrain policy proposal activate failed: {exc}")
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
            f"policy proposal publishing stopped for {self.proposal_topic}",
        )
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self._destroy_configured_entities()
        self.latest_status = None
        self.latest_detection = None
        self.latest_guard = None
        self.latest_mission_state = None
        self.latest_instruction = ""
        self.set_lifecycle_state(
            NodeLifecycleStatus.PRIMARY_STATE_UNCONFIGURED,
            "unconfigured",
            False,
            False,
            "unconfigured policy proposal node",
        )
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._processing_active = False
        self._destroy_publish_timer()
        self.set_lifecycle_state(
            NodeLifecycleStatus.PRIMARY_STATE_FINALIZED,
            "finalized",
            False,
            False,
            "policy proposal node shutdown requested",
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
            "policy proposal lifecycle error",
        )
        return TransitionCallbackReturn.SUCCESS

    def _read_configuration(self) -> None:
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.control_guard_topic = str(self.get_parameter("control_guard_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        self.proposal_topic = str(self.get_parameter("proposal_topic").value)
        self.proposal_json_topic = str(self.get_parameter("proposal_json_topic").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.policy_config = PolicyConfig(
            target_label=str(self.get_parameter("target_label").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            allow_motion_candidates=bool(
                self.get_parameter("allow_motion_candidates").value
            ),
        )

    def _create_configured_entities(self) -> None:
        if self._configured:
            return

        self.proposal_pub = self.create_publisher(
            PolicyProposalMsg, self.proposal_topic, 10
        )
        self.proposal_json_pub = self.create_publisher(String, self.proposal_json_topic, 10)
        self.status_sub = self.create_subscription(
            MotionStatus, self.status_topic, self.on_status, 10
        )
        self.detection_sub = self.create_subscription(
            CameraDetection, self.detection_topic, self.on_detection, 10
        )
        self.control_guard_sub = self.create_subscription(
            ControlGuard, self.control_guard_topic, self.on_guard, 10
        )
        self.mission_state_sub = self.create_subscription(
            MissionState, self.mission_state_topic, self.on_mission_state, 10
        )
        self.instruction_sub = self.create_subscription(
            String, self.instruction_topic, self.on_instruction, 10
        )
        self._configured = True

    def _destroy_configured_entities(self) -> None:
        if self.proposal_pub is not None:
            self.destroy_publisher(self.proposal_pub)
            self.proposal_pub = None
        if self.proposal_json_pub is not None:
            self.destroy_publisher(self.proposal_json_pub)
            self.proposal_json_pub = None
        for subscription_name in [
            "status_sub",
            "detection_sub",
            "control_guard_sub",
            "mission_state_sub",
            "instruction_sub",
        ]:
            subscription = getattr(self, subscription_name)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, subscription_name, None)
        self._configured = False

    def _create_publish_timer(self) -> None:
        self._destroy_publish_timer()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_proposal)

    def _destroy_publish_timer(self) -> None:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

    def on_status(self, msg: MotionStatus) -> None:
        if not self._processing_active:
            return
        self.latest_status = msg
        self.publish_proposal()

    def on_detection(self, msg: CameraDetection) -> None:
        if not self._processing_active:
            return
        self.latest_detection = msg
        self.publish_proposal()

    def on_guard(self, msg: ControlGuard) -> None:
        if not self._processing_active:
            return
        self.latest_guard = msg
        self.publish_proposal()

    def on_mission_state(self, msg: MissionState) -> None:
        if not self._processing_active:
            return
        self.latest_mission_state = msg

    def on_instruction(self, msg: String) -> None:
        if not self._processing_active:
            return
        self.latest_instruction = msg.data
        self.publish_proposal()

    def publish_proposal(self) -> None:
        if (
            not self._processing_active
            or self.proposal_pub is None
            or self.proposal_json_pub is None
        ):
            return

        proposal = propose_policy_action(
            instruction=self.latest_instruction,
            status=self.status_snapshot(),
            detection=self.detection_snapshot(),
            guard=self.guard_snapshot(),
            config=self.policy_config,
        )
        raw_json = proposal.to_json()

        typed_msg = PolicyProposalMsg()
        typed_msg.stamp = self.get_clock().now().to_msg()
        typed_msg.instruction = proposal.instruction
        typed_msg.action = proposal.action
        typed_msg.confidence = float(proposal.confidence)
        typed_msg.reason = proposal.reason
        typed_msg.requires_operator_confirm = proposal.requires_operator_confirm
        typed_msg.physical_motion_candidate = proposal.physical_motion_candidate
        typed_msg.preconditions = list(proposal.preconditions)
        typed_msg.raw_json = raw_json
        self.proposal_pub.publish(typed_msg)

        json_msg = String()
        json_msg.data = raw_json
        self.proposal_json_pub.publish(json_msg)

    def status_snapshot(self) -> PolicyStatusSnapshot:
        if self.latest_status is None:
            return PolicyStatusSnapshot()

        payload = parse_raw_json(self.latest_status.raw_json)
        sensor = payload.get("sensor") if isinstance(payload.get("sensor"), dict) else {}
        base_angle = payload.get("baseAngle") if isinstance(payload.get("baseAngle"), dict) else {}
        return PolicyStatusSnapshot(
            available=bool(self.latest_status.available),
            state=str(self.latest_status.state or "UNKNOWN"),
            moving=bool(self.latest_status.moving),
            faulted=bool(self.latest_status.faulted),
            base_active=as_bool(base_angle.get("active"), False),
            safety_blocked=as_bool(sensor.get("blocked"), False)
            or as_bool(payload.get("sensorBlocked"), False),
            fault_latched=as_bool(sensor.get("faultLatched"), False)
            or as_bool(payload.get("faultLatched"), False),
        )

    def detection_snapshot(self) -> PolicyDetectionSnapshot:
        if self.latest_detection is None:
            return PolicyDetectionSnapshot()

        payload = parse_raw_json(self.latest_detection.raw_json)
        return PolicyDetectionSnapshot(
            available=bool(self.latest_detection.available),
            detected=bool(self.latest_detection.detected),
            fresh=as_bool(payload.get("fresh"), bool(self.latest_detection.available)),
            held=as_bool(payload.get("held"), False),
            alignment=str(self.latest_detection.alignment or "LOST"),
            command_suggestion=str(self.latest_detection.command_suggestion or "none"),
            label=str(self.latest_detection.label or ""),
            color=str(self.latest_detection.color or ""),
            confidence=(
                float(self.latest_detection.confidence)
                if self.latest_detection.confidence >= 0.0
                else None
            ),
            area_ratio=float(self.latest_detection.area_ratio),
        )

    def guard_snapshot(self) -> PolicyGuardSnapshot:
        if self.latest_guard is None:
            return PolicyGuardSnapshot()

        return PolicyGuardSnapshot(
            ready=bool(self.latest_guard.ready),
            reason=str(self.latest_guard.reason or "unknown"),
            status_fresh=bool(self.latest_guard.status_fresh),
            detection_fresh=bool(self.latest_guard.detection_fresh),
        )

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


def parse_raw_json(raw_json: str) -> dict[str, Any]:
    if not raw_json:
        return {}
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on", "ready", "ok"}:
            return True
        if normalized in {"false", "0", "no", "n", "off", "blocked", "stale", ""}:
            return False
    return default


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionBrainPolicyProposalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
