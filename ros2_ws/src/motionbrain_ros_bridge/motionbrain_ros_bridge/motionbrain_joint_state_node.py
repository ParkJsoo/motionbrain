#!/usr/bin/env python3

import math
from typing import Any

import rclpy
from motionbrain_msgs.msg import MotionStatus
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import JointState

from motionbrain_ros_bridge.lifecycle_status import LifecycleStatusPublisher
from motionbrain_ros_bridge.m4_sensor_mapping import SensorJointCalibration
from motionbrain_ros_bridge.m4_sensor_mapping import (
    shoulder_feedback_to_measured_ros_joint_position,
)
from motionbrain_ros_bridge.m4_sensor_mapping import shoulder_feedback_to_ros_joint_position


JOINT_NAMES = [
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "gripper_joint",
]

MEASURED_JOINT_NAMES = [
    "shoulder_pitch_joint",
]

JOINT_STATES_OUTPUT_ESTIMATED = "estimated"
JOINT_STATES_OUTPUT_MEASURED = "measured"
JOINT_STATES_OUTPUT_NONE = "none"
JOINT_STATES_OUTPUT_ALIASES = {
    "": JOINT_STATES_OUTPUT_NONE,
    "0": JOINT_STATES_OUTPUT_NONE,
    "false": JOINT_STATES_OUTPUT_NONE,
    "off": JOINT_STATES_OUTPUT_NONE,
    "disabled": JOINT_STATES_OUTPUT_NONE,
    "open_loop": JOINT_STATES_OUTPUT_ESTIMATED,
    "compat": JOINT_STATES_OUTPUT_ESTIMATED,
}


def normalize_joint_states_output(value: object) -> str:
    output = str(value).strip().lower()
    output = JOINT_STATES_OUTPUT_ALIASES.get(output, output)
    if output not in {
        JOINT_STATES_OUTPUT_ESTIMATED,
        JOINT_STATES_OUTPUT_MEASURED,
        JOINT_STATES_OUTPUT_NONE,
    }:
        raise ValueError("joint_states_output must be estimated, measured, or none")
    return output


class MotionBrainJointStateNode(LifecycleNode):
    def __init__(self, autostart: bool | None = None) -> None:
        super().__init__("motionbrain_joint_state_node")

        self.declare_parameter("source_topic", "/motionbrain/status_typed")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "estimated_joint_states_topic",
            "/motionbrain/estimated_joint_states",
        )
        self.declare_parameter("joint_states_output", JOINT_STATES_OUTPUT_ESTIMATED)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("publish_default_pose", True)
        self.declare_parameter("shoulder_feedback_calibration_enabled", False)
        self.declare_parameter("shoulder_sensor_zero_deg", 0.0)
        self.declare_parameter("shoulder_direction_sign", 1)
        self.declare_parameter("shoulder_ros_joint_zero_rad", 0.0)
        self.declare_parameter("autostart", True if autostart is None else bool(autostart))

        self.estimated_positions = [0.0] * len(JOINT_NAMES)
        self.measured_shoulder_position = math.nan
        self.has_status = False
        self.source_topic = "/motionbrain/status_typed"
        self.joint_states_topic = "/joint_states"
        self.estimated_joint_states_topic = "/motionbrain/estimated_joint_states"
        self.joint_states_output = JOINT_STATES_OUTPUT_ESTIMATED
        self.publish_rate_hz = 5.0
        self.publish_default_pose = True
        self.shoulder_feedback_calibration_enabled = False
        self.shoulder_calibration: SensorJointCalibration | None = None
        self._configured = False
        self._publishing_active = False
        self.timer = None
        self.estimated_publisher = None
        self.joint_states_publisher = None
        self.subscription = None
        self.lifecycle = LifecycleStatusPublisher(
            self,
            detail="unconfigured joint-state bridge",
        )

        if bool(self.get_parameter("autostart").value):
            self.trigger_configure()
            self.trigger_activate()

    def on_configure(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            self._read_configuration()
            self._create_configured_entities()
            self.lifecycle.mark_inactive(
                f"configured {self.estimated_joint_states_topic} and "
                f"{self.joint_states_output} {self.joint_states_topic}; waiting for activation"
            )
            self.get_logger().info(
                f"MotionBrain joint-state bridge configured from {self.source_topic}; "
                "waiting for lifecycle activation"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.lifecycle.mark_error(f"configure failed: {exc}")
            self.get_logger().error(f"MotionBrain joint-state bridge configure failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            if not self._configured:
                self._read_configuration()
                self._create_configured_entities()
            self._create_publish_timer()
            self._publishing_active = True
            self.lifecycle.mark_active(
                f"publishing estimated joint states on {self.estimated_joint_states_topic}; "
                f"joint_states_output={self.joint_states_output} topic={self.joint_states_topic}"
            )
            self._log_active()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.lifecycle.mark_error(f"activate failed: {exc}")
            self.get_logger().error(f"MotionBrain joint-state bridge activate failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._publishing_active = False
        self._destroy_publish_timer()
        self.lifecycle.mark_inactive(
            f"joint-state publishing stopped for {self.joint_states_topic}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._publishing_active = False
        self._destroy_publish_timer()
        self._destroy_configured_entities()
        self.lifecycle.mark_inactive("unconfigured joint-state bridge")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._publishing_active = False
        self._destroy_publish_timer()
        self.lifecycle.mark_inactive("joint-state bridge shutdown requested")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._publishing_active = False
        self._destroy_publish_timer()
        self.lifecycle.mark_error("joint-state bridge lifecycle error")
        return TransitionCallbackReturn.SUCCESS

    def _read_configuration(self) -> None:
        self.source_topic = str(self.get_parameter("source_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.estimated_joint_states_topic = str(
            self.get_parameter("estimated_joint_states_topic").value
        )
        self.joint_states_output = normalize_joint_states_output(
            self.get_parameter("joint_states_output").value
        )
        if (
            self.estimated_joint_states_topic == self.joint_states_topic
            and self.joint_states_output != JOINT_STATES_OUTPUT_ESTIMATED
        ):
            raise ValueError(
                "estimated_joint_states_topic must differ from joint_states_topic "
                "unless joint_states_output is estimated"
            )
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.publish_default_pose = bool(self.get_parameter("publish_default_pose").value)
        self.shoulder_feedback_calibration_enabled = bool(
            self.get_parameter("shoulder_feedback_calibration_enabled").value
        )
        self.shoulder_calibration = None
        if self.shoulder_feedback_calibration_enabled:
            self.shoulder_calibration = SensorJointCalibration(
                sensor_zero_deg=float(
                    self.get_parameter("shoulder_sensor_zero_deg").value
                ),
                direction_sign=self.get_parameter("shoulder_direction_sign").value,
                ros_joint_zero_rad=float(
                    self.get_parameter("shoulder_ros_joint_zero_rad").value
                ),
            )

    def _create_configured_entities(self) -> None:
        if self._configured:
            return

        self.estimated_publisher = self.create_publisher(
            JointState,
            self.estimated_joint_states_topic,
            10,
        )
        if (
            self.joint_states_output != JOINT_STATES_OUTPUT_NONE
            and self.joint_states_topic != self.estimated_joint_states_topic
        ):
            self.joint_states_publisher = self.create_publisher(
                JointState,
                self.joint_states_topic,
                10,
            )
        self.subscription = self.create_subscription(
            MotionStatus,
            self.source_topic,
            self.handle_status,
            10,
        )
        self._configured = True

    def _create_publish_timer(self) -> None:
        self._destroy_publish_timer()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_joint_states)

    def _destroy_publish_timer(self) -> None:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

    def _destroy_configured_entities(self) -> None:
        if self.estimated_publisher is not None:
            self.destroy_publisher(self.estimated_publisher)
            self.estimated_publisher = None
        if self.joint_states_publisher is not None:
            self.destroy_publisher(self.joint_states_publisher)
            self.joint_states_publisher = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self._configured = False
        self.has_status = False
        self.estimated_positions = [0.0] * len(JOINT_NAMES)
        self.measured_shoulder_position = math.nan

    def _log_active(self) -> None:
        self.get_logger().info(
            f"Publishing estimated joint states on {self.estimated_joint_states_topic} "
            f"and joint_states_output={self.joint_states_output} on "
            f"{self.joint_states_topic} from {self.source_topic} for joints: "
            + ", ".join(JOINT_NAMES)
        )

    def handle_status(self, message: MotionStatus) -> None:
        estimated_shoulder_pitch = shoulder_feedback_to_ros_joint_position(
            calibration_enabled=self.shoulder_feedback_calibration_enabled,
            feedback_available=message.shoulder_feedback_available,
            sensor_connected=message.shoulder_sensor_connected,
            sensor_fresh=message.shoulder_sensor_fresh,
            sensor_ready=message.shoulder_sensor_ready,
            shoulder_angle_deg=message.shoulder_angle_deg,
            legacy_tilt_angle_deg=message.tilt_angle_deg,
            calibration=self.shoulder_calibration,
        )
        self.measured_shoulder_position = shoulder_feedback_to_measured_ros_joint_position(
            calibration_enabled=self.shoulder_feedback_calibration_enabled,
            feedback_available=message.shoulder_feedback_available,
            sensor_connected=message.shoulder_sensor_connected,
            sensor_fresh=message.shoulder_sensor_fresh,
            sensor_ready=message.shoulder_sensor_ready,
            shoulder_angle_deg=message.shoulder_angle_deg,
            calibration=self.shoulder_calibration,
        )
        self.estimated_positions = [
            math.radians(float(message.base_angle_deg)),
            estimated_shoulder_pitch,
            math.radians(float(message.pan_angle_deg)),
            math.radians(float(message.wrist_angle_deg)),
            math.radians(float(message.gripper_angle_deg)),
        ]
        self.has_status = True

    def publish_joint_states(self) -> None:
        if not self._publishing_active:
            return

        if self.estimated_publisher is None:
            return

        if not self.has_status and not self.publish_default_pose:
            return

        estimated_message = self.joint_state_message(
            JOINT_NAMES,
            self.estimated_positions,
        )
        self.estimated_publisher.publish(estimated_message)

        if self.joint_states_publisher is None:
            return

        if self.joint_states_output == JOINT_STATES_OUTPUT_ESTIMATED:
            if self.joint_states_topic == self.estimated_joint_states_topic:
                return
            self.joint_states_publisher.publish(estimated_message)
            return

        if not self.has_status:
            return

        measured_message = self.joint_state_message(
            MEASURED_JOINT_NAMES,
            [self.measured_shoulder_position],
        )
        self.joint_states_publisher.publish(measured_message)

    def joint_state_message(
        self,
        names: list[str],
        positions: list[float],
    ) -> JointState:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(names)
        message.position = list(positions)
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionBrainJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
