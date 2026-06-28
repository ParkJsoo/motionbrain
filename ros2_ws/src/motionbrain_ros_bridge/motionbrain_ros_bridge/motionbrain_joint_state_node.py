#!/usr/bin/env python3

import math

import rclpy
from motionbrain_msgs.msg import MotionStatus
from rclpy.node import Node
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


class MotionBrainJointStateNode(Node):
    def __init__(self) -> None:
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

        source_topic = str(self.get_parameter("source_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        estimated_joint_states_topic = str(
            self.get_parameter("estimated_joint_states_topic").value
        )
        self.joint_states_output = normalize_joint_states_output(
            self.get_parameter("joint_states_output").value
        )
        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.shoulder_feedback_calibration_enabled = bool(
            self.get_parameter("shoulder_feedback_calibration_enabled").value
        )
        self.shoulder_calibration: SensorJointCalibration | None = None
        if self.shoulder_feedback_calibration_enabled:
            try:
                self.shoulder_calibration = SensorJointCalibration(
                    sensor_zero_deg=float(
                        self.get_parameter("shoulder_sensor_zero_deg").value
                    ),
                    direction_sign=self.get_parameter("shoulder_direction_sign").value,
                    ros_joint_zero_rad=float(
                        self.get_parameter("shoulder_ros_joint_zero_rad").value
                    ),
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid shoulder feedback calibration: {exc}") from exc

        self.estimated_positions = [0.0] * len(JOINT_NAMES)
        self.measured_shoulder_position = math.nan
        self.has_status = False
        self.publish_default_pose = bool(self.get_parameter("publish_default_pose").value)

        self.estimated_publisher = self.create_publisher(
            JointState,
            estimated_joint_states_topic,
            10,
        )
        self.joint_states_publisher = None
        if self.joint_states_output != JOINT_STATES_OUTPUT_NONE:
            self.joint_states_publisher = self.create_publisher(
                JointState,
                joint_states_topic,
                10,
            )
        self.joint_states_topic = joint_states_topic
        self.estimated_joint_states_topic = estimated_joint_states_topic
        self.subscription = self.create_subscription(
            MotionStatus,
            source_topic,
            self.handle_status,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_joint_states)
        self.lifecycle = LifecycleStatusPublisher(
            self,
            detail=(
                f"configuring {estimated_joint_states_topic} and "
                f"{self.joint_states_output} {joint_states_topic} from {source_topic}"
            ),
        )
        self.lifecycle.mark_active(
            f"publishing estimated joint states on {estimated_joint_states_topic}; "
            f"joint_states_output={self.joint_states_output} topic={joint_states_topic}"
        )

        self.get_logger().info(
            f"Publishing estimated joint states on {estimated_joint_states_topic} "
            f"and joint_states_output={self.joint_states_output} on "
            f"{joint_states_topic} from {source_topic} for joints: "
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
