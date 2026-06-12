#!/usr/bin/env python3

import math
from typing import Iterable

import rclpy
from motionbrain_msgs.msg import MotionStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState

from motionbrain_ros_bridge.lifecycle_status import LifecycleStatusPublisher


JOINT_NAMES = [
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "gripper_joint",
]


def degrees_to_radians(values: Iterable[float]) -> list[float]:
    return [math.radians(float(value)) for value in values]


class MotionBrainJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_joint_state_node")

        self.declare_parameter("source_topic", "/motionbrain/status_typed")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("publish_default_pose", True)

        source_topic = str(self.get_parameter("source_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)

        self.positions = [0.0] * len(JOINT_NAMES)
        self.has_status = False
        self.publish_default_pose = bool(self.get_parameter("publish_default_pose").value)

        self.publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.subscription = self.create_subscription(
            MotionStatus,
            source_topic,
            self.handle_status,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_joint_states)
        self.lifecycle = LifecycleStatusPublisher(
            self,
            detail=f"configuring {joint_states_topic} from {source_topic}",
        )
        self.lifecycle.mark_active(f"publishing {joint_states_topic} from {source_topic}")

        self.get_logger().info(
            f"Publishing {joint_states_topic} from {source_topic} for joints: "
            + ", ".join(JOINT_NAMES)
        )

    def handle_status(self, message: MotionStatus) -> None:
        self.positions = degrees_to_radians(
            [
                message.base_angle_deg,
                message.tilt_angle_deg,
                message.pan_angle_deg,
                message.wrist_angle_deg,
                message.gripper_angle_deg,
            ]
        )
        self.has_status = True

    def publish_joint_states(self) -> None:
        if not self.has_status and not self.publish_default_pose:
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(self.positions)
        self.publisher.publish(message)


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
