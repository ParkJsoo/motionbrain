#!/usr/bin/env python3

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from motionbrain_msgs.msg import KinematicsState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from motionbrain_ros_bridge.motionbrain_kinematics import JointAngles
from motionbrain_ros_bridge.motionbrain_kinematics import forward_kinematics
from motionbrain_ros_bridge.motionbrain_kinematics import inverse_kinematics
from motionbrain_ros_bridge.motionbrain_kinematics import joint_positions_from_message


def quaternion_from_yaw_pitch(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    return (
        -sp * sy,
        sp * cy,
        cp * sy,
        cp * cy,
    )


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


class MotionBrainKinematicsNode(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_kinematics_node")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/motionbrain/end_effector_pose")
        self.declare_parameter("kinematics_topic", "/motionbrain/kinematics")
        self.declare_parameter("kinematics_typed_topic", "/motionbrain/kinematics_typed")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("enable_ik_suggestion", False)
        self.declare_parameter("target_x_m", 0.60)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("target_z_m", 0.18)
        self.declare_parameter("target_tool_pitch_deg", 0.0)

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        kinematics_topic = str(self.get_parameter("kinematics_topic").value)
        kinematics_typed_topic = str(self.get_parameter("kinematics_typed_topic").value)

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.kinematics_pub = self.create_publisher(String, kinematics_topic, 10)
        self.kinematics_typed_pub = self.create_publisher(
            KinematicsState,
            kinematics_typed_topic,
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.handle_joint_state,
            10,
        )

        self.get_logger().info(
            f"Publishing FK pose on {pose_topic}, typed kinematics on "
            f"{kinematics_typed_topic}, and compatibility JSON on {kinematics_topic} "
            f"from {joint_states_topic}"
        )

    def handle_joint_state(self, message: JointState) -> None:
        positions = joint_positions_from_message(message.name, message.position)
        angles = JointAngles.from_positions(positions)
        pose = forward_kinematics(angles)

        pose_message = PoseStamped()
        pose_message.header.stamp = self.get_clock().now().to_msg()
        pose_message.header.frame_id = str(self.get_parameter("frame_id").value)
        pose_message.pose.position.x = pose.x_m
        pose_message.pose.position.y = pose.y_m
        pose_message.pose.position.z = pose.z_m
        qx, qy, qz, qw = quaternion_from_yaw_pitch(pose.yaw_rad, pose.pitch_rad)
        pose_message.pose.orientation.x = qx
        pose_message.pose.orientation.y = qy
        pose_message.pose.orientation.z = qz
        pose_message.pose.orientation.w = qw
        self.pose_pub.publish(pose_message)

        typed_message = KinematicsState()
        typed_message.stamp = pose_message.header.stamp
        typed_message.x_m = pose.x_m
        typed_message.y_m = pose.y_m
        typed_message.z_m = pose.z_m
        typed_message.yaw_rad = pose.yaw_rad
        typed_message.pitch_rad = pose.pitch_rad
        typed_message.radial_reach_m = pose.radial_reach_m
        typed_message.within_joint_limits = pose.within_joint_limits
        typed_message.joint_limit_violations = list(pose.joint_limit_violations)
        typed_message.base_yaw_rad = angles.base_yaw
        typed_message.shoulder_pitch_rad = angles.shoulder_pitch
        typed_message.elbow_pitch_rad = angles.elbow_pitch
        typed_message.wrist_pitch_rad = angles.wrist_pitch
        typed_message.gripper_rad = angles.gripper

        payload: dict[str, Any] = {
            "fk": {
                "xM": pose.x_m,
                "yM": pose.y_m,
                "zM": pose.z_m,
                "yawRad": pose.yaw_rad,
                "pitchRad": pose.pitch_rad,
                "radialReachM": pose.radial_reach_m,
                "withinJointLimits": pose.within_joint_limits,
                "jointLimitViolations": list(pose.joint_limit_violations),
            },
            "jointsRad": {
                "baseYaw": angles.base_yaw,
                "shoulderPitch": angles.shoulder_pitch,
                "elbowPitch": angles.elbow_pitch,
                "wristPitch": angles.wrist_pitch,
                "gripper": angles.gripper,
            },
        }

        typed_message.ik_enabled = bool(self.get_parameter("enable_ik_suggestion").value)
        if typed_message.ik_enabled:
            solution = inverse_kinematics(
                float(self.get_parameter("target_x_m").value),
                float(self.get_parameter("target_y_m").value),
                float(self.get_parameter("target_z_m").value),
                target_tool_pitch_rad=math.radians(
                    float(self.get_parameter("target_tool_pitch_deg").value),
                ),
            )
            payload["ikSuggestion"] = {
                "reachable": solution.reachable,
                "reason": solution.reason,
                "targetXM": solution.target_x_m,
                "targetYM": solution.target_y_m,
                "targetZM": solution.target_z_m,
                "radialReachM": solution.radial_reach_m,
                "withinJointLimits": solution.within_joint_limits,
                "jointLimitViolations": list(solution.joint_limit_violations),
                "jointsRad": {
                    "baseYaw": solution.joint_angles.base_yaw,
                    "shoulderPitch": solution.joint_angles.shoulder_pitch,
                    "elbowPitch": solution.joint_angles.elbow_pitch,
                    "wristPitch": solution.joint_angles.wrist_pitch,
                    "gripper": solution.joint_angles.gripper,
                },
            }
            typed_message.ik_reachable = solution.reachable
            typed_message.ik_reason = solution.reason
            typed_message.ik_target_x_m = solution.target_x_m
            typed_message.ik_target_y_m = solution.target_y_m
            typed_message.ik_target_z_m = solution.target_z_m
            typed_message.ik_radial_reach_m = solution.radial_reach_m
            typed_message.ik_within_joint_limits = solution.within_joint_limits
            typed_message.ik_joint_limit_violations = list(solution.joint_limit_violations)
            typed_message.ik_base_yaw_rad = solution.joint_angles.base_yaw
            typed_message.ik_shoulder_pitch_rad = solution.joint_angles.shoulder_pitch
            typed_message.ik_elbow_pitch_rad = solution.joint_angles.elbow_pitch
            typed_message.ik_wrist_pitch_rad = solution.joint_angles.wrist_pitch
            typed_message.ik_gripper_rad = solution.joint_angles.gripper

        raw_json = compact_json(payload)
        typed_message.raw_json = raw_json
        self.kinematics_typed_pub.publish(typed_message)

        json_message = String()
        json_message.data = raw_json
        self.kinematics_pub.publish(json_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionBrainKinematicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
