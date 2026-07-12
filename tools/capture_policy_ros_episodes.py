#!/usr/bin/env python3

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import ControlGuard
from motionbrain_msgs.msg import MissionState
from sensor_msgs.msg import JointState

from capture_policy_episodes import capture_policy_episodes
from capture_policy_episodes import safe_session_name
from capture_policy_episodes import url_with_path


def stamp_dict(stamp: Any) -> dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def message_dict(message: Any) -> dict[str, Any]:
    if isinstance(message, ControlGuard):
        return {
            "stamp": stamp_dict(message.stamp),
            "ready": message.ready,
            "reason": message.reason,
            "suggestedAction": message.suggested_action,
            "statusFresh": message.status_fresh,
            "detectionFresh": message.detection_fresh,
            "statusAgeSec": float(message.status_age_sec),
            "detectionAgeSec": float(message.detection_age_sec),
            "state": message.state,
            "armed": message.armed,
            "moving": message.moving,
            "faulted": message.faulted,
            "cameraAvailable": message.camera_available,
            "targetDetected": message.target_detected,
            "alignment": message.alignment,
            "rawJson": message.raw_json,
        }
    if isinstance(message, MissionState):
        return {
            "stamp": stamp_dict(message.stamp),
            "state": message.state,
            "reason": message.reason,
            "nextStep": message.next_step,
            "suggestedAction": message.suggested_action,
            "guardReady": message.guard_ready,
            "guardReason": message.guard_reason,
            "statusFresh": message.status_fresh,
            "detectionFresh": message.detection_fresh,
            "targetDetected": message.target_detected,
            "alignment": message.alignment,
            "areaRatio": float(message.area_ratio),
            "rawJson": message.raw_json,
        }
    if isinstance(message, CameraDetection):
        return {
            "stamp": stamp_dict(message.stamp),
            "available": message.available,
            "detected": message.detected,
            "targetType": message.target_type,
            "label": message.label,
            "classId": message.class_id,
            "confidence": float(message.confidence),
            "alignment": message.alignment,
            "commandSuggestion": message.command_suggestion,
            "areaRatio": float(message.area_ratio),
            "cameraUrl": message.camera_url,
            "reason": message.reason,
            "rawJson": message.raw_json,
        }
    if isinstance(message, JointState):
        return {
            "stamp": stamp_dict(message.header.stamp),
            "frameId": message.header.frame_id,
            "name": list(message.name),
            "position": [float(value) for value in message.position],
            "velocity": [float(value) for value in message.velocity],
            "effort": [float(value) for value in message.effort],
        }
    raise TypeError(f"unsupported message type: {type(message).__name__}")


class RosSnapshotCache:
    def __init__(self, max_age_sec: float) -> None:
        self.node = rclpy.create_node("motionbrain_policy_episode_recorder")
        self.max_age_sec = max_age_sec
        self.lock = threading.Lock()
        self.latest: dict[str, tuple[float, dict[str, Any]]] = {}
        subscriptions = [
            ("controlGuard", ControlGuard, "/motionbrain/control_guard_typed"),
            ("missionState", MissionState, "/motionbrain/mission_state_typed"),
            ("rosDetection", CameraDetection, "/camera/detection_typed"),
            ("jointState", JointState, "/joint_states"),
        ]
        self.subscriptions = [
            self.node.create_subscription(
                message_type,
                topic,
                lambda message, key=key: self.on_message(key, message),
                10,
            )
            for key, message_type, topic in subscriptions
        ]
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()

    def on_message(self, key: str, message: Any) -> None:
        with self.lock:
            self.latest[key] = (time.monotonic(), message_dict(message))

    def wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        required = {"controlGuard", "missionState", "rosDetection", "jointState"}
        while time.monotonic() < deadline:
            with self.lock:
                if required.issubset(self.latest):
                    return
            time.sleep(0.05)
        with self.lock:
            missing = sorted(required - set(self.latest))
        raise TimeoutError(f"ROS snapshot topics unavailable: {','.join(missing)}")

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        result: dict[str, Any] = {}
        ages: dict[str, float] = {}
        with self.lock:
            items = dict(self.latest)
        for key, (received_at, payload) in items.items():
            age = now - received_at
            ages[key] = round(age, 6)
            if age > self.max_age_sec:
                raise TimeoutError(f"ROS snapshot stale: {key} age={age:.3f}s")
            result[key] = payload
        result["rosSnapshotMeta"] = {
            "capturedAtMonotonic": now,
            "maxAgeSec": self.max_age_sec,
            "agesSec": ages,
        }
        return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture synchronized HTTP and ROS policy episodes.")
    parser.add_argument("--output-dir", default="datasets/policy")
    parser.add_argument("--session-name", default="")
    parser.add_argument("--label", default="policy-ros")
    parser.add_argument("--dashboard-url", default="http://127.0.0.1:8765")
    parser.add_argument("--status-url", required=True)
    parser.add_argument("--perception-url", default="http://127.0.0.1:8766")
    parser.add_argument("--policy-url", required=True)
    parser.add_argument("--instruction", default="")
    parser.add_argument("--operator-action", default="")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--ros-ready-timeout", type=float, default=10.0)
    parser.add_argument("--ros-max-age", type=float, default=2.0)
    parser.add_argument("--notes", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    cache = RosSnapshotCache(args.ros_max_age)
    cache.start()
    try:
        cache.wait_ready(args.ros_ready_timeout)
        dataset = capture_policy_episodes(
            output_root=Path(args.output_dir),
            session_name=args.session_name.strip() or safe_session_name(args.label),
            label=args.label,
            frame_url=url_with_path(args.dashboard_url, "/api/vision_frame"),
            status_url=args.status_url,
            detection_url=url_with_path(args.perception_url, "/api/detection"),
            policy_url=args.policy_url,
            instruction=args.instruction,
            operator_action=args.operator_action,
            count=args.count,
            interval=args.interval,
            timeout=args.timeout,
            notes=args.notes,
            required_sources=(
                "frame",
                "status",
                "detection",
                "policyProposal",
                "controlGuard",
                "missionState",
                "rosDetection",
                "jointState",
            ),
            derive_control_guard=False,
            snapshot_func=cache.snapshot,
        )
        print(f"captured ROS policy episodes: {dataset}")
    finally:
        cache.close()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
