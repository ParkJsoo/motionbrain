#!/usr/bin/env python3

import json
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import LightCommand
from motionbrain_msgs.msg import LightResult
from motionbrain_msgs.msg import MotionEvent
from motionbrain_msgs.msg import MotionStatus
from motionbrain_msgs.msg import RoutineStatus
from motionbrain_ros_bridge.payload_utils import ALIGN_DEADBAND
from motionbrain_ros_bridge.payload_utils import as_bool
from motionbrain_ros_bridge.payload_utils import as_float
from motionbrain_ros_bridge.payload_utils import as_str
from motionbrain_ros_bridge.payload_utils import as_uint
from motionbrain_ros_bridge.payload_utils import compact_json
from motionbrain_ros_bridge.payload_utils import perception_detection_url
from motionbrain_ros_bridge.payload_utils import parse_light_action
from motionbrain_ros_bridge.vision_detection import detect_colored_target
from rclpy.node import Node
from std_msgs.msg import String


NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)
POLL_EXCEPTIONS = NETWORK_EXCEPTIONS + (json.JSONDecodeError,)
PERCEPTION_EXCEPTIONS = POLL_EXCEPTIONS + (ValueError,)


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> bytes:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def post_motionbrain(base_url: str, path: str, timeout: float, token: str = "") -> dict[str, Any]:
    headers = {"X-MotionBrain": "1"}
    if token:
        headers["X-MotionBrain-Token"] = token
    request = urllib.request.Request(
        f"{base_url}{path}",
        method="POST",
        headers=headers,
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


class MotionBrainStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_status_node")

        self.declare_parameter("motion_host", "192.168.4.1")
        self.declare_parameter("motion_port", 80)
        self.declare_parameter("camera_url", "")
        self.declare_parameter("perception_url", os.environ.get("MOTIONBRAIN_PERCEPTION_URL", ""))
        self.declare_parameter("detect_color", "red")
        self.declare_parameter("poll_interval", 1.0)
        self.declare_parameter("http_timeout", 2.0)
        self.declare_parameter("events_limit", 8)
        self.declare_parameter("http_token", os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""))

        self.motion_base_url = self._motion_base_url()
        self.status_pub = self.create_publisher(String, "/motionbrain/status", 10)
        self.routine_pub = self.create_publisher(String, "/motionbrain/routine", 10)
        self.events_pub = self.create_publisher(String, "/motionbrain/events", 10)
        self.detection_pub = self.create_publisher(String, "/camera/detection", 10)
        self.light_result_pub = self.create_publisher(String, "/motionbrain/light_result", 10)
        self.status_typed_pub = self.create_publisher(MotionStatus, "/motionbrain/status_typed", 10)
        self.routine_typed_pub = self.create_publisher(RoutineStatus, "/motionbrain/routine_typed", 10)
        self.events_typed_pub = self.create_publisher(MotionEvent, "/motionbrain/events_typed", 10)
        self.detection_typed_pub = self.create_publisher(
            CameraDetection,
            "/camera/detection_typed",
            10,
        )
        self.light_result_typed_pub = self.create_publisher(
            LightResult,
            "/motionbrain/light_result_typed",
            10,
        )
        self.light_sub = self.create_subscription(
            String,
            "/motionbrain/light_cmd",
            self.handle_light_cmd,
            10,
        )
        self.light_typed_sub = self.create_subscription(
            LightCommand,
            "/motionbrain/light_cmd_typed",
            self.handle_light_cmd_typed,
            10,
        )

        interval = float(self.get_parameter("poll_interval").value)
        self.timer = self.create_timer(max(interval, 0.1), self.poll_once)
        self.get_logger().info(
            f"MotionBrain ROS2 bridge polling {self.motion_base_url}; "
            "topics: /motionbrain/status /motionbrain/status_typed "
            "/motionbrain/routine /motionbrain/routine_typed "
            "/motionbrain/events /motionbrain/events_typed "
            "/camera/detection /camera/detection_typed "
            "/motionbrain/light_cmd /motionbrain/light_cmd_typed"
        )

    def _motion_base_url(self) -> str:
        host = str(self.get_parameter("motion_host").value)
        port = int(self.get_parameter("motion_port").value)
        return f"http://{host}:{port}"

    def _timeout(self) -> float:
        return float(self.get_parameter("http_timeout").value)

    def publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = compact_json(payload)
        publisher.publish(message)

    def publish_status_typed(self, payload: dict[str, Any]) -> None:
        message = MotionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.available = True
        message.state = as_str(payload.get("state"), "UNKNOWN")
        message.armed = message.state.upper() in {"ARMED", "RUNNING", "MOVING"}
        message.moving = as_bool(payload.get("motorEnabled"))
        message.faulted = message.state.upper() == "FAULT"

        sensor = payload.get("sensor")
        if isinstance(sensor, dict):
            message.faulted = message.faulted or as_bool(sensor.get("faultLatched"))

        base_angle = payload.get("baseAngle")
        if isinstance(base_angle, dict):
            message.base_angle_deg = as_float(base_angle.get("currentDeg"))
            message.moving = message.moving or as_bool(base_angle.get("active"))
            message.stop_reason = as_str(base_angle.get("lastStopReason"))

        teleop = payload.get("teleop")
        if isinstance(teleop, dict):
            message.moving = message.moving or as_bool(teleop.get("controlActive"))
            if not message.stop_reason:
                message.stop_reason = as_str(teleop.get("lastStopReason"))

        message.raw_json = compact_json(payload)
        self.status_typed_pub.publish(message)

    def publish_events_typed(self, payload: dict[str, Any]) -> None:
        events = payload.get("events")
        if not isinstance(events, list):
            return

        for index, event in enumerate(events):
            if not isinstance(event, dict):
                continue
            message = MotionEvent()
            message.stamp = self.get_clock().now().to_msg()
            message.index = as_uint(event.get("id"), index)
            message.event_type = as_str(event.get("code") or event.get("severity"))
            message.message = as_str(event.get("detail"))
            message.source = as_str(event.get("category"))
            message.raw_json = compact_json(event)
            self.events_typed_pub.publish(message)

    def publish_routine_typed(self, payload: dict[str, Any]) -> None:
        message = RoutineStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.available = True
        message.controller_state = as_str(payload.get("state"), "UNKNOWN")
        message.dry_run_only = as_bool(payload.get("dryRunOnly"))
        message.execute_implemented = as_bool(payload.get("executeImplemented"))

        executor = payload.get("executor")
        if isinstance(executor, dict):
            message.executor_enabled = as_bool(executor.get("enabled"))
            message.executor_mode = as_str(executor.get("mode"))
            message.abort_supported = as_bool(executor.get("abortSupported"))
            message.timeout_supported = as_bool(executor.get("timeoutSupported"))
            message.materialization_gate_supported = as_bool(
                executor.get("materializationGateSupported")
            )
            message.queue_apply_allowed = as_bool(executor.get("queueApplyAllowed"))

            status = executor.get("status")
            if isinstance(status, dict):
                message.executor_state = as_str(status.get("state"))
                message.routine_name = as_str(status.get("routineName"))
                message.current_step = as_uint(status.get("currentStep"))
                message.total_steps = as_uint(status.get("totalSteps"))
                message.motion_step_count = as_uint(status.get("motionStepCount"))
                message.remaining_ms = as_uint(status.get("remainingMs"))
                message.executor_last_result = as_str(status.get("lastResult"))
                message.executor_last_detail = as_str(status.get("lastDetail"))

        diagnostics = payload.get("diagnostics")
        if isinstance(diagnostics, dict):
            sensor = diagnostics.get("sensor")
            if isinstance(sensor, dict):
                message.sensor_connected = as_bool(sensor.get("connected"))
                message.sensor_fresh = as_bool(sensor.get("fresh"))
                message.sensor_age_ms = as_uint(sensor.get("ageMs"))

            teleop = diagnostics.get("teleop")
            if isinstance(teleop, dict):
                message.teleop_connected = as_bool(teleop.get("connected"))
                message.teleop_deadman = as_bool(teleop.get("deadman"))
                message.teleop_control_active = as_bool(teleop.get("controlActive"))
                message.teleop_age_ms = as_uint(teleop.get("ageMs"))

            safety = diagnostics.get("safety")
            if isinstance(safety, dict):
                message.safety_motion_blocked = as_bool(safety.get("motionBlocked"))
                message.safety_block_reason = as_str(safety.get("blockReason"))
                message.safety_fault_latched = as_bool(safety.get("faultLatched"))
                message.safety_fault_reason = as_str(safety.get("faultReason"))

        recovery = payload.get("recovery")
        if isinstance(recovery, dict):
            message.recovery_action = as_str(recovery.get("action"))

        last_command = payload.get("lastCommand")
        if isinstance(last_command, dict):
            message.last_command_seen = as_bool(last_command.get("seen"))
            message.last_command_success = as_bool(last_command.get("success"))
            message.last_command_type = as_str(last_command.get("type"))
            message.last_command_source = as_str(last_command.get("source"))
            message.last_command_message = as_str(last_command.get("message"))

        routines = payload.get("routines")
        if isinstance(routines, list):
            routine_names = [
                as_str(routine.get("name"))
                for routine in routines
                if isinstance(routine, dict) and as_str(routine.get("name"))
            ]
            message.routine_names = routine_names
            message.routine_count = len(routine_names)

        message.raw_json = compact_json(payload)
        self.routine_typed_pub.publish(message)

    def publish_detection_typed(self, payload: dict[str, Any]) -> None:
        message = CameraDetection()
        message.stamp = self.get_clock().now().to_msg()
        message.available = as_bool(payload.get("available"))
        message.detected = as_bool(payload.get("detected"))
        message.target_type = as_str(payload.get("targetType"))
        message.label = as_str(payload.get("label") or payload.get("color"))
        try:
            message.class_id = int(payload["classId"]) if payload.get("classId") is not None else -1
        except (TypeError, ValueError):
            message.class_id = -1
        message.confidence = as_float(payload.get("confidence"))
        message.color = as_str(payload.get("color"))
        message.alignment = as_str(payload.get("alignment"), "LOST")
        message.command_suggestion = as_str(payload.get("commandSuggestion"), "none")
        message.area_ratio = as_float(payload.get("areaRatio", payload.get("ratio")))
        message.pixels = as_uint(payload.get("pixels"))
        message.width = as_uint(payload.get("width"))
        message.height = as_uint(payload.get("height"))
        message.frame_bytes = as_uint(payload.get("frameBytes"))
        message.center_x = as_float(payload.get("centerX", payload.get("centroidX")))
        message.center_y = as_float(payload.get("centerY", payload.get("centroidY")))
        message.offset_x = as_float(payload.get("offsetX"))
        message.offset_y = as_float(payload.get("offsetY"))
        message.align_deadband = as_float(payload.get("alignDeadband"), ALIGN_DEADBAND)
        message.camera_url = as_str(payload.get("cameraUrl"))
        message.reason = as_str(payload.get("reason"))
        message.raw_json = compact_json(payload)
        self.detection_typed_pub.publish(message)

    def publish_light_result_typed(self, payload: dict[str, Any]) -> None:
        message = LightResult()
        message.stamp = self.get_clock().now().to_msg()
        message.success = as_bool(payload.get("success"))
        message.requested_action = as_str(payload.get("requestedAction"))
        message.state = as_str(payload.get("state"))
        message.error = as_str(payload.get("error") or payload.get("message"))
        message.raw_json = compact_json(payload)
        self.light_result_typed_pub.publish(message)

    def publish_light_result(self, payload: dict[str, Any]) -> None:
        self.publish_json(self.light_result_pub, payload)
        self.publish_light_result_typed(payload)

    def poll_once(self) -> None:
        self.motion_base_url = self._motion_base_url()
        timeout = self._timeout()

        try:
            status = fetch_json(f"{self.motion_base_url}/status", timeout)
            self.publish_json(self.status_pub, status)
            self.publish_status_typed(status)
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"status poll failed: {exc}")

        try:
            routine = fetch_json(f"{self.motion_base_url}/routine", timeout)
            self.publish_json(self.routine_pub, routine)
            self.publish_routine_typed(routine)
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"routine poll failed: {exc}")

        try:
            limit = int(self.get_parameter("events_limit").value)
            if limit > 0:
                events = fetch_json(f"{self.motion_base_url}/events?limit={limit}", timeout)
                self.publish_json(self.events_pub, events)
                self.publish_events_typed(events)
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"events poll failed: {exc}")

        perception_url = str(self.get_parameter("perception_url").value).strip().rstrip("/")
        camera_url = str(self.get_parameter("camera_url").value).strip().rstrip("/")
        if perception_url:
            self.poll_perception(perception_url, timeout)
        elif camera_url:
            self.poll_camera(camera_url, timeout)

    def poll_perception(self, perception_url: str, timeout: float) -> None:
        try:
            detection = fetch_json(perception_detection_url(perception_url), timeout)
            if not isinstance(detection, dict):
                raise ValueError("perception_detection_not_object")
            detection.setdefault("ts", time.time())
            detection.setdefault("perceptionUrl", perception_url)
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        except PERCEPTION_EXCEPTIONS as exc:
            detection = {
                "detected": False,
                "available": False,
                "perceptionUrl": perception_url,
                "reason": str(exc),
                "ts": time.time(),
            }
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)

    def poll_camera(self, camera_url: str, timeout: float) -> None:
        color = str(self.get_parameter("detect_color").value)
        try:
            frame = fetch_bytes(f"{camera_url}/capture", timeout)
            detection = detect_colored_target(frame, color)
            detection["ts"] = time.time()
            detection["cameraUrl"] = camera_url
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        except NETWORK_EXCEPTIONS as exc:
            detection = {
                "detected": False,
                "available": False,
                "cameraUrl": camera_url,
                "reason": str(exc),
                "ts": time.time(),
            }
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)

    def handle_light_cmd(self, message: String) -> None:
        action = parse_light_action(message.data)
        self.handle_light_action(action, message.data)

    def handle_light_cmd_typed(self, message: LightCommand) -> None:
        raw_payload = message.raw_json or message.action
        action = parse_light_action(message.action or raw_payload)
        self.handle_light_action(action, raw_payload)

    def handle_light_action(self, action: str | None, raw_payload: str) -> None:
        if action is None:
            self.publish_light_result(
                {
                    "success": False,
                    "error": "invalid_light_action",
                    "accepted": ["on", "off", "toggle"],
                    "payload": raw_payload,
                },
            )
            return

        try:
            path = f"/light?action={urllib.parse.quote(action)}"
            token = str(self.get_parameter("http_token").value)
            result = post_motionbrain(self.motion_base_url, path, self._timeout(), token)
            result["requestedAction"] = action
            self.publish_light_result(result)
        except POLL_EXCEPTIONS as exc:
            self.publish_light_result(
                {
                    "success": False,
                    "requestedAction": action,
                    "error": str(exc),
                },
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionBrainStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
