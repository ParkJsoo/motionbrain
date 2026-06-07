#!/usr/bin/env python3

import json
import os
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import as_completed
from ipaddress import ip_network
from typing import Any

import rclpy
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import MotionStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from motionbrain_ros_bridge.payload_utils import ALIGN_DEADBAND
from motionbrain_ros_bridge.payload_utils import as_bool
from motionbrain_ros_bridge.payload_utils import as_float
from motionbrain_ros_bridge.payload_utils import as_str
from motionbrain_ros_bridge.payload_utils import as_uint
from motionbrain_ros_bridge.payload_utils import compact_json


CONFIG_ORIGIN_HEADERS = {"Origin": "http://motionbrain.local"}


def fetch_json(url: str, timeout: float, headers: dict[str, str] | None = None) -> dict[str, Any]:
    request = urllib.request.Request(url, headers=headers or {})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"not_object: {url}")
    return payload


def env_list(name: str, default: str) -> list[str]:
    value = os.environ.get(name, default)
    return [item.strip() for item in value.split(",") if item.strip()]


def is_dashboard_url(url: str, timeout: float) -> bool:
    payload = fetch_json(f"{url.rstrip('/')}/api/config", timeout, CONFIG_ORIGIN_HEADERS)
    return bool(payload.get("ok")) and ("cameraUrl" in payload or "motionBaseUrl" in payload)


def discover_dashboard_url(configured_url: str, timeout: float) -> str:
    candidates: list[str] = []
    default_candidates = "http://motionbrain-pi.local:8765,http://motionbrain-pi.davolink:8765"
    for url in [configured_url, *env_list("MOTIONBRAIN_DASHBOARD_CANDIDATES", default_candidates)]:
        normalized = url.rstrip("/")
        if normalized and normalized not in candidates:
            candidates.append(normalized)

    probe_timeout = min(max(timeout, 0.5), 1.0)
    for url in candidates:
        try:
            if is_dashboard_url(url, probe_timeout):
                return url
        except Exception:
            pass

    port = int(os.environ.get("MOTIONBRAIN_DASHBOARD_PORT", "8765"))
    cidrs = env_list("MOTIONBRAIN_DASHBOARD_DISCOVERY_CIDRS", "192.168.219.0/24")
    scan_urls: list[str] = []
    for cidr in cidrs:
        try:
            network = ip_network(cidr, strict=False)
        except ValueError:
            continue
        for host in network.hosts():
            scan_urls.append(f"http://{host}:{port}")

    if not scan_urls:
        return configured_url.rstrip("/")

    workers = max(1, min(int(os.environ.get("MOTIONBRAIN_DASHBOARD_DISCOVERY_WORKERS", "64")), len(scan_urls)))
    scan_timeout = float(os.environ.get("MOTIONBRAIN_DASHBOARD_DISCOVERY_TIMEOUT", "0.7"))
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(is_dashboard_url, url, scan_timeout): url for url in scan_urls}
        for future in as_completed(futures):
            url = futures[future]
            try:
                if future.result():
                    return url
            except Exception:
                pass
    return configured_url.rstrip("/")


class MotionBrainHttpMirror(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_http_mirror")

        self.declare_parameter("dashboard_url", os.environ.get("MOTIONBRAIN_DASHBOARD_URL", "http://motionbrain-pi.local:8765"))
        self.declare_parameter("poll_interval", float(os.environ.get("MOTIONBRAIN_MIRROR_POLL_INTERVAL", "2.0")))
        self.declare_parameter("timeout", float(os.environ.get("MOTIONBRAIN_MIRROR_TIMEOUT", "5.0")))

        configured_dashboard_url = str(self.get_parameter("dashboard_url").value).rstrip("/")
        self.timeout = float(self.get_parameter("timeout").value)
        self.discovery_enabled = os.environ.get("MOTIONBRAIN_DASHBOARD_DISCOVERY", "1") != "0"
        self.discovery_retry_seconds = float(os.environ.get("MOTIONBRAIN_DASHBOARD_DISCOVERY_RETRY_SECONDS", "30.0"))
        self.last_discovery_attempt = 0.0
        self.dashboard_url = configured_dashboard_url
        if self.discovery_enabled:
            self.dashboard_url = discover_dashboard_url(configured_dashboard_url, self.timeout)
            if self.dashboard_url != configured_dashboard_url:
                self.set_parameters([Parameter("dashboard_url", Parameter.Type.STRING, self.dashboard_url)])

        self.status_pub = self.create_publisher(String, "/motionbrain/status", 10)
        self.status_typed_pub = self.create_publisher(MotionStatus, "/motionbrain/status_typed", 10)
        self.detection_pub = self.create_publisher(String, "/camera/detection", 10)
        self.detection_typed_pub = self.create_publisher(CameraDetection, "/camera/detection_typed", 10)

        poll_interval = max(float(self.get_parameter("poll_interval").value), 0.5)
        self.timer = self.create_timer(poll_interval, self.poll_once)
        self.get_logger().info(f"Mirroring MotionBrain dashboard API from {self.dashboard_url}")

    def maybe_rediscover_dashboard(self) -> None:
        if not self.discovery_enabled:
            return
        now = time.monotonic()
        if now - self.last_discovery_attempt < self.discovery_retry_seconds:
            return
        self.last_discovery_attempt = now
        discovered_url = discover_dashboard_url(self.dashboard_url, self.timeout)
        if discovered_url and discovered_url != self.dashboard_url:
            self.dashboard_url = discovered_url
            self.set_parameters([Parameter("dashboard_url", Parameter.Type.STRING, self.dashboard_url)])
            self.get_logger().info(f"Rediscovered MotionBrain dashboard API at {self.dashboard_url}")

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

    def poll_once(self) -> None:
        failed = False
        try:
            status = fetch_json(f"{self.dashboard_url}/api/status", self.timeout)
            status.setdefault("mirrorTs", time.time())
            self.publish_json(self.status_pub, status)
            self.publish_status_typed(status)
        except Exception as exc:
            failed = True
            self.get_logger().warning(f"status mirror failed: {exc}")

        try:
            detection = fetch_json(f"{self.dashboard_url}/api/detection", self.timeout)
            detection.setdefault("mirrorTs", time.time())
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        except Exception as exc:
            failed = True
            self.get_logger().warning(f"detection mirror failed: {exc}")

        if failed:
            self.maybe_rediscover_dashboard()


def main() -> None:
    rclpy.init()
    node = MotionBrainHttpMirror()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
