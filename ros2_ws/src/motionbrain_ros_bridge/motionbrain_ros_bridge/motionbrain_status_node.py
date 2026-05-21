#!/usr/bin/env python3

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TARGET_RATIO_THRESHOLD = 0.02
ALIGN_DEADBAND = 0.15


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> bytes:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def post_motionbrain(base_url: str, path: str, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(
        f"{base_url}{path}",
        method="POST",
        headers={"X-MotionBrain": "1"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def parse_light_action(payload: str) -> str | None:
    text = payload.strip().lower()
    if text in {"on", "off", "toggle"}:
        return text

    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None

    if not isinstance(data, dict):
        return None

    action = str(data.get("action", "")).strip().lower()
    if action in {"on", "off", "toggle"}:
        return action
    return None


def classify_alignment(offset_x: float | None, deadband: float = ALIGN_DEADBAND) -> str:
    if offset_x is None:
        return "unknown"
    if offset_x < -deadband:
        return "left"
    if offset_x > deadband:
        return "right"
    return "centered"


def detect_colored_target(frame: bytes, color: str) -> dict[str, Any]:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return {
            "detected": False,
            "color": color,
            "available": False,
            "reason": "opencv_unavailable",
            "alignment": "unknown",
        }

    data = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return {
            "detected": False,
            "color": color,
            "available": True,
            "reason": "decode_failed",
            "alignment": "unknown",
        }

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "green":
        mask = cv2.inRange(hsv, (40, 70, 70), (85, 255, 255))
    elif color == "blue":
        mask = cv2.inRange(hsv, (95, 70, 70), (130, 255, 255))
    else:
        return {
            "detected": False,
            "color": color,
            "available": True,
            "reason": "unsupported_color",
            "alignment": "unknown",
        }

    pixels = int(cv2.countNonZero(mask))
    height, width = image.shape[:2]
    area = max(height * width, 1)
    ratio = pixels / area
    detected = ratio >= TARGET_RATIO_THRESHOLD
    centroid_x: float | None = None
    centroid_y: float | None = None
    offset_x: float | None = None
    offset_y: float | None = None

    if detected and pixels > 0:
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            centroid_x = float(moments["m10"] / moments["m00"])
            centroid_y = float(moments["m01"] / moments["m00"])
            center_x = (width - 1) / 2.0
            center_y = (height - 1) / 2.0
            offset_x = (centroid_x - center_x) / max(center_x, 1.0)
            offset_y = (centroid_y - center_y) / max(center_y, 1.0)

    return {
        "detected": detected,
        "color": color,
        "available": True,
        "ratio": ratio,
        "pixels": pixels,
        "width": width,
        "height": height,
        "frameBytes": len(frame),
        "centroidX": centroid_x,
        "centroidY": centroid_y,
        "offsetX": offset_x,
        "offsetY": offset_y,
        "alignDeadband": ALIGN_DEADBAND,
        "alignment": classify_alignment(offset_x) if detected else "not_detected",
    }


class MotionBrainStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("motionbrain_status_node")

        self.declare_parameter("motion_host", "192.168.4.1")
        self.declare_parameter("motion_port", 80)
        self.declare_parameter("camera_url", "")
        self.declare_parameter("detect_color", "red")
        self.declare_parameter("poll_interval", 1.0)
        self.declare_parameter("http_timeout", 2.0)
        self.declare_parameter("events_limit", 8)

        self.motion_base_url = self._motion_base_url()
        self.status_pub = self.create_publisher(String, "/motionbrain/status", 10)
        self.events_pub = self.create_publisher(String, "/motionbrain/events", 10)
        self.detection_pub = self.create_publisher(String, "/camera/detection", 10)
        self.light_result_pub = self.create_publisher(String, "/motionbrain/light_result", 10)
        self.light_sub = self.create_subscription(
            String,
            "/motionbrain/light_cmd",
            self.handle_light_cmd,
            10,
        )

        interval = float(self.get_parameter("poll_interval").value)
        self.timer = self.create_timer(max(interval, 0.1), self.poll_once)
        self.get_logger().info(
            f"MotionBrain ROS2 bridge polling {self.motion_base_url}; "
            "topics: /motionbrain/status /motionbrain/events /camera/detection /motionbrain/light_cmd"
        )

    def _motion_base_url(self) -> str:
        host = str(self.get_parameter("motion_host").value)
        port = int(self.get_parameter("motion_port").value)
        return f"http://{host}:{port}"

    def _timeout(self) -> float:
        return float(self.get_parameter("http_timeout").value)

    def publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        publisher.publish(message)

    def poll_once(self) -> None:
        self.motion_base_url = self._motion_base_url()
        timeout = self._timeout()

        try:
            status = fetch_json(f"{self.motion_base_url}/status", timeout)
            self.publish_json(self.status_pub, status)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"status poll failed: {exc}")

        try:
            limit = int(self.get_parameter("events_limit").value)
            if limit > 0:
                events = fetch_json(f"{self.motion_base_url}/events?limit={limit}", timeout)
                self.publish_json(self.events_pub, events)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"events poll failed: {exc}")

        camera_url = str(self.get_parameter("camera_url").value).strip().rstrip("/")
        if camera_url:
            self.poll_camera(camera_url, timeout)

    def poll_camera(self, camera_url: str, timeout: float) -> None:
        color = str(self.get_parameter("detect_color").value)
        try:
            frame = fetch_bytes(f"{camera_url}/capture", timeout)
            detection = detect_colored_target(frame, color)
            detection["ts"] = time.time()
            detection["cameraUrl"] = camera_url
            self.publish_json(self.detection_pub, detection)
        except (urllib.error.URLError, TimeoutError) as exc:
            self.publish_json(
                self.detection_pub,
                {
                    "detected": False,
                    "available": False,
                    "cameraUrl": camera_url,
                    "reason": str(exc),
                    "ts": time.time(),
                },
            )

    def handle_light_cmd(self, message: String) -> None:
        action = parse_light_action(message.data)
        if action is None:
            self.publish_json(
                self.light_result_pub,
                {
                    "success": False,
                    "error": "invalid_light_action",
                    "accepted": ["on", "off", "toggle"],
                    "payload": message.data,
                },
            )
            return

        try:
            path = f"/light?action={urllib.parse.quote(action)}"
            result = post_motionbrain(self.motion_base_url, path, self._timeout())
            result["requestedAction"] = action
            self.publish_json(self.light_result_pub, result)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            self.publish_json(
                self.light_result_pub,
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
