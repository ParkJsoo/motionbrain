#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROS_BRIDGE_SRC = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "motionbrain_ros_bridge"
if str(ROS_BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(ROS_BRIDGE_SRC))

from motionbrain_ros_bridge.vision_detection import DetectionConfig  # noqa: E402
from motionbrain_ros_bridge.vision_detection import DetectorBackend  # noqa: E402
from motionbrain_ros_bridge.vision_detection import OpenCvDnnObjectDetector  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_frame  # noqa: E402


NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)


def fetch_bytes(url: str, timeout: float) -> tuple[bytes, str]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "application/octet-stream")
        return response.read(), content_type


class PerceptionState:
    def __init__(
        self,
        camera_url: str,
        config: DetectionConfig,
        *,
        detector: DetectorBackend | None = None,
        timeout: float = 2.0,
        interval: float = 0.35,
        stale_seconds: float = 2.0,
    ) -> None:
        self.camera_url = camera_url.rstrip("/")
        self.config = config
        self.detector = detector
        self.timeout = timeout
        self.interval = interval
        self.stale_seconds = stale_seconds
        self.lock = threading.Lock()
        self.latest_frame: tuple[float, bytes, str] | None = None
        self.latest_detection: dict[str, Any] | None = None
        self.last_error = ""
        self.frames_total = 0
        self.detect_total = 0
        self.error_total = 0

    def run_once(self) -> None:
        frame, content_type = fetch_bytes(f"{self.camera_url}/capture", self.timeout)
        detection = detect_frame(frame, self.config, self.detector)
        now = time.time()
        detection["cameraUrl"] = self.camera_url
        detection["ts"] = now
        detection["contentType"] = content_type
        with self.lock:
            self.latest_frame = (now, frame, content_type)
            self.latest_detection = detection
            self.last_error = ""
            self.frames_total += 1
            self.detect_total += 1

    def mark_error(self, exc: BaseException) -> None:
        with self.lock:
            self.last_error = str(exc)
            self.error_total += 1

    def run_loop(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            started = time.monotonic()
            try:
                self.run_once()
            except Exception as exc:
                self.mark_error(exc)
            elapsed = time.monotonic() - started
            stop_event.wait(max(self.interval - elapsed, 0.0))

    def detection_payload(self) -> dict[str, Any]:
        with self.lock:
            detection = dict(self.latest_detection) if self.latest_detection is not None else None
            last_error = self.last_error

        if detection is None:
            return {
                "available": False,
                "detected": False,
                "cameraUrl": self.camera_url,
                "reason": last_error or "no_frame",
                "ts": time.time(),
                "alignment": "LOST",
                "commandSuggestion": "none",
            }

        now = time.time()
        age_ms = max((now - float(detection.get("ts", now))) * 1000.0, 0.0)
        detection["ageMs"] = age_ms
        detection["fresh"] = age_ms <= self.stale_seconds * 1000.0
        if last_error:
            detection["lastError"] = last_error
        return detection

    def perception_payload(self) -> dict[str, Any]:
        detection = self.detection_payload()
        return {
            "ok": bool(detection.get("available")) and bool(detection.get("fresh", False)),
            "cameraUrl": self.camera_url,
            "detection": detection,
            "health": self.health_payload(),
        }

    def health_payload(self) -> dict[str, Any]:
        with self.lock:
            frame = self.latest_frame
            detection = self.latest_detection
            last_error = self.last_error
            frames_total = self.frames_total
            detect_total = self.detect_total
            error_total = self.error_total

        now = time.time()
        frame_age_ms = None if frame is None else max((now - frame[0]) * 1000.0, 0.0)
        fresh = frame_age_ms is not None and frame_age_ms <= self.stale_seconds * 1000.0
        detector = detection.get("detector", {}) if isinstance(detection, dict) else {}
        return {
            "ok": fresh and not last_error,
            "cameraUrl": self.camera_url,
            "mode": self.config.mode,
            "detectColor": self.config.color,
            "objectTarget": self.config.object_target,
            "objectBackend": self.config.object_backend,
            "objectModel": self.config.object_model,
            "objectLabels": self.config.object_labels,
            "targetPolicy": self.config.target_policy,
            "fresh": fresh,
            "frameAgeMs": frame_age_ms,
            "framesTotal": frames_total,
            "detectTotal": detect_total,
            "errorTotal": error_total,
            "lastError": last_error,
            "detector": detector,
            "detectorConfigured": self.detector is not None,
        }

    def frame_payload(self) -> tuple[bytes, str] | None:
        with self.lock:
            if self.latest_frame is None:
                return None
            _, frame, content_type = self.latest_frame
            return frame, content_type


class PerceptionHandler(BaseHTTPRequestHandler):
    server: "PerceptionServer"

    def log_message(self, fmt: str, *args: object) -> None:
        message = fmt % args
        if message.startswith('"GET /') and '" 200 ' in message:
            return
        sys.stderr.write(f"[perception] {self.address_string()} {message}\n")

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/health":
                self.send_json(self.server.state.health_payload(), allow_cross_origin=True)
            elif parsed.path == "/api/config":
                self.send_json(self.server.config_payload(), allow_cross_origin=True)
            elif parsed.path == "/api/perception":
                self.send_json(self.server.state.perception_payload(), allow_cross_origin=True)
            elif parsed.path == "/api/detection":
                self.send_json(self.server.state.detection_payload(), allow_cross_origin=True)
            elif parsed.path == "/api/vision_frame":
                self.handle_vision_frame()
            else:
                self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
        except (ValueError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def handle_vision_frame(self) -> None:
        frame_payload = self.server.state.frame_payload()
        if frame_payload is None:
            self.send_error_json(HTTPStatus.SERVICE_UNAVAILABLE, "no_frame")
            return

        frame, content_type = frame_payload
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_cross_origin_headers()
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def send_json(
        self,
        payload: dict[str, Any],
        status: HTTPStatus = HTTPStatus.OK,
        *,
        allow_cross_origin: bool = False,
    ) -> None:
        body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        if allow_cross_origin:
            self.send_cross_origin_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_cross_origin_headers(self) -> None:
        origin = self.headers.get("Origin", "")
        if "*" in self.server.allowed_origins:
            self.send_header("Access-Control-Allow-Origin", "*")
        elif origin in self.server.allowed_origins:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")

    def send_error_json(self, status: HTTPStatus, error: str) -> None:
        self.send_json({"ok": False, "error": error}, status, allow_cross_origin=True)


class PerceptionServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        state: PerceptionState,
        allowed_origins: set[str],
    ) -> None:
        super().__init__(server_address, handler_class)
        self.state = state
        self.allowed_origins = allowed_origins

    def config_payload(self) -> dict[str, Any]:
        config = self.state.config
        return {
            "ok": True,
            "cameraUrl": self.state.camera_url,
            "detectorMode": config.mode,
            "detectColor": config.color,
            "alignDeadband": config.align_deadband,
            "targetRatioThreshold": config.target_ratio_threshold,
            "objectBackend": config.object_backend,
            "objectModel": config.object_model,
            "objectLabels": config.object_labels,
            "objectTarget": config.object_target,
            "objectMinConfidence": config.object_min_confidence,
            "objectNmsThreshold": config.object_nms_threshold,
            "objectInputSize": config.object_input_size,
            "targetPolicy": config.target_policy,
            "detectorConfigured": self.state.detector is not None,
        }


def build_detection_config(args: argparse.Namespace) -> DetectionConfig:
    return DetectionConfig(
        mode=args.detector_mode,
        color=args.detect_color,
        align_deadband=args.align_deadband,
        object_backend=args.object_backend,
        object_model=args.object_model,
        object_labels=args.object_labels,
        object_target=args.object_target,
        object_min_confidence=args.object_min_confidence,
        object_nms_threshold=args.object_nms_threshold,
        object_input_size=args.object_input_size,
        target_policy=args.target_policy,
    )


def build_detector(config: DetectionConfig) -> DetectorBackend | None:
    mode = config.mode.strip().lower()
    if mode == "color":
        return None
    if mode != "object":
        raise ValueError(f"unsupported detector mode: {config.mode}")

    backend = config.object_backend.strip().lower()
    if backend in {"opencv-dnn", "onnx"}:
        return OpenCvDnnObjectDetector.from_model(
            config.object_model,
            config.object_labels,
            input_size=config.object_input_size,
        )
    if backend == "fake":
        return None
    raise ValueError(f"object backend not implemented: {config.object_backend}")


def run(args: argparse.Namespace) -> int:
    config = build_detection_config(args)
    detector = build_detector(config)
    state = PerceptionState(
        args.camera_url,
        config,
        detector=detector,
        timeout=args.timeout,
        interval=args.interval,
        stale_seconds=args.stale_seconds,
    )
    stop_event = threading.Event()
    worker = threading.Thread(target=state.run_loop, args=(stop_event,), daemon=True)
    worker.start()
    server = PerceptionServer(
        (args.host, args.port),
        PerceptionHandler,
        state,
        set(args.allow_origin),
    )
    print(f"MotionBrain perception service: http://{args.host}:{args.port}")
    print(f"camera={args.camera_url}")
    detector_name = getattr(detector, "name", "-")
    print(
        "detector="
        f"{args.detector_mode} backend={args.object_backend} name={detector_name} "
        f"color={args.detect_color} target={args.object_target or '-'}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping perception service")
    finally:
        stop_event.set()
        server.server_close()
        worker.join(timeout=2.0)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pi-hosted MotionBrain perception service for camera detection and tracked-frame compatibility."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Service bind host")
    parser.add_argument("--port", type=int, default=8766, help="Service bind port")
    parser.add_argument("--camera-url", required=True, help="ESP32-CAM base URL, for example http://192.168.4.2")
    parser.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout in seconds")
    parser.add_argument("--interval", type=float, default=0.35, help="Camera/detection loop interval in seconds")
    parser.add_argument("--stale-seconds", type=float, default=2.0, help="Freshness window for health and action gates")
    parser.add_argument(
        "--detector-mode",
        choices=("color", "object"),
        default=os.environ.get("MOTIONBRAIN_DETECTOR_MODE", "color"),
    )
    parser.add_argument("--detect-color", choices=("red", "green", "blue"), default=os.environ.get("MOTIONBRAIN_DETECT_COLOR", "red"))
    parser.add_argument("--align-deadband", type=float, default=0.15, help="Normalized horizontal center tolerance")
    parser.add_argument(
        "--object-backend",
        choices=("fake", "tflite", "onnx", "opencv-dnn"),
        default=os.environ.get("MOTIONBRAIN_OBJECT_BACKEND", "fake"),
    )
    parser.add_argument("--object-model", default=os.environ.get("MOTIONBRAIN_OBJECT_MODEL", ""))
    parser.add_argument("--object-labels", default=os.environ.get("MOTIONBRAIN_OBJECT_LABELS", ""))
    parser.add_argument("--object-target", default=os.environ.get("MOTIONBRAIN_OBJECT_TARGET", ""))
    parser.add_argument("--object-min-confidence", type=float, default=float(os.environ.get("MOTIONBRAIN_OBJECT_MIN_CONFIDENCE", "0.45")))
    parser.add_argument("--object-nms-threshold", type=float, default=0.45)
    parser.add_argument("--object-input-size", type=int, default=640)
    parser.add_argument("--target-policy", choices=("largest", "center", "highest-confidence"), default="largest")
    parser.add_argument(
        "--allow-origin",
        action="append",
        default=["*"],
        help="Allowed CORS origin for read-only API endpoints; repeatable. Defaults to '*'.",
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be > 0")
    if args.timeout <= 0:
        parser.error("--timeout must be > 0")
    if args.stale_seconds <= 0:
        parser.error("--stale-seconds must be > 0")
    if args.align_deadband < 0.0 or args.align_deadband >= 1.0:
        parser.error("--align-deadband must be >= 0.0 and < 1.0")
    if args.object_min_confidence < 0.0 or args.object_min_confidence > 1.0:
        parser.error("--object-min-confidence must be between 0 and 1")
    if args.object_nms_threshold < 0.0 or args.object_nms_threshold > 1.0:
        parser.error("--object-nms-threshold must be between 0 and 1")
    if args.object_input_size < 32:
        parser.error("--object-input-size must be >= 32")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
