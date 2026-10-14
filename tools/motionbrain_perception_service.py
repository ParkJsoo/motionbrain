#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import hashlib
import uuid
from collections import OrderedDict
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

from motionbrain_ros_bridge.observation import parse_capture_headers, refresh_observation
from motionbrain_ros_bridge.vision_detection import DetectionConfig  # noqa: E402
from motionbrain_ros_bridge.vision_detection import DetectorBackend  # noqa: E402
from motionbrain_ros_bridge.vision_detection import OpenCvDnnObjectDetector  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_frame  # noqa: E402


SOURCE_FIELDS = frozenset({
    "sourceFrameId", "cameraBootId", "capturedUs", "sourceClockDomain", "sourceTimestampSemantics",
    "sourceTimeKnown", "sourceMetadataValid", "sourceMetadataError", "reportedSourceAgeMs",
    "sourceAgeAtReceiveMs", "captureHostBounds", "captureHostBoundsBasis",
    "captureHostBoundsCalibrated", "captureHostBoundsUncertaintySeconds",
    "captureHostBoundsEvidenceRef", "imageSha256", "hostClockId",
    "receivedMonotonic", "requestStartedMonotonic", "inferenceCompletedMonotonic", "observationId",
})

NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)


def fetch_capture(url: str, timeout: float, max_frame_bytes: int = 2 * 1024 * 1024) -> tuple[bytes, str, dict[str, str]]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "application/octet-stream")
        frame = response.read(max_frame_bytes + 1)
        if len(frame) > max_frame_bytes:
            raise ValueError("frame_exceeds_byte_limit")
        return frame, content_type, dict(response.headers.items())


def fetch_bytes(url: str, timeout: float) -> tuple[bytes, str]:
    """Legacy display-only caller compatibility; capture processing uses headers."""
    frame, content_type, _ = fetch_capture(url, timeout)
    return frame, content_type


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
        display_hold_seconds: float = 0.0,
        failure_backoff_initial: float = 0.5,
        failure_backoff_max: float = 5.0,
        opencv_threads: int | None = None,
        observation_cache_size: int = 32,
        max_frame_bytes: int = 2 * 1024 * 1024,
    ) -> None:
        self.camera_url = camera_url.rstrip("/")
        self.config = config
        self.detector = detector
        self.timeout = timeout
        self.interval = interval
        self.stale_seconds = stale_seconds
        self.display_hold_seconds = display_hold_seconds
        self.failure_backoff_initial = failure_backoff_initial
        self.failure_backoff_max = failure_backoff_max
        self.opencv_threads = opencv_threads
        if observation_cache_size < 1 or max_frame_bytes < 1:
            raise ValueError("observation cache limits must be positive")
        self.observation_cache_size = observation_cache_size
        self.max_frame_bytes = max_frame_bytes
        self.host_clock_id = uuid.uuid4().hex
        self.observations: OrderedDict[str, tuple[bytes, str, dict[str, Any]]] = OrderedDict()
        self.current_camera_boot: str | None = None
        self.camera_boot_highwater: dict[str, int] = {}
        # Retired boot identities never revive. Bound registry memory and fail closed
        # after 128 boots until an operator restarts this host process.
        self.max_camera_boots = 128
        self.cycle_lock = threading.Lock()
        self.lock = threading.Lock()
        self.latest_frame: tuple[float, bytes, str] | None = None
        self.latest_detection: dict[str, Any] | None = None
        self.last_live_detection: dict[str, Any] | None = None
        self.last_live_detection_ts = 0.0
        self.stable_frames = 0
        self.last_error = ""
        self.frames_total = 0
        self.detect_total = 0
        self.error_total = 0
        self.last_cycle_ms: float | None = None
        self.consecutive_errors = 0
        self.current_backoff_seconds = 0.0
        self.next_capture_at_monotonic: float | None = None
        self.last_error_at: float | None = None
        self.last_success_at: float | None = None

    def target_key(self, detection: dict[str, Any]) -> tuple[str, str, int | None]:
        return (
            str(detection.get("targetType", "")),
            str(detection.get("label") or detection.get("color") or ""),
            detection.get("classId") if isinstance(detection.get("classId"), int) else None,
        )

    def prepare_detection(self, detection: dict[str, Any], now: float, content_type: str, frame_bytes: int) -> dict[str, Any]:
        if detection.get("detected"):
            previous_key = self.target_key(self.last_live_detection) if self.last_live_detection else None
            current_key = self.target_key(detection)
            self.stable_frames = self.stable_frames + 1 if previous_key == current_key else 1
            detection["held"] = False
            detection["liveDetected"] = True
            detection["holdAgeMs"] = 0.0
            detection["stableFrames"] = self.stable_frames
            self.last_live_detection = copy.deepcopy(detection)
            self.last_live_detection_ts = now
            return detection

        hold_age = now - self.last_live_detection_ts
        if (
            self.display_hold_seconds > 0.0
            and self.last_live_detection is not None
            and hold_age <= self.display_hold_seconds
        ):
            held = copy.deepcopy(self.last_live_detection)
            held["held"] = True
            held["liveDetected"] = False
            held["holdAgeMs"] = max(hold_age * 1000.0, 0.0)
            held["reason"] = "held_last_detection"
            held["cameraUrl"] = self.camera_url
            held["ts"] = detection["ts"]
            held["contentType"] = content_type
            held["frameBytes"] = frame_bytes
            held["stableFrames"] = self.stable_frames
            # Geometry belongs to the previous detection; source/image belongs to
            # this capture. Never claim the held geometry was detected here.
            for key, value in detection.items():
                if key in SOURCE_FIELDS:
                    held[key] = copy.deepcopy(value)
            return held

        self.stable_frames = 0
        detection["held"] = False
        detection["liveDetected"] = False
        detection["holdAgeMs"] = None
        detection["stableFrames"] = 0
        return detection

    def run_once(self) -> None:
        # Serialize capture/inference assembly, while HTTP readers keep serving
        # the last complete atomic record during the potentially slow inference.
        with self.cycle_lock:
            request_started = time.monotonic()
            frame, content_type, headers = fetch_capture(
                f"{self.camera_url}/capture", self.timeout, self.max_frame_bytes)
            received = time.monotonic()
            if len(frame) > self.max_frame_bytes:
                raise ValueError("frame_exceeds_byte_limit")
            metadata = parse_capture_headers(headers)
            frame_id = metadata["sourceFrameId"]
            image_sha = hashlib.sha256(frame).hexdigest()
            with self.lock:
                existing = self.observations.get(frame_id)
                if existing is not None:
                    original, original_type, original_detection = existing
                    if image_sha != original_detection["imageSha256"]:
                        # Quarantine original evidence; never replace the source
                        # identity's JPEG or inference with conflicting bytes.
                        original_detection["sourceMetadataValid"] = False
                        original_detection["sourceMetadataError"] = "source_frame_content_conflict"
                        self.last_live_detection = None
                        self.stable_frames = 0
                    self.latest_frame = (original_detection["ts"], original, original_type)
                    self.latest_detection = original_detection
                    if (metadata["cameraBootId"] == self.current_camera_boot
                            and metadata["capturedUs"] < self.camera_boot_highwater[metadata["cameraBootId"]]):
                        # Historical reads retain their original evidence. A replay
                        # cannot become the latest control observation again.
                        self.latest_detection = copy.deepcopy(original_detection)
                        self.latest_detection["sourceMetadataValid"] = False
                        self.latest_detection["sourceMetadataError"] = "out_of_order_cached_source_frame"
                    self.frames_total += 1
                    self.last_error = ""
                    self.consecutive_errors = 0
                    self.current_backoff_seconds = 0.0
                    return
                if metadata["sourceMetadataValid"]:
                    boot, captured = metadata["cameraBootId"], metadata["capturedUs"]
                    error = None
                    if boot in self.camera_boot_highwater:
                        if boot != self.current_camera_boot:
                            error = "retired_camera_boot"
                        elif captured <= self.camera_boot_highwater[boot]:
                            error = "retired_or_out_of_order_source_frame"
                    elif len(self.camera_boot_highwater) >= self.max_camera_boots:
                        error = "camera_boot_registry_exhausted"
                    if error:
                        metadata["sourceMetadataValid"] = False
                        metadata["sourceMetadataError"] = error
                    else:
                        if self.current_camera_boot is not None and boot != self.current_camera_boot:
                            for _, _, old_detection in self.observations.values():
                                old_detection["sourceMetadataValid"] = False
                                old_detection["sourceMetadataError"] = "retired_camera_boot"
                            self.last_live_detection = None
                            self.stable_frames = 0
                        self.current_camera_boot = boot
                        self.camera_boot_highwater[boot] = captured
                metadata.update({
                    "imageSha256": image_sha,
                    "hostClockId": self.host_clock_id,
                    "receivedMonotonic": received,
                    "requestStartedMonotonic": request_started,
                    "observationId": uuid.uuid4().hex,
                    "captureHostBoundsBasis": "http_request_interval_equal_clock_rate_assumption",
                    "captureHostBoundsCalibrated": False,
                    "captureHostBoundsUncertaintySeconds": None,
                    "captureHostBoundsEvidenceRef": None,
                })
                reported_age = metadata["reportedSourceAgeMs"]
                timing_valid = received >= request_started
                metadata["sourceAgeAtReceiveMs"] = (
                    reported_age + (received - request_started) * 1000.0
                    if reported_age is not None and timing_valid else None)
                metadata["captureHostBounds"] = (
                    {"clockId": self.host_clock_id,
                     "earliest": request_started - reported_age / 1000.0,
                     "latest": received - reported_age / 1000.0}
                    if reported_age is not None and timing_valid else None)
            detection = detect_frame(frame, self.config, self.detector)
            completed = time.monotonic()
            now = time.time()
            metadata["inferenceCompletedMonotonic"] = completed
            if completed < received:
                metadata["sourceMetadataValid"] = False
                metadata["sourceMetadataError"] = "host_monotonic_clock_regressed"
            detection.update(metadata)
            detection.update({"cameraUrl": self.camera_url, "ts": now,
                              "contentType": content_type, "frameBytes": len(frame),
                              "detectionSourceFrameId": frame_id})
            with self.lock:
                # Unknown/retired capture identity cannot contribute to stability.
                detection = self.prepare_detection(detection, completed, content_type, len(frame))
                if not metadata["sourceMetadataValid"]:
                    self.stable_frames = 0
                    detection["stableFrames"] = 0
                    self.last_live_detection = None
                detection = refresh_observation(detection, completed, self.stale_seconds)
                self.latest_frame = (now, frame, content_type)
                self.latest_detection = copy.deepcopy(detection)
                if frame_id and metadata["sourceMetadataValid"]:
                    self.observations[frame_id] = (frame, content_type, copy.deepcopy(detection))
                    self.latest_detection = self.observations[frame_id][2]
                    while len(self.observations) > self.observation_cache_size:
                        self.observations.popitem(last=False)
                self.last_error = ""
                self.frames_total += 1
                self.detect_total += 1
                self.consecutive_errors = 0
                self.current_backoff_seconds = 0.0
                self.last_success_at = now

    def observation_payload(self, frame_id: str) -> dict[str, Any] | None:
        with self.lock:
            record = self.observations.get(frame_id)
            if record is None:
                return None
            return refresh_observation(record[2], time.monotonic(), self.stale_seconds)

    def mark_error(self, exc: BaseException) -> None:
        now = time.time()
        with self.lock:
            self.last_error = str(exc)
            self.error_total += 1
            self.consecutive_errors += 1
            exponent = min(self.consecutive_errors - 1, 16)
            self.current_backoff_seconds = min(
                self.failure_backoff_max,
                self.failure_backoff_initial * (2**exponent),
            )
            self.last_error_at = now

    def mark_cycle_duration(self, elapsed_seconds: float) -> None:
        with self.lock:
            self.last_cycle_ms = max(elapsed_seconds * 1000.0, 0.0)

    def next_cycle_delay(self, had_error: bool, now_monotonic: float | None = None) -> float:
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        with self.lock:
            delay = self.interval
            if had_error:
                delay = max(delay, self.current_backoff_seconds)
            self.next_capture_at_monotonic = now_monotonic + delay
            return delay

    def run_loop(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            started = time.monotonic()
            had_error = False
            try:
                self.run_once()
            except Exception as exc:
                had_error = True
                self.mark_error(exc)
            elapsed = time.monotonic() - started
            self.mark_cycle_duration(elapsed)
            delay = self.next_cycle_delay(had_error)
            stop_event.wait(delay)

    def detection_payload(self) -> dict[str, Any]:
        with self.lock:
            detection = copy.deepcopy(self.latest_detection) if self.latest_detection is not None else None
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
                "sourceFresh": False,
                "controlEligible": False,
            }

        detection = refresh_observation(detection, time.monotonic(), self.stale_seconds)
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
            detection = copy.deepcopy(self.latest_detection)
            last_error = self.last_error
            frames_total = self.frames_total
            detect_total = self.detect_total
            error_total = self.error_total
            last_cycle_ms = self.last_cycle_ms
            consecutive_errors = self.consecutive_errors
            current_backoff_seconds = self.current_backoff_seconds
            next_capture_at_monotonic = self.next_capture_at_monotonic
            last_error_at = self.last_error_at
            last_success_at = self.last_success_at

        now = time.time()
        monotonic_now = time.monotonic()
        frame_age_ms = None if detection is None else (monotonic_now - detection["receivedMonotonic"]) * 1000.0
        fresh = frame_age_ms is not None and 0 <= frame_age_ms <= self.stale_seconds * 1000.0
        next_capture_delay_ms = (
            None
            if next_capture_at_monotonic is None
            else max((next_capture_at_monotonic - monotonic_now) * 1000.0, 0.0)
        )
        last_error_age_ms = None if last_error_at is None else max((now - last_error_at) * 1000.0, 0.0)
        last_success_age_ms = None if last_success_at is None else max((now - last_success_at) * 1000.0, 0.0)
        detector = detection.get("detector", {}) if isinstance(detection, dict) else {}
        return {
            "ok": fresh,
            "cameraUrl": self.camera_url,
            "mode": self.config.mode,
            "detectColor": self.config.color,
            "objectTarget": self.config.object_target,
            "objectBackend": self.config.object_backend,
            "objectModel": self.config.object_model,
            "objectLabels": self.config.object_labels,
            "objectTargetAliases": list(self.config.object_target_aliases),
            "targetPolicy": self.config.target_policy,
            "displayHoldSeconds": self.display_hold_seconds,
            "fresh": fresh,
            "frameAgeMs": frame_age_ms,
            "timeoutSeconds": self.timeout,
            "intervalSeconds": self.interval,
            "lastCycleMs": last_cycle_ms,
            "framesTotal": frames_total,
            "detectTotal": detect_total,
            "errorTotal": error_total,
            "consecutiveErrors": consecutive_errors,
            "failureBackoffInitialSeconds": self.failure_backoff_initial,
            "failureBackoffMaxSeconds": self.failure_backoff_max,
            "currentBackoffSeconds": current_backoff_seconds,
            "nextCaptureDelayMs": next_capture_delay_ms,
            "lastError": last_error,
            "lastErrorAt": last_error_at,
            "lastErrorAgeMs": last_error_age_ms,
            "lastSuccessAt": last_success_at,
            "lastSuccessAgeMs": last_success_age_ms,
            "detector": detector,
            "detectorConfigured": self.detector is not None,
            "opencvThreads": self.opencv_threads,
        }

    def frame_payload(self, frame_id: str | None = None) -> tuple[bytes, str] | None:
        with self.lock:
            if frame_id is not None:
                record = self.observations.get(frame_id)
                return None if record is None else (record[0], record[1])
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
            elif parsed.path == "/api/observation":
                frame_id = urllib.parse.parse_qs(parsed.query).get("frameId", [""])[0]
                if not frame_id:
                    self.send_error_json(HTTPStatus.BAD_REQUEST, "frameId_required")
                else:
                    observation = self.server.state.observation_payload(frame_id)
                    if observation is None:
                        self.send_error_json(HTTPStatus.NOT_FOUND, "source_frame_not_cached")
                    else:
                        self.send_json({"ok": True, "observation": observation}, allow_cross_origin=True)
            elif parsed.path == "/api/vision_frame":
                self.handle_vision_frame()
            else:
                self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
        except (ValueError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def handle_vision_frame(self) -> None:
        query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query, keep_blank_values=True)
        frame_id = query.get("frameId", [None])[0]
        if frame_id == "":
            self.send_error_json(HTTPStatus.BAD_REQUEST, "frameId_required")
            return
        frame_payload = self.server.state.frame_payload(frame_id)
        if frame_payload is None:
            self.send_error_json(HTTPStatus.NOT_FOUND if frame_id is not None else HTTPStatus.SERVICE_UNAVAILABLE, "source_frame_not_cached" if frame_id is not None else "no_frame")
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
            "objectTargetAliases": list(config.object_target_aliases),
            "objectMinConfidence": config.object_min_confidence,
            "objectNmsThreshold": config.object_nms_threshold,
            "objectInputSize": config.object_input_size,
            "displayHoldSeconds": self.state.display_hold_seconds,
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
        object_target_aliases=tuple(args.object_target_aliases),
        object_min_confidence=args.object_min_confidence,
        object_nms_threshold=args.object_nms_threshold,
        object_input_size=args.object_input_size,
        target_policy=args.target_policy,
    )


def parse_label_list(values: list[str] | tuple[str, ...]) -> list[str]:
    labels: list[str] = []
    for value in values:
        for item in str(value).split(","):
            label = " ".join(item.strip().lower().replace("_", " ").split())
            if label and label not in labels:
                labels.append(label)
    return labels


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


def configure_opencv_runtime(thread_count: int) -> int | None:
    if thread_count == 0:
        return None
    try:
        import cv2  # type: ignore
    except ImportError:
        return None

    cv2.setNumThreads(thread_count)
    try:
        return int(cv2.getNumThreads())
    except AttributeError:
        return thread_count


def run(args: argparse.Namespace) -> int:
    opencv_threads = configure_opencv_runtime(args.opencv_threads)
    config = build_detection_config(args)
    detector = build_detector(config)
    state = PerceptionState(
        args.camera_url,
        config,
        detector=detector,
        timeout=args.timeout,
        interval=args.interval,
        stale_seconds=args.stale_seconds,
        display_hold_seconds=args.display_hold_seconds,
        failure_backoff_initial=args.failure_backoff_initial,
        failure_backoff_max=args.failure_backoff_max,
        opencv_threads=opencv_threads,
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
    if opencv_threads is not None:
        print(f"opencv_threads={opencv_threads}")
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
        "--failure-backoff-initial",
        type=float,
        default=float(os.environ.get("MOTIONBRAIN_PERCEPTION_FAILURE_BACKOFF_INITIAL", "0.5")),
        help="Seconds to wait after the first camera fetch failure before retrying",
    )
    parser.add_argument(
        "--failure-backoff-max",
        type=float,
        default=float(os.environ.get("MOTIONBRAIN_PERCEPTION_FAILURE_BACKOFF_MAX", "5.0")),
        help="Maximum retry backoff after repeated camera fetch failures",
    )
    parser.add_argument(
        "--display-hold-seconds",
        type=float,
        default=0.0,
        help="Seconds to keep showing the last live detection after a transient miss; held detections are marked held=true",
    )
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
    parser.add_argument(
        "--object-target-alias",
        action="append",
        default=[os.environ.get("MOTIONBRAIN_OBJECT_TARGET_ALIASES", "")],
        help="Additional labels accepted as the selected target, comma-separated or repeatable; reports the canonical target label.",
    )
    parser.add_argument("--object-min-confidence", type=float, default=float(os.environ.get("MOTIONBRAIN_OBJECT_MIN_CONFIDENCE", "0.45")))
    parser.add_argument("--object-nms-threshold", type=float, default=0.45)
    parser.add_argument("--object-input-size", type=int, default=640)
    parser.add_argument("--target-policy", choices=("largest", "center", "highest-confidence"), default="largest")
    parser.add_argument(
        "--opencv-threads",
        type=int,
        default=int(os.environ.get("MOTIONBRAIN_OPENCV_THREADS", "1")),
        help="OpenCV worker thread count; use 0 to leave OpenCV defaults unchanged",
    )
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
    if args.failure_backoff_initial < 0:
        parser.error("--failure-backoff-initial must be >= 0")
    if args.failure_backoff_max < args.failure_backoff_initial:
        parser.error("--failure-backoff-max must be >= --failure-backoff-initial")
    if args.display_hold_seconds < 0:
        parser.error("--display-hold-seconds must be >= 0")
    if args.align_deadband < 0.0 or args.align_deadband >= 1.0:
        parser.error("--align-deadband must be >= 0.0 and < 1.0")
    if args.object_min_confidence < 0.0 or args.object_min_confidence > 1.0:
        parser.error("--object-min-confidence must be between 0 and 1")
    if args.object_nms_threshold < 0.0 or args.object_nms_threshold > 1.0:
        parser.error("--object-nms-threshold must be between 0 and 1")
    if args.object_input_size < 32:
        parser.error("--object-input-size must be >= 32")
    if args.opencv_threads < 0:
        parser.error("--opencv-threads must be >= 0")
    args.object_target_aliases = parse_label_list(args.object_target_alias)
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
