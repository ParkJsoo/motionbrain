#!/usr/bin/env python3

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


TARGET_RATIO_THRESHOLD = 0.02


def fetch_json(url: str, timeout: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def fetch_bytes_with_retries(url: str, timeout: float, attempts: int, retry_delay: float) -> bytes:
    last_exc: Exception | None = None
    for attempt in range(1, attempts + 1):
        try:
            return fetch_bytes(url, timeout)
        except (urllib.error.URLError, TimeoutError) as exc:
            last_exc = exc
            if attempt < attempts:
                time.sleep(retry_delay)
    assert last_exc is not None
    raise last_exc


def post_motionbrain(base_url: str, path: str, timeout: float, token: str = "") -> dict:
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


def status_allows_demo_command(status: dict) -> bool:
    sensor = status.get("sensor", {})
    if sensor.get("blocked", False):
        return False
    return status.get("state") in ("IDLE", "ARMED")


def status_allows_base_alignment(status: dict) -> bool:
    sensor = status.get("sensor", {})
    base = status.get("baseAngle", {})
    if sensor.get("blocked", False):
        return False
    if base.get("active", False):
        return False
    return status.get("state") == "ARMED"


def classify_alignment(offset_x: float | None, deadband: float) -> str:
    if offset_x is None:
        return "LOST"
    if offset_x < -deadband:
        return "LEFT"
    if offset_x > deadband:
        return "RIGHT"
    return "CENTER"


def command_suggestion_for_alignment(alignment: str) -> str:
    if alignment == "LEFT":
        return "base_left"
    if alignment == "RIGHT":
        return "base_right"
    if alignment == "CENTER":
        return "hold"
    return "none"


def detect_colored_target(frame: bytes, color: str, align_deadband: float) -> dict:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return {
            "detected": False,
            "available": False,
            "color": color,
            "reason": "opencv_unavailable",
            "frameBytes": len(frame),
            "alignment": "LOST",
            "commandSuggestion": "none",
        }

    data = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return {
            "detected": False,
            "available": True,
            "color": color,
            "reason": "decode_failed",
            "frameBytes": len(frame),
            "alignment": "LOST",
            "commandSuggestion": "none",
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
        raise ValueError(f"unsupported color: {color}")

    pixels = int(cv2.countNonZero(mask))
    height, width = image.shape[:2]
    area = max(height * width, 1)
    ratio = pixels / max(area, 1)
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

    alignment = classify_alignment(offset_x, align_deadband) if detected else "LOST"
    command_suggestion = command_suggestion_for_alignment(alignment)
    return {
        "detected": detected,
        "available": True,
        "color": color,
        "ratio": ratio,
        "areaRatio": ratio,
        "pixels": pixels,
        "width": width,
        "height": height,
        "frameBytes": len(frame),
        "centerX": centroid_x,
        "centerY": centroid_y,
        "centroidX": centroid_x,
        "centroidY": centroid_y,
        "offsetX": offset_x,
        "offsetY": offset_y,
        "alignDeadband": align_deadband,
        "alignment": alignment,
        "commandSuggestion": command_suggestion,
    }


def format_detection_detail(detection: dict) -> str:
    reason = detection.get("reason")
    if reason:
        return str(reason)

    ratio = detection.get("ratio")
    offset_x = detection.get("offsetX")
    alignment = detection.get("alignment", "LOST")
    suggestion = detection.get("commandSuggestion", "none")
    if isinstance(ratio, (int, float)) and isinstance(offset_x, (int, float)):
        return f"{detection.get('color')}_ratio={ratio:.3f} offset_x={offset_x:+.2f} align={alignment} suggest={suggestion}"
    if isinstance(ratio, (int, float)):
        return f"{detection.get('color')}_ratio={ratio:.3f} align={alignment} suggest={suggestion}"
    return f"align={alignment} suggest={suggestion}"


def run(args: argparse.Namespace) -> int:
    motion_base = f"http://{args.motion_host}:{args.motion_port}"
    camera_capture = args.camera_url.rstrip("/") + "/capture"
    last_light_action_time = 0.0
    last_align_action_time = 0.0

    print("MotionBrain Phase 4 vision host MVP")
    print(f"motion={motion_base}")
    print(f"camera={camera_capture}")
    print(f"detect={args.detect_color}")
    print(f"action={'enabled' if args.enable_action else 'dry-run'} {args.action}")
    print(
        "alignment="
        f"{'enabled' if args.enable_align_action else 'dry-run'} "
        f"deadband={args.align_deadband:.2f} step={args.align_degrees:.1f}deg "
        f"speed={args.align_percent}%"
    )
    print(
        f"capture timeout={args.timeout:.1f}s retries={args.capture_retries} "
        f"retry_delay={args.capture_retry_delay:.1f}s interval={args.interval:.1f}s"
    )

    while True:
        timestamp = time.strftime("%H:%M:%S")
        try:
            status = fetch_json(f"{motion_base}/status", args.timeout)
            frame = fetch_bytes_with_retries(
                camera_capture,
                args.timeout,
                args.capture_retries,
                args.capture_retry_delay,
            )
            if args.assume_detected:
                detection = {
                    "detected": True,
                    "available": False,
                    "color": args.detect_color,
                    "reason": "assume_detected",
                    "frameBytes": len(frame),
                    "alignment": "LOST",
                    "commandSuggestion": "none",
                }
            else:
                detection = detect_colored_target(frame, args.detect_color, args.align_deadband)
            detected = bool(detection.get("detected"))
            detail = format_detection_detail(detection)
            allowed = status_allows_demo_command(status)
            align_allowed = status_allows_base_alignment(status)

            print(
                f"[{timestamp}] state={status.get('state')} "
                f"detected={'Y' if detected else 'N'} {detail} "
                f"frame={len(frame)}B allowed={'Y' if allowed else 'N'} "
                f"align_allowed={'Y' if align_allowed else 'N'}",
                flush=True,
            )

            now = time.monotonic()
            if detected and allowed and args.enable_action and now - last_light_action_time >= args.cooldown:
                response = post_motionbrain(
                    motion_base,
                    f"/light?action={urllib.parse.quote(args.action)}",
                    args.timeout,
                    args.http_token,
                )
                print(f"[{timestamp}] ACTION light.{args.action} success={response.get('success')}", flush=True)
                last_light_action_time = now
            alignment = detection.get("alignment")
            if (
                detected
                and align_allowed
                and args.enable_align_action
                and alignment in {"LEFT", "RIGHT"}
                and now - last_align_action_time >= args.cooldown
            ):
                direction = alignment.lower()
                path = (
                    f"/base?action=angle&direction={direction}"
                    f"&degrees={args.align_degrees:.1f}&percent={args.align_percent}"
                )
                response = post_motionbrain(motion_base, path, args.timeout, args.http_token)
                print(
                    f"[{timestamp}] ACTION base.{direction} "
                    f"{args.align_degrees:.1f}deg success={response.get('success')} "
                    f"message={response.get('message')}",
                    flush=True,
                )
                last_align_action_time = now

        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError) as exc:
            print(f"[{timestamp}] ERROR {exc}", file=sys.stderr, flush=True)

        if args.once:
            return 0
        time.sleep(args.interval)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read ESP32-CAM frames and MotionBrain status, then optionally trigger safe light or base-alignment actions."
    )
    parser.add_argument("--motion-host", default="192.168.4.1", help="MotionBrain motion-controller IP")
    parser.add_argument("--motion-port", type=int, default=80, help="MotionBrain HTTP port")
    parser.add_argument("--camera-url", required=True, help="ESP32-CAM base URL, for example http://192.168.4.2")
    parser.add_argument("--detect-color", choices=("red", "green", "blue"), default="red")
    parser.add_argument("--assume-detected", action="store_true", help="Skip OpenCV and treat every fetched frame as a detection")
    parser.add_argument("--interval", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--capture-retries", type=int, default=2, help="Camera capture attempts per loop")
    parser.add_argument("--capture-retry-delay", type=float, default=1.0, help="Seconds to wait before retrying camera capture")
    parser.add_argument("--cooldown", type=float, default=5.0, help="Minimum seconds between actions")
    parser.add_argument("--http-token", default=os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""), help="Optional X-MotionBrain-Token for controller POST endpoints")
    parser.add_argument("--action", choices=("on", "off", "toggle"), default="toggle", help="POST /light?action=...")
    parser.add_argument("--enable-action", action="store_true", help="Actually send /light action when target is detected")
    parser.add_argument("--align-deadband", type=float, default=0.15, help="Normalized horizontal center tolerance")
    parser.add_argument("--align-degrees", type=float, default=5.0, help="Relative base angle step for alignment")
    parser.add_argument("--align-percent", type=int, default=35, help="Base speed percent for alignment")
    parser.add_argument("--enable-align-action", action="store_true", help="Actually send /base angle action when target is off-center")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    if args.align_deadband < 0.0 or args.align_deadband >= 1.0:
        parser.error("--align-deadband must be >= 0.0 and < 1.0")
    if args.align_degrees < 3.0 or args.align_degrees > 180.0:
        parser.error("--align-degrees must be between 3 and 180")
    if args.align_percent < 1 or args.align_percent > 100:
        parser.error("--align-percent must be between 1 and 100")
    if args.capture_retries < 1:
        parser.error("--capture-retries must be >= 1")
    if args.capture_retry_delay < 0:
        parser.error("--capture-retry-delay must be >= 0")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
