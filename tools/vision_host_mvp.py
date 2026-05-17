#!/usr/bin/env python3

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def fetch_json(url: str, timeout: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def post_motionbrain(base_url: str, path: str, timeout: float) -> dict:
    request = urllib.request.Request(
        f"{base_url}{path}",
        method="POST",
        headers={"X-MotionBrain": "1"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def status_allows_demo_command(status: dict) -> bool:
    sensor = status.get("sensor", {})
    if sensor.get("blocked", False):
        return False
    return status.get("state") in ("IDLE", "ARMED")


def detect_colored_target(frame: bytes, color: str) -> tuple[bool, str]:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return False, "opencv unavailable"

    data = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return False, "decode failed"

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
    area = image.shape[0] * image.shape[1]
    ratio = pixels / max(area, 1)
    return ratio >= 0.02, f"{color}_ratio={ratio:.3f}"


def run(args: argparse.Namespace) -> int:
    motion_base = f"http://{args.motion_host}:{args.motion_port}"
    camera_capture = args.camera_url.rstrip("/") + "/capture"
    last_action_time = 0.0

    print("MotionBrain Phase 4 vision host MVP")
    print(f"motion={motion_base}")
    print(f"camera={camera_capture}")
    print(f"detect={args.detect_color}")
    print(f"action={'enabled' if args.enable_action else 'dry-run'} {args.action}")

    while True:
        timestamp = time.strftime("%H:%M:%S")
        try:
            status = fetch_json(f"{motion_base}/status", args.timeout)
            frame = fetch_bytes(camera_capture, args.timeout)
            if args.assume_detected:
                detected, detail = True, "assume_detected"
            else:
                detected, detail = detect_colored_target(frame, args.detect_color)
            allowed = status_allows_demo_command(status)

            print(
                f"[{timestamp}] state={status.get('state')} "
                f"detected={'Y' if detected else 'N'} {detail} "
                f"frame={len(frame)}B allowed={'Y' if allowed else 'N'}",
                flush=True,
            )

            now = time.monotonic()
            if detected and allowed and args.enable_action and now - last_action_time >= args.cooldown:
                response = post_motionbrain(motion_base, f"/light?action={urllib.parse.quote(args.action)}", args.timeout)
                print(f"[{timestamp}] ACTION light.{args.action} success={response.get('success')}", flush=True)
                last_action_time = now

        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError) as exc:
            print(f"[{timestamp}] ERROR {exc}", file=sys.stderr, flush=True)

        if args.once:
            return 0
        time.sleep(args.interval)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read ESP32-CAM frames and MotionBrain status, then optionally trigger a safe demo action."
    )
    parser.add_argument("--motion-host", default="192.168.4.1", help="MotionBrain motion-controller IP")
    parser.add_argument("--motion-port", type=int, default=80, help="MotionBrain HTTP port")
    parser.add_argument("--camera-url", required=True, help="ESP32-CAM base URL, for example http://192.168.4.2")
    parser.add_argument("--detect-color", choices=("red", "green", "blue"), default="red")
    parser.add_argument("--assume-detected", action="store_true", help="Skip OpenCV and treat every fetched frame as a detection")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--cooldown", type=float, default=5.0, help="Minimum seconds between actions")
    parser.add_argument("--action", choices=("on", "off", "toggle"), default="toggle", help="POST /light?action=...")
    parser.add_argument("--enable-action", action="store_true", help="Actually send /light action when target is detected")
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
