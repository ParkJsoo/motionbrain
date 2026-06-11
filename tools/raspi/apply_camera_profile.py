#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)
PROFILE_FETCH_EXCEPTIONS = (ValueError, json.JSONDecodeError) + NETWORK_EXCEPTIONS


def normalize_base_url(url: str) -> str:
    candidate = url.strip()
    if not candidate:
        raise ValueError("camera URL is required")
    if not urllib.parse.urlparse(candidate).scheme:
        candidate = f"http://{candidate}"
    parsed = urllib.parse.urlparse(candidate)
    if not parsed.netloc:
        raise ValueError(f"invalid camera URL: {url}")
    return urllib.parse.urlunparse((parsed.scheme, parsed.netloc, "", "", "", "")).rstrip("/")


def camera_profile_url(base_url: str, framesize: str, quality: int) -> str:
    query = urllib.parse.urlencode({"framesize": framesize, "quality": str(quality)})
    return f"{normalize_base_url(base_url)}/camera?{query}"


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = response.read()
    decoded = json.loads(payload.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError(f"expected JSON object from {url}")
    return decoded


def post_json(url: str, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url, data=b"", method="POST")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = response.read()
    decoded = json.loads(payload.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError(f"expected JSON object from {url}")
    return decoded


def needs_profile_update(status: dict[str, Any], framesize: str, quality: int) -> bool:
    current_framesize = str(status.get("frameSize", "")).strip().lower()
    try:
        current_quality = int(status.get("jpegQuality"))
    except (TypeError, ValueError):
        return True
    return current_framesize != framesize.strip().lower() or current_quality != quality


def apply_camera_profile(camera_url: str, framesize: str, quality: int, timeout: float) -> str:
    base_url = normalize_base_url(camera_url)
    status_url = f"{base_url}/status"
    try:
        status = fetch_json(status_url, timeout)
    except PROFILE_FETCH_EXCEPTIONS:
        status = {}

    if status and not needs_profile_update(status, framesize, quality):
        return "unchanged"

    result = post_json(camera_profile_url(base_url, framesize, quality), timeout)
    if str(result.get("status", "")).lower() != "ok":
        raise RuntimeError(f"camera profile update failed: {result}")
    return "updated"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply the ESP32-CAM profile expected by MotionBrain perception.")
    parser.add_argument("--camera-url", required=True, help="ESP32-CAM base URL")
    parser.add_argument("--framesize", default="qvga", help="ESP32-CAM frame size, for example qvga")
    parser.add_argument("--quality", type=int, default=10, help="ESP32-CAM JPEG quality, 4..30")
    parser.add_argument("--timeout", type=float, default=3.0, help="HTTP timeout in seconds")
    args = parser.parse_args()
    if args.quality < 4 or args.quality > 30:
        parser.error("--quality must be between 4 and 30")
    if args.timeout <= 0:
        parser.error("--timeout must be > 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        result = apply_camera_profile(args.camera_url, args.framesize, args.quality, args.timeout)
    except Exception as exc:
        print(f"failed {exc}", file=sys.stderr)
        return 1
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
