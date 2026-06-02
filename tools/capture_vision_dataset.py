#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Callable


FetchBytes = Callable[[str, float], tuple[bytes, str]]
FetchJson = Callable[[str, float], dict[str, Any]]


NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)
SCHEMA_VERSION = "motionbrain.vision_dataset.v1"


def utc_timestamp() -> str:
    return datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def safe_session_name(label: str) -> str:
    normalized = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in label.strip())
    normalized = normalized.strip("_") or "unlabeled"
    return f"{datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')}_{normalized}"


def fetch_bytes(url: str, timeout: float) -> tuple[bytes, str]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "application/octet-stream")
        return response.read(), content_type


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("json_payload_not_object")
    return payload


def url_with_path(base_url: str, path: str) -> str:
    stripped = base_url.strip().rstrip("/")
    if not stripped:
        return ""
    parsed = urllib.parse.urlparse(stripped)
    current_path = parsed.path.rstrip("/")
    target_path = path.rstrip("/")
    if current_path.endswith(target_path):
        return stripped
    if current_path.endswith("/api") and path.startswith("/api/"):
        return f"{stripped}{path[len('/api'):]}"
    return f"{stripped}{path}"


def resolve_frame_url(args: argparse.Namespace) -> str:
    if args.frame_url:
        return args.frame_url.strip()
    if args.perception_url:
        return url_with_path(args.perception_url, "/api/vision_frame")
    if args.camera_url:
        return url_with_path(args.camera_url, "/capture")
    raise ValueError("one of --frame-url, --perception-url, or --camera-url is required")


def resolve_detection_url(args: argparse.Namespace) -> str:
    if args.detection_url:
        return args.detection_url.strip()
    if args.perception_url:
        return url_with_path(args.perception_url, "/api/detection")
    return ""


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")


def capture_dataset(
    *,
    output_root: Path,
    session_name: str,
    label: str,
    frame_url: str,
    detection_url: str = "",
    count: int = 30,
    interval: float = 0.5,
    timeout: float = 6.0,
    notes: str = "",
    fetch_bytes_func: FetchBytes = fetch_bytes,
    fetch_json_func: FetchJson = fetch_json,
) -> Path:
    if count <= 0:
        raise ValueError("count must be positive")
    if interval < 0:
        raise ValueError("interval must be non-negative")

    dataset_dir = output_root / session_name
    frames_dir = dataset_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=False)

    labels_path = dataset_dir / "labels.jsonl"
    manifest = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": utc_timestamp(),
        "label": label,
        "notes": notes,
        "source": {
            "frameUrl": frame_url,
            "detectionUrl": detection_url,
        },
        "requestedFrames": count,
        "capturedFrames": 0,
        "framesDir": "frames",
        "labelsPath": "labels.jsonl",
    }
    write_json(dataset_dir / "manifest.json", manifest)

    captured = 0
    for index in range(count):
        started = time.time()
        entry: dict[str, Any] = {
            "index": index,
            "label": label,
            "capturedAt": utc_timestamp(),
            "ts": started,
        }
        try:
            frame, content_type = fetch_bytes_func(frame_url, timeout)
            frame_name = f"{index:06d}.jpg"
            frame_path = frames_dir / frame_name
            frame_path.write_bytes(frame)
            entry.update(
                {
                    "frame": f"frames/{frame_name}",
                    "frameBytes": len(frame),
                    "contentType": content_type,
                    "ok": True,
                }
            )
            if detection_url:
                try:
                    entry["detection"] = fetch_json_func(detection_url, timeout)
                except Exception as exc:  # keep frame capture useful even if detection polling fails
                    entry["detectionError"] = str(exc)
            captured += 1
        except Exception as exc:
            entry.update({"ok": False, "error": str(exc)})
        append_jsonl(labels_path, entry)

        elapsed = time.time() - started
        if index != count - 1 and interval > elapsed:
            time.sleep(interval - elapsed)

    manifest["capturedFrames"] = captured
    manifest["completedAt"] = utc_timestamp()
    write_json(dataset_dir / "manifest.json", manifest)
    return dataset_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture MotionBrain vision frames with labels for offline detector evaluation."
    )
    source = parser.add_argument_group("frame source")
    source.add_argument("--frame-url", default="", help="Direct JPEG endpoint, e.g. http://pi:8765/api/vision_frame")
    source.add_argument("--perception-url", default="", help="Pi perception/dashboard base URL; uses /api/vision_frame")
    source.add_argument("--camera-url", default="", help="ESP32-CAM base URL; uses /capture")
    source.add_argument("--detection-url", default="", help="Optional detection JSON endpoint; defaults to perception /api/detection")

    parser.add_argument("--output-dir", default="datasets/vision", help="Root directory for captured sessions")
    parser.add_argument("--session-name", default="", help="Dataset session directory name")
    parser.add_argument("--label", default="unlabeled", help="Human label for all captured frames, e.g. cup/bottle/background")
    parser.add_argument("--count", type=int, default=30, help="Number of frames to capture")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between captures")
    parser.add_argument("--timeout", type=float, default=6.0, help="HTTP timeout seconds")
    parser.add_argument("--notes", default="", help="Free-form capture notes stored in manifest.json")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    frame_url = resolve_frame_url(args)
    detection_url = resolve_detection_url(args)
    session_name = args.session_name.strip() or safe_session_name(args.label)
    dataset_dir = capture_dataset(
        output_root=Path(args.output_dir),
        session_name=session_name,
        label=args.label.strip() or "unlabeled",
        frame_url=frame_url,
        detection_url=detection_url,
        count=args.count,
        interval=args.interval,
        timeout=args.timeout,
        notes=args.notes,
    )
    print(f"captured dataset: {dataset_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
