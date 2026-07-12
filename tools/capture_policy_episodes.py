#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Callable


FetchBytes = Callable[[str, float], tuple[bytes, str]]
FetchJson = Callable[[str, float], dict[str, Any]]

SCHEMA_VERSION = "motionbrain.policy_episode.v1"


def utc_timestamp() -> str:
    return datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def safe_session_name(label: str) -> str:
    normalized = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in label.strip())
    normalized = normalized.strip("_") or "policy"
    return f"{datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')}_{normalized}"


def fetch_bytes(url: str, timeout: float) -> tuple[bytes, str]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "application/octet-stream")
        return response.read(), content_type


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON payload is not an object: {url}")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")


def url_with_path(base_url: str, path: str) -> str:
    stripped = base_url.strip().rstrip("/")
    if not stripped:
        return ""
    parsed_path = path if path.startswith("/") else f"/{path}"
    return f"{stripped}{parsed_path}"


def capture_policy_episodes(
    *,
    output_root: Path,
    session_name: str,
    label: str = "policy",
    frame_url: str = "",
    status_url: str = "",
    detection_url: str = "",
    guard_url: str = "",
    mission_url: str = "",
    instruction: str = "",
    operator_action: str = "",
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

    episodes_path = dataset_dir / "episodes.jsonl"
    manifest = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": utc_timestamp(),
        "label": label,
        "notes": notes,
        "source": {
            "frameUrl": frame_url,
            "statusUrl": status_url,
            "detectionUrl": detection_url,
            "guardUrl": guard_url,
            "missionUrl": mission_url,
        },
        "instruction": instruction,
        "operatorAction": operator_action,
        "requestedSamples": count,
        "capturedSamples": 0,
        "framesDir": "frames",
        "episodesPath": "episodes.jsonl",
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
            "instruction": instruction,
            "operatorAction": operator_action,
            "ok": True,
        }

        sample_errors: list[dict[str, str]] = []
        if frame_url:
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
                    }
                )
            except Exception as exc:
                sample_errors.append({"source": "frame", "error": str(exc)})

        for key, url in [
            ("status", status_url),
            ("detection", detection_url),
            ("controlGuard", guard_url),
            ("missionState", mission_url),
        ]:
            if not url:
                continue
            try:
                entry[key] = fetch_json_func(url, timeout)
            except Exception as exc:
                sample_errors.append({"source": key, "error": str(exc)})

        if sample_errors:
            entry["ok"] = False
            entry["errors"] = sample_errors
        else:
            captured += 1

        append_jsonl(episodes_path, entry)

        elapsed = time.time() - started
        if index != count - 1 and interval > elapsed:
            time.sleep(interval - elapsed)

    manifest["capturedSamples"] = captured
    manifest["completedAt"] = utc_timestamp()
    write_json(dataset_dir / "manifest.json", manifest)
    return dataset_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture MotionBrain policy episodes for offline proposal replay."
    )
    parser.add_argument("--output-dir", default="datasets/policy", help="Root directory for captured sessions")
    parser.add_argument("--session-name", default="", help="Dataset session directory name")
    parser.add_argument("--label", default="policy", help="Human label for this session")
    parser.add_argument("--frame-url", default="", help="Optional JPEG frame endpoint")
    parser.add_argument("--status-url", default="", help="Optional MotionBrain status JSON endpoint")
    parser.add_argument("--detection-url", default="", help="Optional detection JSON endpoint")
    parser.add_argument("--guard-url", default="", help="Optional control guard JSON endpoint")
    parser.add_argument("--mission-url", default="", help="Optional mission state JSON endpoint")
    parser.add_argument("--perception-url", default="", help="Optional Pi perception base URL; supplies /api/detection")
    parser.add_argument("--dashboard-url", default="", help="Optional Pi dashboard base URL; supplies /api/vision_frame")
    parser.add_argument("--instruction", default="", help="Instruction stored with every sample")
    parser.add_argument("--operator-action", default="", help="Operator action label stored with every sample")
    parser.add_argument("--count", type=int, default=30, help="Number of samples to capture")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between samples")
    parser.add_argument("--timeout", type=float, default=6.0, help="HTTP timeout seconds")
    parser.add_argument("--notes", default="", help="Free-form capture notes stored in manifest.json")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    frame_url = args.frame_url.strip()
    detection_url = args.detection_url.strip()
    if args.dashboard_url and not frame_url:
        frame_url = url_with_path(args.dashboard_url, "/api/vision_frame")
    if args.perception_url and not detection_url:
        detection_url = url_with_path(args.perception_url, "/api/detection")

    session_name = args.session_name.strip() or safe_session_name(args.label)
    dataset_dir = capture_policy_episodes(
        output_root=Path(args.output_dir),
        session_name=session_name,
        label=args.label.strip() or "policy",
        frame_url=frame_url,
        status_url=args.status_url.strip(),
        detection_url=detection_url,
        guard_url=args.guard_url.strip(),
        mission_url=args.mission_url.strip(),
        instruction=args.instruction,
        operator_action=args.operator_action,
        count=args.count,
        interval=args.interval,
        timeout=args.timeout,
        notes=args.notes,
    )
    print(f"captured policy episodes: {dataset_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
