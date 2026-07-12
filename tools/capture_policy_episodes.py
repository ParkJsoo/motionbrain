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
SnapshotFunc = Callable[[], dict[str, Any]]

SCHEMA_VERSION = "motionbrain.policy_episode.v2"


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
    policy_url: str = "",
    joint_state_url: str = "",
    events_url: str = "",
    instruction: str = "",
    operator_action: str = "",
    count: int = 30,
    interval: float = 0.5,
    timeout: float = 6.0,
    notes: str = "",
    required_sources: tuple[str, ...] = ("frame", "status", "detection", "policyProposal"),
    derive_control_guard: bool = True,
    fetch_bytes_func: FetchBytes = fetch_bytes,
    fetch_json_func: FetchJson = fetch_json,
    snapshot_func: SnapshotFunc | None = None,
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
            "policyUrl": policy_url,
            "jointStateUrl": joint_state_url,
            "eventsUrl": events_url,
        },
        "requiredSources": list(required_sources),
        "deriveControlGuard": derive_control_guard,
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
            ("policyProposal", policy_url),
            ("jointState", joint_state_url),
            ("events", events_url),
        ]:
            if not url:
                continue
            try:
                entry[key] = fetch_json_func(url, timeout)
            except Exception as exc:
                sample_errors.append({"source": key, "error": str(exc)})

        if snapshot_func is not None:
            try:
                snapshot = snapshot_func()
                if not isinstance(snapshot, dict):
                    raise ValueError("snapshot_not_object")
                for key, value in snapshot.items():
                    if key not in entry:
                        entry[key] = value
            except Exception as exc:
                sample_errors.append({"source": "rosSnapshot", "error": str(exc)})

        if derive_control_guard and "controlGuard" not in entry:
            status = entry.get("status") if isinstance(entry.get("status"), dict) else {}
            detection = entry.get("detection") if isinstance(entry.get("detection"), dict) else {}
            status_available = bool(status) and not bool(status.get("degraded", False))
            detection_fresh = bool(detection.get("fresh", False))
            entry["controlGuard"] = {
                "ready": status_available and detection_fresh,
                "reason": "derived_ready" if status_available and detection_fresh else "derived_inputs_not_ready",
                "statusFresh": status_available,
                "detectionFresh": detection_fresh,
                "derived": True,
                "provenance": "episode_recorder_http_snapshot",
            }

        present_sources = set(entry)
        if entry.get("frame"):
            present_sources.add("frame")
        for source in required_sources:
            if source and source not in present_sources and not any(
                error.get("source") == source for error in sample_errors
            ):
                sample_errors.append({"source": source, "error": "required_source_missing"})

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
    parser.add_argument("--policy-url", default="", help="Optional read-only policy proposal JSON endpoint")
    parser.add_argument("--joint-state-url", default="", help="Optional joint-state JSON endpoint")
    parser.add_argument("--events-url", default="", help="Optional event summary JSON endpoint")
    parser.add_argument("--perception-url", default="", help="Optional Pi perception base URL; supplies /api/detection")
    parser.add_argument("--dashboard-url", default="", help="Optional Pi dashboard base URL; supplies /api/vision_frame")
    parser.add_argument("--instruction", default="", help="Instruction stored with every sample")
    parser.add_argument("--operator-action", default="", help="Operator action label stored with every sample")
    parser.add_argument("--count", type=int, default=30, help="Number of samples to capture")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between samples")
    parser.add_argument("--timeout", type=float, default=6.0, help="HTTP timeout seconds")
    parser.add_argument("--notes", default="", help="Free-form capture notes stored in manifest.json")
    parser.add_argument(
        "--required-source",
        action="append",
        default=[],
        help="Required sample source key; repeatable. Defaults to frame/status/detection/policyProposal.",
    )
    parser.add_argument(
        "--no-derived-control-guard",
        action="store_true",
        help="Do not derive a conservative guard snapshot when --guard-url is absent.",
    )
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
        policy_url=args.policy_url.strip(),
        joint_state_url=args.joint_state_url.strip(),
        events_url=args.events_url.strip(),
        instruction=args.instruction,
        operator_action=args.operator_action,
        count=args.count,
        interval=args.interval,
        timeout=args.timeout,
        notes=args.notes,
        required_sources=tuple(args.required_source) if args.required_source else (
            "frame",
            "status",
            "detection",
            "policyProposal",
        ),
        derive_control_guard=not args.no_derived_control_guard,
    )
    print(f"captured policy episodes: {dataset_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
