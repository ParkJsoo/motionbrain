#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
MISSION_SRC = ROOT / "ros2_ws" / "src" / "motionbrain_mission"
if str(MISSION_SRC) not in sys.path:
    sys.path.insert(0, str(MISSION_SRC))

from motionbrain_mission.policy_proposal import PolicyConfig  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyDetectionSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyGuardSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyStatusSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import propose_policy_action  # noqa: E402


SCHEMA_VERSION = "motionbrain.policy_replay.v1"
UNSAFE_ACTIONS = {
    "motor_run",
    "joint_run",
    "sequence_run",
    "routine_run",
    "routine_execute",
    "ros2_control_write",
}


def utc_timestamp() -> str:
    return datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        payload = json.loads(line)
        if not isinstance(payload, dict):
            raise ValueError(f"jsonl entry is not an object: {path}")
        entries.append(payload)
    return entries


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_episode_entries(dataset_dir: Path) -> list[dict[str, Any]]:
    episodes_path = dataset_dir / "episodes.jsonl"
    if not episodes_path.exists():
        raise FileNotFoundError(f"episodes file not found: {episodes_path}")
    return [entry for entry in read_jsonl(episodes_path) if entry.get("ok", True)]


def evaluate_policy_replay(
    *,
    dataset_dir: Path,
    output_path: Path,
    results_path: Path,
    config: PolicyConfig | None = None,
) -> dict[str, Any]:
    entries = load_episode_entries(dataset_dir)
    config = config or PolicyConfig()
    results_path.parent.mkdir(parents=True, exist_ok=True)
    if results_path.exists():
        results_path.unlink()

    results: list[dict[str, Any]] = []
    for entry in entries:
        proposal = propose_policy_action(
            instruction=str(entry.get("instruction", "")),
            status=status_snapshot(entry.get("status")),
            detection=detection_snapshot(entry.get("detection")),
            guard=guard_snapshot(entry.get("controlGuard")),
            config=config,
        )
        operator_action = str(entry.get("operatorAction", "")).strip()
        result = {
            "index": entry.get("index"),
            "instruction": entry.get("instruction", ""),
            "operatorAction": operator_action,
            "proposal": proposal.to_dict(),
            "operatorAgreement": bool(operator_action) and operator_action == proposal.action,
            "unsafeProposal": proposal.action in UNSAFE_ACTIONS,
            "physicalMotionCandidate": proposal.physical_motion_candidate,
            "staleRejected": (
                proposal.action == "hold"
                and any(
                    item in proposal.preconditions
                    for item in ("status_stale", "detection_stale")
                )
            ),
            "heldRejected": (
                proposal.action == "hold" and "held_detection" in proposal.preconditions
            ),
        }
        results.append(result)
        append_jsonl(results_path, result)

    summary = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": utc_timestamp(),
        "dataset": str(dataset_dir),
        "resultsPath": str(results_path),
        "policy": {
            "targetLabel": config.target_label,
            "minConfidence": config.min_confidence,
            "allowMotionCandidates": config.allow_motion_candidates,
        },
        "metrics": summarize_results(results),
    }
    write_json(output_path, summary)
    return summary


def summarize_results(results: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(results)
    operator_labeled = [item for item in results if item.get("operatorAction")]
    action_counts = Counter(str(item["proposal"]["action"]) for item in results)
    reason_counts = Counter(str(item["proposal"]["reason"]) for item in results)
    unsafe_count = sum(1 for item in results if item["unsafeProposal"])
    physical_candidates = sum(1 for item in results if item["physicalMotionCandidate"])
    agreements = sum(1 for item in operator_labeled if item["operatorAgreement"])
    return {
        "total": total,
        "operatorLabeled": len(operator_labeled),
        "operatorAgreements": agreements,
        "operatorAgreementRate": agreements / len(operator_labeled) if operator_labeled else None,
        "unsafeProposals": unsafe_count,
        "unsafeProposalRate": unsafe_count / total if total else None,
        "physicalMotionCandidates": physical_candidates,
        "physicalMotionCandidateRate": physical_candidates / total if total else None,
        "staleRejections": sum(1 for item in results if item["staleRejected"]),
        "heldRejections": sum(1 for item in results if item["heldRejected"]),
        "actionCounts": dict(sorted(action_counts.items())),
        "reasonCounts": dict(sorted(reason_counts.items())),
    }


def status_snapshot(payload: Any) -> PolicyStatusSnapshot:
    data = payload if isinstance(payload, dict) else {}
    sensor = data.get("sensor") if isinstance(data.get("sensor"), dict) else {}
    base = data.get("baseAngle") if isinstance(data.get("baseAngle"), dict) else {}
    state = str(data.get("state", "UNKNOWN"))
    return PolicyStatusSnapshot(
        available=as_bool(data.get("available"), bool(data)),
        state=state,
        moving=as_bool(data.get("moving"), False),
        faulted=as_bool(data.get("faulted"), state == "FAULT"),
        base_active=as_bool(base.get("active"), False),
        safety_blocked=as_bool(sensor.get("blocked"), as_bool(data.get("sensorBlocked"), False)),
        fault_latched=as_bool(sensor.get("faultLatched"), as_bool(data.get("faultLatched"), False)),
    )


def detection_snapshot(payload: Any) -> PolicyDetectionSnapshot:
    data = payload if isinstance(payload, dict) else {}
    detection = data.get("detection") if isinstance(data.get("detection"), dict) else data
    return PolicyDetectionSnapshot(
        available=as_bool(detection.get("available"), bool(detection)),
        detected=as_bool(detection.get("detected"), False),
        fresh=as_bool(detection.get("fresh"), as_bool(detection.get("available"), bool(detection))),
        held=as_bool(detection.get("held"), False),
        alignment=str(detection.get("alignment", "LOST")),
        command_suggestion=str(detection.get("commandSuggestion", "none")),
        label=str(detection.get("label", "")),
        color=str(detection.get("color", "")),
        confidence=as_optional_float(detection.get("confidence")),
        area_ratio=float(detection.get("areaRatio", detection.get("ratio", 0.0)) or 0.0),
    )


def guard_snapshot(payload: Any) -> PolicyGuardSnapshot:
    data = payload if isinstance(payload, dict) else {}
    return PolicyGuardSnapshot(
        ready=as_bool(data.get("ready"), False),
        reason=str(data.get("reason", "missing_guard")),
        status_fresh=as_bool(data.get("statusFresh"), False),
        detection_fresh=as_bool(data.get("detectionFresh"), False),
    )


def as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on", "ready", "ok"}:
            return True
        if normalized in {"false", "0", "no", "n", "off", "blocked", "stale", ""}:
            return False
    return default


def as_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay MotionBrain policy episodes and summarize proposal safety metrics."
    )
    parser.add_argument("--dataset", required=True, help="Dataset directory created by capture_policy_episodes.py")
    parser.add_argument("--output", default="", help="Summary JSON path; default is <dataset>/policy_replay_<timestamp>.json")
    parser.add_argument("--results-jsonl", default="", help="Per-sample JSONL path; default follows --output")
    parser.add_argument("--target-label", default="cup")
    parser.add_argument("--min-confidence", type=float, default=0.5)
    parser.add_argument(
        "--disable-motion-candidates",
        action="store_true",
        help="Replay with physical motion candidates disabled.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    dataset_dir = Path(args.dataset)
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    output_path = Path(args.output) if args.output else dataset_dir / f"policy_replay_{timestamp}.json"
    results_path = (
        Path(args.results_jsonl)
        if args.results_jsonl
        else output_path.with_suffix(".jsonl")
    )
    summary = evaluate_policy_replay(
        dataset_dir=dataset_dir,
        output_path=output_path,
        results_path=results_path,
        config=PolicyConfig(
            target_label=args.target_label,
            min_confidence=args.min_confidence,
            allow_motion_candidates=not args.disable_motion_candidates,
        ),
    )
    metrics = summary["metrics"]
    print(
        "policy replay: "
        f"total={metrics['total']} "
        f"unsafe={metrics['unsafeProposals']} "
        f"physical_candidates={metrics['physicalMotionCandidates']} "
        f"operator_agreement={metrics['operatorAgreementRate']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
