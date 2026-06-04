#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from collections import Counter
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ROS_BRIDGE_SRC = ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"
if str(ROS_BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(ROS_BRIDGE_SRC))

from motionbrain_ros_bridge.vision_detection import DetectionConfig  # noqa: E402
from motionbrain_ros_bridge.vision_detection import DetectorBackend  # noqa: E402
from motionbrain_ros_bridge.vision_detection import OpenCvDnnObjectDetector  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_frame  # noqa: E402


SCHEMA_VERSION = "motionbrain.vision_evaluation.v1"
NEGATIVE_LABELS = {"", "background", "negative", "none", "unlabeled"}


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


def is_positive_label(label: str) -> bool:
    return label.strip().lower() not in NEGATIVE_LABELS


def parse_label_list(values: list[str] | tuple[str, ...]) -> list[str]:
    labels: list[str] = []
    for value in values:
        for item in str(value).split(","):
            label = " ".join(item.strip().lower().replace("_", " ").split())
            if label and label not in labels:
                labels.append(label)
    return labels


def load_dataset_entries(dataset_dir: Path) -> list[dict[str, Any]]:
    labels_path = dataset_dir / "labels.jsonl"
    if not labels_path.exists():
        raise FileNotFoundError(f"labels file not found: {labels_path}")
    entries = []
    for entry in read_jsonl(labels_path):
        if not entry.get("ok", True):
            continue
        frame = entry.get("frame")
        if not isinstance(frame, str) or not frame:
            continue
        entry = dict(entry)
        entry["framePath"] = str(dataset_dir / frame)
        entries.append(entry)
    return entries


def build_detector(config: DetectionConfig) -> DetectorBackend | None:
    if config.mode != "object":
        return None
    if config.object_backend != "opencv-dnn":
        raise ValueError(f"unsupported object backend for offline evaluation: {config.object_backend}")
    return OpenCvDnnObjectDetector.from_model(
        config.object_model,
        config.object_labels,
        input_size=config.object_input_size,
    )


def result_from_payload(
    *,
    entry: dict[str, Any],
    payload: dict[str, Any],
    elapsed_ms: float,
) -> dict[str, Any]:
    expected = str(entry.get("label", "")).strip().lower()
    detected_label = str(payload.get("label") or "").strip().lower()
    detected = bool(payload.get("detected"))
    positive = is_positive_label(expected)
    target_match = positive and detected and detected_label == expected
    false_negative = positive and not target_match
    wrong_label = positive and detected and detected_label != expected
    false_positive = (not positive) and detected
    true_negative = (not positive) and not detected
    detector = payload.get("detector", {}) if isinstance(payload.get("detector"), dict) else {}
    latency_ms = detector.get("latencyMs")
    if not isinstance(latency_ms, (int, float)):
        latency_ms = elapsed_ms

    return {
        "index": entry.get("index"),
        "frame": entry.get("frame"),
        "expectedLabel": expected,
        "positive": positive,
        "detected": detected,
        "detectedLabel": detected_label,
        "targetMatch": target_match,
        "falseNegative": false_negative,
        "wrongLabel": wrong_label,
        "falsePositive": false_positive,
        "trueNegative": true_negative,
        "confidence": payload.get("confidence"),
        "reason": payload.get("reason", ""),
        "latencyMs": latency_ms,
        "alignment": payload.get("alignment", "LOST"),
        "targetBox": payload.get("targetBox"),
        "objectCount": len(payload.get("objects", [])) if isinstance(payload.get("objects"), list) else 0,
    }


def summarize_results(results: list[dict[str, Any]]) -> dict[str, Any]:
    positives = [item for item in results if item["positive"]]
    negatives = [item for item in results if not item["positive"]]
    confidences = [float(item["confidence"]) for item in results if isinstance(item.get("confidence"), (int, float))]
    latencies = [float(item["latencyMs"]) for item in results if isinstance(item.get("latencyMs"), (int, float))]
    wrong_labels = Counter(
        str(item["detectedLabel"]) for item in results if item["wrongLabel"] and item.get("detectedLabel")
    )
    detected_labels = Counter(str(item["detectedLabel"]) for item in results if item["detected"] and item.get("detectedLabel"))
    reasons = Counter(str(item.get("reason", "")) for item in results if item.get("reason"))

    target_matches = sum(1 for item in positives if item["targetMatch"])
    false_positives = sum(1 for item in negatives if item["falsePositive"])
    return {
        "total": len(results),
        "positives": len(positives),
        "negatives": len(negatives),
        "detected": sum(1 for item in results if item["detected"]),
        "targetMatches": target_matches,
        "falseNegatives": sum(1 for item in positives if item["falseNegative"]),
        "wrongLabels": sum(1 for item in positives if item["wrongLabel"]),
        "falsePositives": false_positives,
        "trueNegatives": sum(1 for item in negatives if item["trueNegative"]),
        "targetFoundRate": target_matches / len(positives) if positives else None,
        "falsePositiveRate": false_positives / len(negatives) if negatives else None,
        "averageConfidence": statistics.fmean(confidences) if confidences else None,
        "averageLatencyMs": statistics.fmean(latencies) if latencies else None,
        "wrongLabelCounts": dict(sorted(wrong_labels.items())),
        "detectedLabelCounts": dict(sorted(detected_labels.items())),
        "reasonCounts": dict(sorted(reasons.items())),
    }


def evaluate_dataset(
    *,
    dataset_dir: Path,
    output_path: Path,
    results_path: Path,
    base_config: DetectionConfig,
    detector: DetectorBackend | None,
    object_target: str = "",
    filter_target_from_label: bool = False,
) -> dict[str, Any]:
    entries = load_dataset_entries(dataset_dir)
    results_path.parent.mkdir(parents=True, exist_ok=True)
    if results_path.exists():
        results_path.unlink()

    results: list[dict[str, Any]] = []
    errors = []
    for entry in entries:
        frame_path = Path(str(entry["framePath"]))
        try:
            frame = frame_path.read_bytes()
            target = object_target.strip()
            if filter_target_from_label and not target and is_positive_label(str(entry.get("label", ""))):
                target = str(entry.get("label", "")).strip()
            config = DetectionConfig(
                mode=base_config.mode,
                color=base_config.color,
                align_deadband=base_config.align_deadband,
                target_ratio_threshold=base_config.target_ratio_threshold,
                object_target=target,
                object_min_confidence=base_config.object_min_confidence,
                object_nms_threshold=base_config.object_nms_threshold,
                object_input_size=base_config.object_input_size,
                object_backend=base_config.object_backend,
                object_model=base_config.object_model,
                object_labels=base_config.object_labels,
                object_target_aliases=base_config.object_target_aliases,
                target_policy=base_config.target_policy,
            )
            started = time.monotonic()
            payload = detect_frame(frame, config, detector)
            elapsed_ms = (time.monotonic() - started) * 1000.0
            result = result_from_payload(entry=entry, payload=payload, elapsed_ms=elapsed_ms)
            results.append(result)
            append_jsonl(results_path, result)
        except Exception as exc:
            errors.append({"index": entry.get("index"), "frame": entry.get("frame"), "error": str(exc)})

    summary = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": utc_timestamp(),
        "dataset": str(dataset_dir),
        "resultsPath": str(results_path),
        "detector": {
            "mode": base_config.mode,
            "color": base_config.color,
            "objectBackend": base_config.object_backend,
            "objectModel": base_config.object_model,
            "objectLabels": base_config.object_labels,
            "objectTarget": object_target,
            "objectTargetAliases": list(base_config.object_target_aliases),
            "filterTargetFromLabel": filter_target_from_label,
            "objectMinConfidence": base_config.object_min_confidence,
            "objectInputSize": base_config.object_input_size,
            "targetPolicy": base_config.target_policy,
            "name": getattr(detector, "name", "color" if base_config.mode == "color" else ""),
        },
        "metrics": summarize_results(results),
        "errors": errors,
    }
    write_json(output_path, summary)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate MotionBrain detector settings against a captured vision dataset."
    )
    parser.add_argument("--dataset", required=True, help="Dataset directory created by capture_vision_dataset.py")
    parser.add_argument("--output", default="", help="Summary JSON path; default is <dataset>/evaluation_<timestamp>.json")
    parser.add_argument("--results-jsonl", default="", help="Per-frame JSONL path; default follows --output")
    parser.add_argument("--detector-mode", choices=("color", "object"), default="object")
    parser.add_argument("--detect-color", default="red")
    parser.add_argument("--object-backend", choices=("opencv-dnn",), default="opencv-dnn")
    parser.add_argument("--object-model", default="")
    parser.add_argument("--object-labels", default=str(ROOT / "config" / "coco80.labels"))
    parser.add_argument("--object-target", default="", help="Optional fixed target label, e.g. cup")
    parser.add_argument(
        "--object-target-alias",
        action="append",
        default=[],
        help="Additional labels accepted as the selected target, comma-separated or repeatable.",
    )
    parser.add_argument(
        "--filter-target-from-label",
        action="store_true",
        help="Use each positive frame label as the detector target filter when --object-target is empty.",
    )
    parser.add_argument("--object-min-confidence", type=float, default=0.5)
    parser.add_argument("--object-nms-threshold", type=float, default=0.45)
    parser.add_argument("--object-input-size", type=int, default=640)
    parser.add_argument("--target-policy", choices=("largest", "center", "highest-confidence"), default="largest")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    dataset_dir = Path(args.dataset)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    output_path = Path(args.output) if args.output else dataset_dir / f"evaluation_{stamp}.json"
    results_path = Path(args.results_jsonl) if args.results_jsonl else output_path.with_suffix(".jsonl")

    config = DetectionConfig(
        mode=args.detector_mode,
        color=args.detect_color,
        object_backend=args.object_backend,
        object_model=args.object_model,
        object_labels=args.object_labels,
        object_target=args.object_target,
        object_target_aliases=tuple(parse_label_list(args.object_target_alias)),
        object_min_confidence=args.object_min_confidence,
        object_nms_threshold=args.object_nms_threshold,
        object_input_size=args.object_input_size,
        target_policy=args.target_policy,
    )
    detector = build_detector(config)
    summary = evaluate_dataset(
        dataset_dir=dataset_dir,
        output_path=output_path,
        results_path=results_path,
        base_config=config,
        detector=detector,
        object_target=args.object_target,
        filter_target_from_label=args.filter_target_from_label,
    )
    metrics = summary["metrics"]
    print(
        "evaluated "
        f"{metrics['total']} frames, targetFoundRate={metrics['targetFoundRate']}, "
        f"falsePositiveRate={metrics['falsePositiveRate']}, output={output_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
