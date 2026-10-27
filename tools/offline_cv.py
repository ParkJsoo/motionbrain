#!/usr/bin/env python3
"""Inspect reviewed images, render annotation references, or evaluate saved predictions."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.cv_dataset import CVDataset, load_candidate_frames, read_json  # noqa: E402
from learning.cv_evaluation import evaluate  # noqa: E402


def write_json(path, report):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("inspect", "render", "evaluate"):
        command = commands.add_parser(name)
        command.add_argument("--manifest", type=Path, required=True)
        command.add_argument("--split", choices=("train", "train_development_pilot", "validation", "test"))
        command.add_argument("--output-dir", type=Path, required=True)
        if name == "evaluate":
            command.add_argument("--predictions", type=Path, required=True)
            command.add_argument("--score-threshold", type=float, required=True)
            command.add_argument("--nms-threshold", type=float, required=True)
    reference = commands.add_parser("render-reference")
    reference.add_argument("--labels", type=Path, required=True)
    reference.add_argument("--captures", type=Path, required=True)
    reference.add_argument("--source-root", type=Path, required=True)
    reference.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists():
            raise ValueError("output directory already exists; outputs are never overwritten")
        if args.command == "render-reference":
            frames = load_candidate_frames(args.labels, args.captures, args.source_root)
        else:
            dataset = CVDataset(args.manifest, split=args.split)
            frames = dataset.frames
        if args.command in ("render", "render-reference"):
            from learning.cv_display import render_frames
            report = render_frames(frames, args.output_dir)
        else:
            if args.command == "evaluate":
                report = evaluate(dataset, read_json(args.predictions), score_threshold=args.score_threshold,
                                  nms_threshold=args.nms_threshold)
            else:
                # Fail closed rather than silently omitting images from a training loader.
                records = []
                for i, frame in enumerate(frames):
                    image, target = dataset[i]
                    records.append({"id": frame.id, "imageId": target["image_id"], "shape": list(image.shape),
                                    "sourceSha256": frame.sha256, "sessionId": frame.session,
                                    "leakageGroup": frame.group, "split": frame.split,
                                    "boxes": len(target["boxes"]), "labels": target["labels"]})
                report = {"schema": "motionbrain.cv_inspection.v1", "frames": records,
                          "manifestSha256": dataset.manifest_sha256, "trainingExecuted": False}
            args.output_dir.mkdir(parents=True, exist_ok=False)
        write_json(args.output_dir / "report.json", report)
        print(json.dumps({"report": str(args.output_dir / "report.json"), "frames": len(frames)}, ensure_ascii=False))
        failures = report.get("processingFailures", report.get("metrics", {}).get("processingFailures", 0))
        return 2 if failures else 0
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(json.dumps({"status": "error", "reason": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
