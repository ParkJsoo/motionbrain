#!/usr/bin/env python3
"""Review saved collection evidence only; no devices, capture, retry or execution."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore, StorageError, describe_artifact, read_json_artifact
from learning.collection_readiness import (load_transition, review_demo, review_device_capture,
                                           review_transition, scene_template)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="saved evidence root; never a device URL")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", action="store_true", help="audit a manual demonstration archive")
    mode.add_argument("--device-capture", action="store_true", help="audit saved commissioning capture; never run it")
    mode.add_argument("--transition", help="relative JSON path, or CaptureRunner summary containing transition")
    parser.add_argument("--scene", help="relative scene evidence JSON path within root")
    parser.add_argument("--output-dir", type=Path, help="new private directory for report and unfilled template")
    args = parser.parse_args(argv)
    if not args.transition and args.scene:
        parser.error("--scene requires --transition")
    try:
        template = None
        if args.demo:
            report = review_demo(args.root)
        elif args.device_capture:
            report = review_device_capture(args.root)
        else:
            scene = (read_json_artifact(args.root, describe_artifact(args.root, args.scene))
                     if args.scene else None)
            report = review_transition(args.root, args.transition, scene)
            transition, ref = load_transition(args.root, args.transition)
            template = scene_template(transition, ref)
        if args.output_dir:
            # Never changes source files, resumes, or overwrites an old result.
            store = ArtifactStore(args.output_dir)
            if template is not None:
                store.write_json("scene-template.json", template)
            store.write_json("report.json", report)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        # A reviewable report is not a ready-to-train result. Blocked is exit 2.
        return 0 if report["recordedEvidenceComplete"] else 2
    except (StorageError, OSError, ValueError, TypeError) as exc:
        print(json.dumps(dict(error=type(exc).__name__, detail=str(exc), readOnly=True,
                              trainingEligible=False, executionAllowed=False)), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
