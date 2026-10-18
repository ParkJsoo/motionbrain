#!/usr/bin/env python3
"""Run a synthetic optical model only; never display, capture, or contact devices."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore, StorageError, describe_artifact, read_json_artifact
from learning.optical_challenge import evaluate_case, synthetic_case


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--synthetic-example", action="store_true")
    mode.add_argument("--root", type=Path, help="saved synthetic packet directory")
    parser.add_argument("--output-dir", type=Path, required=True, help="new private output directory")
    args = parser.parse_args(argv)
    try:
        if args.synthetic_example:
            case = synthetic_case()
        else:
            manifest = read_json_artifact(args.root, describe_artifact(args.root, "manifest.json"))
            case = read_json_artifact(args.root, manifest["case"])
        report = evaluate_case(case)
        store = ArtifactStore(args.output_dir)
        case_ref = store.write_json("case.json", case)
        report_ref = store.write_json("report.json", report)
        store.write_json("manifest.json", dict(case=case_ref, report=report_ref,
                                              caseOrigin="synthetic", trainingEligible=False))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        # Exit 0 means a consistent hypothetical model, never physical eligibility.
        return 0 if report["modelConsistent"] else 2
    except (StorageError, OSError, ValueError, TypeError, KeyError) as exc:
        print(json.dumps(dict(error=str(exc), trainingEligible=False, executionAllowed=False)), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
