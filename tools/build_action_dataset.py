#!/usr/bin/env python3
"""Build or validate a local action dataset inventory. No training or devices."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore, StorageError, describe_artifact, read_json_artifact
from learning.dataset import build_manifest, validate_manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--runs", nargs="+", type=Path, help="explicit saved run directories")
    mode.add_argument("--validate", type=Path, help="manifest JSON; reload and verify every source")
    parser.add_argument("--output-dir", type=Path, help="new immutable manifest directory")
    parser.add_argument("--seed", default="motionbrain-v1")
    parser.add_argument("--require-ready", action="store_true", help="also require nonempty physical train/validation/test")
    args = parser.parse_args(argv)
    if args.runs and args.output_dir is None:
        parser.error("--runs requires --output-dir")
    if args.validate and args.output_dir is not None:
        parser.error("--validate is read-only; --output-dir is not accepted")
    try:
        if args.runs:
            manifest = build_manifest(args.runs, seed=args.seed)
            report = validate_manifest(manifest)
            store = ArtifactStore(args.output_dir)
            store.write_json("manifest.json", manifest)
            store.write_json("validation.json", report)
            report["manifestPath"] = str(store.root / "manifest.json")
        else:
            path = args.validate.absolute()
            manifest = read_json_artifact(path.parent, describe_artifact(path.parent, path.name))
            report = validate_manifest(manifest)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["valid"] and (not args.require_ready or report["readyForTraining"]) else 2
    except (StorageError, ValueError, OSError) as exc:
        print(json.dumps({"valid": False, "readyForTraining": False, "errors": [str(exc)]}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
