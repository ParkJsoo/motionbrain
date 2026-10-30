#!/usr/bin/env python3
"""Record synthetic visual actions, import saved observations, or audit local data."""
import argparse
import json
from pathlib import Path
import signal
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.artifacts import StorageError
from learning.visual_action_collection import audit_collection, replay_bundle
from learning.visual_action_examples import SCENARIOS, run_synthetic, write_example


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    dry = modes.add_parser("dry-run", help="synthetic images/time/output; no device access")
    dry.add_argument("--output-dir", type=Path, required=True)
    dry.add_argument("--scenario", choices=SCENARIOS, default="complete")
    example = modes.add_parser("example", help="write a synthetic bundle showing the input contract")
    example.add_argument("--output-dir", type=Path, required=True)
    replay = modes.add_parser("replay", help="copy and link saved local BGRA frames and events")
    replay.add_argument("--bundle", type=Path, required=True)
    replay.add_argument("--output-dir", type=Path, required=True)
    audit = modes.add_parser("audit", help="read-only; never resume or retry")
    audit.add_argument("run_dir", type=Path)
    args = parser.parse_args(argv)
    previous = signal.getsignal(signal.SIGTERM)
    def terminate(signum, frame):
        raise KeyboardInterrupt("terminated")
    signal.signal(signal.SIGTERM, terminate)
    try:
        if args.mode == "example":
            report = {"bundle": str(write_example(args.output_dir)), "evidence": "synthetic", "trainingEligible": False}
            status = 0
        else:
            if args.mode == "audit":
                report = audit_collection(args.run_dir)
            elif args.mode == "replay":
                report = replay_bundle(args.bundle, args.output_dir)
            else:
                report = run_synthetic(args.output_dir, args.scenario)
            status = 0 if report["recordingComplete"] else 2
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        return status
    except KeyboardInterrupt:
        print(json.dumps({"error": "interrupted_audit_saved_run", "automaticRetry": False}), file=sys.stderr)
        return 130
    except (StorageError, OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"error": type(exc).__name__, "detail": "recording_failed_audit_saved_run",
                          "automaticRetry": False}), file=sys.stderr)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
