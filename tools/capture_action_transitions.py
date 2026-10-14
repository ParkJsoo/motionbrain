#!/usr/bin/env python3
"""Collect synthetic action transitions locally, or audit an existing run."""
import argparse
import json
from pathlib import Path
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore, StorageError
from learning.dry_run import SCENARIOS, SyntheticAdapter
from learning.runner import CapturePersistenceError, CaptureRunner, audit_run


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="synthetic only; no devices or network")
    mode.add_argument("--audit-run", type=Path, help="read-only audit; never resume execution")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "datasets/action_transitions/dry_run")
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--scenario", choices=SCENARIOS, default="complete")
    args = parser.parse_args(argv)
    if not 1 <= args.steps <= 32:
        parser.error("--steps must be between 1 and 32")
    if args.audit_run:
        report = audit_run(args.audit_run)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["ok"] else 2
    run_dir = args.output_dir / uuid.uuid4().hex
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM")

    signal.signal(signal.SIGTERM, terminate)
    try:
        adapter = SyntheticAdapter(args.scenario)
        runner = CaptureRunner(ArtifactStore(run_dir), adapter)
        reports = []
        for index in range(args.steps):
            result = runner.capture(episode_id="episode-1")
            reports.append({"transitionId": result["transitionId"], "status": result["status"],
                            "executionState": result["executionState"], "missing": result["missing"]})
            if runner.halted:
                break
        runner.close()
        print(json.dumps({"runDir": str(run_dir), "mode": "dry_run", "evidence": "synthetic",
                          "trainingEligible": False, "executeCalls": adapter.execute_calls,
                          "attempts": reports}, indent=2, allow_nan=False))
        if any(item["status"] == "interrupted" for item in reports):
            return 130
        return 0 if all(item["status"] == "completed" for item in reports) else 2
    except (StorageError, CapturePersistenceError, OSError) as exc:
        print(json.dumps({"runDir": str(run_dir), "error": str(exc), "automaticRetry": False}), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print(json.dumps({"runDir": str(run_dir), "error": "interrupted; audit persisted evidence",
                          "automaticRetry": False}), file=sys.stderr)
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
