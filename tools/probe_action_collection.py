#!/usr/bin/env python3
"""Collect real camera/API evidence using GET only. Never drives the robot."""
import argparse
import json
from pathlib import Path
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore, StorageError
from learning.device_adapter import ReadOnlyHTTP
from learning.device_probe import audit_device_probe, run_device_probe


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-url")
    parser.add_argument("--camera-url")
    parser.add_argument("--perception-url", default="http://127.0.0.1:8766")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "datasets/device_probes")
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=4)
    parser.add_argument("--audit-run", type=Path)
    args = parser.parse_args(argv)
    if args.audit_run:
        result = audit_device_probe(args.audit_run)
        print(json.dumps(result, indent=2))
        return 0 if result["ok"] else 2
    if not args.controller_url or not args.camera_url:
        parser.error("--controller-url and --camera-url are required")
    if not 2 <= args.samples <= 100 or not 0 <= args.interval <= 10:
        parser.error("samples must be 2..100 and interval 0..10 seconds")
    run_dir = args.output_dir / uuid.uuid4().hex
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM")

    signal.signal(signal.SIGTERM, terminate)
    try:
        transport = ReadOnlyHTTP(controller_url=args.controller_url, camera_url=args.camera_url,
                                 perception_url=args.perception_url, timeout=args.timeout)
        report = run_device_probe(ArtifactStore(run_dir), transport, samples=args.samples, interval=args.interval)
        audit = audit_device_probe(run_dir)
        print(json.dumps({"runDir": str(run_dir), "status": report["status"], "audit": audit,
                          "completedSamples": report["completedSamples"], "clock": report["clock"],
                          "outputLogUnchanged": report["outputLogUnchanged"],
                          "motorCommandsSent": 0, "trainingEligible": False,
                          "blockers": report["blockers"], "errors": report["errors"]}, indent=2))
        if report["status"] == "interrupted":
            return 130
        return 0 if report["status"] == "completed" and audit["ok"] else 2
    except (StorageError, OSError, ValueError) as exc:
        print(json.dumps({"runDir": str(run_dir), "error": type(exc).__name__,
                          "automaticRetry": False, "instruction": "audit partial evidence; do not resume"}))
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
