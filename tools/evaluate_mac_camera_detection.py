#!/usr/bin/env python3
"""Run one fixed CPU detector replay; no camera, control, or learning access."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.mac_camera_detection import run_evaluation, worker


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-dir', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--labels', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--worker-output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker_output is not None:
        worker(args.worker_output)
        return 0
    if any(getattr(args, key) is None for key in ('capture_dir', 'model', 'labels', 'output')):
        parser.error('--capture-dir, --model, --labels, and --output are required')
    report = run_evaluation(args.capture_dir, args.model, args.labels, args.output)
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report['offlineEvaluationComplete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
