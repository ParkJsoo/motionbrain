#!/usr/bin/env python3
"""One bounded Mac direct-camera probe or immutable offline evidence audit."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.mac_camera_probe import evaluate_capture, run_probe


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='mode', required=True)
    listing = commands.add_parser('list', help='Enumerate devices without capturing images')
    listing.add_argument('--helper', required=True)
    capture = commands.add_parser('capture', help='Capture one finite commissioning run; no retries')
    capture.add_argument('--helper', required=True)
    capture.add_argument('--device-id', required=True)
    capture.add_argument('--view-id', required=True, help='New configuration identity for the changed camera/desk view')
    capture.add_argument('--output', required=True, type=Path)
    capture.add_argument('--session-id')
    audit = commands.add_parser('audit', help='Verify existing evidence without camera access or mutation')
    audit.add_argument('directory', type=Path)
    args = parser.parse_args(argv)
    if args.mode == 'list':
        return subprocess.run([str(Path(args.helper).resolve(strict=True)), '--list'], timeout=15, check=False).returncode
    if args.mode == 'audit':
        report = evaluate_capture(args.directory)
    else:
        report = run_probe(args.helper, args.output, args.device_id, args.view_id, session_id=args.session_id)
    print(json.dumps(report, sort_keys=True, indent=2, allow_nan=False))
    # Success means acquisition/processing evidence only; overall freshness stays unverified.
    return 0 if report['captureProcessingPassed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
