#!/usr/bin/env python3
"""Explicit finite ESP32-CAM register request, or offline integrity audit."""
import argparse
import json
from pathlib import Path
import signal
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.ov2640_register_probe import audit, collect


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--url')
    parser.add_argument('--expected-app-elf-sha256')
    args = parser.parse_args(argv)
    if args.audit:
        if args.execute or args.output_dir or args.url or args.expected_app_elf_sha256:
            parser.error('audit cannot be combined with execution arguments')
        result = audit(args.audit)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['ok'] else 2
    if not args.execute or not all((args.output_dir, args.url, args.expected_app_elf_sha256)):
        parser.error('execution requires --execute --output-dir --url --expected-app-elf-sha256; fresh hardware approval is required')
    def terminate(signum, frame):
        raise KeyboardInterrupt('SIGTERM')
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        result = collect(args.output_dir, args.url, args.expected_app_elf_sha256)
    finally:
        signal.signal(signal.SIGTERM, previous)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 130 if result['failure'] == 'cancelled' else 0 if result['successfulRun'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
