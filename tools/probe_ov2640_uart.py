#!/usr/bin/env python3
"""Approved single-handle power-on observation, or an offline audit. No uploader."""
import argparse
import json
from pathlib import Path
import signal
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.ov2640_uart_session import audit, collect


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--port')
    parser.add_argument('--usb-serial')
    parser.add_argument('--usb-vid', type=lambda s: int(s, 0))
    parser.add_argument('--usb-pid', type=lambda s: int(s, 0))
    parser.add_argument('--usb-location')
    parser.add_argument('--expected-app-elf-sha256')
    args = parser.parse_args(argv)
    execution_values = (args.output_dir, args.port, args.usb_serial, args.usb_vid, args.usb_pid,
                        args.usb_location, args.expected_app_elf_sha256)
    if args.audit:
        if args.execute or any(v is not None for v in execution_values):
            parser.error('audit cannot be combined with execution arguments')
        result = audit(args.audit)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['ok'] else 2
    if (not args.execute or any(v is None for v in (args.output_dir, args.port, args.usb_vid,
                                                   args.usb_pid, args.usb_location, args.expected_app_elf_sha256))):
        parser.error('explicit execution, output, port, USB identity and firmware hash required; obtain fresh approval first')
    identity = dict(serial_number=args.usb_serial, vid=args.usb_vid, pid=args.usb_pid, location=args.usb_location)
    def terminate(signum, frame):
        raise KeyboardInterrupt('SIGTERM')
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        result = collect(args.output_dir, args.port, identity, args.expected_app_elf_sha256)
    finally:
        signal.signal(signal.SIGTERM, previous)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 130 if result['failure'] == 'cancelled' else 0 if result['successfulRun'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
