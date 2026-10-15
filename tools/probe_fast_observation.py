#!/usr/bin/env python3
"""Measure current-frame ROI correspondence in a GET-only shadow run. No motor commands."""
import argparse
import json
from pathlib import Path
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from learning.artifacts import ArtifactStore
from learning.device_adapter import ReadOnlyHTTP
from learning.fast_probe import run_fast_probe


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-url", required=True)
    parser.add_argument("--camera-url", required=True)
    parser.add_argument("--perception-url", default="http://127.0.0.1:8766")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "datasets/fast_probes")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--match-workers", type=int, choices=(1, 2, 3), default=3,
                        help="parallel scale matches within one frame; 1 reproduces serial matching")
    parser.add_argument("--fresh-capture", action="store_true",
                        help="opt-in post-request first-DMA capture; requires supported camera firmware")
    parser.add_argument("--serial-persistence", action="store_true",
                        help="baseline: sequential receipt/JPEG storage, no directory preparation")
    parser.add_argument("--capture-timing", action="store_true",
                        help="record header/body timing and guarded camera status association (GET only)")
    parser.add_argument("--connection-timing", action="store_true",
                        help="also split HTTP camera resolution/connect/send intervals; requires --capture-timing")
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 100:
        parser.error("samples must be 1..100")
    if args.connection_timing and not args.capture_timing:
        parser.error("--connection-timing requires --capture-timing")
    transport = ReadOnlyHTTP(controller_url=args.controller_url, camera_url=args.camera_url,
                             perception_url=args.perception_url, connection_timing=args.connection_timing)
    run_dir = args.output_dir / uuid.uuid4().hex
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM")

    signal.signal(signal.SIGTERM, terminate)
    try:
        report = run_fast_probe(ArtifactStore(run_dir), transport, samples=args.samples,
                                capture_timing=args.capture_timing, match_workers=args.match_workers,
                                parallel_persistence=not args.serial_persistence, fresh_capture=args.fresh_capture)
        print(json.dumps({"runDir": str(run_dir), **{k: v for k, v in report.items()
                          if k not in ("artifacts", "results")},
                          "measurements": [{k: v for k, v in r.items() if k not in ("image", "source")}
                                           for r in report["results"]]}, indent=2))
        return 130 if report["status"] == "interrupted" else (0 if report["status"] == "completed" else 2)
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
