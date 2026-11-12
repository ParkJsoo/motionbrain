#!/usr/bin/env python3
"""개발 관측기의 fresh 원본 연결을 검사한다. 기본 촬영·모터 명령은 없다."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.development_gripper_check import run_check


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--frame-root', type=Path)
    mode.add_argument('--capture', action='store_true')
    args = parser.parse_args()
    result = run_check(args.config, args.output, frame_root=args.frame_root, capture=args.capture)
    print(json.dumps({key: result[key] for key in ('status', 'sourceMode', 'newCameraCaptures',
        'hardwareMotionCommands', 'trainingEligible')}, ensure_ascii=False))
    if result['status'] == 'failed':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
