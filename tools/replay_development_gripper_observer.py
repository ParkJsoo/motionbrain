#!/usr/bin/env python3
"""저장된 집게 영상만 새 개발 관측기로 재생한다. 촬영·구동·학습은 하지 않는다."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.development_gripper_replay import run_replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = run_replay(args.manifest, args.output)
    print(json.dumps({key: report[key] for key in ('kind', 'frames', 'completeFrames',
        'boundaryObjectRejections', 'trainingEligible', 'hardwareRequests', 'motorCommands')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
