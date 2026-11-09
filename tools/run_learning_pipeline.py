#!/usr/bin/env python3
"""비공개 manifest 하나로 수집 원본 감사·관측·학습·평가를 재실행한다."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.experiment_pipeline import run_pipeline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'.local'):
        parser.error('원본 경로·관측·실험 보고서는 .local 아래에 저장합니다')
    report = run_pipeline(args.manifest, output)
    print(json.dumps({'counts': report['counts'], 'report': str(output/'latest.json'),
                      'controlEligible': False, 'motorCommands': 0}, ensure_ascii=False))


if __name__ == '__main__':
    main()
