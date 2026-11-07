#!/usr/bin/env python3
"""저장된 집게 카메라 원본의 관측 계약을 오프라인 진단한다."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.grasp_observation_diagnostic import run_study

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--config-sha256', required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    a = p.parse_args()
    r = run_study(a.config, a.config_sha256, a.output_dir, root=ROOT)
    print(json.dumps({k: r[k] for k in ('listedFrames', 'extractedFrames', 'failures',
                                      'flaggedFrames', 'modelRefit', 'controlEligible')}, ensure_ascii=False))

if __name__ == '__main__': main()
