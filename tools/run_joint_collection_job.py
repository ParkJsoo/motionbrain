#!/usr/bin/env python3
"""모든 관절의 독립 작업을 순차 실행하는 공통 CLI. 기본은 장치 없는 구조 검사."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from learning.batch_joint_job import JointSuite,validate_suite

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);p.add_argument('--run',action='store_true')
    a=p.parse_args(argv);config=validate_suite(json.loads(a.config.read_bytes()))
    if not a.run:
        print(json.dumps(dict(status='structure_validated',deviceAccess=False,executionAuthorized=False,staticAdaptersChecked=False,joints=[j['trainingPlan']['conditions'][0]['command']['joint'] for j in config['jobs']])));return 0
    report=JointSuite(ROOT,config).run();print(json.dumps(report,ensure_ascii=False));return 0 if report['status']=='completed' else 2
if __name__=='__main__':raise SystemExit(main())
