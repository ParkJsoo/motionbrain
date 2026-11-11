#!/usr/bin/env python3
"""유한 양턱 수집→학습→고정→새 평가. --run 없이 장치 접근 없는 계획 검사만 수행."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from learning.batch_jaw_job import JawJob,validate_job


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args(argv)
    config=validate_job(json.loads(args.config.read_bytes()))
    if not args.run:
        print(json.dumps(dict(status='validated',deviceAccess=False,executionAuthorized=False)))
        return 0
    from learning.batch_jaw_driver import create_driver
    report=JawJob(ROOT,config,create_driver).run()
    print(json.dumps(report,ensure_ascii=False))
    return 0 if report['status']=='completed' else 2


if __name__=='__main__': raise SystemExit(main())
