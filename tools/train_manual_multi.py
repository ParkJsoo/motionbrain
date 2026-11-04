#!/usr/bin/env python3
"""명시한 조건별 수집 자료로 다조건 모델을 개발 진단한다."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.artifacts import StorageError
from learning.manual_multi_learning import export


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',action='append',nargs=2,metavar=('INDEX','DATASET'),required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if not a.output_dir.resolve().is_relative_to(Path(__file__).resolve().parents[1]/'.local'):
        p.error('출력은 저장소 .local 아래여야 합니다')
    try:
        r=export(a.source,a.output_dir)
        print(json.dumps(dict(metrics=r['diagnostics']['metrics'],trainingPerformed=True,independentTest=False,controlEligible=False)))
    except (ValueError,TypeError,KeyError,OSError,StorageError) as e:
        print(json.dumps(dict(status='refused',reason=str(e))),file=sys.stderr);return 2
    return 0

if __name__=='__main__':sys.exit(main())
