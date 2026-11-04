#!/usr/bin/env python3
"""수동 수집 영상·예정 명령 기준 모델을 오프라인 학습하고 개발용 진단을 저장한다."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.artifacts import StorageError
from learning.manual_collection_learning import export


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True,help='비공개 .local 아래의 새 결과 폴더')
    args=parser.parse_args(argv)
    if not args.output_dir.resolve().is_relative_to(Path(__file__).resolve().parents[1]/'.local'):
        parser.error('학습 출력은 저장소 .local 아래여야 합니다')
    try:
        result=export(args.index,args.dataset,args.output_dir)
        print(json.dumps(dict(counts=result['counts'],metrics=result['diagnostics']['metrics'],
            trainingPerformed=True,independentTest=False,controlEligible=False),allow_nan=False))
        return 0
    except (ValueError,TypeError,KeyError,OSError,StorageError) as exc:
        print(json.dumps(dict(status='refused',reason=str(exc),controlEligible=False)),file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())
