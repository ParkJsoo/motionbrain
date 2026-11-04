#!/usr/bin/env python3
"""후속 수집을 고정된 기존 모델로 평가하며 다시 학습하지 않는다."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.artifacts import StorageError
from learning.manual_collection_learning import export_frozen


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('index','dataset','training-index','training-dataset','training-report','model','plan','output-dir'):
        parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args(argv)
    if not args.output_dir.resolve().is_relative_to(Path(__file__).resolve().parents[1]/'.local'):
        parser.error('평가 출력은 저장소 .local 아래여야 합니다')
    try:
        result=export_frozen(args.index,args.dataset,args.training_index,args.training_dataset,
                             args.training_report,args.model,args.plan,args.output_dir)
        print(json.dumps({k:result[k] for k in ('status','counts','metrics','trainingPerformed','independentTest')},allow_nan=False))
        return 0
    except (ValueError,TypeError,KeyError,OSError,StorageError) as exc:
        print(json.dumps(dict(status='refused',reason=str(exc),controlEligible=False)),file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())
