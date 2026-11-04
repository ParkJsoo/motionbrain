#!/usr/bin/env python3
"""저장된 수동 수집 기록을 오프라인 영상·명령 데이터셋으로 검증한다."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.manual_collection_dataset import export_dataset


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index',type=Path,required=True,help='명시적인 세션 목록과 원본 해시')
    parser.add_argument('--output-dir',type=Path,required=True,help='새 로컬 결과 디렉터리')
    parser.add_argument('--export-png',action='store_true',help='선택된 입력·목표 영상만 무손실 PNG로 내보내기')
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[1]
    if not args.output_dir.resolve().is_relative_to(repo/'.local'):
        parser.error('비공개 데이터셋 출력은 저장소 .local 아래여야 합니다')
    result=export_dataset(args.index,args.output_dir,export_png=args.export_png)
    print(json.dumps(dict(counts=result['counts'],trainingPerformed=False,independentTest=False),ensure_ascii=False))
    return 0 if result['counts']['rejected']==0 else 2


if __name__=='__main__':raise SystemExit(main())
