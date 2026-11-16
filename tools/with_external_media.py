#!/usr/bin/env python3
"""승인된 외장 미디어 명세로 기존 오프라인 스크립트를 실행한다."""
import argparse
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.external_media import external_media_reads


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--receipt-sha256', required=True)
    parser.add_argument('--volume-uuid', required=True)
    parser.add_argument('script', type=Path)
    parser.add_argument('script_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    sys.argv = [str(args.script), *args.script_args]
    sys.path.insert(0, str(args.script.resolve().parent))
    with external_media_reads(args.receipt, args.receipt_sha256, args.volume_uuid):
        runpy.run_path(str(args.script), run_name='__main__')


if __name__ == '__main__':
    main()
