#!/usr/bin/env python3
"""종료된 수동 수집 세션을 원본 삭제 없이 FFV1으로 보관·검증·복원한다."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.session_archive import create, restore, verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    for operation in ('create', 'verify', 'restore'):
        command = sub.add_parser(operation)
        command.add_argument('source', type=Path)
        if operation != 'verify':
            command.add_argument('destination', type=Path)
        command.add_argument('--ffmpeg', default='ffmpeg')
    args = parser.parse_args()
    action = dict(create=create, verify=verify, restore=restore)[args.operation]
    positional = [args.source] + ([] if args.operation == 'verify' else [args.destination])
    print(json.dumps(action(*positional, ffmpeg=args.ffmpeg), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
