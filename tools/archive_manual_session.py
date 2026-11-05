#!/usr/bin/env python3
"""종료된 수집 세션을 무손실 보관·검증·복원하고 검증된 프레임 사본을 정리한다."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.session_archive import create, prune, restore, restore_missing, verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    for operation in ('create', 'verify', 'restore', 'prune', 'restore-missing'):
        command = sub.add_parser(operation)
        command.add_argument('source', type=Path)
        if operation != 'verify':
            command.add_argument('destination', type=Path)
        command.add_argument('--ffmpeg', default='ffmpeg')
    args = parser.parse_args()
    action = {'create': create, 'verify': verify, 'restore': restore,
              'prune': prune, 'restore-missing': restore_missing}[args.operation]
    positional = [args.source] + ([] if args.operation == 'verify' else [args.destination])
    print(json.dumps(action(*positional, ffmpeg=args.ffmpeg), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
