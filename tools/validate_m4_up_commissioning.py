#!/usr/bin/env python3
"""M4 위 단발 시운전 범위를 오프라인으로만 검증한다. 실행 옵션은 없다."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.artifacts import _decode_json, _read
from learning.m4_up_commissioning import implementation_digest, validate_scope
from learning.supervised_collection import plan_digest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--scope', type=Path, help='.local 아래의 시운전 범위 JSON')
    mode.add_argument('--implementation-sha256', action='store_true', help='현재 구현 해시만 출력')
    args = parser.parse_args(argv)
    if args.implementation_sha256:
        print(implementation_digest())
        return 0
    try:
        path = args.scope.absolute()
        root = (ROOT/'.local').resolve(strict=True)
        # _read checks every path component without following symlinks.
        relative = path.relative_to(root).as_posix()
        scope = validate_scope(_decode_json(_read(root, relative)))
        result = dict(status='validated_offline', scopeSha256=plan_digest(scope),
                      pulse=scope['pulse'], defaultSpeed=scope['defaultSpeed'], maxCommands=1,
                      hardwareCalls=0, generalM4UpEnabled=False, liveReadinessVerified=False)
    except (Exception, KeyboardInterrupt) as exc:
        # Never echo a private scope or parser exception containing source values.
        print(json.dumps(dict(status='rejected_offline', errorType=type(exc).__name__, hardwareCalls=0)))
        return 2
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
