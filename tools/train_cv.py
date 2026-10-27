#!/usr/bin/env python3
"""Check a local pilot plan or explicitly execute one bounded development fit."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.cv_training import FINISHED, execute, load_plan, preflight


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('check', 'run'))
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--execute', action='store_true', help='required for run; does not replace user approval')
    args = parser.parse_args(argv)
    try:
        if args.command == 'check':
            if args.execute:
                raise ValueError('--execute is only valid with run')
            plan, digest = load_plan(args.plan)
            result = dict(preflight(plan), planSha256=digest, trainingExecuted=False)
        else:
            result = execute(args.plan, confirmed=args.execute)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0 if args.command == 'check' or result['status'] in FINISHED else 2
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(json.dumps({'status':'error','reason':str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
