#!/usr/bin/env python3
"""Check an immutable plan, or explicitly run one bounded saved-model evaluation."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from learning.cv_checkpoint import execute, load_plan, preflight


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "run"))
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="explicit run only; does not replace user authorization")
    args = parser.parse_args(argv)
    try:
        if args.command == "check":
            if args.execute:
                raise ValueError("--execute is only valid with run")
            plan, digest = load_plan(args.plan)
            lock, dataset = preflight(plan)
            result = {"status": "checked", "planSha256": digest, "frames": [f.id for f in dataset.frames],
                      "checkpointSha256": lock["checkpointSha256"], "modelExecuted": False,
                      "imageDecodingChecked": False, "scope": plan.scope}
        else:
            result = execute(args.plan, confirmed=args.execute)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0 if result["status"] in ("checked", "completed") else 2
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(json.dumps({"status": "error", "reason": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
