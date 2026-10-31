#!/usr/bin/env python3
"""Rank offline M2 candidates from saved audited states; never execute or fit."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.joint_action_selection import replay_saved_states


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--goals', type=float, nargs='+', required=True)
    parser.add_argument('--split', choices=('train', 'validation', 'test'), default='test')
    parser.add_argument('--tolerance-deg', type=float, default=1.0)
    parser.add_argument('--output', type=Path, required=True, help='new private JSON report')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output_already_exists')
    try:
        report = replay_saved_states(json.loads(args.model.read_text()),
                                     json.loads(args.dataset.read_text()), args.goals,
                                     split=args.split, tolerance_deg=args.tolerance_deg)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            stream.write(encoded)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(dict(output=str(args.output), counts=report['counts'],
                          loggedActionMatches=report['loggedActionMatches'],
                          trainingPerformed=False, controlEligible=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
