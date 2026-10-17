#!/usr/bin/env python3
"""Archive private manual hardware evidence; never sends motor commands."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.hardware_demo import archive_demo, audit_demo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--observation', choices=('unknown', 'grasped', 'placed', 'unsuccessful'), default='unknown')
    parser.add_argument('--note', default='')
    args = parser.parse_args()
    result = archive_demo(args.source, args.destination, observation=args.observation, note=args.note)
    errors = audit_demo(args.destination)
    print(json.dumps({'steps': len(result['attempts']), 'trainingEligible': False,
                      'controlEligible': False, 'auditErrors': errors}))
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
