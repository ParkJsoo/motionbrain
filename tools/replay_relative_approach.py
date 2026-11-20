"""Replay a JSON array of audited, same-contract open-jaw transitions."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.relative_approach import RelativeApproachModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('transitions', type=Path)
    parser.add_argument('--query', type=Path, help='JSON with observation, goal_bands and commands')
    args = parser.parse_args()
    model = RelativeApproachModel(json.loads(args.transitions.read_text()))
    result = model.select(**json.loads(args.query.read_text())) if args.query else model.replay()
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
