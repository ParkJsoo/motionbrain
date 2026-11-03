#!/usr/bin/env python3
"""Fit settled-endpoint development models and retrospective diagnostics offline."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.artifacts import ArtifactStore, StorageError
from learning.supervised_prediction import run


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True, help='new immutable local directory')
    args = parser.parse_args(argv)
    store = None
    try:
        store = ArtifactStore(args.output_dir)
        report = run(args.index)
        ref = store.write_json('development-evaluation.json', report)
        result = dict(status='trained_and_explored', artifact=ref,
                      count=report['metadata']['count'], metrics=report['evaluation']['metrics'],
                      independentTest=False, controlEligible=False)
        store.write_json('result.json', result)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (ValueError, TypeError, KeyError, OSError, StorageError) as exc:
        result = dict(status='refused', reason=str(exc), controlEligible=False)
        if store is not None:
            store.write_json('refusal.json', result)
        print(json.dumps(result), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
