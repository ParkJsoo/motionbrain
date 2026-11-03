#!/usr/bin/env python3
"""Freeze a verified supervised model and baselines offline; no capture or scoring."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.artifacts import ArtifactStore, StorageError
from learning.supervised_evaluation import freeze


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, required=True)
    parser.add_argument('--training-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True, help='new immutable local directory')
    args = parser.parse_args(argv)
    store = None
    try:
        store = ArtifactStore(args.output_dir)
        bundle = freeze(args.index, args.training_run)
        ref = store.write_json('evaluation-bundle.json', bundle)
        result = dict(status='frozen_pending_new_data', artifact=ref, modelSha256=bundle['modelSha256'],
                      baselines=bundle['baselines'], evaluationPerformed=False, collectionAuthorized=False)
        store.write_json('result.json', result)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (ValueError, TypeError, KeyError, OSError, StorageError) as exc:
        result = dict(status='refused', reason=str(exc), collectionAuthorized=False)
        if store is not None:
            store.write_json('refusal.json', result)
        print(json.dumps(result), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
