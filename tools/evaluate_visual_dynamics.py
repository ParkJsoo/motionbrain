#!/usr/bin/env python3
"""Revalidate physical originals and compare no-change and fixed linear dynamics."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.artifacts import ArtifactStore, StorageError
from learning.visual_dynamics import evaluate, load_model, DatasetUnavailable


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args(argv)
    store=None
    try:
        store=ArtifactStore(args.output_dir)
        report=evaluate(args.manifest,load_model(args.model))
        store.write_json('evaluation.json',report)
        print(json.dumps(report,indent=2,allow_nan=False))
        return 0
    except (DatasetUnavailable,StorageError,ValueError,TypeError,KeyError,OSError) as exc:
        report={'status':'unavailable','reason':str(exc),'validation':getattr(exc,'validation',None),
                'physicalManipulationImprovementEstablished':False}
        if store is not None:
            try: store.write_json('unavailable.json',report)
            except StorageError: pass
        print(json.dumps(report,indent=2,allow_nan=False),file=sys.stderr)
        return 2


if __name__=='__main__': raise SystemExit(main())
