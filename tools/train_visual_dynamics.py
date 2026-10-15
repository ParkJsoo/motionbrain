#!/usr/bin/env python3
"""Train a fixed CPU linear dynamics model from a verified physical manifest."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.artifacts import ArtifactStore, StorageError
from learning.visual_dynamics import train, DatasetUnavailable


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True,help='new immutable output directory')
    args=p.parse_args(argv)
    store=None
    try:
        store=ArtifactStore(args.output_dir)
        artifact=train(args.manifest)
        ref=store.write_json('model.json',artifact)
        result={'status':'trained','model':ref,'metadata':artifact['metadata'],
                'physicalManipulationImprovementEstablished':False}
        store.write_json('result.json',result)
        print(json.dumps(result,indent=2,allow_nan=False))
        return 0
    except (DatasetUnavailable,StorageError,ValueError,TypeError,KeyError,OSError) as exc:
        result={'status':'refused','reason':str(exc),'validation':getattr(exc,'validation',None),
                'physicalManipulationImprovementEstablished':False}
        if store is not None:
            try: store.write_json('refusal.json',result)
            except StorageError: pass
        print(json.dumps(result,indent=2,allow_nan=False),file=sys.stderr)
        return 2


if __name__=='__main__': raise SystemExit(main())
