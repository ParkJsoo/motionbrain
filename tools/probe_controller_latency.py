#!/usr/bin/env python3
"""One 14-GET latency comparison. No camera, ARM, outputs or retry."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.controller_latency import collect,summarize

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller-url',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=collect(args.output,args.controller_url)
    print(json.dumps(summarize(result),indent=2,allow_nan=False))
    raise SystemExit(0 if result['status']=='recorded' else 2)
