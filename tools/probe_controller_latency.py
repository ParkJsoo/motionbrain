#!/usr/bin/env python3
"""One 14-GET latency comparison. No camera, ARM, outputs or retry."""
import argparse
import json
from pathlib import Path
import sys
import signal
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.controller_latency import collect,summarize

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller-url',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    def terminate(signum,frame):raise KeyboardInterrupt('SIGTERM')
    previous=signal.signal(signal.SIGTERM,terminate)
    try:result=collect(args.output,args.controller_url)
    finally:signal.signal(signal.SIGTERM,previous)
    print(json.dumps(summarize(result),indent=2,allow_nan=False))
    raise SystemExit(130 if result.get('cancelled') else 0 if result['status']=='recorded' else 2)
