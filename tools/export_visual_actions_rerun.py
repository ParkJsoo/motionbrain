#!/usr/bin/env python3
"""Export audited local observations for Rerun Viewer; does not install its SDK."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.visual_action_rerun import export_recording

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path)
    p.add_argument('--output',type=Path,required=True,help='new private directory/recording.rrd')
    a=p.parse_args()
    if not a.output.resolve().is_relative_to((ROOT/'.local').resolve()):
        p.error('RRD contains source images: keep it under .local')
    print(json.dumps(export_recording(a.recording,a.output),ensure_ascii=False))
