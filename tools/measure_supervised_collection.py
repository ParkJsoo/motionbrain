#!/usr/bin/env python3
"""Measure recorded supervised endpoints without devices or training."""
import argparse
import csv
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.supervised_measurement import measure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path,help='New output directory')
    args=parser.parse_args()
    report=json.loads(args.report.read_text());config=json.loads(args.config.read_text())
    result=measure(report,config,args.report.parent/'native-camera')
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'measurements.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    with (args.output/'measurements.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['id','captureGroup','split','executionStatus','status','reason','before','after','delta','pulse'])
        writer.writeheader()
        for row in result['rows']:
            writer.writerow({key:json.dumps(row.get(key),ensure_ascii=False) if key in ('before','after','pulse') else row.get(key) for key in writer.fieldnames})
    print(json.dumps(result['summary']))

if __name__=='__main__':main()
