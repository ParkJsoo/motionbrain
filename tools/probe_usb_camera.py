#!/usr/bin/env python3
"""One finite same-session USB/camera observation, or an offline audit."""
import argparse
import json
from pathlib import Path
import signal
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.usb_camera_probe import collect,audit


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path);p.add_argument('--output',type=Path)
    for flag in ('controller-url','port','usb-serial','helper','device-id','view-id'):p.add_argument('--'+flag)
    p.add_argument('--initialize-port',action='store_true',help='allow startup reset before new baseline, 20s drain, total 80s')
    a=p.parse_args(argv)
    if a.audit:
        r=audit(a.audit);print(json.dumps(r,indent=2));return 0 if r['ok'] else 2
    required=('output','controller_url','port','usb_serial','helper','device_id','view_id')
    if any(getattr(a,k) is None for k in required):p.error('output, controller, explicit USB identity and camera identity required')
    def terminate(signum,frame):raise KeyboardInterrupt('SIGTERM')
    previous=signal.signal(signal.SIGTERM,terminate)
    try:
        r=collect(**{k:getattr(a,k) for k in required},initialize_port=a.initialize_port)
        print(json.dumps(r,indent=2,allow_nan=False))
        return 130 if r.get('cancelled') else 0 if r['status']=='recorded' else 2
    finally:signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':raise SystemExit(main())
