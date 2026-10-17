#!/usr/bin/env python3
"""Six framed read-only USB M4 requests, eight HTTP checks, no camera or motion."""
import argparse
import json
from pathlib import Path
import signal
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from learning.serial_m4_probe import collect,audit

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--controller-url');p.add_argument('--port');p.add_argument('--usb-serial')
    p.add_argument('--initialize-port',action='store_true',help='permit a driver reset before a new baseline: 20s startup, 12 HTTP checks, 32s total')
    p.add_argument('--fresh-sample',action='store_true',help='S2: one reply after the next regular sensor update, with MCU wait diagnostics')
    p.add_argument('--transport-trace',action='store_true',help='record bounded host write/read/parse observations; not wire timestamps')
    p.add_argument('--compact-sample',action='store_true',help='S3: fresh sample with full identity/timestamps and CRC32 in a 58-byte response')
    p.add_argument('--firmware-timing',action='store_true',help='two read-only RAM stage snapshots; requires compact sample')
    a=p.parse_args()
    if a.audit:
        r=audit(a.audit);print(json.dumps(r,indent=2));return 0 if r['ok'] else 2
    if not all((a.output,a.controller_url,a.port,a.usb_serial)):p.error('output, controller-url, port and usb-serial required')
    def terminate(signum,frame):raise KeyboardInterrupt('SIGTERM')
    old=signal.signal(signal.SIGTERM,terminate)
    try:
        r=collect(a.output,a.controller_url,a.port,a.usb_serial,initialize_port=a.initialize_port,fresh_sample=a.fresh_sample,transport_trace=a.transport_trace,compact_sample=a.compact_sample,firmware_timing=a.firmware_timing)
        print(json.dumps(r,indent=2));return 130 if r.get('cancelled') is True else (0 if r['status']=='recorded' else 2)
    finally:signal.signal(signal.SIGTERM,old)

if __name__=='__main__':raise SystemExit(main())
