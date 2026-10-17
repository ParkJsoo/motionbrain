#!/usr/bin/env python3
"""One finite iPhone/controller observation session, or a read-only saved audit."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.mac_controller_probe import run_probe, audit_probe
from learning.artifacts import StorageError


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--audit",type=Path)
    p.add_argument("--output",type=Path)
    p.add_argument("--controller-url")
    p.add_argument("--helper")
    p.add_argument("--device-id")
    p.add_argument("--view-id")
    p.add_argument("--direct-http",action="store_true",help="use main-thread HTTP with a total request deadline")
    p.add_argument("--clock-bridge",action="store_true",help="record same-Mac PTS clock conversion; not optical exposure")
    args=p.parse_args(argv)
    if args.audit:
        r=audit_probe(args.audit);print(json.dumps(r,indent=2));return 0 if r["ok"] else 2
    if any(getattr(args,k) is None for k in ("output","controller_url","helper","device_id","view_id")):
        p.error("capture requires output, controller-url, helper, device-id and view-id")
    previous=signal.getsignal(signal.SIGTERM)
    def terminate(signum,frame):
        raise KeyboardInterrupt("SIGTERM")
    signal.signal(signal.SIGTERM,terminate)
    try:
        options={}
        if args.direct_http:
            from learning.controller_latency import direct_get
            options['read']=direct_get
        if args.clock_bridge:
            from learning.mac_clock_bridge import MacHostClock
            options['clock_probe']=MacHostClock()
        r=run_probe(args.output,controller_url=args.controller_url,helper=args.helper,
                    device_id=args.device_id,view_id=args.view_id,**options)
        print(json.dumps(r,indent=2,allow_nan=False))
        return 130 if r.get("cancelled") else 0 if r["status"]=="recorded" else 2
    except (StorageError,OSError,ValueError,subprocess.SubprocessError,KeyboardInterrupt) as exc:
        print(json.dumps(dict(error=type(exc).__name__,automaticRetry=False,trainingEligible=False)))
        return 130 if isinstance(exc,KeyboardInterrupt) else 2
    finally:
        signal.signal(signal.SIGTERM,previous)


if __name__=="__main__":
    raise SystemExit(main())
