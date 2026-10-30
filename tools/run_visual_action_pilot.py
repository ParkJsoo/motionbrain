#!/usr/bin/env python3
"""Validate a pilot offline; explicit execute requires exact plan confirmation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.artifacts import _decode_json
from learning.visual_action_live import validate_plan, plan_digest, LiveActionSession, RecordingPort, observe_stream


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan',type=Path,required=True)
    ap.add_argument('--helper',type=Path,required=True)
    modes=ap.add_mutually_exclusive_group()
    modes.add_argument('--execute',action='store_true')
    modes.add_argument('--observe-only',action='store_true',help='explicit new camera capture; never constructs a motor port')
    modes.add_argument('--check-journal',action='store_true',help='offline readiness check before manual ARM; no camera or controller access')
    ap.add_argument('--confirm-plan-sha256')
    ap.add_argument('--controller-url')
    ap.add_argument('--output-dir',type=Path)
    ap.add_argument('--journal',type=Path)
    args=ap.parse_args(argv)
    plan=validate_plan(_decode_json(args.plan.read_bytes()))
    if hashlib.sha256(args.helper.read_bytes()).hexdigest()!=plan['helperSha256']:
        raise ValueError('camera_helper_hash_mismatch')
    digest=plan_digest(plan)
    if args.check_journal or args.execute:
        from learning.action_journal import read_journal
        if args.journal is None:raise ValueError('journal_path_required')
        check=read_journal(args.journal,command_id=plan['request']['commandId'])
        if args.check_journal or not check['ready']:
            print(json.dumps({'mode':'journal_readiness',**check},ensure_ascii=False,indent=2))
            return 0 if check['ready'] else 2
    if not args.execute and not args.observe_only:
        print(json.dumps({'mode':'validate_only','planSha256':digest,'request':plan['request'],
                          'camera':plan['camera'],'motorCalls':0,'captureCalls':0,'automaticArm':False},indent=2))
        return 0
    if args.confirm_plan_sha256!=digest or not args.output_dir:
        raise ValueError('capture_requires_exact_plan_hash_and_output')
    private=(ROOT/'.local').resolve()
    if not args.output_dir.resolve().is_relative_to(private):
        raise ValueError('physical_artifacts_and_journal_must_be_private')
    from learning.visual_camera_stream import NativeCameraStream
    from learning.visual_features import PointTracker
    c=plan['camera']
    tracker=PointTracker(c['width'],c['height'],plan['points'],max_displacement_px=plan['maxTravelPx'],reference_limit_px=plan['referenceLimitPx'])
    if args.observe_only:
        from learning.artifacts import ArtifactStore
        store=ArtifactStore(args.output_dir)
        store.write_json('pilot-plan.json',plan)
        camera=NativeCameraStream(args.output_dir/'native-camera',args.helper,c,tracker,host_clock_id=uuid.uuid4().hex)
        def stop_observation(signum,frame): raise KeyboardInterrupt()
        previous=signal.signal(signal.SIGTERM,stop_observation)
        try: report=observe_stream(camera)
        finally: signal.signal(signal.SIGTERM,previous)
        store.write_json('observation-result.json',report)
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 130 if report['operatorInterrupted'] else (0 if report['observationComplete'] else 2)
    if not args.controller_url or not args.journal or not args.journal.resolve().is_relative_to(private):
        raise ValueError('execution_requires_target_and_private_journal')
    token=os.environ.get('MOTIONBRAIN_HTTP_TOKEN','')
    if not token: raise ValueError('runtime_token_required')
    from learning.action_execution import ActionLimits,BoundedActionExecutor
    from learning.action_port import BoundedActionHTTP
    port=RecordingPort(BoundedActionHTTP(args.controller_url,writes_enabled=True,token_provider=lambda:token,timeout=.8,
                                       required_profile="m2_up_pwm180_ms1500",total_deadline=True))
    # One durable journal must be shared across runs on this controller; never reset it.
    args.journal.parent.mkdir(parents=True,exist_ok=True)
    executor=BoundedActionExecutor(args.journal.resolve(),port,ActionLimits(motor=2,max_duration_ms=1500,
                                   max_pwm=180,max_percent=0,min_degrees=0,max_degrees=0),max_polls=100)
    # Recheck under the executor's exclusive journal lock (the offline check is advisory).
    if not executor.journal.readiness(plan['request']['commandId'])['ready']:
        executor.close()
        raise ValueError('journal_changed_before_capture')
    camera=NativeCameraStream(args.output_dir/'native-camera',args.helper,c,tracker,
        host_clock_id=port.host_clock_id,on_fault=executor.request_stop)
    session=None
    def stop(signum,frame):
        executor.request_stop()
        raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        session=LiveActionSession(args.output_dir,plan,camera,executor,port)
        report=session.run(confirmed_request=plan['request'])
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 130 if report['operatorInterrupted'] else (0 if report['recordingComplete'] else 2)
    finally:
        signal.signal(signal.SIGTERM,previous)
        if session is None: executor.close()


if __name__=='__main__':
    try: raise SystemExit(main())
    except KeyboardInterrupt: raise SystemExit(130)
    except Exception as exc:
        print(json.dumps({'error':type(exc).__name__,'detail':'pilot_failed_preserve_output_and_inspect_journal','automaticRetry':False}),file=sys.stderr)
        raise SystemExit(2)
