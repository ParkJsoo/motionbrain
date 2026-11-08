#!/usr/bin/env python3
"""Prepare/validate offline. Explicit run captures and executes one attended block."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.artifacts import ArtifactStore, _decode_json
from learning.supervised_collection import PROFILES, make_plan, plan_digest, validate_plan, SupervisedCollection


def parser():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    sub.add_parser('profiles',help='historical ceilings only; no device access')
    a=sub.add_parser('prepare',help='write a private plan without accessing devices')
    for name in ('session-id','capture-group','device-id','view-id','pose-context','boot-id','controller-url','direction'):
        a.add_argument('--'+name,required=True)
    a.add_argument('--joint',choices=list(PROFILES),required=True)
    a.add_argument('--split',choices=['development','train','validation','test'],default='development')
    a.add_argument('--percent',type=int,required=True)
    a.add_argument('--durations-ms',type=int,nargs='+',required=True)
    a.add_argument('--helper',type=Path,required=True);a.add_argument('--output-plan',type=Path,required=True)
    a.add_argument('--arm-reference-template',type=Path,help='optional fixed ROI JSON; binds its original SHA to the plan')
    for name in ('validate','run'):
        a=sub.add_parser(name);a.add_argument('--plan',type=Path,required=True);a.add_argument('--helper',type=Path,required=True)
        a.add_argument('--arm-reference-template',type=Path)
        if name=='run':
            a.add_argument('--confirm-plan-sha256',required=True);a.add_argument('--onsite',action='store_true',required=True)
            a.add_argument('--output-dir',type=Path,required=True)
    return p


def private_path(p):
    p=p.resolve();private=(ROOT/'.local').resolve()
    if not p.is_relative_to(private):raise ValueError('private_artifact_path_required')
    return p


def arm_template(args,plan):
    path=getattr(args,'arm_reference_template',None)
    expected=plan.get('armReferenceTemplateSha256')
    if path is None and expected is None:return None
    if path is None or expected is None:raise ValueError('arm_reference_template_and_plan_pin_required')
    from learning.arm_session_reference import load_template
    raw=path.read_bytes();load_template(raw,expected)
    return raw


def run(args,plan):
    if not args.onsite or args.confirm_plan_sha256!=plan_digest(plan):raise ValueError('explicit_attended_plan_required')
    reference_template=arm_template(args,plan)
    out=private_path(args.output_dir)
    if out.exists():raise ValueError('new_output_directory_required')
    token=os.environ.get('MOTIONBRAIN_HTTP_TOKEN','')
    if not token or not all(33<=ord(c)<=126 for c in token):raise ValueError('runtime_token_required')
    if shutil.disk_usage(ROOT).free<plan['camera']['maxBytes']+2*1024**3:raise ValueError('storage_reserve_required')
    from learning.action_journal import BoundedActionJournal
    from learning.action_recorder import ActionRecorder
    from learning.supervised_transport import SequencePort
    # Same host lock as previous bounded-action users. No optional alternate ledger.
    journal=BoundedActionJournal(str(ROOT/'.local/visual-actions/controller-commands.sqlite'),10000)
    handlers={}
    try:
        if not journal.readiness()['ready']:raise ValueError('legacy_journal_unresolved')
        db=journal.connection
        with db:
            db.execute('CREATE TABLE IF NOT EXISTS supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT NOT NULL, output_path TEXT NOT NULL, phase TEXT NOT NULL, result_json TEXT)')
        if db.execute("SELECT 1 FROM supervised_blocks WHERE phase='pending' LIMIT 1").fetchone():
            raise ValueError('interrupted_block_requires_readonly_recovery_no_replay')
        if db.execute('SELECT 1 FROM supervised_blocks WHERE session_id=?',(plan['sessionId'],)).fetchone():raise ValueError('session_already_consumed')
        if db.execute("SELECT 1 FROM supervised_blocks WHERE phase='off_unverified' LIMIT 1").fetchone():raise ValueError('previous_output_off_unverified')
        store=ArtifactStore(out)
        camera=ActionRecorder(out/'native-camera',args.helper,plan['camera'])
        port=SequencePort(plan['controllerUrl'],token,plan['controllerBootId'],store)
        with db:
            db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,NULL)',(plan['sessionId'],plan_digest(plan),str(out),'pending'))
        def interrupted(signum,frame):
            for sig in handlers:signal.signal(sig,signal.SIG_IGN)
            raise KeyboardInterrupt()
        for sig in (signal.SIGINT,signal.SIGTERM):
            handlers[sig]=signal.signal(sig,interrupted)
        options={'arm_reference_template':reference_template} if reference_template is not None else {}
        report=SupervisedCollection(plan,camera,port,store,**options).run(plan_digest(plan))
        with db:
            db.execute('UPDATE supervised_blocks SET phase=?,result_json=? WHERE session_id=?',
                ('closed' if report['finalIdleOffVerified'] else 'off_unverified',json.dumps(report,allow_nan=False),plan['sessionId']))
        print(json.dumps({'status':report['status'],'summary':report['summary'],'finalIdleOffVerified':report['finalIdleOffVerified'],'report':str(out/'report.json')},ensure_ascii=False))
        return 0 if report['status']=='completed' else 2
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        journal.close()


def main(argv=None):
    args=parser().parse_args(argv)
    if args.mode=='profiles':
        print(json.dumps({'profiles':PROFILES,'maxPulses':3,'shoulderMaxPulses':1,'shoulderUpEnabled':False,'travelSafetyGuaranteed':False},indent=2));return 0
    if args.mode=='prepare':
        plan=make_plan(session_id=args.session_id,capture_group=args.capture_group,split=args.split,
            controller_url=args.controller_url,boot_id=args.boot_id,helper_sha256=hashlib.sha256(args.helper.read_bytes()).hexdigest(),
            device_id=args.device_id,view_id=args.view_id,pose_context=args.pose_context,joint=args.joint,direction=args.direction,
            percent=args.percent,durations=args.durations_ms)
        if args.arm_reference_template is not None:
            from learning.arm_session_reference import load_template
            raw=args.arm_reference_template.read_bytes();load_template(raw)
            plan['armReferenceTemplateSha256']=hashlib.sha256(raw).hexdigest()
            plan=validate_plan(plan)
        path=private_path(args.output_plan);path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('x') as f:json.dump(plan,f,ensure_ascii=False,indent=2);f.write('\n')
    else:
        plan=validate_plan(_decode_json(args.plan.read_bytes()))
        if hashlib.sha256(args.helper.read_bytes()).hexdigest()!=plan['helperSha256']:raise ValueError('helper_hash_mismatch')
        arm_template(args,plan)
        if args.mode=='run':return run(args,plan)
    print(json.dumps({'planSha256':plan_digest(plan),'pulses':plan['pulses'],'requiredFreeBytes':plan['camera']['maxBytes']+2*1024**3,'hardwareCalls':0,'captures':0},indent=2));return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:raise SystemExit(130)
    except Exception as exc:
        print(json.dumps({'error':type(exc).__name__,'reason':str(exc) if isinstance(exc,ValueError) else 'failed_preserve_private_evidence','automaticRetry':False}),file=sys.stderr)
        raise SystemExit(2)
