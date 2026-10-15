#!/usr/bin/env python3
"""One fixed GET-only 20-request evaluation, or offline audit of its original files."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from learning.artifacts import ArtifactStore, read_events, read_json_artifact, verify_artifacts
from learning.device_adapter import ReadOnlyHTTP
from learning.fast_probe import run_fast_probe
from learning.observation_attempt import POLICY
from learning.source_age import source_age_basis, source_age_upper_ms
from learning.anchor_age import causal_anchor_basis, anchor_age_upper_ms


def audit(root):
    events, errors = read_events(root)
    if errors or not events or events[-1]['kind'] != 'finished':
        raise ValueError(f'missing/invalid terminal: {errors}')
    report = read_json_artifact(root, events[-1]['payload']['report'])
    if verify_artifacts(root, report['artifacts']):
        raise ValueError('artifact integrity failure')
    refs = {r['artifactRef']:r for r in report['artifacts']}
    session = read_json_artifact(root, refs['session.json'])
    for name, digest in session['sourceSha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'source hash mismatch: {name}')
    attempt = report['observationAttempt']
    if attempt['policy'] != POLICY:
        raise ValueError('policy mismatch')
    requested = [e for e in events if e['kind']=='capture_requested']
    if not (attempt['currentCaptures'] <= len(requested) <= attempt['currentCaptures'] + (attempt['status'] != 'success')):
        # An interruption between intent and GET cannot be called a confirmed capture.
        raise ValueError('capture intent/count mismatch')
    rows = report.get('consumptions',[])
    seen = set()
    previous = None
    anchor = None
    if rows:
        evidence = read_json_artifact(root, refs['anchor_evidence.json'])
        anchor = causal_anchor_basis(evidence,session['hostClockId'])
    receipts = [read_json_artifact(root,r) for name,r in refs.items() if name.startswith('raw/')]
    current_receipts = [r for r in receipts if r.get('path')=='/capture'][1:]
    for index, (m,row) in enumerate(zip(report['results'], rows)):
        source = m['source']
        if source['sourceFrameId'] in seen:
            raise ValueError('duplicate frame')
        seen.add(source['sourceFrameId'])
        r = current_receipts[index]
        if r['sha256'] != source['imageSha256'] or m['image']['sha256'] != r['sha256']:
            raise ValueError('original/measurement mismatch')
        if previous is not None and r['requestStartedMonotonic'] < previous:
            raise ValueError('capture precedes persistence barrier')
        previous = row.get('frameEvidenceCompletedMonotonic')
        basis = source_age_basis(r,source,request_started=r['requestStartedMonotonic'],
            received=r['receivedMonotonic'],host_clock_id=session['hostClockId'],fresh_age_mode='send_start')
        age = source_age_upper_ms(basis,source['reportedSourceAgeMs'],row['consumedMonotonic'])
        if abs(age-row['sourceAgeAtConsumptionMs'])>1e-6:
            raise ValueError('age mismatch')
        if row['consumed'] and (age>100 or row['consumedMonotonic']>attempt['deadlineMonotonic'] or
                anchor_age_upper_ms(anchor,row['consumedMonotonic'],session['hostClockId'],session['hostClockId'])>15000):
            raise ValueError('invalid consumption')
    if attempt['status']=='success':
        successes=[r for r in rows if r['consumed'] and r['storageStatus']=='completed']
        if (report['status']!='completed' or len(successes)!=1 or not report['outputLogUnchanged'] or
                not 1<=attempt['currentCaptures']<=5 or not 0<=attempt['operationElapsedMs']<=3000 or
                successes[0]['persistenceElapsedMs']>250 or len(rows)!=attempt['currentCaptures']):
            raise ValueError('invalid success')
    return report


def stats(values):
    values=[v for v in values if v is not None]
    return {'n':len(values), 'p50':float(np.percentile(values,50)) if values else None,
            'p95':float(np.percentile(values,95)) if values else None, 'max':max(values) if values else None}


def summarize(reports, wall_times):
    attempts=[r['observationAttempt'] for r in reports]
    success=sum(a['status']=='success' for a in attempts)
    rows=[row for r in reports for row in r.get('consumptions',[])]
    return {'policy':POLICY,'plannedRequests':20,'completedRequests':len(reports),
            'unexecutedRequests':20-len(reports),'successes':success,'successRateAllPlanned':success/20,
            'gatePassed':len(reports)==20 and success>=16,
            'acquisitionFailures':sum(a['status']=='acquisition_failed' for a in attempts),
            'currentCaptures':sum(a['currentCaptures'] for a in attempts),
            'checkpointCaptures':sum(any(x['artifactRef']=='anchor_checkpoint.json' for x in r['artifacts']) for r in reports),
            'budgetOverruns':sum(a['operationBudgetExceeded'] for a in attempts),
            'storageLate':sum(r['storageStatus']=='late' for r in rows),
            'failureReasons':dict(Counter(a['reason'] for a in attempts if a['status']!='success')),
            'rejectionReasons':dict(Counter(m.get('reason') for r in reports for m in r['results'] if not m['accepted'])),
            'timingsMs':{**{key:stats([a[key] for a in attempts]) for key in (
                'anchorAcquisitionMs','requestToConsumptionMs','requestToCoreEndMs','operationElapsedMs')},
                'requestToTerminalReturnMs':stats(wall_times)},
            'requests':attempts,'motorCommandsSent':0,'trainingEligible':False,'controlEligible':False}



def stop_batch(report):
    error = report.get('error') or ''
    # Only known observation rejections may start the next independent request.
    allowed = ('anchor_wait_exhausted', 'anchor_expired', 'anchor_not_live_detection',
               'anchor_texture_insufficient', 'appearance_mismatch_or_target_lost',
               'ambiguous_match', 'scale_change_requires_new_anchor', 'translation_out_of_range',
               'capture_budget_exhausted', 'operation_budget_exceeded')
    return (not report['outputLogUnchanged'] or report['status']=='interrupted' or
            bool(report['integrityErrors']) or
            any(r['storageStatus']!='completed' for r in report.get('consumptions',[])) or
            (report['status']!='completed' and not any(error.endswith(reason) for reason in allowed)))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--audit-only',action='store_true')
    p.add_argument('--controller-url'); p.add_argument('--camera-url')
    p.add_argument('--perception-url',default='http://127.0.0.1:8766')
    args=p.parse_args(argv)
    if args.audit_only:
        batch=json.loads((args.output_dir/'batch.json').read_text())
        reports=[audit(args.output_dir/f'{i+1:02d}') for i in range(batch['completedRequests'])]
        result=summarize(reports,batch['terminalReturnMs'])
    else:
        if not args.controller_url or not args.camera_url: p.error('device URLs required')
        args.output_dir.mkdir(parents=True,exist_ok=False)
        (args.output_dir/'policy.json').write_text(json.dumps(POLICY,sort_keys=True,indent=2)+'\n')
        reports,wall_times=[],[]
        def interrupted(*_): raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM,interrupted)
        for index in range(20):
            transport=ReadOnlyHTTP(controller_url=args.controller_url,camera_url=args.camera_url,
                perception_url=args.perception_url,connection_timing=True)
            root=args.output_dir/f'{index+1:02d}'
            start=time.monotonic()
            report=run_fast_probe(ArtifactStore(root),transport,request_id=f'request-{index+1:02d}',
                fresh_capture=True,capture_timing=True,shadow_consumption=True,
                fresh_age_mode='send_start',causal_anchor=True,max_persistence_ms=250)
            elapsed=(time.monotonic()-start)*1000
            reports.append(report); wall_times.append(elapsed)
            audit_error = None
            try:
                audit(root)
            except Exception as exc:
                audit_error = f'{type(exc).__name__}: {exc}'
                report['observationAttempt'].update(status='failed', reason='audit_failed')
            result=summarize(reports,wall_times)
            result['terminalReturnMs']=wall_times
            result['auditError']=audit_error
            (args.output_dir/'batch.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps({'request':index+1,'result':report['observationAttempt'],'terminalMs':elapsed}),flush=True)
            if audit_error or stop_batch(report):
                break
        result=summarize(reports,wall_times)
        result['auditError']=audit_error
    print(json.dumps(result,indent=2))
    return 0 if result['gatePassed'] else 2


if __name__=='__main__': raise SystemExit(main())
