"""Verify the preserved, rejected fixed-wait experiment against its archived source."""
import hashlib
import json
from pathlib import Path
import sys


def verify(repo,root,archived_source):
    sys.path.insert(0,str(repo))
    from learning.artifacts import read_events,read_json_artifact,verify_artifacts
    from learning.anchor_age import causal_anchor_basis
    def load(p): return json.loads(p.read_text())
    run=root/'runs/normal'; report=load(run/'run.json'); session=load(run/'session.json')
    events,errors=read_events(run)
    assert not errors and read_json_artifact(run,events[-1]['payload']['report'])==report
    assert not verify_artifacts(run,report['artifacts'])
    for name,digest in session['sourceSha256'].items():
        assert hashlib.sha256((archived_source/name).read_bytes()).hexdigest()==digest,name
    before=load(root/'before.json'); after=load(root/'after-normal.json')
    assert before['outputs']==after['outputs'] and before['camera']==after['camera']
    assert after['controller']['state']=='IDLE' and after['controller']['motorEnabled'] is False
    assert all(m['speed']==0 and not m['enabled'] for m in after['controller']['motors'].values())
    for k in ('id','type','source','executedAtMs'):
        assert before['controller']['lastCommand'][k]==after['controller']['lastCommand'][k]
    assert report['status']=='failed' and 'anchor_not_after_checkpoint' in report['error']
    assert report['completedSamples']==report['consumedObservations']==report['motorCommandsSent']==0
    assert report['outputLogUnchanged'] and not report['trainingEligible'] and not report['controlEligible']
    captures=[load(p) for p in (run/'raw').glob('*.json') if load(p)['path']=='/capture']
    assert len(captures)==1 and not any(e['kind']=='capture_requested' for e in events)
    assert len(load(root/'runs.json'))==1 and load(root/'stop.json')['skipped']==['expiry']
    e=load(run/'anchor_evidence.json'); cp=load(run/'anchor_checkpoint.json'); anchor=load(run/'anchor.json')
    assert e['checkpointReceipt']==captures[0]==cp['receipt']
    assert e['anchorObservation']==anchor['observation']
    for a,digest in ((cp['image'],cp['receipt']['sha256']),(anchor['image'],e['anchorObservation']['imageSha256'])):
        assert hashlib.sha256((run/a['artifactRef']).read_bytes()).hexdigest()==digest
    assert e['anchorObservation']['cameraBootId']==cp['source']['cameraBootId']
    assert e['anchorObservation']['capturedUs']<cp['source']['capturedUs']
    try: causal_anchor_basis(e,session['hostClockId'])
    except ValueError as error: assert str(error)=='anchor_not_after_checkpoint'
    else: raise AssertionError('old anchor accepted')
    return dict(status=report['status'],error=report['error'],currentCaptures=0,checkpointCaptures=1,
        consumedObservations=0,motorCommandsSent=0,expiryRunSkipped=True,artifactCount=len(report['artifacts']),
        eventCount=len(events),jpegCount=len(list((run/'images').rglob('*.jpg'))),
        checkpointCapturedUs=cp['source']['capturedUs'],anchorCapturedUs=e['anchorObservation']['capturedUs'],
        sourceSha256=session['sourceSha256'],fixedWaitSeconds=session['anchorCheckpointWaitSeconds'])


if __name__=='__main__': print(json.dumps(verify(*map(Path,sys.argv[1:4])),indent=2))
