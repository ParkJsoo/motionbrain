"""Replay both age modes against identical original JPEGs and recorded times.

Arguments: REPO EXPERIMENT [HISTORICAL_SOURCE_COMMIT]
No device requests; replay times are not a processing-speed benchmark.
"""
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys


def replay(repo,experiment,source_commit=None):
    sys.path.insert(0,str(repo))
    from learning.artifacts import verify_artifacts,read_events,read_json_artifact
    from learning.fast_observation import AnchoredRoiMeasurement
    from learning.observation_consumer import ShadowObservationConsumer,ConsumptionStopped
    def load(p): return json.loads(p.read_text())
    runs=[]
    for root in sorted((experiment/'runs').iterdir()):
        session=load(root/'session.json')
        if not session['freshCaptureRequested']: continue
        original=load(root/'run.json')
        assert original['outputLogUnchanged'] and original['motorCommandsSent']==0
        assert not original['trainingEligible'] and not original['controlEligible']
        assert not verify_artifacts(root,original['artifacts'])
        events,errors=read_events(root)
        assert not errors and read_json_artifact(root,events[-1]['payload']['report'])==original
        for name,digest in session['sourceSha256'].items():
            content=(subprocess.check_output(['git','-C',str(repo),'show',source_commit+':'+name])
                     if source_commit else (repo/name).read_bytes())
            assert hashlib.sha256(content).hexdigest()==digest,name
        anchor=load(root/'anchor.json')
        anchor_body=(root/anchor['image']['artifactRef']).read_bytes()
        receipts=[load(p) for p in sorted((root/'raw').glob('*.json'))]
        captures=[r for r in receipts if r.get('device')=='camera' and r.get('path')=='/capture']
        assert len(captures)==len(original['results'])==len(original['consumptions'])
        modes={}
        for mode in ('clock_margin','send_start'):
            class Clock:
                now=0
                def monotonic(self): return self.now
            clock=Clock()
            tracker=AnchoredRoiMeasurement(anchor['observation'],anchor_body,clock=clock,fresh_age_mode=mode)
            consumer=ShadowObservationConsumer(clock,session['hostClockId'],fresh_age_mode=mode,max_persistence_ms=250)
            rows=[]
            for i,(old,r,saved) in enumerate(zip(original['results'],captures,original['consumptions'])):
                body=(root/old['image']['artifactRef']).read_bytes()
                assert hashlib.sha256(body).hexdigest()==r['sha256']==saved['imageSha256']
                assert old['sourceFrameId']==saved['sourceFrameId']
                # Current function reads start, matching-complete and final-complete.
                times=iter([old['measurementStartedMonotonic'],old['measurementCompletedMonotonic'],old['measurementCompletedMonotonic']])
                clock.monotonic=lambda:next(times)
                measured=tracker.measure(body,old['source'],request_started=r['requestStartedMonotonic'],
                    received=r['receivedMonotonic'],host_clock_id=session['hostClockId'],capture_receipt=r)
                for field in ('score','peakMargin','bestScaleProbe'):
                    assert abs(measured[field]-old[field])<1e-5,field
                clock.monotonic=lambda:clock.now
                clock.now=saved['consumedMonotonic']
                row=consumer.consume(measured,old['source'],r,body)
                if session.get('freshAgeMode','clock_margin')==mode:
                    assert measured['accepted']==old['accepted']
                    assert abs(measured['reportedAgePlusElapsedMs']-old['reportedAgePlusElapsedMs'])<1e-6
                    assert row['consumed']==saved['consumed']
                    assert abs(row['sourceAgeAtConsumptionMs']-saved['sourceAgeAtConsumptionMs'])<1e-6
                clock.now=saved['frameEvidenceCompletedMonotonic']
                try: consumer.finish_persistence(saved['sourceFrameId'],session['hostClockId'])
                except ConsumptionStopped:
                    assert i==len(captures)-1 and saved['storageStatus']=='late'
                final=consumer.snapshot()[-1]
                assert final['storageStatus']==saved['storageStatus']
                rows.append(dict(frameId=saved['sourceFrameId'],measurementAccepted=measured['accepted'],
                    consumed=row['consumed'],reason=row['reason'],measurementAgeMs=measured['reportedAgePlusElapsedMs'],
                    consumptionAgeMs=row['sourceAgeAtConsumptionMs'],storageStatus=final['storageStatus'],
                    persistenceElapsedMs=final['persistenceElapsedMs']))
            modes[mode]=rows
        runs.append(dict(runId=root.name,recordedMode=session.get('freshAgeMode','clock_margin'),
                         actualRunStatus=original['status'],artifactCount=len(original['artifacts']),
                         eventCount=len(events),jpegCount=len(list((root/'images').rglob('*.jpg'))),modes=modes))
    summary={}
    for mode in ('clock_margin','send_start'):
        rows=[row for run in runs for row in run['modes'][mode]]
        values=[r['consumptionAgeMs'] for r in rows]
        summary[mode]=dict(samples=len(rows),measurementAccepted=sum(r['measurementAccepted'] for r in rows),
            consumed=sum(r['consumed'] for r in rows),consumptionAgeMs=dict(min=min(values),median=statistics.median(values),max=max(values)))
    for run in runs:
        for old,new in zip(run['modes']['clock_margin'],run['modes']['send_start']):
            assert new['consumptionAgeMs']>=old['consumptionAgeMs']-1e-6
            assert not new['consumed'] or old['consumed']
    return dict(summary=summary,runs=runs,sourceAgeLimitMs=100,maxPersistenceMs=250,
                motorCommandsSent=0,recordedTimeReplay=True,processingSpeedBenchmark=False)


if __name__=='__main__':
    print(json.dumps(replay(Path(sys.argv[1]),Path(sys.argv[2]),sys.argv[3] if len(sys.argv)>3 else None),indent=2))
