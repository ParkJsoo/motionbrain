"""Offline raw artifact/identity/causal-anchor and consumption replay verification."""
import hashlib
import json
from pathlib import Path
import statistics
import sys
from unittest.mock import Mock


def verify(repo,root):
    sys.path.insert(0,str(repo))
    from learning.artifacts import read_events,read_json_artifact,verify_artifacts
    from learning.anchor_age import causal_anchor_basis,semantic_anchor
    from learning.fast_observation import AnchoredRoiMeasurement
    from learning.observation_consumer import ShadowObservationConsumer,ConsumptionStopped
    from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers
    def load(p):return json.loads(p.read_text())
    def stats(v): return dict(min=min(v),median=statistics.median(v),max=max(v)) if v else None
    plan=load(root/'plan.json'); entries=load(root/'runs.json'); before=load(root/'before.json')
    assert plan['sourceAgeLimitMs']==100 and plan['anchorAgeLimitMs']==15000 and plan['maxPersistenceMs']==250
    assert plan['scriptSha256']==hashlib.sha256(Path(__file__).with_name('collect.py').read_bytes()).hexdigest()
    assert 1<=len(entries)<=2
    output=dict(sourceAgeLimitMs=100,anchorAgeLimitMs=15000,maxPersistenceMs=250,runs=[],
        motorCommandsSent=0,physicalCalibration=False,controlEligible=False,trainingEligible=False)
    for index,entry in enumerate(entries):
        mode=entry['case']; assert mode==plan['cases'][index]
        run=root/'runs'/mode; report=load(run/'run.json'); session=load(run/'session.json')
        after=load(root/f'after-{mode}.json')
        assert after['camera']==before['camera'] and after['outputs']==before['outputs']
        assert after['controller']['state']=='IDLE' and after['controller']['motorEnabled'] is False
        assert all(m['speed']==0 and not m['enabled'] for m in after['controller']['motors'].values())
        for k in ('id','type','source','executedAtMs'):
            assert after['controller']['lastCommand'][k]==before['controller']['lastCommand'][k]
        assert not report['integrityErrors'] and not verify_artifacts(run,report['artifacts'])
        events,errors=read_events(run)
        assert not errors and events[-1]['kind']=='finished'
        assert read_json_artifact(run,events[-1]['payload']['report'])==report
        for name,digest in session['sourceSha256'].items():
            assert hashlib.sha256((repo/name).read_bytes()).hexdigest()==digest,name
        assert session['causalAnchorEnabled'] and session['freshAgeMode']=='send_start'
        assert session['shadowConsumptionEnabled'] and session['maxPersistenceMs']==250
        assert session['policy']['maxSourceAgeMs']==100 and session['policy']['maxAnchorAgeMs']==15000
        assert report['outputLogUnchanged'] and report['motorCommandsSent']==0
        assert not report['controlEligible'] and not report['trainingEligible']
        for k in ('status','error','completedSamples','acceptedMeasurements','consumedObservations','consumptionHalted'):
            assert report[k]==entry[k]
        evidence=load(run/'anchor_evidence.json'); anchor=load(run/'anchor.json'); cp=load(run/'anchor_checkpoint.json')
        assert evidence['anchorObservation']==anchor['observation']
        assert evidence['anchorRetrievedMonotonic']==anchor['retrievedMonotonic']
        assert evidence['hostClockId']==session['hostClockId']==anchor['retrievalHostClockId']
        assert evidence['checkpointReceipt']==cp['receipt'] and evidence['checkpointSource']==cp['source']
        anchor_body=(run/anchor['image']['artifactRef']).read_bytes()
        checkpoint_body=(run/cp['image']['artifactRef']).read_bytes()
        assert hashlib.sha256(anchor_body).hexdigest()==anchor['observation']['imageSha256']
        assert hashlib.sha256(checkpoint_body).hexdigest()==cp['source']['imageSha256']==cp['receipt']['sha256']
        basis=causal_anchor_basis(evidence,session['hostClockId'])
        receipts=[load(p) for p in sorted((run/'raw').glob('*.json'))]
        captures=[r for r in receipts if r['device']=='camera' and r['path']=='/capture']
        assert captures[0]==cp['receipt']
        assert len(captures)==entry['cameraGetCount']==len(report['results'])+1
        assert sum(e['kind']=='capture_requested' for e in events)==len(report['results'])
        polls=[r for r in receipts if r['path']=='/api/detection'][:-1]
        poll_events=[e for e in events if e['kind']=='anchor_wait_polled']
        assert session['anchorWaitMaxPolls']==8 and session['anchorWaitIntervalSeconds']==1
        assert 1<=len(polls)<=8 and len(polls)==len(poll_events)
        for j,(poll,event) in enumerate(zip(polls,poll_events)):
            candidate=poll['json']; is_new=candidate['capturedUs']>cp['source']['capturedUs']
            assert candidate['cameraBootId']==cp['source']['cameraBootId']
            assert candidate['sourceFrameId']==event['payload']['sourceFrameId']
            assert event['payload']['afterCheckpoint']==is_new==(j==len(polls)-1)
            if j: assert poll['requestStartedMonotonic']-polls[j-1]['receivedMonotonic']>=1
        exact=[r for r in receipts if r['path']=='/api/observation']
        images=[r for r in receipts if r['path']=='/api/vision_frame']
        assert len(exact)==len(images)==1
        assert exact[0]['json']['observation']==anchor['observation']
        assert images[0]['query']=={'frameId':anchor['observation']['sourceFrameId']}
        assert images[0]['sha256']==anchor['observation']['imageSha256']
        clock=Mock(); clock.monotonic.return_value=anchor['retrievedMonotonic']
        tracker=AnchoredRoiMeasurement(anchor['observation'],anchor_body,clock=clock,
            fresh_age_mode='send_start',anchor_evidence=evidence)
        consumer=ShadowObservationConsumer(clock,session['hostClockId'],fresh_age_mode='send_start',anchor_evidence=evidence)
        rows=[]
        assert len(report['consumptions'])==len(report['results'])
        for i,(saved,m,r) in enumerate(zip(report['consumptions'],report['results'],captures[1:])):
            source={**parse_capture_headers(r['headers']),'imageSha256':r['sha256']}
            assert source==m['source'] and m['semanticAnchor']==semantic_anchor(anchor['observation'])
            body=(run/m['image']['artifactRef']).read_bytes()
            assert hashlib.sha256(body).hexdigest()==r['sha256']==saved['imageSha256']
            clock.monotonic.side_effect=[m['measurementStartedMonotonic'],m['measurementCompletedMonotonic'],m['measurementCompletedMonotonic']]
            replayed=tracker.measure(body,source,request_started=r['requestStartedMonotonic'],
                received=r['receivedMonotonic'],host_clock_id=session['hostClockId'],capture_receipt=r)
            assert replayed['accepted']==m['accepted'] and replayed.get('reason')==m.get('reason')
            for field in ('score','peakMargin','bestScaleProbe','anchorAgeAtMeasurementUpperMs','reportedAgePlusElapsedMs'):
                if field in m: assert abs(m[field]-replayed[field])<1e-5,field
            clock.monotonic.side_effect=None; clock.monotonic.return_value=saved['consumedMonotonic']
            pending=consumer.consume(m,source,r,body)
            assert pending==load(run/f'consumption/{i:04d}.json')
            clock.monotonic.return_value=saved['frameEvidenceCompletedMonotonic']
            try: consumer.finish_persistence(saved['sourceFrameId'],session['hostClockId'])
            except ConsumptionStopped:
                assert mode=='expiry' and i==len(report['results'])-1
            assert consumer.snapshot()[-1]==saved
            expected=(saved['consumedMonotonic']-cp['receipt']['connectionTiming']['requestSendStartedMonotonic'])*1000
            assert abs(saved['anchorAgeAtConsumptionUpperMs']-expected)<1e-6
            assert saved['anchorAgeClockRateAssumptionUsed'] is False and saved['anchorAgeTimestampMarginUsed'] is False
            assert saved['sourceAgeClockRateAssumptionUsed'] is False and saved['sourceAgeTimestampMarginUsed'] is False
            assert saved['storageStatus']=='completed' and saved['persistenceElapsedMs']<=250
            if saved['consumed']:
                assert expected<=15000 and saved['sourceAgeAtConsumptionMs']<=100 and m['accepted']
            if i+1<len(report['results']):
                assert not saved['haltAfterPersistence']
                assert captures[i+2]['requestStartedMonotonic']>=saved['frameEvidenceCompletedMonotonic']
            rows.append(dict(frameId=saved['sourceFrameId'],measurementAccepted=m['accepted'],
                consumed=saved['consumed'],reason=saved['reason'],measurementReason=m.get('reason'),
                sourceAgeAtConsumptionMs=saved['sourceAgeAtConsumptionMs'],anchorAgeUpperMs=expected,
                persistenceElapsedMs=saved['persistenceElapsedMs']))
        assert report['consumedObservations']==sum(r['consumed'] for r in rows)
        if mode=='normal':
            assert report['status']=='completed' and len(rows)==12 and not consumer.halted
            assert entry['injection'] is None
        else:
            assert report['status']=='failed' and len(rows)==1 and consumer.halted
            assert rows[0]['reason']=='anchor_expired_at_consumption' and not rows[0]['consumed']
            inj=entry['injection']; assert inj['checkpointSendMonotonic']==basis['anchorCaptureLowerBoundMonotonic']
            assert inj['completedMonotonic']>=inj['checkpointSendMonotonic']+15.05
            assert captures[1]['requestStartedMonotonic']>=inj['completedMonotonic']
        output['runs'].append(dict(case=mode,status=report['status'],error=report['error'],
            artifactCount=len(report['artifacts']),eventCount=len(events),jpegCount=len(list((run/'images').rglob('*.jpg'))),
            captures=len(captures),samples=len(rows),consumed=report['consumedObservations'],anchorMetadataPolls=len(polls),
            acquisitionUpperMs=(anchor['retrievedMonotonic']-basis['anchorCaptureLowerBoundMonotonic'])*1000,
            anchorCheckpointCameraOrderOnly=True,
            statistics={k:stats([r[k] for r in rows]) for k in ('sourceAgeAtConsumptionMs','anchorAgeUpperMs','persistenceElapsedMs')},rows=rows))
    return output


if __name__=='__main__':
    print(json.dumps(verify(Path(sys.argv[1]),Path(sys.argv[2])),indent=2))
