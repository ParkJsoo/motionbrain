"""Pi GET-only normal/intentional-anchor-expiry experiment. Never actuates.

One normal 12-frame run, then one expiry run requesting 3 frames. Unexpected
failures stop the entire sequence, without retry. Services/firmware unchanged.
"""
import hashlib
import json
from pathlib import Path
import sys
import time


def snapshot(http):
    def get(device,path):
        receipt,body=http.get(device,path)
        assert receipt['status']==200
        return json.loads(body)
    c=get('controller','/status'); cam=get('camera','/status')
    assert c['state']=='IDLE' and c['motorEnabled'] is False
    assert all(m['speed']==0 and not m['enabled'] for m in c['motors'].values())
    return dict(controller={k:c[k] for k in ('state','motorEnabled','motors','lastCommand')},
                camera={k:cam[k] for k in ('cameraBootId','captureFailures','cameraRecoveries')},
                outputs=get('controller','/action_outputs'))


class ExpiryInjection:
    def __init__(self,http,expire,clock=time):
        self.original=http.get
        self.expire,self.clock=expire,clock
        self.checkpoint_send=None
        self.status_calls=0
        self.captures=0
        self.injection=None

    def get(self,device,path,query=None):
        if device=='controller' and path=='/status':
            self.status_calls+=1
            if self.expire and self.status_calls==2:
                assert self.checkpoint_send is not None
                started=self.clock.monotonic()
                wait=max(0,self.checkpoint_send+15.05-started)
                self.clock.sleep(wait)
                self.injection=dict(startedMonotonic=started,requestedSleepSeconds=wait,
                    completedMonotonic=self.clock.monotonic(),checkpointSendMonotonic=self.checkpoint_send)
        result=self.original(device,path,query)
        if device=='camera' and path=='/capture':
            self.captures+=1
            if self.captures==1:
                self.checkpoint_send=result[0].get('connectionTiming',{}).get('requestSendStartedMonotonic')
        return result


def collect(root):
    sys.path.insert(0,str(root))
    from learning.artifacts import ArtifactStore
    from learning.device_adapter import ReadOnlyHTTP
    from learning.fast_probe import run_fast_probe
    def http():
        return ReadOnlyHTTP(controller_url='http://192.168.219.113',camera_url='http://192.168.219.116',
                            perception_url='http://127.0.0.1:8766',connection_timing=True)
    def save(name,value): (root/name).write_text(json.dumps(value,indent=2)+'\n')
    observer=http(); before=snapshot(observer); save('before.json',before)
    save('plan.json',dict(cases=['normal','expiry'],requestedSamples=[12,3],sourceAgeLimitMs=100,
        anchorAgeLimitMs=15000,maxPersistenceMs=250,scriptSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    summaries=[]
    for mode,samples in [('normal',12),('expiry',3)]:
        transport=http(); injection=ExpiryInjection(transport,mode=='expiry'); transport.get=injection.get
        try:
            report=run_fast_probe(ArtifactStore(root/'runs'/mode),transport,samples=samples,
                fresh_capture=True,fresh_age_mode='send_start',causal_anchor=True,shadow_consumption=True,
                max_persistence_ms=250,capture_timing=True)
            item=dict(case=mode,injection=injection.injection,cameraGetCount=injection.captures,
                **{k:report[k] for k in ('status','error','completedSamples','acceptedMeasurements',
                    'consumedObservations','consumptionHalted','outputLogUnchanged','motorCommandsSent')})
            summaries.append(item); save('runs.json',summaries); print(json.dumps(item),flush=True)
        finally:
            after=snapshot(observer); save('after-'+mode+'.json',after)
        assert before['outputs']==after['outputs'] and before['camera']==after['camera']
        for k in ('id','type','source','executedAtMs'):
            assert before['controller']['lastCommand'][k]==after['controller']['lastCommand'][k]
        assert report['outputLogUnchanged'] and report['motorCommandsSent']==0
        if mode=='normal':
            if report['status']!='completed' or report['consumptionHalted']:
                save('stop.json',dict(reason=report['error'],skipped=['expiry'])); return False
            assert report['completedSamples']==12 and injection.captures==13
        else:
            assert report['status']=='failed' and report['consumptionHalted']
            assert report['completedSamples']==1 and injection.captures==2
            row=report['consumptions'][0]
            assert not row['consumed'] and row['reason']=='anchor_expired_at_consumption'
            assert row['storageStatus']=='completed' and row['anchorAgeAtConsumptionUpperMs']>15000
    return True


if __name__=='__main__':
    raise SystemExit(0 if collect(Path(sys.argv[1]).resolve()) else 2)
