"""91 GET-only clock samples over >=180s in an isolated Pi checkout. No retries."""
import hashlib
import json
from pathlib import Path
import sys
import time

from clock_analysis import REPO, sample_from_receipt, checked
from learning.artifacts import ArtifactStore, verify_artifacts
from learning.device_adapter import ReadOnlyHTTP
from learning.device_probe import _idle


def snapshot(http):
    def get(device,path):
        r,b = http.get(device,path)
        assert r['status']==200
        return json.loads(b)
    c,cam = get('controller','/status'),get('camera','/status')
    assert _idle(c) and cam['freshCaptureSupported'] is True
    return dict(controller={k:c[k] for k in ('state','motorEnabled','motors')},
        lastCommand={k:c['lastCommand'][k] for k in ('id','type','source','executedAtMs')},
        camera={k:cam[k] for k in ('cameraBootId','captureFailures','cameraRecoveries','freshCaptureSupported')},
        outputs=get('controller','/action_outputs'))


def collect(root):
    store = ArtifactStore(root/'clock-run')
    refs=[]
    def save(path,obj):
        ref=store.write_json(path,obj); refs.append(ref); return ref
    http=ReadOnlyHTTP(controller_url='http://192.168.219.113',camera_url='http://192.168.219.116',
                      perception_url='http://127.0.0.1:8766',connection_timing=True)
    sources=['learning/device_adapter.py','learning/http_timing.py','learning/fresh_capture.py',
             'learning/artifacts.py','ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/observation.py',
             'collect.py','clock_analysis.py']
    save('session.json',dict(hostClockId=http.host_clock_id,requestedSamples=91,intervalSeconds=2,
        sourceAgeLimitMs=100,rateAssumptionUsedForSelection=False,trainingEligible=False,controlEligible=False,
        sourceSha256={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sources}))
    store.append('started','clock',{})
    samples=[]; before=after=None; status='completed'; error=None
    try:
        before=snapshot(http); save('before.json',before)
        for i in range(91):
            # No burst catch-up. At least two seconds between request starts.
            if i:
                time.sleep(max(0,previous_start+2-time.monotonic()))
            if i % 15 == 0:
                state=snapshot(http); save(f'status/{i:04d}.json',state)
                assert state==before, 'device_state_changed'
            previous_start=time.monotonic()
            r,body=http.get('camera','/capture',{'fresh':'1'} if i%2 else None)
            save(f'receipts/{i:04d}.json',r)  # Preserve even a later rejected receipt.
            if body and r.get('status')==200:
                refs.append(store.save_image(str(i),'source',body,r['sha256']))
            sample=sample_from_receipt(r)  # No +/-1000ppm acceptance filter.
            assert sample['cameraBootId']==before['camera']['cameraBootId']
            if samples: checked([samples[-1],sample])
            samples.append(sample); save(f'samples/{i:04d}.json',sample)
            store.append('sampled',str(i),{})
            if i%15==0: print(json.dumps({'samples':i+1,'requested':91}),flush=True)
    except KeyboardInterrupt:
        status,error='interrupted','KeyboardInterrupt'
    except Exception as exc:
        status,error='failed',f'{type(exc).__name__}: {exc}'
    finally:
        try:
            after=snapshot(http); save('after.json',after)
            if before!=after: status,error='failed','device_state_changed_at_end'
        except Exception as exc:
            status,error='failed',f'final_status: {type(exc).__name__}: {exc}'
    integrity=verify_artifacts(store.root,refs)
    if integrity: status,error='failed','artifact_integrity'
    report=dict(status=status,error=error,completedSamples=len(samples),samples=samples,artifacts=refs,
        outputLogUnchanged=before is not None and after==before,integrityErrors=integrity,
        motorCommandsSent=0,controlEligible=False,trainingEligible=False)
    ref=store.write_json('run.json',report); store.append('finished','clock',{'report':ref})
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','artifacts')}),flush=True)
    return status=='completed'


if __name__=='__main__':
    root=Path(sys.argv[1]).resolve()
    assert root==REPO
    raise SystemExit(0 if collect(root) else 2)
