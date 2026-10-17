"""One bounded USB/camera session with producer-owned clock identities.

Only six S3 sensor requests and read-only HTTP are available. Clock linkage is
post-hoc presentation/poll-marker evidence, never optical or training evidence.
"""
import base64
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

from learning.artifacts import (ArtifactStore, StorageError, describe_artifact,
                               read_events, read_json_artifact, verify_artifacts)
from learning.controller_latency import direct_get, deadline
from learning.mac_camera_probe import create_plan, evaluate_capture
from learning.mac_clock_bridge import MacHostClock, finite
from learning.mac_controller_probe import ENDPOINTS, _check, ProbeRejected, stop_camera
from learning.serial_m4_probe import (open_usb, read_sample, validate_receipt,
                                    initialization_boot_changed, unexpected_tail, trace_summary, wire_request)
from tools.diagnose_controller_http import validate_target

SCHEMA='motionbrain.usb_camera_probe.v1'
ROOT=Path(__file__).resolve().parents[1]


def policy_for(initialize_port):
    if type(initialize_port) is not bool:raise ValueError('initialization_choice_invalid')
    return dict(samples=6,frames=21,httpRequests=12 if initialize_port else 8,
                diagnosticRequests=2,sessionSeconds=80 if initialize_port else 60,
                cameraProcessSeconds=47,clockBridgeSeconds=50,cleanupGraceSeconds=4,
                requestSeconds=.8,intervalSeconds=.025,drainSeconds=20 if initialize_port else .2,
                startupMaxRawBytes=65536 if initialize_port else 4096,maxRawBytes=4096,
                maxTraceReadCalls=256,maxPersistenceMs=250,maxSourceAgeMs=100,
                maxAnchorAgeMs=15000,maxM4AgeMs=20,retries=0,motorRequests=0)


def launch_camera(helper, root, device_id, view_id, session_id):
    with (root/'camera-wrapper.stdout').open('xb') as out, (root/'camera-wrapper.stderr').open('xb') as err:
        return subprocess.Popen([sys.executable,str(ROOT/'tools/probe_mac_camera.py'),'capture',
                                 '--helper',str(helper),'--device-id',device_id,'--view-id',view_id,
                                 '--session-id',session_id,'--output',str(root/'camera')],
                                stdout=out,stderr=err,start_new_session=True)


def collect(output, *, controller_url, port, usb_serial, helper, device_id, view_id,
            initialize_port=False, clock=time, clock_probe=None, http=direct_get,
            open_port=open_usb, launch=launch_camera, stop=stop_camera):
    from learning.m4_stage_trace import check_diagnostics, link_stages
    from learning.usb_camera_clock_link import link_saved_session
    policy=policy_for(initialize_port)
    controller_url=validate_target(controller_url)
    session_id=uuid.uuid4().hex;host_id=uuid.uuid4().hex
    create_plan(device_id,view_id,session_id)
    helper=Path(helper).resolve(strict=True)
    if not helper.is_file() or not os.access(helper,os.X_OK):raise ValueError('executable_camera_helper_required')
    clock_probe=clock_probe if clock_probe is not None else MacHostClock()
    store=ArtifactStore(Path(output));refs=[];http_rows=[];serial_rows=[];diagnostics=[];samples=[]
    device=camera=None;start=clock.monotonic();nonces=[uuid.uuid4().hex[:8] for _ in range(6)]
    if len(set(nonces))!=6:raise ValueError('nonce_collision')
    result=dict(schemaVersion=SCHEMA,status='rejected',failure=None,trainingEligible=False,
                controlEligible=False,physicalSampleAlignmentVerified=False,opticalExposureVerified=False,
                automaticRetry=False,motorRequests=0,samples=samples,startedMonotonic=start,
                controllerBootChangedDuringInitialization=False,cleanupErrors=[])
    def save(path,value):
        begun=clock.monotonic();ref=store.write_json(path,value);refs.append(ref)
        if clock.monotonic()-begun>.25:raise StorageError('persistence_exceeds_250ms')
        return ref
    def budget():
        if clock.monotonic()-start>policy['sessionSeconds']:raise ValueError('session_deadline')
    def camera_budget():
        budget()
        if clock.monotonic()-result['cameraProcessStartedMonotonic']>47:raise ValueError('camera_deadline')
    def get(endpoint,phase):
        budget()
        if len(http_rows)>=policy['httpRequests']:raise ValueError('http_request_budget')
        row=http(controller_url,endpoint,clock=clock)
        row.update(hostClockId=host_id,collectionSessionId=session_id,phase=phase)
        http_rows.append(row);save(f'http/{len(http_rows):02d}.json',row)
        if row.get('cancelled') is True:raise KeyboardInterrupt()
        return _check(row,endpoint)
    def diagnostic(phase):
        budget()
        if len(diagnostics)>=2:raise ValueError('diagnostic_request_budget')
        row=http(controller_url,'m4_diagnostics',clock=clock)
        row.update(hostClockId=host_id,collectionSessionId=session_id,phase=phase)
        diagnostics.append(row);save(f'firmware/{phase}.json',row)
        if row.get('cancelled') is True:raise KeyboardInterrupt()
        return check_diagnostics(row)
    def baseline_check(values):
        if values['actions']['enabled'] or values['actions']['bootId']!=values['action_outputs']['bootId']:
            raise ValueError('baseline_boot_or_action_feature_invalid')
    def drain(name,seconds,limit):
        raw=bytearray();begun=clock.monotonic()
        try:
            with deadline(seconds+.1):
                while clock.monotonic()-begun<seconds:
                    raw.extend(device.read(1))
                    if len(raw)>limit:raise ValueError('serial_drain_raw_limit')
        finally:
            save(f'serial/{name}.json',dict(rawBase64=base64.b64encode(raw).decode(),
                                          startedMonotonic=begun,endedMonotonic=clock.monotonic()))
        return raw
    save('plan.json',dict(schemaVersion=SCHEMA,policy=policy,initializePort=initialize_port,
                          collectionSessionId=session_id,hostClockId=host_id,controllerUrl=controller_url,
                          port=port,usbSerial=usb_serial,deviceId=device_id,viewId=view_id,nonces=nonces))
    try:
        if initialize_port:
            pre={e:get(e,'pre_open') for e in ENDPOINTS};baseline_check(pre)
            device=open_port(port,usb_serial)
            startup=drain('startup',20,65536)
            if startup.count(b'=== MotionBrain System Boot ===')>1:raise ValueError('multiple_boots_during_initialization')
        baseline={e:get(e,'baseline') for e in ENDPOINTS};baseline_check(baseline)
        boot=baseline['actions']['bootId']
        if initialize_port:
            result['controllerBootChangedDuringInitialization']=initialization_boot_changed(pre['action_outputs'],baseline['action_outputs'])
        else:
            device=open_port(port,usb_serial);drain('startup',.2,4096)
        stage_before=diagnostic('before')
        if stage_before['bootId']!=boot or stage_before['active']:raise ValueError('diagnostic_boot_or_active_mismatch')
        before=clock_probe.sample(host_id);before_ref=save('clock/before.json',before)
        host_boot=before['hostBootId']
        result['cameraProcessStartedMonotonic']=clock.monotonic()
        camera=launch(helper,store.root,device_id,view_id,session_id)
        while not (store.root/'camera/frame-0000.json').is_file():
            camera_budget()
            if camera.poll() is not None:raise ValueError('camera_exited_before_first_frame')
            clock.sleep(.02)
        result['firstFrameFileObservedMonotonic']=clock.monotonic()
        serial_refs=[]
        for nonce in nonces:
            camera_budget()
            if camera.poll() is not None:raise ValueError('camera_ended_before_serial_samples')
            row=read_sample(device,nonce,boot,host_id,clock=clock,compact_sample=True,transport_trace=True)
            row.update(collectionSessionId=session_id,hostBootId=host_boot)
            serial_rows.append(row);serial_refs.append(save(f'serial/{len(serial_rows):02d}.json',row))
            if row.get('cancelled') is True:raise KeyboardInterrupt()
            sample=validate_receipt(row,nonce,boot,compact_sample=True,transport_trace=True)
            if samples:
                for key in ('polledMs','reportedMs'):
                    if not 0<((sample[key]-samples[-1][key])&0xffffffff)<1000:raise ValueError('serial_sample_not_new')
            samples.append(sample)
            if len(samples)<6:clock.sleep(.025)
        tail=drain('tail',.2,4096)
        if unexpected_tail(tail):raise ValueError('unexpected_serial_tail_reply')
        device.close();device=None
        camera_budget()
        remaining=min(start+policy['sessionSeconds'],result['cameraProcessStartedMonotonic']+47)-clock.monotonic()
        if remaining<=0:raise ValueError('camera_deadline')
        camera.wait(timeout=remaining)
        result['cameraProcessEndedMonotonic']=clock.monotonic()
        after_ref=save('clock/after.json',clock_probe.sample(host_id))
        if camera.returncode!=0:raise ValueError('camera_nonzero_exit')
        stage_after=diagnostic('after')
        result['firmwareStages']=link_stages(stage_before,stage_after,samples)
        final={e:get(e,'final') for e in ENDPOINTS}
        if final['actions']['bootId']!=boot or final['action_outputs']!=baseline['action_outputs']:
            raise ValueError('controller_boot_or_outputs_changed')
        budget();result.update(outputHistoryUnchanged=True,observationEndedMonotonic=clock.monotonic())
        # Large pixel audits are offline work, outside the acquisition budget.
        result['camera']=evaluate_capture(store.root/'camera')
        if not result['camera']['captureProcessingPassed']:raise ValueError('camera_processing_not_passed')
        link=link_saved_session(store.root,before_ref,after_ref,serial_refs,session_id=session_id,
                               host_clock_id=host_id,controller_boot_id=boot,nonces=nonces,
                               camera_process_window=[result['cameraProcessStartedMonotonic'],result['cameraProcessEndedMonotonic']],
                               transport_trace=True)
        result['clockLink']=save('clock/link.json',link)
        result['status']='recorded'
    except KeyboardInterrupt:result.update(failure='cancelled',cancelled=True)
    except (OSError,ValueError,StorageError,ProbeRejected,subprocess.SubprocessError) as exc:
        result['failure']=str(exc)
    finally:
        result.setdefault('observationEndedMonotonic',clock.monotonic())
        for name,resource,cleanup in (('usb',device,lambda p:p.close()),('camera',camera,stop)):
            if resource is not None:
                try:cleanup(resource)
                except (Exception,KeyboardInterrupt) as exc:
                    result['cleanupErrors'].append(dict(resource=name,error=type(exc).__name__))
                    result.update(status='rejected',failure='resource_cleanup_failed')
        result['cleanupEndedMonotonic']=clock.monotonic()
    for directory in ('camera',):
        if (store.root/directory).is_dir():
            for path in sorted((store.root/directory).iterdir()):refs.append(describe_artifact(store.root,f'{directory}/{path.name}'))
    for name in ('camera-wrapper.stdout','camera-wrapper.stderr'):
        if (store.root/name).exists():refs.append(describe_artifact(store.root,name))
    result.update(httpCount=len(http_rows),serialCount=len(serial_rows),diagnosticCount=len(diagnostics),
                  within20ms=sum(s['within20ms'] for s in samples),artifacts=list(refs))
    ref=save('result.json',result);store.append('usb_camera_probe_sealed','observation',{'summary':ref})
    return result


def audit(root):
    """Recompute all successful links; retain partial failed attempts as evidence."""
    from learning.m4_stage_trace import check_diagnostics, link_stages
    from learning.usb_camera_clock_link import link_saved_session
    root=Path(root);events,issues=read_events(root)
    try:
        if len(events)!=1 or events[0]['kind']!='usb_camera_probe_sealed':raise ValueError('combined_seal_missing')
        r=read_json_artifact(root,events[0]['payload']['summary'])
        issues+=verify_artifacts(root,r['artifacts'])
        refs={ref['artifactRef']:ref for ref in r['artifacts']}
        if len(refs)!=len(r['artifacts']):raise ValueError('duplicate_artifacts')
        p=read_json_artifact(root,refs['plan.json']);policy=policy_for(p['initializePort'])
        if (p['schemaVersion']!=SCHEMA or p['policy']!=policy or r['schemaVersion']!=SCHEMA
                or any(r.get(k) is not False for k in ('trainingEligible','controlEligible','automaticRetry',
                    'physicalSampleAlignmentVerified','opticalExposureVerified')) or r.get('motorRequests')!=0
                or r['status'] not in ('recorded','rejected')):raise ValueError('combined_contract_invalid')
        if any(not isinstance(p.get(k),str) or not p[k] for k in ('collectionSessionId','hostClockId')):
            raise ValueError('combined_identity_missing')
        nonces=p['nonces']
        if (not isinstance(nonces,list) or len(nonces)!=6 or len(set(nonces))!=6
                or any(not isinstance(n,str) or not re.fullmatch('[0-9a-f]{8}',n) for n in nonces)):
            raise ValueError('combined_nonce_set_invalid')
        rows={}
        for prefix,count,maximum in (('http',r['httpCount'],policy['httpRequests']),
                                     ('serial',r['serialCount'],6)):
            if type(count) is not int or not 0<=count<=maximum:raise ValueError('combined_attempt_count_invalid')
            paths=[f'{prefix}/{i:02d}.json' for i in range(1,count+1)]
            if {path for path in refs if re.fullmatch(prefix+r'/\d+\.json',path)}!=set(paths):
                raise ValueError('combined_attempt_set_invalid')
            rows[prefix]=[read_json_artifact(root,refs[path]) for path in paths]
            previous=r['startedMonotonic']
            for row in rows[prefix]:
                if (row.get('hostClockId')!=p['hostClockId'] or row.get('collectionSessionId')!=p['collectionSessionId']
                        or not all(finite(row.get(k)) for k in ('requestStartedMonotonic','receivedMonotonic'))
                        or not previous<=row['requestStartedMonotonic']<=row['receivedMonotonic']):
                    raise ValueError('combined_receipt_identity_or_order')
                previous=row['receivedMonotonic']
        for row,nonce in zip(rows['serial'],nonces):
            if (row.get('requestBase64')!=base64.b64encode(wire_request(nonce,compact_sample=True)).decode()
                    or row.get('schemaVersion')!='motionbrain.serial_m4_receipt.v1'
                    or row.get('clockDomain')!='python_monotonic' or type(row.get('transportCompleted')) is not bool):
                raise ValueError('combined_serial_request_invalid')
            raw=base64.b64decode(row['rawBase64'],validate=True)
            if len(raw)>4097 or hashlib.sha256(raw).hexdigest()!=row['rawSha256']:raise ValueError('combined_serial_raw_invalid')
            trace_summary(row,True)
        dc=r['diagnosticCount']
        if type(dc) is not int or not 0<=dc<=2:raise ValueError('combined_diagnostic_count_invalid')
        names=['before','after'][:dc]
        if {path for path in refs if path.startswith('firmware/')}!={f'firmware/{n}.json' for n in names}:
            raise ValueError('combined_diagnostic_set_invalid')
        diagnostic_rows=[read_json_artifact(root,refs[f'firmware/{n}.json']) for n in names]
        for row,phase in zip(diagnostic_rows,names):
            if (row.get('phase')!=phase or row.get('hostClockId')!=p['hostClockId']
                    or row.get('collectionSessionId')!=p['collectionSessionId']):raise ValueError('combined_diagnostic_identity')
        if r['status']=='recorded':
            if r['failure'] is not None or r['cleanupErrors'] or r['outputHistoryUnchanged'] is not True:
                raise ValueError('combined_success_contract_invalid')
            if (r['httpCount']!=policy['httpRequests'] or r['serialCount']!=6 or dc!=2):
                raise ValueError('combined_success_count_invalid')
            expected={'plan.json','serial/startup.json','serial/tail.json','clock/before.json',
                      'clock/after.json','clock/link.json','firmware/before.json','firmware/after.json',
                      'camera-wrapper.stdout','camera-wrapper.stderr'}
            expected|={f'http/{i:02d}.json' for i in range(1,policy['httpRequests']+1)}
            expected|={f'serial/{i:02d}.json' for i in range(1,7)}
            expected|={'camera/'+n for n in ('plan.json','session.json','terminal.json','execution.json',
                                            'evaluation.json','helper.stdout','helper.stderr')}
            expected|={f'camera/frame-{i:04d}.{suffix}' for i in range(21) for suffix in ('json','bgra')}
            if set(refs)!=expected:raise ValueError('combined_artifact_set_invalid')
            phases=(['pre_open']*4 if p['initializePort'] else [])+['baseline']*4+['final']*4
            for i,(row,phase) in enumerate(zip(rows['http'],phases)):
                _check(row,ENDPOINTS[i%4])
                if row['phase']!=phase:raise ValueError('combined_http_phase_invalid')
            pre=rows['http'][:4] if p['initializePort'] else []
            baseline=rows['http'][-8:-4];final=rows['http'][-4:]
            boot=baseline[2]['response']['bootId']
            if (baseline[2]['response']['enabled'] or boot!=baseline[3]['response']['bootId']
                    or final[2]['response']['bootId']!=boot or baseline[3]['response']!=final[3]['response']):
                raise ValueError('combined_boot_or_outputs_changed')
            if pre and (pre[2]['response']['enabled'] or pre[2]['response']['bootId']!=pre[3]['response']['bootId']):
                raise ValueError('combined_preopen_invalid')
            changed=initialization_boot_changed(pre[3]['response'],baseline[3]['response']) if pre else False
            if r['controllerBootChangedDuringInitialization'] is not changed:raise ValueError('combined_reboot_summary_invalid')
            a,b=r['cameraProcessStartedMonotonic'],r['cameraProcessEndedMonotonic']
            first=r['firstFrameFileObservedMonotonic']
            end=r['observationEndedMonotonic'];start=r['startedMonotonic']
            if (not all(finite(v) for v in (a,b,first,end,start,r['cleanupEndedMonotonic']))
                    or not 0<=start<=a<=first<=rows['serial'][0]['requestStartedMonotonic']
                    or not rows['serial'][-1]['receivedMonotonic']<=b<=end<=start+policy['sessionSeconds']
                    or b-a>47 or r['cleanupEndedMonotonic']<end):raise ValueError('combined_observation_window_invalid')
            ordered=rows['http'][:-4]+[diagnostic_rows[0]]+rows['serial']+[diagnostic_rows[1]]+final
            previous=start
            for row in ordered:
                if not previous<=row['requestStartedMonotonic']<=row['receivedMonotonic']<=end:
                    raise ValueError('combined_acquisition_order_invalid')
                previous=row['receivedMonotonic']
            if not diagnostic_rows[0]['receivedMonotonic']<=a or not b<=diagnostic_rows[1]['requestStartedMonotonic']:
                raise ValueError('combined_camera_diagnostic_order')
            for name,lower,upper,seconds,limit in (
                    ('startup',pre[-1]['receivedMonotonic'] if pre else baseline[-1]['receivedMonotonic'],
                     baseline[0]['requestStartedMonotonic'] if pre else diagnostic_rows[0]['requestStartedMonotonic'],
                     policy['drainSeconds'],policy['startupMaxRawBytes']),
                    ('tail',rows['serial'][-1]['receivedMonotonic'],b,.2,4096)):
                drain=read_json_artifact(root,refs[f'serial/{name}.json']);raw=base64.b64decode(drain['rawBase64'],validate=True)
                x,y=drain['startedMonotonic'],drain['endedMonotonic']
                if (not all(finite(v) for v in (x,y)) or not lower<=x<=y<=upper
                        or not seconds<=y-x<=seconds+.1 or len(raw)>limit):raise ValueError('combined_drain_invalid')
                if name=='tail' and unexpected_tail(raw):raise ValueError('combined_tail_invalid')
                if name=='startup' and pre and raw.count(b'=== MotionBrain System Boot ===')>1:
                    raise ValueError('combined_multiple_initialization_boots')
            camera_plan=read_json_artifact(root,refs['camera/plan.json'])
            if camera_plan['deviceUniqueId']!=p['deviceId'] or camera_plan['viewConfigurationId']!=p['viewId']:
                raise ValueError('combined_camera_identity_invalid')
            linked=link_saved_session(root,refs['clock/before.json'],refs['clock/after.json'],
                [refs[f'serial/{i:02d}.json'] for i in range(1,7)],session_id=p['collectionSessionId'],
                host_clock_id=p['hostClockId'],controller_boot_id=boot,nonces=nonces,
                camera_process_window=[a,b],transport_trace=True)
            if r['clockLink']!=refs['clock/link.json'] or read_json_artifact(root,r['clockLink'])!=linked:
                raise ValueError('combined_clock_link_recomputation_mismatch')
            samples=[item['sample'] for item in linked['sensors']]
            stages=link_stages(*(check_diagnostics(row) for row in diagnostic_rows),samples)
            if (samples!=r['samples'] or stages!=r['firmwareStages'] or r['within20ms']!=sum(s['within20ms'] for s in samples)
                    or r['camera']!=evaluate_capture(root/'camera')):raise ValueError('combined_summary_recomputation_mismatch')
    except (OSError,ValueError,StorageError,ProbeRejected,KeyError,TypeError,AttributeError,OverflowError) as exc:
        issues.append(str(exc))
    return dict(ok=not issues,issues=issues,trainingEligible=False,controlEligible=False)
