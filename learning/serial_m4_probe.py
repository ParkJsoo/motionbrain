"""Finite, explicitly selected USB sensor diagnostic. No legacy command writer.

RS frames cannot dispatch ARM/motor commands. Firmware poll markers, not physical
AS5600 conversion timestamps, are bounded in the collector's monotonic clock.
"""
import base64
import hashlib
import re
import time
import uuid
from pathlib import Path

from learning.artifacts import ArtifactStore, StorageError, read_events, read_json_artifact, verify_artifacts
from learning.controller_latency import direct_get, deadline
from learning.mac_controller_probe import ENDPOINTS, _check, ProbeRejected
from learning.mac_clock_bridge import finite

POLICY=dict(samples=6, httpRequests=8, serialRequestSeconds=.8, sessionSeconds=12,
            intervalSeconds=.025, drainSeconds=.2, tailDrainSeconds=.2, maxRawBytes=4096, baud=115200,
            maxPersistenceMs=250, maxM4AgeMs=20, retries=0, motorRequests=0)
LINE=re.compile(rb'M4S1 ([0-9a-f]{8}) ([0-9a-f]{32}) ([0-9a-f]{4}) ([0-9a-f]{8}) ([0-9a-f]{8}) ([0-9a-f]{2})\r?\n')


def unexpected_tail(raw):
    return bool(raw and not raw.endswith(b'\n')) or any(
        line.startswith(b'M4S1') or b'M4S1'.startswith(line)
        for line in raw.splitlines() if line)


def policy_for(initialize_port):
    if type(initialize_port) is not bool:raise ValueError('initialization_choice_invalid')
    policy=dict(POLICY)
    if initialize_port:policy.update(httpRequests=12,sessionSeconds=32,drainSeconds=20, startupMaxRawBytes=65536)
    return policy


def initialization_boot_changed(before,after):
    if before['bootId']==after['bootId']:
        if before!=after:raise ValueError('outputs_changed_without_initialization_reboot')
        return False
    # A new boot can establish a new diagnostic baseline only with a complete
    # all-OFF history. Never use reopening to hide output activity or loss.
    entries=after['entries']
    if (after['dropped']!=0 or not entries
            or any(not isinstance(e,dict) or type(e.get('pwm')) is not int or e['pwm']!=0
                   or type(e.get('motor')) is not int or e['motor'] not in range(1,6)
                   or e.get('sequence')!=i or e.get('actionOrdinal')!=0
                   for i,e in enumerate(entries,1))
            or {e['motor'] for e in entries}!={1,2,3,4,5}):
        raise ValueError('new_boot_output_history_not_complete_off')
    return True


def open_usb(port, serial_number):
    import serial
    from serial.tools import list_ports
    if not serial_number:raise ValueError('explicit_usb_serial_required')
    selected=[p for p in list_ports.comports() if p.device==port]
    if len(selected)!=1 or (selected[0].vid,selected[0].pid,selected[0].serial_number)!=(0x10C4,0xEA60,serial_number):
        raise ValueError('usb_identity_mismatch')
    device=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.1,exclusive=True)
    # Set before open. No toggling/reset and no input-buffer erasure. Any
    # platform/driver reset is detected by the baseline boot ID in each reply.
    device.dtr=False;device.rts=False;device.port=port
    try:device.open()
    except BaseException:
        device.close();raise
    return device


def parse_reply(raw, nonce, boot_id):
    if len(raw)>4096:raise ValueError('serial_raw_limit')
    candidates=[line for line in raw.splitlines(keepends=True) if line.startswith(b'M4S1 ')]
    if len(candidates)!=1:raise ValueError('serial_reply_count_invalid')
    m=LINE.fullmatch(candidates[0])
    if not m:raise ValueError('serial_reply_shape_invalid')
    token,boot,raw_angle,polled,reported,flags=[v.decode('ascii') for v in m.groups()]
    if token!=nonce or boot!=boot_id:raise ValueError('serial_nonce_or_boot_mismatch')
    raw_angle,polled,reported,flags=[int(v,16) for v in (raw_angle,polled,reported,flags)]
    age=(reported-polled)&0xffffffff
    if raw_angle>4095 or polled==0 or flags!=31 or age>150:raise ValueError('serial_sensor_or_idle_invalid')
    return dict(nonce=nonce,bootId=boot_id,raw=raw_angle,polledMs=polled,reportedMs=reported,
                reportedAgeMs=age,flags=flags,angleDegrees=(raw_angle*360/4096-24.35)%360)


def validate_receipt(row, nonce, boot_id):
    if (not isinstance(row,dict) or row.get('schemaVersion')!='motionbrain.serial_m4_receipt.v1' or row.get('transportCompleted') is not True
            or row.get('clockDomain')!='python_monotonic' or not isinstance(row.get('hostClockId'),str)
            or not row['hostClockId'] or row.get('requestBase64')!=base64.b64encode(b'\x1e'+nonce.encode()+b'\n').decode()):
        raise ValueError('serial_receipt_invalid')
    a,b=row['requestStartedMonotonic'],row['receivedMonotonic']
    if (not all(finite(v) for v in (a,b)) or not 0<=a<=b or b-a>.8):
        raise ValueError('serial_receipt_clock_invalid')
    raw=base64.b64decode(row['rawBase64'],validate=True)
    if hashlib.sha256(raw).hexdigest()!=row.get('rawSha256'):raise ValueError('serial_raw_hash_mismatch')
    sample=parse_reply(raw,nonce,boot_id)
    if row.get('sample')!=sample:raise ValueError('serial_sample_recomputation_mismatch')
    age=sample['reportedAgeMs']
    upper=(b-a)*1000+age+1
    return dict(**sample,transportMs=(b-a)*1000,ageUpperDiagnosticMs=upper,within20ms=upper<=20,
                sampledHostBounds=dict(clockId=row['hostClockId'],earliest=a-(age+1)/1000,
                                       latest=b-max(0,age-1)/1000),
                timestampSemantics='firmware_update_marker_before_adc_not_physical_conversion',
                sensorPhysicalSampleTimeVerified=False,trainingEligible=False,controlEligible=False)


def read_sample(device, nonce, boot_id, host_id, *, clock=time):
    if not re.fullmatch('[0-9a-f]{8}',nonce):raise ValueError('nonce_invalid')
    command=b'\x1e'+nonce.encode()+b'\n'
    row=dict(schemaVersion='motionbrain.serial_m4_receipt.v1',clockDomain='python_monotonic',hostClockId=host_id,
             requestBase64=base64.b64encode(command).decode(),requestStartedMonotonic=clock.monotonic(),
             transportCompleted=False)
    raw=bytearray()
    try:
        with deadline(.8):
            if device.write(command)!=len(command):raise OSError('partial_serial_write')
            while clock.monotonic()-row['requestStartedMonotonic']<=.8:
                chunk=device.read(1)
                if not chunk:continue
                raw.extend(chunk)
                if len(raw)>4096:raise ValueError('serial_raw_limit')
                if chunk==b'\n' and any(line.startswith(b'M4S1 ') for line in raw.splitlines(keepends=True)):
                    row['sample']=parse_reply(bytes(raw),nonce,boot_id)
                    row['transportCompleted']=True
                    break
            if not row['transportCompleted']:raise TimeoutError('serial_request_deadline')
    except (OSError,ValueError) as exc:row['error']=str(exc)
    finally:
        row['receivedMonotonic']=clock.monotonic()
        row['rawBase64']=base64.b64encode(raw).decode();row['rawSha256']=hashlib.sha256(raw).hexdigest()
    return row


def collect(output,url,port,serial_number,*,open_port=open_usb,http=direct_get,clock=time,initialize_port=False):
    policy=policy_for(initialize_port)
    store=ArtifactStore(Path(output));refs=[];http_rows=[];serial_rows=[];samples=[];device=None
    start=clock.monotonic();host_id=uuid.uuid4().hex;nonces=[uuid.uuid4().hex[:8] for _ in range(6)]
    if len(set(nonces))!=6:raise ValueError('nonce_collision')
    def save(path,value):
        at=clock.monotonic();ref=store.write_json(path,value);refs.append(ref)
        if clock.monotonic()-at>.25:raise StorageError('persistence_exceeds_250ms')
        return ref
    def budget():
        if clock.monotonic()-start>policy['sessionSeconds']:raise ValueError('session_deadline')
    def get(endpoint,phase):
        budget();r=http(url,endpoint,clock=clock);r.update(phase=phase,hostClockId=host_id)
        http_rows.append(r);save(f'http/{len(http_rows):02d}.json',r);return _check(r,endpoint)
    def check_baseline(values):
        if values['actions']['bootId']!=values['action_outputs']['bootId'] or values['actions']['enabled']:
            raise ValueError('baseline_boot_or_action_feature_invalid')
    def startup():
        drained=bytearray();drain_start=clock.monotonic()
        try:
            with deadline(policy['drainSeconds']+.1):
                while clock.monotonic()-drain_start<policy['drainSeconds']:
                    drained.extend(device.read(1))
                    if len(drained)>policy.get('startupMaxRawBytes',4096):raise ValueError('serial_startup_raw_limit')
        finally:
            save('serial/startup.json',dict(rawBase64=base64.b64encode(drained).decode(),
                                           startedMonotonic=drain_start,endedMonotonic=clock.monotonic()))
        if initialize_port and drained.count(b'=== MotionBrain System Boot ===')>1:
            raise ValueError('multiple_boots_during_initialization')
    save('plan.json',dict(schemaVersion='motionbrain.serial_m4_probe.v1',policy=policy,hostClockId=host_id,
                          url=url,port=port,usbSerial=serial_number,nonces=nonces,initializePort=initialize_port))
    result=dict(schemaVersion='motionbrain.serial_m4_probe.v1',status='rejected',failure=None,
                trainingEligible=False,controlEligible=False,motorRequests=0,automaticRetry=False,
                startedMonotonic=start,samples=samples,controllerBootChangedDuringInitialization=False)
    try:
        if initialize_port:
            pre_open={e:get(e,'pre_open') for e in ENDPOINTS};check_baseline(pre_open)
            device=open_port(port,serial_number);startup()
        baseline={e:get(e,'baseline') for e in ENDPOINTS}
        check_baseline(baseline)
        boot=baseline['actions']['bootId']
        if initialize_port:
            result['controllerBootChangedDuringInitialization']=initialization_boot_changed(
                pre_open['action_outputs'],baseline['action_outputs'])
        else:
            device=open_port(port,serial_number);startup()
        last_sample=None
        for nonce in nonces:
            budget();row=read_sample(device,nonce,boot,host_id,clock=clock)
            serial_rows.append(row);save(f'serial/{len(serial_rows):02d}.json',row)
            sample=validate_receipt(row,nonce,boot)
            if last_sample is not None:
                for key in ('polledMs','reportedMs'):
                    if not 0<((sample[key]-last_sample[key])&0xffffffff)<1000:
                        raise ValueError('serial_sample_not_new')
            last_sample=sample;samples.append(sample)
            if len(samples)<6:clock.sleep(.025)
        # Preserve and inspect a bounded tail, including a duplicate final
        # reply. Measuring delivery at the first complete line is unchanged.
        tail=bytearray();tail_start=clock.monotonic()
        try:
            with deadline(.25):
                while clock.monotonic()-tail_start<.2:
                    tail.extend(device.read(1))
                    if len(tail)>4096:raise ValueError('serial_tail_raw_limit')
        finally:
            save('serial/tail.json',dict(rawBase64=base64.b64encode(tail).decode(),
                                        startedMonotonic=tail_start,endedMonotonic=clock.monotonic()))
        if unexpected_tail(tail):raise ValueError('unexpected_serial_tail_reply')
        device.close();device=None
        final={e:get(e,'final') for e in ENDPOINTS}
        if final['actions']['bootId']!=boot or final['action_outputs']!=baseline['action_outputs']:
            raise ValueError('controller_boot_or_outputs_changed')
        budget();result.update(status='recorded',outputHistoryUnchanged=True)
    except StorageError:raise
    except (OSError,ValueError,ProbeRejected) as exc:result['failure']=str(exc)
    finally:
        if device is not None:device.close()
    result.update(endedMonotonic=clock.monotonic(),httpCount=len(http_rows),serialCount=len(serial_rows),
                  within20ms=sum(s['within20ms'] for s in samples),artifacts=list(refs))
    ref=save('result.json',result);store.append('serial_probe_sealed','observation',{'summary':ref})
    return result


def audit(root):
    root=Path(root);events,issues=read_events(root)
    try:
        if len(events)!=1 or events[0]['kind']!='serial_probe_sealed':raise ValueError('serial_seal_missing')
        r=read_json_artifact(root,events[0]['payload']['summary']);issues+=verify_artifacts(root,r['artifacts'])
        refs={x['artifactRef']:x for x in r['artifacts']}
        if len(refs)!=len(r['artifacts']):raise ValueError('duplicate_artifacts')
        p=read_json_artifact(root,refs['plan.json'])
        policy=policy_for(p.get('initializePort',False))
        if (p.get('schemaVersion')!='motionbrain.serial_m4_probe.v1' or r.get('schemaVersion')!=p['schemaVersion']
                or p['policy']!=policy or r['trainingEligible'] is not False or r['controlEligible'] is not False
                or r['motorRequests']!=0 or r['automaticRetry'] is not False):raise ValueError('serial_contract_invalid')
        if r['status']=='recorded':
            expected={'plan.json','serial/startup.json','serial/tail.json'}|{f'http/{i:02d}.json' for i in range(1,policy['httpRequests']+1)}|{f'serial/{i:02d}.json' for i in range(1,7)}
            if set(refs)!=expected:raise ValueError('serial_artifact_set_invalid')
            rows=[read_json_artifact(root,refs[f'http/{i:02d}.json']) for i in range(1,policy['httpRequests']+1)]
            phases=['pre_open']*4+['baseline']*4+['final']*4 if p.get('initializePort',False) else ['baseline']*4+['final']*4
            for i,row in enumerate(rows):
                _check(row,ENDPOINTS[i%4])
                if row['phase']!=phases[i] or row['hostClockId']!=p['hostClockId']:
                    raise ValueError('http_identity_invalid')
            pre_open,rows=(rows[:4],rows[4:]) if p.get('initializePort',False) else ([],rows)
            boot=rows[2]['response']['bootId']
            if pre_open:
                if (pre_open[2]['response']['bootId']!=pre_open[3]['response']['bootId']
                        or pre_open[2]['response']['enabled']):raise ValueError('pre_open_state_invalid')
            changed=initialization_boot_changed(pre_open[3]['response'],rows[3]['response']) if pre_open else False
            if r['controllerBootChangedDuringInitialization'] is not changed:raise ValueError('initialization_boot_summary_invalid')
            if (boot!=rows[3]['response']['bootId'] or boot!=rows[6]['response']['bootId']
                    or rows[3]['response']!=rows[7]['response'] or rows[2]['response']['enabled']):
                raise ValueError('controller_boot_or_outputs_changed')
            serial=[read_json_artifact(root,refs[f'serial/{i:02d}.json']) for i in range(1,7)]
            if len(p['nonces'])!=6 or len(set(p['nonces']))!=6:raise ValueError('nonce_set_invalid')
            samples=[validate_receipt(x,n,boot) for x,n in zip(serial,p['nonces'])]
            startup_lower=pre_open[3]['receivedMonotonic'] if pre_open else rows[3]['receivedMonotonic']
            startup_upper=rows[0]['requestStartedMonotonic'] if pre_open else serial[0]['requestStartedMonotonic']
            for name,lower,upper in (('startup',startup_lower,startup_upper),
                                     ('tail',serial[-1]['receivedMonotonic'],rows[4]['requestStartedMonotonic'])):
                drain=read_json_artifact(root,refs[f'serial/{name}.json'])
                a,b=drain['startedMonotonic'],drain['endedMonotonic']
                raw=base64.b64decode(drain['rawBase64'],validate=True)
                seconds=policy['drainSeconds'] if name=='startup' else .2
                limit=policy.get('startupMaxRawBytes',4096) if name=='startup' else 4096
                if (not all(finite(x) for x in (a,b)) or not lower<=a<=b<=upper
                        or not seconds<=b-a<=seconds+.1 or len(raw)>limit):raise ValueError('serial_drain_invalid')
                if name=='startup' and pre_open and raw.count(b'=== MotionBrain System Boot ===')>1:
                    raise ValueError('multiple_boots_during_initialization')
                if name=='tail' and unexpected_tail(raw):
                    raise ValueError('unexpected_serial_tail_reply')
            ordered=pre_open+rows[:4]+serial+rows[4:];previous=r['startedMonotonic']
            if not all(finite(r[k]) for k in ('startedMonotonic','endedMonotonic')):raise ValueError('session_clock_invalid')
            for row in ordered:
                if row['hostClockId']!=p['hostClockId'] or row['requestStartedMonotonic']<previous:raise ValueError('receipt_order_invalid')
                previous=row['receivedMonotonic']
            if not previous<=r['endedMonotonic']<=r['startedMonotonic']+policy['sessionSeconds']:raise ValueError('session_deadline')
            for a,b in zip(samples,samples[1:]):
                for key in ('polledMs','reportedMs'):
                    if not 0<((b[key]-a[key])&0xffffffff)<1000:raise ValueError('serial_sample_not_new')
            if (r['samples']!=samples or r['within20ms']!=sum(s['within20ms'] for s in samples)
                    or r['httpCount']!=policy['httpRequests'] or r['serialCount']!=6 or r['outputHistoryUnchanged'] is not True
                    or r['failure'] is not None):raise ValueError('serial_summary_mismatch')
        elif r['status']!='rejected':raise ValueError('serial_status_invalid')
    except (OSError,ValueError,KeyError,TypeError,AttributeError,OverflowError,StorageError,ProbeRejected) as exc:issues.append(str(exc))
    return dict(ok=not issues,issues=issues,trainingEligible=False,controlEligible=False)
