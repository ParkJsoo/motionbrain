"""Finite, explicitly selected USB sensor diagnostic. No legacy command writer.

RS frames cannot dispatch ARM/motor commands. Firmware poll markers, not physical
AS5600 conversion timestamps, are bounded in the collector's monotonic clock.
"""
from contextlib import nullcontext
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
FRESH_LINE=re.compile(rb'M4S2 ([0-9a-f]{8}) ([0-9a-f]{32}) ([0-9a-f]{4}) ([0-9a-f]{8}) ([0-9a-f]{8}) ([0-9a-f]{8}) ([0-9a-f]{2})\r?\n')
LINE=re.compile(rb'M4S1 ([0-9a-f]{8}) ([0-9a-f]{32}) ([0-9a-f]{4}) ([0-9a-f]{8}) ([0-9a-f]{8}) ([0-9a-f]{2})\r?\n')


def unexpected_tail(raw):
    return bool(raw and not raw.endswith(b'\n')) or any(
        any(line.startswith(magic) or magic.startswith(line) for magic in (b'M4S1',b'M4S2'))
        for line in raw.splitlines() if line)


def policy_for(initialize_port, fresh_sample=False, transport_trace=False):
    if any(type(v) is not bool for v in (initialize_port,fresh_sample,transport_trace)):raise ValueError('initialization_choice_invalid')
    policy=dict(POLICY)
    if initialize_port:policy.update(httpRequests=12,sessionSeconds=32,drainSeconds=20, startupMaxRawBytes=65536)
    if fresh_sample:policy.update(freshSample=True,firmwareWaitLimitMs=150,mcuIntervalMarginMs=2)
    if transport_trace:policy.update(transportTrace=True,maxTraceReadCalls=256)
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


def parse_reply(raw, nonce, boot_id, *, fresh_sample=False):
    if len(raw)>4096:raise ValueError('serial_raw_limit')
    if not raw.endswith(b'\n'):raise ValueError('serial_partial_trailing_bytes')
    candidates=[line for line in raw.splitlines(keepends=True) if unexpected_tail(line)]
    if len(candidates)!=1:raise ValueError('serial_reply_count_invalid')
    m=(FRESH_LINE if fresh_sample else LINE).fullmatch(candidates[0])
    if not m:raise ValueError('serial_reply_shape_invalid')
    fields=[v.decode('ascii') for v in m.groups()]
    received=int(fields.pop(5),16) if fresh_sample else None
    token,boot,raw_angle,polled,reported,flags=fields
    if token!=nonce or boot!=boot_id:raise ValueError('serial_nonce_or_boot_mismatch')
    raw_angle,polled,reported,flags=[int(v,16) for v in (raw_angle,polled,reported,flags)]
    age=(reported-polled)&0xffffffff
    if raw_angle>4095 or polled==0 or flags!=31 or age>150:raise ValueError('serial_sensor_or_idle_invalid')
    result=dict(nonce=nonce,bootId=boot_id,raw=raw_angle,polledMs=polled,reportedMs=reported,
                reportedAgeMs=age,flags=flags,angleDegrees=(raw_angle*360/4096-24.35)%360)
    if fresh_sample:
        wait=(reported-received)&0xffffffff;advance=(polled-received)&0xffffffff
        if not 0<advance<=wait<=150:raise ValueError('fresh_sample_order_invalid')
        result.update(requestReceivedMs=received,firmwareWaitMs=wait)
    return result


def trace_summary(row, enabled):
    if ('transportTrace' in row) != enabled:raise ValueError('transport_trace_presence_mismatch')
    if not enabled:return None
    from learning.serial_transport_trace import validate_trace
    raw=base64.b64decode(row['rawBase64'],validate=True)
    if hashlib.sha256(raw).hexdigest()!=row.get('rawSha256'):raise ValueError('serial_raw_hash_mismatch')
    return validate_trace(row['transportTrace'],row,raw)


def validate_receipt(row, nonce, boot_id, *, fresh_sample=False, transport_trace=False):
    if (not isinstance(row,dict) or row.get('schemaVersion')!='motionbrain.serial_m4_receipt.v1' or row.get('transportCompleted') is not True
            or row.get('clockDomain')!='python_monotonic' or not isinstance(row.get('hostClockId'),str)
            or not row['hostClockId'] or row.get('requestBase64')!=base64.b64encode(b'\x1e'+(b'2' if fresh_sample else b'')+nonce.encode()+b'\n').decode()):
        raise ValueError('serial_receipt_invalid')
    trace_summary(row,transport_trace)
    a,b=row['requestStartedMonotonic'],row['receivedMonotonic']
    if (not all(finite(v) for v in (a,b)) or not 0<=a<=b or b-a>.8):
        raise ValueError('serial_receipt_clock_invalid')
    raw=base64.b64decode(row['rawBase64'],validate=True)
    if hashlib.sha256(raw).hexdigest()!=row.get('rawSha256'):raise ValueError('serial_raw_hash_mismatch')
    sample=parse_reply(raw,nonce,boot_id,fresh_sample=fresh_sample)
    if row.get('sample')!=sample:raise ValueError('serial_sample_recomputation_mismatch')
    age=sample['reportedAgeMs']
    upper=(b-a)*1000+age+1
    bounds=dict(clockId=row['hostClockId'],earliest=a-(age+1)/1000,
                latest=b-max(0,age-1)/1000)
    extra={}
    if fresh_sample:
        # 1ms integer quantization plus 1ms allowance for MCU rate error over
        # <=150ms. This is a stated diagnostic assumption, not calibrated time.
        wait_lower=max(0,sample['firmwareWaitMs']-2)
        if (b-a)*1000<wait_lower:raise ValueError('firmware_wait_exceeds_host_rtt')
        upper-=wait_lower
        advance=(sample['polledMs']-sample['requestReceivedMs'])&0xffffffff
        bounds.update(earliest=a+max(0,advance-2)/1000,latest=b-max(0,age-2)/1000)
        if bounds['earliest']>bounds['latest']:raise ValueError('fresh_marker_bounds_invalid')
        # Use the same conservative rate allowance for the age itself.
        upper+=1
        extra=dict(firmwareWaitLowerDiagnosticMs=wait_lower,
                   mcuClockRateCalibrated=False,mcuIntervalMarginMs=2,
                   ageBoundBasis='host_rtt_minus_mcu_wait_lower_plus_marker_age_upper')
    return dict(**sample,transportMs=(b-a)*1000,ageUpperDiagnosticMs=upper,within20ms=upper<=20,
                sampledHostBounds=bounds,**extra,
                timestampSemantics='firmware_update_marker_before_adc_not_physical_conversion',
                sensorPhysicalSampleTimeVerified=False,trainingEligible=False,controlEligible=False)


def read_sample(device, nonce, boot_id, host_id, *, clock=time, fresh_sample=False, transport_trace=False):
    if not re.fullmatch('[0-9a-f]{8}',nonce):raise ValueError('nonce_invalid')
    command=b'\x1e'+(b'2' if fresh_sample else b'')+nonce.encode()+b'\n'
    row=dict(schemaVersion='motionbrain.serial_m4_receipt.v1',clockDomain='python_monotonic',hostClockId=host_id,
             requestBase64=base64.b64encode(command).decode(),requestStartedMonotonic=clock.monotonic(),
             transportCompleted=False)
    raw=bytearray()
    if transport_trace:
        from learning.serial_transport_trace import TransportTrace
        trace=TransportTrace(clock)
        operation=trace.operation
    else:
        operation=lambda kind,**fields:nullcontext(fields)
    try:
        with deadline(.8):
            with operation('write',requestedBytes=len(command)) as step:
                written=device.write(command);step['returnedBytes']=written
            if written!=len(command):raise OSError('partial_serial_write')
            while clock.monotonic()-row['requestStartedMonotonic']<=.8:
                with operation('buffer_query') as step:
                    available=getattr(device,'in_waiting',0)
                    if type(available) is not int or available<0:raise ValueError('serial_buffer_count_invalid')
                    step['availableBytes']=available
                requested=max(1,min(4097-len(raw),available))
                offset=len(raw);chunk=None;step=None
                try:
                    with operation('read',requestedBytes=requested,rawOffset=offset) as step:
                        chunk=device.read(requested)
                        raw.extend(chunk);step['returnedBytes']=len(chunk)
                finally:
                    # Preserve a returned chunk even if cancellation interrupts
                    # bookkeeping. A read that raises before returning has no
                    # observed bytes; never invent bytes from the driver.
                    if chunk is not None:
                        if len(raw)==offset:raw.extend(chunk)
                        if step is not None:
                            step['returnedBytes']=len(chunk)
                            if transport_trace:step['rawSha256']=hashlib.sha256(chunk).hexdigest()
                if not chunk:continue
                if len(raw)>4096:raise ValueError('serial_raw_limit')
                if b'\n' in chunk and any(line.startswith((b'M4S1 ',b'M4S2 ')) for line in raw.splitlines(keepends=True)):
                    with operation('parse',rawBytes=len(raw),rawSha256=hashlib.sha256(raw).hexdigest() if transport_trace else None):
                        row['sample']=parse_reply(bytes(raw),nonce,boot_id,fresh_sample=fresh_sample)
                    row['transportCompleted']=True
                    break
            if not row['transportCompleted']:raise TimeoutError('serial_request_deadline')
    except KeyboardInterrupt:
        row.update(error='cancelled',cancelled=True)
    except (OSError,ValueError) as exc:row['error']=str(exc)
    finally:
        row['receivedMonotonic']=clock.monotonic()
        row['rawBase64']=base64.b64encode(raw).decode();row['rawSha256']=hashlib.sha256(raw).hexdigest()
        if transport_trace:row['transportTrace']=trace.snapshot()
    return row


def collect(output,url,port,serial_number,*,open_port=open_usb,http=direct_get,clock=time,initialize_port=False,fresh_sample=False,transport_trace=False):
    policy=policy_for(initialize_port,fresh_sample,transport_trace)
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
            budget();row=read_sample(device,nonce,boot,host_id,clock=clock,fresh_sample=fresh_sample,transport_trace=transport_trace)
            serial_rows.append(row);save(f'serial/{len(serial_rows):02d}.json',row)
            if row.get('cancelled') is True:raise KeyboardInterrupt()
            sample=validate_receipt(row,nonce,boot,fresh_sample=fresh_sample,transport_trace=transport_trace)
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
    except KeyboardInterrupt:result.update(failure='cancelled',cancelled=True)
    except (OSError,ValueError,ProbeRejected) as exc:result['failure']=str(exc)
    finally:
        if device is not None:device.close()
    result.update(endedMonotonic=clock.monotonic(),httpCount=len(http_rows),serialCount=len(serial_rows),
                  within20ms=sum(s['within20ms'] for s in samples),artifacts=list(refs))
    ref=save('result.json',result);store.append('serial_probe_sealed','observation',{'summary':ref})
    return result


def audit(root):
    root=Path(root);events,issues=read_events(root);traces=[]
    try:
        if len(events)!=1 or events[0]['kind']!='serial_probe_sealed':raise ValueError('serial_seal_missing')
        r=read_json_artifact(root,events[0]['payload']['summary']);issues+=verify_artifacts(root,r['artifacts'])
        refs={x['artifactRef']:x for x in r['artifacts']}
        if len(refs)!=len(r['artifacts']):raise ValueError('duplicate_artifacts')
        p=read_json_artifact(root,refs['plan.json'])
        fresh_sample=p['policy'].get('freshSample',False)
        transport_trace=p['policy'].get('transportTrace',False)
        policy=policy_for(p.get('initializePort',False),fresh_sample,transport_trace)
        if (p.get('schemaVersion')!='motionbrain.serial_m4_probe.v1' or r.get('schemaVersion')!=p['schemaVersion']
                or p['policy']!=policy or r['trainingEligible'] is not False or r['controlEligible'] is not False
                or r['motorRequests']!=0 or r['automaticRetry'] is not False):raise ValueError('serial_contract_invalid')
        # Trace evidence is audited even for rejected/cancelled attempts.
        if (not isinstance(p['nonces'],list) or len(p['nonces'])!=6 or
                any(not isinstance(n,str) or not re.fullmatch('[0-9a-f]{8}',n) for n in p['nonces']) or
                len(set(p['nonces']))!=6):raise ValueError('nonce_set_invalid')
        count=r['serialCount']
        if type(count) is not int or not 0<=count<=6:raise ValueError('serial_attempt_count_invalid')
        serial_paths={name for name in refs if re.fullmatch(r'serial/\d+\.json',name)}
        if serial_paths!={f'serial/{i:02d}.json' for i in range(1,count+1)}:
            raise ValueError('serial_attempt_artifact_mismatch')
        for i in range(1,count+1):
            row=read_json_artifact(root,refs[f'serial/{i:02d}.json'])
            expected_request=base64.b64encode(b'\x1e'+(b'2' if fresh_sample else b'')+p['nonces'][i-1].encode()+b'\n').decode()
            if (not isinstance(row,dict) or row.get('schemaVersion')!='motionbrain.serial_m4_receipt.v1'
                    or row.get('requestBase64')!=expected_request or row.get('hostClockId')!=p['hostClockId']
                    or row.get('clockDomain')!='python_monotonic' or type(row.get('transportCompleted')) is not bool):
                raise ValueError('serial_attempt_identity_invalid')
            raw=base64.b64decode(row['rawBase64'],validate=True)
            if len(raw)>4097 or hashlib.sha256(raw).hexdigest()!=row['rawSha256']:
                raise ValueError('serial_attempt_raw_invalid')
            a,b=row['requestStartedMonotonic'],row['receivedMonotonic']
            if not all(finite(v) for v in (a,b)) or not 0<=a<=b:raise ValueError('serial_attempt_clock_invalid')
            item=trace_summary(row,transport_trace)
            if item is not None:traces.append(item)
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
            samples=[validate_receipt(x,n,boot,fresh_sample=fresh_sample,transport_trace=transport_trace) for x,n in zip(serial,p['nonces'])]
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
    return dict(ok=not issues,issues=issues,transportTraceSummaries=traces,trainingEligible=False,controlEligible=False)
