"""Offline, receipt-bound video/command examples; no device access or model fitting.

Sensor changes are supervision, not visual angle calibration. Camera PTS and HTTP
receipt brackets do not establish exposure time or exact controller-clock sync.
"""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
import statistics

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore, StorageError, _decode_json, _read, _parts
from learning.supervised_measurement import clock_evidence, finite, validate_execution
from learning.supervised_transport import SequencePort, response_json, validate_pulse
from learning.supervised_collection import PROFILES
from learning.http_phase_trace import http_inventory_matches

INDEX = 'motionbrain.manual_dataset_index.v1'
SCHEMA = 'motionbrain.manual_video_command_dataset.v1'
POLICY = dict(preSeconds=[-3., -.1], postSeconds=[3., 6.], minimumFrames=3,
              minimumSensorPoints=3, maximumFrameGapSeconds=.3,
              inputAvailability='stdout_read_plus_conservative_persistence_margin_before_command',
              settlingVerified=False)


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class Evidence:
    def __init__(self, root):
        self.root = Path(root)
        self.hashes = {}

    def raw(self, name):
        raw = _read(self.root, name)
        self.hashes[name] = digest(raw)
        return raw

    def json(self, name):
        return _decode_json(self.raw(name))


class _CompareStore:
    """ActionRecorder's persisted-frame validation reused without writing anything."""
    def __init__(self, evidence, expected):
        self.evidence, self.expected = evidence, expected

    def write_json(self, name, value):
        require(self.evidence.json('native-camera/host-ingress/'+name) == value,
                'ingress_frame_changed')
        require(value == self.expected[len(self.expected_seen)], 'session_frame_changed')
        self.expected_seen.append(name)



def audit_camera(ev, plan, session):
    camera = plan['camera']; frames = session['frames']
    require(isinstance(frames, list) and bool(frames), 'frames_required')
    capture = ev.root/'native-camera'
    # Existing bracket validator checks the same host boot/timebase before/after.
    clock_hashes = {}
    clock = clock_evidence(capture, clock_hashes)
    ev.hashes.update({'native-camera/'+k:v for k,v in clock_hashes.items()})
    require(clock['basis'] == 'before_after_same_mach_clock'
            and clock['hostClockId'] == camera['sessionId'], 'closed_camera_clock_required')
    rec = ActionRecorder(capture, 'unused-offline-helper', camera)
    rec.started = clock['validPythonBounds'][0]
    rec.offset_low, rec.offset_high = clock['initialOffsets']
    rec.store = _CompareStore(ev, frames); rec.store.expected_seen = []
    lines = [ _decode_json(line) for line in ev.raw('native-camera/helper.stdout').splitlines() ]
    require(len(lines) == len(frames)+1 and lines[-1].get('event') == 'terminal', 'camera_receipt_count')
    for i, (frame, line) in enumerate(zip(frames, lines)):
        require(line.get('event') == 'frame' and frame.get('frameIndex') == i, 'camera_frame_order')
        require(all(finite(frame.get(k)) for k in ('stdoutReadAt','sourceEarliest','sourceLatest','persistenceMs')),
                'camera_time_invalid')
        rec._packet(line['receipt'], frame['stdoutReadAt'])
        require(clock['validPythonBounds'][0] <= frame['sourceEarliest'] <= frame['sourceLatest']
                <= frame['stdoutReadAt'] <= clock['validPythonBounds'][1], 'camera_clock_bounds')
        # _packet already authenticates these bytes; retain their source bindings.
        ev.hashes['native-camera/'+frame['metadataFile']] = frame['metadataSha256']
        ev.hashes['native-camera/'+frame['imageFile']] = frame['imageSha256']
    rec.closing = True
    rec._read_terminal(lines[-1])
    terminal = ev.json('native-camera/terminal.json')
    require(terminal.get('droppedFrames') == 0 and session['recording'].get('reason') is None,
            'camera_drop_or_fault')
    require(session['recording'].get('recordedFrames') == len(frames)
            and session['recording'].get('terminalStatus') == terminal['status'], 'recording_terminal_mismatch')
    require(session['recording'].get('clockBridge', {}).get('hostClockId') == clock['hostClockId'],
            'recording_clock_mismatch')
    return frames, dict(viewId=camera['viewId'], deviceId=camera['deviceId'],
                       configuration=list(rec.configuration), helperSha256=plan['helperSha256'])


def _http_records(ev, session):
    """Keep every failed GET and its bounded quiet recovery in the source audit."""
    events = [e for e in session['events'] if e.get('kind') == 'http']
    failures = [e for e in session['events'] if e.get('kind') == 'http_error']
    retries = [e for e in session['events'] if e.get('kind') == 'http_retry']
    recovery_files = list(ev.root.glob('quiet-monitor-*-recovery-intent.json'))
    recovery = None
    if failures or retries or recovery_files:
        require(len(failures) == len(retries) == len(recovery_files) == 1, 'http_failure_or_retry')
        recovery = ev.json(recovery_files[0].name)
        require(set(recovery) == {'reason','failedRequestSerial','startedAt','deadline',
                'lastVerifiedSequence','readOnly','offVerified','maxRecoveriesPerSession'} and
                recovery['readOnly'] is True and recovery['offVerified'] is False and
                type(recovery['maxRecoveriesPerSession']) is int and recovery['maxRecoveriesPerSession'] == 1 and
                type(recovery['failedRequestSerial']) is int and recovery['failedRequestSerial'] > 1 and
                type(recovery['lastVerifiedSequence']) is int and recovery['lastVerifiedSequence'] > 0 and
                recovery['reason'] in ('TimeoutError','RemoteDisconnected','URLError') and
                finite(recovery['startedAt']) and finite(recovery['deadline']), 'quiet_recovery_contract')
        serial = recovery['failedRequestSerial']; name = f'http-{serial:06d}'
        require(recovery_files[0].name == f'quiet-monitor-{serial:06d}-recovery-intent.json', 'quiet_recovery_file')
        intent = ev.json(name+'-intent.json'); error = ev.json(name+'-error.json')
        require(intent['method'] == 'GET' and intent['payload'] is None and
                intent['path'].split('?')[0] in ('/status','/collection_status','/action_outputs') and
                error['error'] == recovery['reason'], 'quiet_recovery_read_only')
        times = [recovery['deadline']-4.5, failures[0]['requestStartedAt'], intent['startedAt'], error['endedAt'],
                 failures[0]['at'], recovery['startedAt'], recovery['deadline']]
        require(all(finite(t) for t in times) and times == sorted(times) and
                recovery['startedAt'] < recovery['deadline'], 'quiet_recovery_timing')
        require(failures[0] == dict(kind='http_error', at=failures[0]['at'],
                requestStartedAt=failures[0]['requestStartedAt'], path=intent['path'],payload=None,error=error['error']) and
                retries[0] == dict(kind='http_retry',at=recovery['startedAt'],path=intent['path'],payload=None,
                attempt=2,evidenceFile=recovery_files[0].name,detail=recovery), 'quiet_recovery_event_binding')
    serials = [i for i in range(1, len(events)+len(failures)+1)
               if recovery is None or i != recovery['failedRequestSerial']]
    expected = {f'http-{i:06d}-{suffix}.json' for i in serials for suffix in ('intent','response')}
    if recovery:
        expected.update(f"http-{recovery['failedRequestSerial']:06d}-{suffix}.json" for suffix in ('intent','error'))
    require(http_inventory_matches(ev, expected), 'http_evidence_count_or_retry')
    return list(zip(serials, events)), recovery


def _audit_quiet_recovery(records, recovery, command, receipt, session, boot):
    if recovery is None:
        return
    serial = recovery['failedRequestSerial']; start = recovery['deadline']-4.5
    earlier = [e for i,e in records if i < serial]
    failure = next(e for e in session['events'] if e.get('kind') == 'http_error')
    require(earlier and earlier[-1]['responseReceivedAt'] <= failure['requestStartedAt'],
            'quiet_recovery_failure_chronology')
    later = [e for i,e in records if i > serial]
    require(later and later[0]['path'] in ('/status','/collection_status') and
            later[0]['requestStartedAt'] >= recovery['startedAt'], 'quiet_recovery_fresh_status')
    resumed = [later[0]]
    for event in later[1:]:
        require(event['path'].split('?')[0] == '/action_outputs' and event['payload'] is None,
                'quiet_recovery_output_read_required')
        resumed.append(event)
        if not event['response'].get('hasMore',False):
            break
    require(len(resumed) >= 2 and not resumed[-1]['response'].get('hasMore',False) and
            resumed[-1]['responseReceivedAt'] <= recovery['deadline'], 'quiet_recovery_deadline')
    end = resumed[-1]['responseReceivedAt']
    results = [e for e in session['events'] if e.get('kind') == 'result']
    post = len(results) == 1 and results[0]['at'] <= start and receipt['offAt'] <= results[0]['at']
    require(end < command['at'] or post, 'quiet_recovery_during_command')
    window = [e for _,e in records if e['responseReceivedAt'] >= start and e['requestStartedAt'] <= end]
    require(all(e['payload'] is None and e['path'].split('?')[0] in
                ('/status','/collection_status','/action_outputs') for e in window), 'quiet_recovery_write_in_window')
    # A successful prior quiet snapshot is mandatory, not just a claimed OFF flag.
    statuses = [e for e in earlier if e['path'] in ('/status','/collection_status') and e['responseReceivedAt'] <= start]
    outputs = [e for e in earlier if e['path'].split('?')[0] == '/action_outputs']
    require(statuses and any(e['responseReceivedAt'] <= start and not e['response'].get('hasMore',False)
                            for e in outputs), 'quiet_recovery_prior_off_missing')
    port = object.__new__(SequencePort)
    port.boot_id=boot; port.last_sequence=None; port._known_entries={}; port._last_output_time=None
    port.last_outputs={}; port._active_pulse=None
    port._check_status(statuses[-1]['response'],'IDLE')
    prior_off = False
    for e in outputs:
        port._consume(e['response'])
        if e['responseReceivedAt'] <= start and not e['response'].get('hasMore',False):
            prior_off = port.last_outputs == {i:0 for i in range(1,6)}
    require(prior_off, 'quiet_recovery_prior_output_not_off')
    require(port.last_outputs == {i:0 for i in range(1,6)} and
            port.last_sequence == recovery['lastVerifiedSequence'], 'quiet_recovery_prior_output_changed')
    port._check_status(resumed[0]['response'],'IDLE')
    for e in resumed[1:]:
        port._consume(e['response'],quiet=True)
    require(port.last_outputs == {i:0 for i in range(1,6)}, 'quiet_recovery_output_not_off')


def audit_http(ev, session, command, receipt):
    records, recovery = _http_records(ev, session)
    boot = None; output = {}; baseline = session['outputBaselineSequence']
    require(type(baseline) is int and baseline > 0, 'output_baseline')
    posts = []; statuses = []; last_queue = None; run_bounds = None
    previous_response = None; first_clear_bounds = None; previous_event = None; initial_latest = None; off_latest = None
    for i, event in records:
        name = f'http-{i:06d}'
        intent = ev.json(name+'-intent.json'); response = ev.json(name+'-response.json')
        path = intent['path']; payload = intent['payload']; parsed = response_json(response['rawUtf8'], path)
        require(response.get('httpStatus') == 200 and parsed == response['parsed'] == event['response'], 'http_response_changed')
        require(event['path'] == path and event['payload'] == payload
                and intent['method'] == ('GET' if payload is None else 'POST'), 'http_intent_changed')
        times = [event['requestStartedAt'], intent['startedAt'], response['receivedAt'], event['responseReceivedAt']]
        require(all(finite(t) for t in times) and times == sorted(times), 'http_time_order')
        require(previous_response is None or previous_response <= times[0], 'http_chronology')
        previous_response=times[-1]
        if payload is not None:
            require(parsed.get('success') is True, 'http_command_rejected')
        if path == '/sequence' and payload == {'action':'clear'} and first_clear_bounds is None:
            require(previous_event is not None and previous_event['path'].split('?')[0] == '/action_outputs',
                    'off_snapshot_missing')
            first_clear_bounds=(previous_event['responseReceivedAt'],event['requestStartedAt'])
            off_latest=max(output)
        if payload is not None: posts.append((path,payload))
        if path == '/sequence' and payload == {'action':'run'}:
            run_bounds=(intent['startedAt'],response['receivedAt'])
        if path == '/actions':
            if boot is None: boot=parsed.get('bootId')
            require(parsed.get('bootId') == boot and parsed.get('active') is False, 'controller_boot_changed')
        if path in ('/status','/collection_status'):
            statuses.append(event)
        if path.split('?')[0] == '/action_outputs':
            require(parsed.get('bootId') == boot and parsed.get('evidence') == 'driver_write'
                    and parsed.get('clockDomain') == 'controller_boot_millis' and parsed.get('gap',False) is False,
                    'output_boot_or_gap')
            if initial_latest is None and not parsed.get('hasMore',False):
                initial_latest=max(e['sequence'] for e in parsed['entries'])
            for entry in parsed['entries']:
                n = entry['sequence']
                require(n not in output or output[n] == entry, 'output_history_changed')
                output[n] = entry
        if path == '/sequence': last_queue=parsed.get('sequence')
        previous_event=event
    require(boot is not None and len(boot)==32 and statuses, 'controller_evidence_missing')
    require(run_bounds is not None and run_bounds[0] <= receipt['dispatchBefore']
            <= receipt['dispatchAfter'] <= run_bounds[1], 'run_receipt_bracket_changed')
    require(first_clear_bounds is not None and first_clear_bounds[0] <= receipt['offAt'] <= first_clear_bounds[1],
            'off_receipt_bracket_changed')
    pulse=command['pulse']
    expected=[('/motor',{'action':'default','speed':255}),('/sequence',{'action':'add','joint':pulse['joint'],
        'direction':pulse['direction'],'speed':pulse['percent'],'duration':pulse['durationMs']}),
        ('/command',{'cmd':'arm'}),('/sequence',{'action':'run'})]
    cleanup=[('/command',{'cmd':'stop'}),('/sequence',{'action':'clear'}),('/motor',{'action':'default','speed':100})]
    require(posts == expected+cleanup+cleanup, 'unexpected_command_sequence')
    require(last_queue and last_queue.get('state') == 'IDLE' and last_queue.get('totalCount') == 0, 'final_queue_not_empty')
    port=object.__new__(SequencePort)
    port.boot_id=boot; port.last_sequence=None; port._known_entries={}; port._last_output_time=None
    port.last_outputs={}; port._active_pulse=None
    entries=[output[k] for k in sorted(output)]
    port._consume(dict(bootId=boot,evidence='driver_write',clockDomain='controller_boot_millis',entries=entries))
    require(port.last_outputs == {i:0 for i in range(1,6)}, 'final_output_not_zero')
    baseline_outputs={e['motor']:e['pwm'] for e in entries if e['sequence']<=baseline}
    require(baseline_outputs=={i:0 for i in range(1,6)},'baseline_output_not_zero')
    before_statuses=[e for e in statuses if e['responseReceivedAt'] < command['at']]
    require(bool(before_statuses),'pre_command_status_missing')
    port._check_status(before_statuses[-1]['response'],'IDLE')
    port._check_status(statuses[-1]['response'], 'IDLE')
    trace=receipt['outputTrace']
    require(all(output.get(e['sequence']) == e for e in trace), 'receipt_trace_changed')
    require(baseline == initial_latest and trace[0]['sequence'] == baseline+1, 'receipt_baseline_gap')
    require(trace == [e for e in entries if baseline < e['sequence'] <= off_latest], 'receipt_trace_incomplete')
    require(all(e['pwm']==0 for e in entries if e['sequence']>trace[-1]['sequence']), 'output_after_receipt')
    require(all(e['motor']==pulse['motor'] or e['pwm']==0 for e in entries if e['sequence']>baseline), 'other_joint_output')
    _audit_quiet_recovery(records, recovery, command, receipt, session, boot)
    return statuses, entries, boot


def select_frames(frames, window, command_at=None):
    selected=[f for f in frames if window[0] <= f['sourceEarliest'] and f['sourceLatest'] <= window[1]
              and not f['qualityFlags'] and (command_at is None or
              f['stdoutReadAt']+f['persistenceMs']/1000 < command_at)]
    require(len(selected)>=POLICY['minimumFrames'], 'video_window_missing')
    require(selected[0]['sourceLatest']-window[0] <= .3 and window[1]-selected[-1]['sourceEarliest'] <= .3,
            'video_window_coverage')
    require(all(b['sourceLatest']-a['sourceEarliest']<=POLICY['maximumFrameGapSeconds']
                for a,b in zip(selected,selected[1:])), 'video_window_gap')
    return selected


def sensor_window(statuses, window):
    samples=[]
    for event in statuses:
        angle=event['response'].get('shoulderAngle',{})
        if not all(angle.get(k) is True for k in ('available','sensorConnected','sensorFresh','sensorReady')):
            continue
        value,age=angle.get('angleDeg'),angle.get('ageMs')
        require(finite(value) and finite(age) and 0<=age<=150, 'sensor_value_or_age')
        bounds=[event['requestStartedAt']-age/1000,event['responseReceivedAt']]
        if window[0]<=bounds[0] and bounds[1]<=window[1]:
            samples.append(dict(valueDeg=value,raw=angle.get('raw'),hostBracket=bounds))
    require(len(samples)>=POLICY['minimumSensorPoints'], 'sensor_window_missing')
    values=[s['valueDeg'] for s in samples]
    return dict(medianDeg=statistics.median(values),rangeDeg=max(values)-min(values),
                window=window,samples=samples)


def frame_ref(frame):
    return {k:copy.deepcopy(frame[k]) for k in ('frameIndex','frameId','imageFile','imageSha256','metadataFile',
        'metadataSha256','pixelSha256','width','height','rawByteLength','storageEncoding','sourceEarliest',
        'sourceLatest','stdoutReadAt','persistenceMs')}


def _visual_session(root, expected_hash):
    ev=Evidence(root); raw=ev.raw('session.json')
    require(digest(raw)==expected_hash,'session_hash_mismatch')
    s=_decode_json(raw);plan=ev.json('plan.json')
    require(s.get('schemaVersion')=='motionbrain.manual_collection.v1' and s.get('phase')=='completed'
            and s.get('error') is None and s.get('finalIdleOffVerified') is True,'completed_session_required')
    require(s.get('datasetEligible') is False and s.get('trainingPerformed') is False,'original_flags_changed')
    require(plan.get('sessionId')==s.get('sessionId') and plan.get('joint')==s.get('joint') and s.get('joint') in PROFILES, 'session_plan_identity')
    commands=[e for e in s['events'] if e.get('kind')=='command'];results=[e for e in s['events'] if e.get('kind')=='result']
    require(len(commands)==len(results)==1,'single_command_required')
    command=ev.json('command-001-intent.json');result=ev.json('command-001-result.json')
    require(commands[0]==command and results[0]==result and command['commandId']==result['commandId'], 'command_artifact_changed')
    require(len(list(ev.root.glob('command-*')))==2,'command_artifact_count')
    pulse=validate_pulse(command['pulse']);receipt=result['receipt']
    require(pulse['joint']==s['joint'], 'session_command_joint_mismatch')
    sign,cap,duration=PROFILES[pulse['joint']]['directions'][pulse['direction']]
    require(pulse['percent']<=cap and pulse['durationMs']<=duration and pulse['expectedPwm']*sign>0,'profile_limit')
    require(receipt.get('offVerified') is True,'receipt_off_required')
    validate_execution(dict(receipt,pulse=pulse),pulse)
    times=[command['at'],receipt['dispatchBefore'],receipt['dispatchAfter'],receipt['offAt'],result['at'],s['endedAt']]
    require(all(finite(t) for t in times) and times==sorted(times),'command_timing')
    frames,identity=audit_camera(ev,plan,s)
    statuses,outputs,boot=audit_http(ev,s,command,receipt)
    pre=[command['at']+x for x in POLICY['preSeconds']];post=[receipt['offAt']+x for x in POLICY['postSeconds']]
    before_frames=select_frames(frames,pre,command['at']);after_frames=select_frames(frames,post)
    input_frame=before_frames[-1];target_frame=min(after_frames,key=lambda f:abs((f['sourceEarliest']+f['sourceLatest'])/2-statistics.mean(post)))
    # The only prospective fields are a bound image and the intended command.
    inputs=dict(image=frame_ref(input_frame),command={k:pulse[k] for k in ('joint','direction','percent','durationMs')})
    row=dict(sessionId=s['sessionId'],status='eligible',inputs=inputs,
        targets=dict(image=frame_ref(target_frame)),
        evidence=dict(commandId=command['commandId'],commandAt=command['at'],dispatchHostBracket=[receipt['dispatchBefore'],receipt['dispatchAfter']],
            offReceiptAt=receipt['offAt'],controllerBootId=boot,actualOutputTrace=receipt['outputTrace'],
            preFrameIds=[f['frameIndex'] for f in before_frames],postFrameIds=[f['frameIndex'] for f in after_frames],
            auditedFrames=len(frames),cameraIdentity=identity,sourceHashes=ev.hashes),
        originalDatasetEligible=False,trainingPerformed=False,controlEligible=False)
    return row, statuses, pre, post, [frames[0]['sourceEarliest'], s['endedAt']]


def build_visual_session(root, expected_hash):
    """Audit one allowed single-joint image transition without a sensor label.

    Keeps the existing camera chronology, OFF evidence and fixed windows.
    Multi-pulse sessions are refused rather than silently split into examples.
    """
    row, _, _, _, bounds = _visual_session(root, expected_hash)
    row['evidence']['cameraHostBounds'] = bounds
    return row


def build_session(root, expected_hash):
    row, statuses, pre, post, _ = _visual_session(root, expected_hash)
    require(row['inputs']['command']['joint']=='shoulder', 'session_plan_identity')
    before=sensor_window(statuses,pre);after=sensor_window(statuses,post)
    row['targets'].update(sensorBefore=before,sensorAfter=after,
                          sensorDeltaDeg=after['medianDeg']-before['medianDeg'])
    return row


def build_dataset(index_path):
    index_path=Path(index_path).resolve();raw=index_path.read_bytes();index=_decode_json(raw)
    require(index.get('schemaVersion')==INDEX and index.get('split')=='development'
            and isinstance(index.get('captureGroup'),str) and index['captureGroup'],'development_group_required')
    sources=index.get('sessions');require(isinstance(sources,list) and bool(sources),'explicit_sessions_required')
    seen=set();canonical=set();episodes=set();session_ids=set();rows=[];identity=None;command=None
    for source in sources:
        require(set(source)=={'path','sessionSha256'},'session_reference_fields')
        relative=source['path'];require(isinstance(relative,str) and relative not in seen,'duplicate_session')
        _parts(relative)
        seen.add(relative);root=index_path.parent/relative
        require(root.resolve().is_relative_to(index_path.parent) and not root.is_symlink(),'session_path_escape')
        require(root.resolve() not in canonical,'duplicate_session_path')
        canonical.add(root.resolve())
        try:
            row=build_session(root,source['sessionSha256'])
            episode=(row['evidence']['controllerBootId'],row['evidence']['commandId'])
            require(row['sessionId'] not in session_ids and episode not in episodes,'duplicate_episode')
            session_ids.add(row['sessionId']);episodes.add(episode)
            this_identity=row['evidence']['cameraIdentity'];this_command=row['inputs']['command']
            require(identity is None or identity==this_identity,'capture_configuration_changed')
            require(command is None or command==this_command,'command_group_changed')
            identity,command=this_identity,this_command
        except (ValueError,StorageError,OSError,KeyError,TypeError) as exc:
            row=dict(status='rejected',reason=str(exc),trainingPerformed=False,controlEligible=False)
        row.update(source=copy.deepcopy(source),captureGroup=index['captureGroup'],split='development')
        rows.append(row)
    return dict(schemaVersion=SCHEMA,indexSha256=digest(raw),policy=copy.deepcopy(POLICY),rows=rows,
                counts=dict(listed=len(rows),eligible=sum(r['status']=='eligible' for r in rows),
                            rejected=sum(r['status']=='rejected' for r in rows)),
                excluded=copy.deepcopy(index.get('excluded',[])),trainingPerformed=False,
                independentTest=False,controlEligible=False,
                timingSemantics='camera_pts_and_http_receipt_bounds_not_exposure_or_exact_controller_sync',
                labelSemantics='M4_sensor_delta_not_independent_visual_angle_ground_truth')


def export_dataset(index_path, output, *, export_png=False):
    result=build_dataset(index_path);store=ArtifactStore(Path(output))
    if export_png:
        import cv2
        import numpy as np
        from learning.lossless_frame import decode
        for i,row in enumerate(result['rows']):
            if row['status']!='eligible':continue
            root=Path(index_path).resolve().parent/row['source']['path']/'native-camera'
            for section in ('inputs','targets'):
                frame=row[section]['image'];raw=_read(root,frame['imageFile'])
                require(digest(raw)==frame['imageSha256'],'export_frame_changed')
                pixels=decode(raw,frame['rawByteLength'],frame['pixelSha256'])
                image=np.frombuffer(pixels,dtype=np.uint8).reshape(frame['height'],frame['width'],4)
                ok,png=cv2.imencode('.png',image);require(ok,'png_export_failed')
                frame['png']=store._write(f'images/{i:03d}-{section}.png',png.tobytes())
    store.write_json('dataset.json',result)
    return result
