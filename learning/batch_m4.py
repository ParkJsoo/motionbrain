"""정확한 일회 M4 위 범위를 기존 시운전 코어에 위임한다. 일반 출력 제한은 유지한다.

생성·정적 검사·원본 감사·재개는 장치에 접근하지 않는다. 현장/접촉 검토자는
명시적으로 주입해야 하며 이 모듈은 긍정 접촉·파지·들기 판정을 생성하지 않는다.
"""
from __future__ import annotations
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from learning import m4_up_commissioning as core
from learning.m4_up_observer import M4DualCameraObserver
from learning.m4_lift_data import _http, _native
from learning.manual_collection_dataset import Evidence
from learning.artifacts import ArtifactStore, StorageError, _read, _decode_json
from learning.batch_observation import observe_frame, validate_spec, digest as observer_digest
from learning.batch_learning import prospective_evidence, evaluate, validate_model, model_sha256
from learning.supervised_collection import plan_digest
from learning.experiment_pipeline import encoded, atomic


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def sha(raw): return hashlib.sha256(raw).hexdigest()


def private_path(repository, name):
    require(isinstance(name, str) and bool(name), 'm4_private_path_required')
    root = Path(repository).resolve()
    path = (root/name).resolve()
    require(path.is_relative_to((root/'.local').resolve()), 'm4_private_path_required')
    return path


def validate_route_command(command, binding):
    """이 helper의 통과는 특수 경로 후보임만 뜻한다. 파일·scope 심사가 별도로 필요하다."""
    require(isinstance(binding, dict) and binding.get('route') == 'm4_up' and
            isinstance(binding.get('scopePath'), str) and bool(binding['scopePath']) and
            core._hash(binding.get('scopeSha256')), 'm4_route_binding_required')
    require(isinstance(command, dict) and set(command) == {'joint','direction','percent','durationMs'},
            'm4_route_command_fields')
    allowed = core.CONTACT_PROBE_PULSES + (core.LIFT_PULSE, core.LEARNING_PULSE, core.SHORT_PULSE)
    require(any(command == {k:p[k] for k in command} and
                all(type(command[k]) is type(p[k]) for k in command) for p in allowed),
            'm4_route_exact_command_required')
    return True


def validate_binding(plan, condition, binding, repository):
    validate_route_command(condition['command'], binding)
    require(set(binding) in ({'route','interaction','scopePath','scopeSha256','camera'},
                            {'route','interaction','scopePath','scopeSha256','camera','retentionCaptures'}), 'm4_binding_fields')
    if 'retentionCaptures' in binding:
        retention=binding['retentionCaptures']
        require(isinstance(retention,dict) and set(retention)=={'count','intervalSeconds'} and
                type(retention['count']) is int and retention['count']==2 and
                core.transport.finite_number(retention['intervalSeconds']) and
                1<=retention['intervalSeconds']<=3,'m4_retention_configuration')
    require(condition['target'] == condition['maxAttempts'] == 1, 'm4_single_scope_one_attempt')
    require(condition['phase'] in ('training','preparation','evaluation'), 'm4_generic_prospective_not_supported')
    require(isinstance(plan.get('runtime'),dict) and
            plan['runtime'].get('controllerUrl') is not None, 'm4_batch_controller_binding_required')
    scope_path = private_path(repository, binding['scopePath'])
    raw = scope_path.read_bytes()
    require(sha(raw) == binding['scopeSha256'], 'm4_scope_file_changed')
    scope = core.validate_scope(_decode_json(raw))
    require(scope['schemaVersion'] in (core.SCHEMA, core.LIFT_SCHEMA, core.CONTACT_PROBE_SCHEMA),
            'm4_legacy_learning_scope_not_generic_batch')
    expected_interaction = 'contact_probe' if scope['schemaVersion'] == core.CONTACT_PROBE_SCHEMA else 'grasped'
    require(binding['interaction'] == expected_interaction, 'm4_interaction_scope_mismatch')
    require(condition['command'] == {k:scope['pulse'][k] for k in condition['command']}, 'm4_scope_command_changed')
    require(scope['controllerUrl'] == plan['runtime']['controllerUrl'] and
            scope['controllerBootId'] == plan['controllerBootId'] and
            scope['context']['externalViewId'] == plan['viewId'], 'm4_scope_batch_context_changed')
    config = binding['camera']
    require(isinstance(config, dict) and set(config) == {'helperPath','helperSha256','cameraUrl'}, 'm4_camera_fields')
    helper = (Path(repository).resolve()/config['helperPath']).resolve(strict=True)
    require(helper.is_relative_to(Path(repository).resolve()) and helper.is_file() and
            sha(helper.read_bytes()) == config['helperSha256'], 'm4_camera_helper_changed')
    core.transport.validate_target(config['cameraUrl'])
    return dict(scope=scope, scopePath=scope_path, camera={**config,'helperPath':str(helper)},
                interaction=expected_interaction)


def audit_source(root, expected_result_sha, expected_scope_sha=None):
    """M4 전용 원본을 재감사한다. ManualCollection session으로 변환하지 않는다."""
    ev = Evidence(Path(root).resolve())
    result_raw, scope_raw = ev.raw('result.json'), ev.raw('scope.json')
    require(sha(result_raw) == expected_result_sha, 'm4_result_hash_changed')
    if expected_scope_sha is not None:
        require(sha(scope_raw) == expected_scope_sha, 'm4_source_scope_file_changed')
    scope = core.validate_scope(_decode_json(scope_raw))
    require(scope['schemaVersion'] in (core.SCHEMA, core.LIFT_SCHEMA, core.CONTACT_PROBE_SCHEMA), 'm4_batch_source_schema')
    result = _decode_json(result_raw)
    expected_status = 'probe_completed_contact_unverified' if scope['schemaVersion'] == core.CONTACT_PROBE_SCHEMA else 'completed'
    require(result.get('schemaVersion') == scope['schemaVersion'] and result.get('scopeSha256') == plan_digest(scope),
            'm4_source_scope_binding_changed')
    require(result.get('status') == expected_status and result.get('errors') == [] and
            result.get('finalIdleOffVerified') is True and result.get('mayHaveRun') is True and
            result.get('commandOutcome') == 'receipt_recorded' and result.get('modelSelected') is False,
            'm4_source_not_completed')
    receipt = result['receipt']
    statuses, outputs, boot, first = _http(ev, receipt, scope['pulse'])
    require(boot == scope['controllerBootId'], 'm4_source_boot_changed')
    require(bool(statuses),'m4_source_status_missing')
    baseline_angle=core.transport.SequencePort._shoulder_angle(statuses[0]['response'])
    require(abs(baseline_angle-scope['expectedStartAngleDeg']) <= scope['startAngleToleranceDeg'], 'm4_source_start_angle_changed')
    angles = result.get('angleObservations')
    require(isinstance(angles,list) and bool(angles) and all(isinstance(a,dict) and
            core.transport.finite_number(a.get('angleDeg')) and
            -.25 <= a['angleDeg']-baseline_angle <= scope['maxObservedDeltaDeg'] for a in angles),
            'm4_source_angle_evidence_invalid')
    frames, bridge = _native(ev, scope, result['observerClose'])
    pairs = {}
    for label in ('before','after'):
        record = ev.json(label+'-observation.json')
        require(record.get('hostClockId') == scope['hostClockId'] and record.get('context') == scope['context']
                and record.get('stable') is True, 'm4_source_observation_context')
        require(sha(ev.raw(record['evidenceFile'])) == record['evidenceSha256'] and
                ev.json(record['evidenceFile']) == {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')},
                'm4_source_review_changed')
        onsite = record.get('onsite',{})
        require(set(onsite) == {'operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified'}
                and all(type(v) is bool for v in onsite.values()) and
                onsite['operatorPresent'] and onsite['clearanceVerified'], 'm4_source_onsite_missing')
        if label == 'before':
            require(onsite['tableSupportVerified'] is True and onsite['jawContactVerified'] is
                    (scope['schemaVersion'] != core.CONTACT_PROBE_SCHEMA), 'm4_source_contact_semantics_changed')
        require(record['external'].get('root')=='external','m4_source_external_root_changed')
        ev.raw('external/'+record['external']['ingressFile'])
        ev.raw('external/host-ingress/clock-before.json')
        for suffix in ('.json','.jpg'):
            ev.raw(record['gripper']['root']+'/esp32-'+label+suffix)
        external = core._external_frame(ev.root, record['external'], scope)
        gripper = core._gripper_frame(ev.root, record['gripper'], scope, label)
        full = next((f for f in frames if f['frameId'] == external['frameId']), None)
        require(full is not None, 'm4_source_selected_frame_not_recorded')
        for frame in (external,gripper):
            require(bridge['validPythonBounds'][0] <= frame['sourceEarliest'] <= frame['sourceLatest'] <=
                    frame['persistedAt'] <= bridge['validPythonBounds'][1], 'm4_source_frame_time_order')
            if label == 'before':
                require(receipt['dispatchBefore']-scope['maxObservationAgeSeconds'] <= frame['sourceEarliest'] and
                        frame['persistedAt'] <= receipt['dispatchBefore']-.1, 'm4_source_input_after_dispatch')
            else:
                require(frame['sourceEarliest'] >= receipt['offAt']+scope['postOffSeconds'], 'm4_source_after_too_early')
        pairs[label] = dict(record=record, external=external, gripper=gripper, frame=full)
    after_record=pairs['after']['record']['gripper']
    metadata=ev.json(after_record['root']+'/esp32-after.json')
    headers={k.lower():v for k,v in metadata['headers'].items()}
    require(pairs['before']['external']['frameIndex'] < pairs['after']['external']['frameIndex'] and
            Fraction(*pairs['before']['external']['pts']) < Fraction(*pairs['after']['external']['pts']) and
            pairs['before']['gripper']['frameId'] != pairs['after']['gripper']['frameId'] and
            int(headers['x-motionbrain-capture-requested-us']) > pairs['before']['gripper']['capturedUs'], 'm4_source_frame_reused')
    require(all(sha(_read(ev.root,name)) == value for name,value in ev.hashes.items()), 'm4_source_changed_during_audit')
    return dict(sourceKind='m4_scope', scope=scope, result=result, pairs=pairs,
                execution=dict(status='verified',offVerified=True,episode=[boot,str(first)],
                               command={k:scope['pulse'][k] for k in ('joint','direction','percent','durationMs')}),
                recording=dict(status='verified'), sourceSha256=expected_result_sha,
                sourceProofs=dict(ev.hashes), physicalSuccessClaimed=False)


def _audit_no_run(ev,scope,*,allow_guarded_run=False):
    """미시작 주장도 HTTP 원본·영출력·빈 큐로 검증한다. RUN 의도 하나도 허용하지 않는다."""
    intents=sorted(ev.root.glob('http-*-intent.json'))
    require(bool(intents) and len(list(ev.root.glob('http-*')))==len(intents)*2,'m4_no_run_http_incomplete')
    entries={};statuses=[];queue=None;last_at=None;guarded_run=0
    allowed=[('/motor',{'action':'default','speed':255}),
             ('/motor',{'action':'default','speed':100}),('/command',{'cmd':'arm'}),
             ('/command',{'cmd':'stop'}),('/sequence',{'action':'clear'}),
             ('/sequence',{'action':'add','joint':'shoulder','direction':'up',
                           'speed':scope['pulse']['percent'],'duration':scope['pulse']['durationMs']})]
    for i,path in enumerate(intents,1):
        prefix=f'http-{i:06d}'
        require(path.name==prefix+'-intent.json','m4_no_run_http_gap')
        intent=ev.json(path.name)
        if not (ev.root/(prefix+'-response.json')).is_file():
            require(allow_guarded_run and guarded_run==0 and intent['path']=='/sequence' and
                    intent['payload']=={'action':'run'} and intent['method']=='POST','m4_no_run_missing_response')
            error=ev.json(prefix+'-error.json')
            require(error['error'] in ('ValueError','TimeoutError') and intent['startedAt']<=error['endedAt'] and
                    (last_at is None or last_at<=intent['startedAt']),'m4_no_run_guard_error_invalid')
            last_at=error['endedAt'];guarded_run+=1;continue
        response=ev.json(prefix+'-response.json')
        parsed=core.transport.response_json(response['rawUtf8'],intent['path'])
        require(response['httpStatus']==200 and parsed==response['parsed'] and
                intent['method']==('GET' if intent['payload'] is None else 'POST'),'m4_no_run_response_changed')
        require(intent['startedAt']<=response['receivedAt'] and
                (last_at is None or last_at<=intent['startedAt']),'m4_no_run_time_order')
        last_at=response['receivedAt']
        if intent['payload'] is not None:
            require((intent['path'],intent['payload']) in allowed and parsed.get('success') is True,
                    'm4_no_run_unexpected_write')
        route=intent['path'].split('?')[0]
        if route=='/actions':require(parsed.get('bootId')==scope['controllerBootId'],'m4_no_run_boot_changed')
        if route in ('/status','/collection_status'):statuses.append(parsed)
        if route=='/sequence' and intent['payload'] is None:queue=parsed.get('sequence')
        if route=='/action_outputs':
            require(parsed.get('bootId')==scope['controllerBootId'] and parsed.get('evidence')=='driver_write' and
                    parsed.get('clockDomain')=='controller_boot_millis' and parsed.get('gap',False) is False,
                    'm4_no_run_output_binding')
            for entry in parsed['entries']:
                require(entry['pwm']==0 and (entry['sequence'] not in entries or entries[entry['sequence']]==entry),
                        'm4_no_run_nonzero_or_changed_output')
                entries[entry['sequence']]=entry
    port=object.__new__(core.transport.SequencePort)
    port.boot_id=scope['controllerBootId'];port.last_sequence=None;port._known_entries={};port._last_output_time=None
    port.last_outputs={};port._active_pulse=None
    port._consume(dict(bootId=scope['controllerBootId'],evidence='driver_write',clockDomain='controller_boot_millis',
                       entries=[entries[n] for n in sorted(entries)]))
    require(port.last_outputs=={i:0 for i in range(1,6)} and statuses and queue and
            queue.get('state')=='IDLE' and queue.get('totalCount')==0,'m4_no_run_final_off_missing')
    port._check_status(statuses[-1],'IDLE')
    require(all(sha(_read(ev.root,name))==value for name,value in ev.hashes.items()),'m4_no_run_source_changed')


def audit_retention_source(root,audited):
    """추가 관측을 기본 원본·정지 후 시간·독립 snapshot 사본과 상호 대조한다."""
    root=Path(root).resolve();path=root/'batch-retention-captures.json'
    if not path.is_file():return None
    raw=_decode_json(_read(root,path.name))
    if raw.get('status')!='recorded':return dict(status='rejected',reason=raw.get('reason','extra_capture_incomplete'))
    config=raw.get('configuration')
    require(isinstance(config,dict) and config.get('count')==2 and
            core.transport.finite_number(config.get('intervalSeconds')) and 1<=config['intervalSeconds']<=3 and
            len(raw.get('entries',[]))==2,'m4_retention_source_configuration')
    scope=audited['scope'];off=audited['result']['receipt']['offAt']
    previous=audited['pairs']['after']['gripper'];last_end=previous['persistedAt']
    previous_external_end=audited['pairs']['after']['external']['sourceLatest']
    frames=audited['result']['observerClose']['native']['frames'];seen={previous['frameId']};entries=[]
    for index,entry in enumerate(raw['entries'],1):
        require(entry.get('snapshotRoot')==f'retention-{index:02d}','m4_retention_snapshot_path')
        snapshot=root/entry['snapshotRoot'];sources=_decode_json(_read(snapshot,'sources.json'))
        require(sources==entry['sources'],'m4_retention_snapshot_sources_changed')
        external=core._external_frame(snapshot,sources['external'],scope)
        gripper=core._gripper_frame(snapshot,sources['gripper'],scope,'after')
        full=next((f for f in frames if f['frameId']==external['frameId']),None)
        require(full is not None and external['frameId'] not in seen and gripper['frameId'] not in seen,
                'm4_retention_frame_reused_or_missing')
        origin=entry['gripperFolder']
        require(isinstance(origin,str) and (root/origin).resolve().is_relative_to((root/'gripper-source').resolve()),
                'm4_retention_original_folder')
        for suffix in ('.jpg','.json'):
            require(_read(snapshot,'gripper/esp32-after'+suffix)==_read(root,origin+'/esp32-frame'+suffix),
                    'm4_retention_gripper_snapshot_changed')
        for name in (full['metadataFile'],full['imageFile'],sources['external']['ingressFile'],'host-ingress/clock-before.json'):
            require(_read(snapshot,'external/'+name)==_read(root,'external/'+name),'m4_retention_external_snapshot_changed')
        metadata=_decode_json(_read(snapshot,'gripper/esp32-after.json'))
        require(metadata['requestStartedAt']>=last_end+config['intervalSeconds'] and
                external['sourceEarliest']>=metadata['requestStartedAt'] and
                external['sourceEarliest']>=previous_external_end+config['intervalSeconds'] and
                external['sourceEarliest']>=off+3 and gripper['sourceEarliest']>=off+3 and
                gripper['capturedUs']>previous['capturedUs'],'m4_retention_sample_timing_changed')
        require(entry['external']==external and entry['gripper']==gripper,'m4_retention_frame_binding_changed')
        # The preview must be a rendering of these exact native pixels.
        pixels=core.decode(_read(snapshot,'external/'+full['imageFile']),full['rawByteLength'],full['pixelSha256'])
        image=np.frombuffer(pixels,np.uint8).reshape(full['height'],full['width'],4)
        preview=cv2.imdecode(np.frombuffer(_read(snapshot,'after-external.png'),np.uint8),cv2.IMREAD_UNCHANGED)
        require(preview is not None and np.array_equal(image,preview),'m4_retention_preview_changed')
        proof_names=['sources.json','after-external.png','gripper/esp32-after.jpg','gripper/esp32-after.json']+[
            'external/'+name for name in (full['metadataFile'],full['imageFile'],sources['external']['ingressFile'],'host-ingress/clock-before.json')]
        hashes={name:sha(_read(snapshot,name)) for name in proof_names}
        require(entry['snapshotHashes']==hashes,'m4_retention_snapshot_hash_changed')
        entries.append(copy.deepcopy(entry));seen.update((external['frameId'],gripper['frameId']))
        previous,last_end=gripper,metadata['persistedAt']
        previous_external_end=external['sourceLatest']
    return dict(status='recorded',configuration=config,entries=entries,physicalOutcome='unreviewed',
                exposureSynchronizationVerified=False)


def capture_retention(observer,scope,configuration,store,*,deadline,tick,clock,initial_external=None):
    folder=Path(store.root).resolve()
    result=dict(status='collecting',configuration=configuration,entries=[])
    previous=_decode_json(_read(folder,'gripper/esp32-after.json'))
    if initial_external is None:
        record=_decode_json(_read(folder,'after-observation.json'))
        initial_external=core._external_frame(folder,record['external'],scope)
    previous_external_end=initial_external['sourceLatest']
    for i in range(1,3):
        next_at=max(previous['persistedAt'],previous_external_end)+configuration['intervalSeconds']
        observer._wait(lambda: True if clock.monotonic()>=next_at else None,deadline,tick)
        tick();record=observer.gripper.capture(deadline=deadline);observer.gripper.verify(record);tick()
        full=observer._external(max(record['sourceEarliest'],previous_external_end+configuration['intervalSeconds']),
                                min(deadline,clock.monotonic()+15),tick)
        snapshot=ArtifactStore(folder/f'retention-{i:02d}')
        ingress=f"host-ingress/{full['frameIndex']:04d}.json"
        for name in (full['metadataFile'],full['imageFile'],ingress,'host-ingress/clock-before.json'):
            snapshot._write('external/'+name,_read(observer.recorder.root,name))
        capture_folder=observer.gripper._bindings[record['frame']['frameId']]['folder']
        for suffix in ('.jpg','.json'):
            snapshot._write('gripper/esp32-after'+suffix,_read(observer.gripper.store.root,capture_folder+'/esp32-frame'+suffix))
        sources=dict(external=dict(root='external',ingressFile=ingress,
            ingressSha256=sha(_read(snapshot.root,'external/'+ingress)),
            clockSha256=sha(_read(snapshot.root,'external/host-ingress/clock-before.json'))),
            gripper=dict(root='gripper',label='after',imageSha256=sha(_read(snapshot.root,'gripper/esp32-after.jpg')),
                         metadataSha256=sha(_read(snapshot.root,'gripper/esp32-after.json'))))
        snapshot.write_json('sources.json',sources)
        pixels=core.decode(_read(snapshot.root,'external/'+full['imageFile']),full['rawByteLength'],full['pixelSha256'])
        image=np.frombuffer(pixels,np.uint8).reshape(full['height'],full['width'],4)
        ok,png=cv2.imencode('.png',image);require(ok,'m4_retention_preview_encode')
        snapshot._write('after-external.png',png.tobytes())
        external=core._external_frame(snapshot.root,sources['external'],scope)
        gripper=core._gripper_frame(snapshot.root,sources['gripper'],scope,'after')
        names=['sources.json','after-external.png','gripper/esp32-after.jpg','gripper/esp32-after.json']+[
            'external/'+name for name in (full['metadataFile'],full['imageFile'],ingress,'host-ingress/clock-before.json')]
        result['entries'].append(dict(snapshotRoot=snapshot.root.name,sources=sources,external=external,gripper=gripper,
            gripperFolder=str((observer.gripper.store.root/capture_folder).relative_to(folder)),
            snapshotHashes={name:sha(_read(snapshot.root,name)) for name in names}))
        previous=_decode_json(_read(snapshot.root,'gripper/esp32-after.json'))
        previous_external_end=external['sourceLatest']
    result['status']='recorded';store.write_json('batch-retention-captures.json',result)
    return result


def _feature(root, pair, spec, clock_id):
    frame = pair['frame']
    raw = _read(root, 'external/'+frame['imageFile'])
    require(sha(raw) == frame['imageSha256'], 'm4_feature_raw_changed')
    pixels = core.decode(raw,frame['rawByteLength'],frame['pixelSha256'])
    image = np.frombuffer(pixels,np.uint8).reshape(frame['height'],frame['width'],4)[:,:,:3].copy()
    ok,png = cv2.imencode('.png',image);require(ok,'m4_feature_encode')
    data = png.tobytes();available=frame['stdoutReadAt']+frame['persistenceMs']/1000
    metadata=dict(frameId=frame['frameId'],imageSha256=sha(data),pixelSha256=sha(image.tobytes()),
                  capturedAt=frame['sourceEarliest'],persistedAt=available,clockId=clock_id,
                  cameraIdentity=spec['cameraIdentity'])
    context={k:spec[k] for k in ('viewId','poseRegime','object','load')}
    observed=observe_frame(data,metadata,spec,context,now=available,clock_id=clock_id)
    require(observed.get('status') == 'accepted' and 'shoulder' in observed['states'],'m4_batch_feature_rejected')
    return dict(observation=observed,derivedMetadata=metadata,
                source=dict(folder=str(Path(root)/'external'),frame=frame))


def make_transition(root, audited, spec, condition):
    root=Path(root).resolve()
    spec=validate_spec(spec);scope=audited['scope'];ctx=scope['context']
    require(spec['cameraIdentity']==ctx['externalDeviceId'] and spec['viewId']==ctx['externalViewId'] and
            spec['poseRegime']==ctx['poseRegime'] and spec['object']==ctx['object'],'m4_generic_observer_context_changed')
    require(condition['command']==audited['execution']['command'],'m4_generic_command_changed')
    before=_feature(root,audited['pairs']['before'],spec,scope['hostClockId'])
    after=_feature(root,audited['pairs']['after'],spec,scope['hostClockId'])
    a,b=before['observation']['states']['shoulder'],after['observation']['states']['shoulder']
    require(a['stateFields']==b['stateFields'],'m4_generic_state_fields_changed')
    require(bool(set(a.get('nodeMatches',[])) & set(condition['fromNodes'])),'m4_generic_before_node_changed')
    row=dict(sessionId=scope['trialId'],episodeId=':'.join(audited['execution']['episode']),
             sourceHash=audited['sourceSha256'],frameIds=[before['derivedMetadata']['frameId'],after['derivedMetadata']['frameId']],
             pairedFrameIds=[audited['pairs'][label]['gripper']['frameId'] for label in ('before','after')],
             captureGroup=condition['captureGroup'],joint='shoulder',
             context={**{k:spec[k] for k in ('viewId','poseRegime','object','load')},'observerSha256':observer_digest(spec)},
             stateFields=a['stateFields'],before=a['state'],after=b['state'],command=condition['command'],
             commandAt=audited['result']['receipt']['dispatchBefore'],beforePersistedAt=before['derivedMetadata']['persistedAt'])
    supplemental=audit_retention_source(root,audited)
    if supplemental and supplemental.get('status')=='recorded':
        row['supplementalFrameIds']=[entry[camera]['frameId'] for entry in supplemental['entries'] for camera in ('external','gripper')]
    transition=dict(schemaVersion='motionbrain.batch_visual_transition.v1',sourceKind='m4_scope',
                sourceSha256=audited['sourceSha256'],scopeSha256=sha(_read(root,'scope.json')),
                observerSpec=spec,condition=copy.deepcopy(condition),conditionId=condition['id'],
                phase=condition['phase'],captureGroup=condition['captureGroup'],
                learningRow=row,before=before,after=after,controlEligible=False,modelSelected=False,
                contactState='unknown' if scope['schemaVersion']==core.CONTACT_PROBE_SCHEMA else 'confirmed_by_supplied_reviewer')
    if supplemental is not None:transition['supplementalObservations']=supplemental
    if condition['phase']=='evaluation':
        model=validate_model(_decode_json(_read(root,'batch-frozen-model.json')))
        evidence=_decode_json(_read(root,'batch-prospective.json'))
        require(evidence['beforeFrameId']==row['frameIds'][0] and
                evidence['beforePixelSha256']==audited['pairs']['before']['frame']['pixelSha256'],
                'm4_prospective_frame_changed')
        result=evaluate(model,[row],prospective={row['sessionId']:evidence['proof']})
        require(result['prospectiveEvidenceVerified'] is True,'m4_prospective_not_verified')
        transition.update(evaluation=result,modelSha256=model_sha256(model))
    return transition


def audit_transition_source(root, expected_result_sha, transition):
    require(transition.get('sourceKind')=='m4_scope','m4_transition_source_kind')
    audited=audit_source(root,expected_result_sha,transition.get('scopeSha256'))
    rebuilt=make_transition(Path(root),audited,transition['observerSpec'],transition['condition'])
    require(transition==rebuilt,'m4_batch_transition_changed')
    return copy.deepcopy(rebuilt['learningRow']),dict(sourceKind='m4_scope',sourceSha256=expected_result_sha,
                                                    sourceProofs=audited['sourceProofs'])


def load_learning_transition(repository,entry,split):
    require(isinstance(entry,dict) and set(entry)=={'sourceKind','transitionPath','transitionSha256',
            'sourcePath','sourceSha256','scopeSha256'},'m4_training_manifest_fields')
    require(entry['sourceKind']=='m4_scope' and split in ('training','evaluation'),'m4_training_source_kind')
    path=private_path(repository,entry['transitionPath']);raw=path.read_bytes()
    require(sha(raw)==entry['transitionSha256'],'m4_transition_hash_changed')
    transition=_decode_json(raw)
    require(transition.get('phase')==split and transition.get('sourceSha256')==entry['sourceSha256'] and
            transition.get('scopeSha256')==entry['scopeSha256'],'m4_transition_manifest_binding_changed')
    require(transition.get('condition',{}).get('phase')==split,'m4_transition_condition_phase_changed')
    # Generic prospective M4 support is intentionally absent. These are development rows.
    row,provenance=audit_transition_source(private_path(repository,entry['sourcePath']),entry['sourceSha256'],transition)
    provenance['transitionSha256']=entry['transitionSha256']
    return row,provenance


class BatchM4Route:
    def __init__(self,plan,repository,root,credential,reviewer_factory=None,*,observer_spec,frozen_model=None,after_result=None,before_dispatch=None,clock=time):
        self.plan,self.repository,self.root=copy.deepcopy(plan),Path(repository).resolve(),Path(root).resolve()
        require(self.root.is_relative_to((self.repository/'.local').resolve()),'m4_private_root_required')
        self.credential,self.reviewer_factory,self.spec,self.clock=credential,reviewer_factory,validate_spec(observer_spec),clock
        self.active=None
        self.after_result=after_result
        require(before_dispatch is None or callable(before_dispatch),'m4_before_dispatch_callback')
        self.before_dispatch=before_dispatch
        require(after_result is None or callable(after_result),'m4_after_result_callback')
        self.model_provider=frozen_model if callable(frozen_model) else None
        self.frozen_model=copy.deepcopy(frozen_model) if frozen_model is not None and not callable(frozen_model) else None

    def _write(self,path,value):
        path.parent.mkdir(parents=True,exist_ok=True);raw=encoded(value)
        if path.exists():require(path.read_bytes()==raw,'m4_batch_immutable_changed')
        else:atomic(path,raw)

    def execute(self,condition,request_id,binding,*,deadline_wall):
        checked=validate_binding(self.plan,condition,binding,self.repository)
        require(callable(self.reviewer_factory),'m4_explicit_reviewer_required')
        require(self.clock.time()<deadline_wall,'batch_dispatch_deadline_expired')
        if condition['phase']=='evaluation' and self.frozen_model is None and self.model_provider is not None:
            self.frozen_model=validate_model(copy.deepcopy(self.model_provider()))
        folder=self.root/'m4-scopes'/request_id
        require(not folder.exists(),'m4_batch_request_consumed')
        store=ArtifactStore(folder)
        self._write(folder/'batch-binding.json',dict(requestId=request_id,condition=condition,binding=binding,
                                                    deadlineWall=deadline_wall))
        require(condition['phase']!='evaluation' or self.frozen_model is not None,'m4_generic_frozen_model_required')
        if condition['phase']=='evaluation':
            validate_model(self.frozen_model)
            store.write_json('batch-frozen-model.json',self.frozen_model)
        reviewer=self.reviewer_factory(checked['scope'],store)
        retention=copy.deepcopy(binding.get('retentionCaptures'))
        route=self
        class RetentionObserver(M4DualCameraObserver):
            def start(owner,*,deadline):
                super().start(deadline=deadline)
                if retention is not None:
                    require(owner.gripper.max_captures==2,'m4_original_capture_budget_changed')
                    owner.gripper.max_captures=4
            def capture_retention(owner,after_off,deadline,tick,initial_external):
                return capture_retention(owner,checked['scope'],retention,store,deadline=deadline,tick=tick,clock=clock,
                                         initial_external=initial_external)

        observer_type=RetentionObserver if retention is not None else M4DualCameraObserver
        observer=observer_type(checked['scope'],checked['camera'],store,reviewer,clock=self.clock)
        clock=self.clock
        deadline_mono=clock.monotonic()+max(0.,deadline_wall-clock.time())
        route=self
        class DeadlineCommissioning(core.M4UpCommissioning):
            generic_record=None
            generic_evidence=None
            retention_collected=False
            def _guard(owner):
                require(clock.time()<deadline_wall and clock.monotonic()<deadline_mono,'batch_dispatch_deadline_expired')
                return super()._guard()
            def _observe(owner,record,*,after_off=None):
                frames=super()._observe(record,after_off=after_off)
                if after_off is None and condition['phase']=='evaluation':
                    if owner.generic_record is None:
                        f=_decode_json(_read(folder,record['external']['root']+'/'+record['external']['ingressFile']))
                        feature=_feature(folder,dict(frame=f),route.spec,checked['scope']['hostClockId'])
                        state=feature['observation']['states']['shoulder']
                        require(bool(set(state.get('nodeMatches',[])) & set(condition['fromNodes'])),
                                'm4_generic_before_node_changed')
                        row=dict(sessionId=checked['scope']['trialId'],before=state['state'],
                                 context={**{k:route.spec[k] for k in ('viewId','poseRegime','object','load')},
                                          'observerSha256':observer_digest(route.spec)},
                                 command=condition['command'],beforePersistedAt=feature['derivedMetadata']['persistedAt'])
                        proof=prospective_evidence(route.frozen_model,row,clock.monotonic())
                        evidence=dict(beforeFrameId=f['frameId'],beforePixelSha256=f['pixelSha256'],proof=proof)
                        store.write_json('batch-prospective.json',evidence)
                        owner.generic_record=copy.deepcopy(record)
                        owner.generic_evidence=copy.deepcopy(evidence)
                        if route.before_dispatch is not None:
                            route.before_dispatch(copy.deepcopy(feature),store)
                    else:
                        require(owner.generic_record==record,'m4_generic_pinned_input_changed')
                if after_off is not None and retention is not None and not owner.retention_collected:
                    owner.retention_collected=True
                    owner.observer.capture_retention(after_off,owner.deadline,lambda:owner._tick(True),frames['external'])
                return frames
            def _before_run(owner):
                owner._guard()
                if condition['phase']=='evaluation':
                    require(owner.generic_record==owner.before and owner.generic_evidence is not None,
                            'm4_generic_prediction_missing')
                result=super()._before_run()
                now=clock.monotonic()
                require(all(0<=now-frame['sourceEarliest']<=route.plan['maxObservationAgeSeconds']
                            for frame in owner.before_frames.values()),'batch_dispatch_observation_stale')
                owner._guard()
                return result
        token=self.credential();require(isinstance(token,str) and token,'m4_credential_required')
        self.active=DeadlineCommissioning(checked['scope'],observer,token,store,evidence_root=folder,clock=self.clock)
        try:
            report=self.active.run(confirmed_scope_sha256=plan_digest(checked['scope']))
            require((folder/'result.json').is_file(),'m4_result_not_durable')
            self._write(folder/'batch-dispatch-proof.json',dict(requestId=request_id,
                resultSha256=sha(_read(folder,'result.json')),scopeSha256=plan_digest(checked['scope']),
                mayHaveRun=self.active.port.may_have_run,dispatchBefore=self.active.port.last_dispatch_before,
                dispatchAfter=self.active.port.last_dispatch_after,evidenceFailed=self.active.port.evidence_failed,
                finalIdleOffVerified=report.get('finalIdleOffVerified') is True))
            result=self._classify(condition,request_id,binding,folder)
            if self.after_result is not None:
                try:
                    interaction=self.after_result(copy.deepcopy(condition),request_id,folder,copy.deepcopy(result))
                    require(isinstance(interaction,dict),'m4_after_result_report_required')
                    result['interactionAfter']=copy.deepcopy(interaction)
                except (ValueError,KeyError,TypeError,OSError,StorageError):
                    result['interactionAfter']=dict(status='unknown',reason='after_review_unavailable')
            self._write(folder/'batch-outcome.json',result)
            return result
        finally:
            self.active.port.token='';self.active=None;token=''

    def _classify(self,condition,request_id,binding,folder):
        from learning.batch_runtime import journal_clear
        report=_decode_json(_read(folder,'result.json'));result_sha=sha(_read(folder,'result.json'))
        result=dict(execution='unknown',offVerified=report.get('finalIdleOffVerified') is True,
                    journalClear=journal_clear(self.repository/'.local/visual-actions/controller-commands.sqlite'),
                    capture='failed',observation='rejected',reason='m4_scope_incomplete',
                    evidence=dict(requestId=request_id,sourceKind='m4_scope',sourcePath=str(folder),sourceSha256=result_sha))
        if any(error.get('code')=='m4_observed_excursion' for error in report.get('errors',[]) if isinstance(error,dict)):
            result.update(failureScope='joint',blockedJoint='shoulder',failureReason='m4_observed_excursion')
        try:
            require(report.get('scopeSha256')==plan_digest(core.validate_scope(_decode_json(_read(folder,'scope.json')))),
                    'm4_result_scope_changed')
            if report.get('commandOutcome')=='receipt_recorded':
                ev=Evidence(folder);scope=ev.json('scope.json')
                statuses,outputs,boot,first=_http(ev,report['receipt'],scope['pulse'])
                result['evidence']['outputSequence']=max(entry['sequence'] for entry in outputs)
                result['evidence']['hostClockId']=scope['hostClockId']
                result['execution']='completed'
            elif report.get('commandOutcome')=='not_started' and report.get('mayHaveRun') is False and result['offVerified']:
                ev=Evidence(folder);scope=ev.json('scope.json')
                proof=_decode_json(_read(folder,'batch-dispatch-proof.json'))
                require(proof.get('requestId')==request_id and proof.get('resultSha256')==result_sha and
                        proof.get('scopeSha256')==plan_digest(scope) and proof.get('mayHaveRun') is False and
                        proof.get('dispatchBefore') is None and proof.get('dispatchAfter') is None and
                        proof.get('evidenceFailed') is False and proof.get('finalIdleOffVerified') is True,
                        'm4_no_run_dispatch_proof_invalid')
                _audit_no_run(ev,scope,allow_guarded_run=True)
                result['execution']='not_dispatched'
            try:
                _native(Evidence(folder),_decode_json(_read(folder,'scope.json')),report['observerClose'])
                result['capture']='recorded'
            except (ValueError,KeyError,TypeError,OSError,StorageError):
                pass
            checked=validate_binding(self.plan,condition,binding,self.repository)
            audited=audit_source(folder,result_sha)
            result['capture']='recorded'
            require(audited['scope']==checked['scope'],'m4_source_original_scope_changed')
            transition=make_transition(folder,audited,self.spec,condition)
            identity=sha(encoded(transition))
            self._write(self.root/'transitions'/(identity+'.json'),transition)
            result.update(capture='recorded',observation='eligible',transitionId=identity,reason='m4_verified_transition',
                          prospectiveEvidenceVerified=bool(transition.get('evaluation',{}).get('prospectiveEvidenceVerified')),
                          dataIdentity={k:copy.deepcopy(transition['learningRow'][k]) for k in ('sourceHash','sessionId','episodeId','frameIds','pairedFrameIds')})
            if 'supplementalFrameIds' in transition['learningRow']:
                result['dataIdentity']['supplementalFrameIds']=copy.deepcopy(transition['learningRow']['supplementalFrameIds'])
        except (ValueError,KeyError,TypeError,OSError,StorageError):
            result['reason']='m4_scope_or_observation_rejected'
        return result

    def reconcile(self,condition,request_id,binding):
        folder=self.root/'m4-scopes'/request_id
        if not (folder/'batch-binding.json').is_file() or not (folder/'result.json').is_file():return None
        saved=_decode_json(_read(folder,'batch-binding.json'))
        require(saved['requestId']==request_id and saved['condition']==condition and saved['binding']==binding,
                'm4_batch_saved_binding_changed')
        validate_binding(self.plan,condition,binding,self.repository)
        return self._classify(condition,request_id,binding,folder)
