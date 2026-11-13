"""Explicit, single-use M4-up commissioning core. No live CLI or default observer.

General collection restrictions remain unchanged. The caller must supply current
dual-camera acquisition/review and a private runtime token; this module never
discovers devices, extracts tokens or fits a model. A separately frozen model
may select only the two explicitly registered learning candidates.
"""
from __future__ import annotations

import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import time

from learning.action_journal import BoundedActionJournal
from learning.artifacts import _decode_json, _read
from learning.gripper_camera_dataset import read_frame
from learning.lossless_frame import ENCODING, SUFFIX, decode
from learning.mac_clock_bridge import validate_sample
from learning.manual_collection_dataset import require
from learning.supervised_collection import identifier, plan_digest
from learning import supervised_transport as transport

REPO = Path(__file__).resolve().parents[1]
SCHEMA = 'motionbrain.m4_up_commissioning.v1'
NONCONTACT_SCHEMA = 'motionbrain.m4_up_noncontact.v1'
CONTACT_PROBE_SCHEMA = 'motionbrain.m4_up_contact_uncertain_probe.v1'
PULSE = dict(joint='shoulder', motor=4, direction='up', percent=16,
             durationMs=200, expectedPwm=40)
CONTACT_RESPONSE_PULSE = dict(joint='shoulder', motor=4, direction='up', percent=50,
                              durationMs=350, expectedPwm=127)
CONTACT_PROBE_PULSES = (PULSE, CONTACT_RESPONSE_PULSE,
    dict(joint='shoulder', motor=4, direction='up', percent=50, durationMs=450, expectedPwm=127))
LIFT_SCHEMA = 'motionbrain.m4_up_manual_lift.v1'
LIFT_PULSE = dict(joint='shoulder', motor=4, direction='up', percent=50,
                  durationMs=750, expectedPwm=127)
LEARNING_SCHEMA = 'motionbrain.m4_up_learning.v1'
MODEL_SCHEMA = 'motionbrain.m4_up_model_selection.v1'
LEARNING_PULSE = dict(joint='shoulder', motor=4, direction='up', percent=50,
                      durationMs=650, expectedPwm=127)
LEARNING_CANDIDATES = [PULSE, LEARNING_PULSE]
LEARNING_SHORT_SCHEMA = 'motionbrain.m4_up_learning_short500.v1'
MODEL_SHORT_SCHEMA = 'motionbrain.m4_up_model_selection_short500.v1'
SHORT_PULSE = dict(joint='shoulder', motor=4, direction='up', percent=50,
                   durationMs=500, expectedPwm=127)
SHORT_CANDIDATES = [PULSE, SHORT_PULSE]
LEARNING_SCHEMAS = (LEARNING_SCHEMA, MODEL_SCHEMA, LEARNING_SHORT_SCHEMA, MODEL_SHORT_SCHEMA)
MODEL_SCHEMAS = (MODEL_SCHEMA, MODEL_SHORT_SCHEMA)
MAX_CONTROLLER_PROOF_AGE_SECONDS = .25
PREPARED_CONTROLLER_PROOF_AGE_SECONDS = .20


def prepare_controller_proof(port, *, clock, guard, save, expected_state='ARMED'):
    """성공한 오래된 GET 근거만 최대 네 번 갱신해 최종 검사 여유를 둔다.

    실패한 GET은 전파한다. 50ms 여유는 최종 250ms 검사를 대체하거나 모든
    파일 저장·검증 시간이 들어맞는다고 보장하지 않는다. IDLE은 무출력 진단용이다.
    """
    require(expected_state in ('ARMED','IDLE'),'m4_proof_expected_state')
    for cycle in range(4):
        guard()
        port._refresh_run_proof(require_ready=False,expected_state=expected_state)
        guard();now=clock.monotonic()
        if not getattr(port,'_cursor_requires_output_read',False) and set(port.proof_started)=={'status','outputs'} and all(
                0 <= now-at <= PREPARED_CONTROLLER_PROOF_AGE_SECONDS for at in port.proof_started.values()):
            save('controller-pre-dispatch-admission.json',dict(preparedAt=now,preparationCycles=cycle+1,
                expectedState=expected_state,controllerProofStarted=copy.deepcopy(port.proof_started),
                preparedMaximumAgeSeconds=PREPARED_CONTROLLER_PROOF_AGE_SECONDS,
                finalMaximumAgeSeconds=MAX_CONTROLLER_PROOF_AGE_SECONDS))
            now=clock.monotonic()
            require(all(0 <= now-at <= PREPARED_CONTROLLER_PROOF_AGE_SECONDS for at in port.proof_started.values()),
                    'm4_controller_proof_stale')
            return
    raise ValueError('m4_controller_proof_stale')
PUBLIC_FAILURE_CODES = frozenset({
    'm4_native_pre_dispatch_unavailable', 'm4_controller_proof_stale',
    'm4_pre_run_guard_stale', 'm4_pinned_before_outside_dispatch_window',
    'm4_start_context_changed', 'm4_observed_excursion', 'm4_camera_wait_expired',
    'm4_output_cursor_invalid','m4_output_cursor_regressed','m4_output_cursor_baseline_changed'})


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def implementation_digest():
    return sha(b'\0'.join(p.read_bytes() for p in (
        Path(__file__),Path(transport.__file__),Path(__file__).with_name('m4_up_observer.py'),
        Path(__file__).with_name('m4_lift_learning.py'))))


def _hash(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def validate_scope(scope):
    is_learning = isinstance(scope,dict) and scope.get('schemaVersion') in LEARNING_SCHEMAS
    candidates = (SHORT_CANDIDATES if isinstance(scope,dict) and scope.get('schemaVersion') in
                  (LEARNING_SHORT_SCHEMA,MODEL_SHORT_SCHEMA) else LEARNING_CANDIDATES)
    is_probe = isinstance(scope, dict) and scope.get('schemaVersion') == CONTACT_PROBE_SCHEMA
    is_noncontact = isinstance(scope, dict) and scope.get('schemaVersion') == NONCONTACT_SCHEMA
    extra = ({'phase','observationContractSha256','modelPath','modelSha256'} if is_learning
             else {'contactState'} if is_probe else {'load'} if is_noncontact else set())
    require(isinstance(scope, dict) and set(scope) == extra | {
        'schemaVersion', 'trialId', 'controllerUrl', 'controllerBootId', 'hostClockId',
        'implementationSha256', 'context', 'pulse', 'defaultSpeed', 'maxCommands',
        'expectedStartAngleDeg', 'startAngleToleranceDeg', 'maxObservedDeltaDeg',
        'maxSeconds', 'maxObservationAgeSeconds', 'postOffSeconds'}, 'm4_scope_fields')
    require(scope['schemaVersion'] in (SCHEMA,LIFT_SCHEMA,CONTACT_PROBE_SCHEMA,NONCONTACT_SCHEMA)+LEARNING_SCHEMAS and identifier(scope['trialId'])
            and identifier(scope['hostClockId']) and isinstance(scope['controllerBootId'], str)
            and re.fullmatch('[0-9a-f]{32}', scope['controllerBootId']) is not None,
            'm4_scope_identity')
    transport.validate_target(scope['controllerUrl'])
    require(scope['implementationSha256'] == implementation_digest(), 'm4_implementation_changed')
    profile = (next((p for p in candidates if p==scope['pulse']),{}) if is_learning
               else next((p for p in CONTACT_PROBE_PULSES if p==scope['pulse']),{}) if is_probe
               else PULSE if scope['schemaVersion'] in (SCHEMA, NONCONTACT_SCHEMA) else LIFT_PULSE)
    if is_probe:
        require(scope['contactState'] == 'unknown', 'm4_probe_unknown_contact_required')
    if is_learning:
        require(scope['pulse'] in candidates and _hash(scope['observationContractSha256']), 'm4_learning_candidates_required')
        require(scope['phase'] in ('training','evaluation','model_selection')
                and (scope['schemaVersion'] in MODEL_SCHEMAS)==(scope['phase']=='model_selection'), 'm4_learning_phase')
        require(scope['schemaVersion'] not in MODEL_SCHEMAS or scope['pulse']==candidates[1],'m4_model_ceiling_required')
        if scope['phase']=='training':
            require(scope['modelPath'] is None and scope['modelSha256'] is None,'m4_training_no_model_binding')
        else:
            require(_hash(scope['modelSha256']) and isinstance(scope['modelPath'],str)
                    and Path(scope['modelPath']).resolve().is_relative_to((REPO/'.local').resolve()),'m4_frozen_model_binding')
    require(bool(profile) and scope['pulse'] == profile and all(type(scope['pulse'][k]) is type(v) for k, v in profile.items())
            and type(scope['defaultSpeed']) is int and scope['defaultSpeed'] == 255
            and type(scope['maxCommands']) is int and scope['maxCommands'] == 1,
            'm4_exact_single_pulse_required')
    for k, value in [('startAngleToleranceDeg', .5), ('maxObservedDeltaDeg', 3. if is_learning else 1.),
                     ('maxSeconds', 60.), ('maxObservationAgeSeconds', 10.), ('postOffSeconds', 3.)]:
        require(transport.finite_number(scope[k]) and scope[k] == value, 'm4_fixed_limits_required')
    require(transport.finite_number(scope['expectedStartAngleDeg'])
            and 127.58 < scope['expectedStartAngleDeg'] < 296.12, 'm4_start_angle_invalid')
    if scope['schemaVersion'] in (LEARNING_SHORT_SCHEMA, MODEL_SHORT_SCHEMA):
        require(192.61 <= scope['expectedStartAngleDeg'] <= 197.61, 'm4_short_reference_band')
    ctx = scope['context']
    require(isinstance(ctx, dict) and set(ctx) == {
        'physicalView', 'poseRegime', 'object', 'supportMode', 'externalDeviceId',
        'externalViewId', 'gripperCameraBootId'} and
        ctx['supportMode'] == ('empty_no_object' if is_noncontact else 'table_supported')
        and all(isinstance(v, str) and bool(v) for v in ctx.values())
        and re.fullmatch('[0-9a-f]{32}', ctx['gripperCameraBootId']) is not None,
        'm4_current_contact_context_required')
    if is_noncontact:
        require(ctx['object'] == 'none' and scope['load'] == 'empty', 'm4_noncontact_empty_no_object_required')
    return copy.deepcopy(scope)


# 이전 scope·proof 준비 구현의 원본 재감사 전용이다.
# 실행 승인·현재 구현 검사를 대체하지 않는다.
RECORDED_IMPLEMENTATION_DIGESTS = frozenset({
    'ed3333408c4662d442c7bd61bec579c1da16dd8e9eae79448f261f57d9b249ba',
    '4848a96eb9411bf486fa3bc662fafa3687e723910a59e1d6032a25e7c131f179',
    '4c4b01c097b5f34b2db7cfdcac147462f398698b0827b09c07996b7f3127f942',
})


def validate_recorded_scope(scope):
    """알려진 이전 원본만 현재의 동일 안전 계약으로 검사하고 원본을 반환한다."""
    require(isinstance(scope, dict), 'm4_scope_fields')
    if scope.get('implementationSha256') == implementation_digest():
        return validate_scope(scope)
    require(scope.get('implementationSha256') in RECORDED_IMPLEMENTATION_DIGESTS and not
            (scope.get('schemaVersion') == NONCONTACT_SCHEMA and scope.get('implementationSha256') ==
             'ed3333408c4662d442c7bd61bec579c1da16dd8e9eae79448f261f57d9b249ba'),
            'm4_recorded_implementation_unknown')
    candidate = copy.deepcopy(scope)
    candidate['implementationSha256'] = implementation_digest()
    validate_scope(candidate)
    return copy.deepcopy(scope)


def _external_frame(root, source, scope):
    require(isinstance(source, dict) and set(source) == {'root', 'ingressFile', 'ingressSha256', 'clockSha256'}
            and _hash(source['ingressSha256']) and _hash(source['clockSha256']), 'm4_external_source_fields')
    prefix = source['root']+'/'
    raw = _read(root, prefix+source['ingressFile'])
    require(sha(raw) == source['ingressSha256'], 'm4_external_ingress_changed')
    f = _decode_json(raw)
    require(type(f.get('frameIndex')) is int and f['frameIndex'] >= 0
            and source['ingressFile'] == f"host-ingress/{f['frameIndex']:04d}.json"
            and f['frameId'] == f"{scope['hostClockId']}:{f['frameIndex']}"
            and f['qualityFlags'] == [] and f['metadataFile'] == f"frame-{f['frameIndex']:04d}.json"
            and f['imageFile'] == f"frame-{f['frameIndex']:04d}{SUFFIX}",
            'm4_external_frame_invalid')
    meta_raw = _read(root, prefix+f['metadataFile']); image = _read(root, prefix+f['imageFile'])
    require(sha(meta_raw) == f['metadataSha256'] and sha(image) == f['imageSha256'], 'm4_external_source_changed')
    meta = _decode_json(meta_raw)
    require(meta.get('schemaVersion') == 'motionbrain.mac_camera_frame.v2'
            and meta.get('clockDomain') == 'coremedia_host_clock'
            and meta.get('timestampSemantics') == 'cmsamplebuffer_presentation_timestamp'
            and meta.get('storageEncoding') == ENCODING and f.get('storageEncoding') == ENCODING
            and meta.get('sessionId') == scope['hostClockId'] and meta.get('sourceFrameId') == f['frameId']
            and type(meta.get('frameIndex')) is int and meta['frameIndex'] == f['frameIndex']
            and meta.get('deviceUniqueId') == scope['context']['externalDeviceId']
            and meta.get('viewConfigurationId') == scope['context']['externalViewId']
            and meta.get('imageFile') == f['imageFile'] and meta.get('imageSha256') == f['imageSha256']
            and meta.get('pixelSha256') == f['pixelSha256'] and meta.get('rawByteLength') == f['rawByteLength']
            and type(f['width']) is int and type(f['height']) is int
            and 1 <= f['width'] <= 1920 and 1 <= f['height'] <= 1080
            and meta.get('width') == f['width'] and meta.get('height') == f['height']
            and meta.get('pixelFormat') == 'BGRA8' and meta.get('rowBytes') == f['width']*4
            and type(meta.get('videoMirrored')) is bool and transport.finite_number(meta.get('videoRotationAngle'))
            and len(image) == meta.get('storedByteLength') == f.get('storedByteLength')
            and f['rawByteLength'] == f['width']*f['height']*4, 'm4_external_metadata_invalid')
    pts = meta.get('pts', {})
    require(isinstance(pts, dict) and all(type(pts.get(k)) is int for k in ('value','timescale','epoch','flags'))
            and pts['value'] >= 0 and pts['timescale'] > 0 and pts['epoch'] == 0
            and pts['flags'] & 1 and not pts['flags'] & ~3, 'm4_external_pts_invalid')
    native, callback = meta.get('convertedHostPtsSeconds'), meta.get('callbackHostSeconds')
    persistence, read_at = f.get('persistenceMs'), f.get('stdoutReadAt')
    require(all(transport.finite_number(v) for v in (native, callback, persistence, read_at))
            and 0 <= native <= callback and 0 <= persistence <= 250
            and f.get('convertedHostPtsSeconds') == native and f.get('callbackHostSeconds') == callback,
            'm4_external_receipt_invalid')
    require(isinstance(meta.get('actualConfigurationJson'), str)
            and sha(meta['actualConfigurationJson'].encode()) == meta.get('configurationSha256'),
            'm4_external_configuration_invalid')
    decode(image, f['rawByteLength'], f['pixelSha256'])
    clock_raw = _read(root, prefix+'host-ingress/clock-before.json')
    require(sha(clock_raw) == source['clockSha256'], 'm4_external_clock_changed')
    clock = _decode_json(clock_raw); low, high = validate_sample(clock)
    require(clock['hostClockId'] == scope['hostClockId'] and clock['pairs'][-1]['after'] <= f['sourceEarliest']
            and f['clockOffsetBoundsSeconds'] == [low, high]
            and f['sourceEarliest'] == meta['convertedHostPtsSeconds']+low
            and f['sourceLatest'] == meta['convertedHostPtsSeconds']+high
            and f['sourceLatest'] <= read_at+.01 and 0 <= read_at-f['sourceEarliest'] <= 2
            and clock['pairs'][-1]['after'] <= f['sourceEarliest'] <= clock['pairs'][-1]['after']+120,
            'm4_external_clock_binding')
    return dict(frameId=f['frameId'], frameIndex=f['frameIndex'], pts=[pts['value'],pts['timescale']],
                pixelSha256=f['pixelSha256'], width=f['width'], height=f['height'],
                videoMirrored=meta['videoMirrored'], videoRotationAngle=meta['videoRotationAngle'],
                configurationSha256=meta['configurationSha256'],
                sourceEarliest=f['sourceEarliest'], sourceLatest=f['sourceLatest'],
                persistedAt=f['stdoutReadAt']+f['persistenceMs']/1000)


def _gripper_frame(root, source, scope, label):
    require(isinstance(source, dict) and set(source) == {'root', 'label', 'metadataSha256', 'imageSha256'}
            and source['label'] == label, 'm4_gripper_source_fields')
    prefix = source['root']+'/'
    meta = _read(root, prefix+f'esp32-{label}.json'); image = _read(root, prefix+f'esp32-{label}.jpg')
    require(sha(meta) == source['metadataSha256'] and sha(image) == source['imageSha256'], 'm4_gripper_source_changed')
    location = (root/source['root']).resolve(strict=True)
    require(location.is_relative_to(root), 'm4_private_source_required')
    frame, receipt, boot = read_frame(location, label, scope['hostClockId'])
    require(boot == scope['context']['gripperCameraBootId'] and frame['imageSha256'] == sha(image),
            'm4_gripper_boot_or_bytes_changed')
    require(sha(_read(root, prefix+f'esp32-{label}.json')) == sha(meta), 'm4_gripper_receipt_changed')
    return dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'], capturedUs=frame['capturedUs'],
                width=frame['width'], height=frame['height'],
                sourceEarliest=receipt['requestStartedAt'], sourceLatest=receipt['receivedAt'],
                persistedAt=receipt['persistedAt'])


class _CommissioningPort(transport.SequencePort):
    def __init__(self, scope, token, store, owner, *, clock):
        super().__init__(scope['controllerUrl'], token, scope['controllerBootId'], store,
                         clock=clock, compact_reads=True)
        self.owner = owner
        self.proving_off = False
        self.proof_started = {}
        self.proof_refresh_count = 0
        self._driver_cursor_baseline = None
        self._cursor_requires_output_read = False
        self._cursor_baseline_count = self._cursor_evidence_count = 0

    def execute(self, *args, **kwargs):
        raise ValueError('m4_use_scoped_commissioning_only')

    def _execute_validated(self, pulse, tick, *, before_run=None):
        self.owner._guard()
        require(self.owner.in_pulse and pulse == self.owner.selected_pulse, 'm4_scope_not_active')
        return super()._execute_validated(pulse, tick, before_run=before_run)

    def _call(self, path, payload=None, **kwargs):
        if not kwargs.get('emergency'):
            self.owner._guard()
            if kwargs.get('run'):
                require(path == '/sequence' and payload == {'action': 'run'}
                        and self.owner.in_pulse and not self.may_have_run,
                        'm4_refresh_run_scope')
                self.owner._prepare_dispatch_inputs()
                self.owner._guard()
            remaining = self.owner.deadline-self.clock.monotonic()
            kwargs['request_timeout'] = min(kwargs.get('request_timeout', remaining), remaining)
        return super()._call(path, payload, **kwargs)

    def _refresh_run_proof(self, *, require_ready=True, expected_state='ARMED'):
        # Successful reads can age during another HTTP round trip. Refresh only
        # one older successful proof per bounded preparation cycle, before the
        # durable RUN intent and final guard. Failed reads are never retried.
        require(expected_state=='ARMED' or
                (expected_state=='IDLE' and getattr(self,'read_only_proof',False) is True),
                'm4_proof_expected_state')
        require(set(self.proof_started) == {'status', 'outputs'}, 'm4_controller_proof_stale')
        now = self.clock.monotonic()
        if not self._cursor_requires_output_read and all(0 <= now-at <= .15 for at in self.proof_started.values()):
            return
        before = copy.deepcopy(self.proof_started)
        self.proof_refresh_count += 1
        require(self.proof_refresh_count <= 4, 'm4_proof_refresh_budget')
        oldest = 'outputs' if self._cursor_requires_output_read else min(before, key=before.get)
        if oldest == 'status':
            self._check_status(self._status(), expected_state)
        else:
            self._outputs(quiet=True)
            require(self.last_outputs == {i: 0 for i in range(1, 6)}, 'driver_off_not_proven')
        refresh_file = ('controller-proof-refresh.json' if self.proof_refresh_count == 1
                        else f'controller-proof-refresh-{self.proof_refresh_count:02d}.json')
        self._save(refresh_file, dict(
            refreshed=oldest, before=before, after=copy.deepcopy(self.proof_started),
            recordedAt=self.clock.monotonic(), maximumProofAgeSeconds=MAX_CONTROLLER_PROOF_AGE_SECONDS))
        if require_ready:
            self.owner._require_fresh_controller_proof(self.clock.monotonic())

    def _snapshot(self, *, emergency=False):
        old = self.proving_off; self.proving_off = emergency
        try: return super()._snapshot(emergency=emergency)
        finally: self.proving_off = old

    def _status(self, **kwargs):
        started = self.clock.monotonic()
        result = super()._status(**kwargs)
        self.proof_started['status'] = started
        if 'outputProof' in result:
            cursor=result['outputProof']
            require(isinstance(cursor,dict) and set(cursor)=={'schemaVersion','bootId','sequenceBefore','sequenceAfter','stable'}
                    and cursor.get('schemaVersion')=='driver_cursor.v1' and cursor.get('bootId')==self.boot_id
                    and cursor.get('stable') is True
                    and all(type(cursor.get(key)) is int and 0 < cursor[key] <= 4294967295
                            for key in ('sequenceBefore','sequenceAfter'))
                    and cursor['sequenceBefore']==cursor['sequenceAfter'],'m4_output_cursor_invalid')
            baseline=self._driver_cursor_baseline
            if baseline is not None and not self.may_have_run and not kwargs.get('emergency'):
                sequence=cursor['sequenceAfter']
                require(sequence>=baseline['latestSequence'],'m4_output_cursor_regressed')
                if sequence==baseline['latestSequence']:
                    require(baseline['bootId']==self.boot_id and baseline['complete'] is True and
                            self.last_sequence==sequence and self.last_outputs==baseline['lastOutputs']=={i:0 for i in range(1,6)},
                            'm4_output_cursor_baseline_changed')
                    baseline_raw=_read(self.store.root,baseline['evidenceFile'])
                    require(sha(baseline_raw)==baseline['evidenceSha256'] and
                            plan_digest(_decode_json(baseline_raw))==plan_digest({k:v for k,v in baseline.items()
                                if k not in ('evidenceFile','evidenceSha256')}),
                            'm4_output_cursor_baseline_changed')
                    self._cursor_evidence_count+=1
                    self._save(f'controller-output-cursor-{self._cursor_evidence_count:03d}.json',dict(
                        evidence='same_boot_complete_driver_history_cursor',statusHttpSerial=self.serial,
                        statusRequestStartedAt=started,outputProof=copy.deepcopy(cursor),
                        historyEvidenceFile=baseline['evidenceFile'],historyEvidenceSha256=baseline['evidenceSha256']))
                    self.proof_started['outputs']=started
                    self._cursor_requires_output_read=False
                else:
                    # A newer cursor can contain 0→nonzero→0. Never forget it
                    # just because current status says every motor is OFF.
                    self._cursor_requires_output_read=True
        return result

    def _outputs(self, **kwargs):
        started = self.clock.monotonic()
        first_serial=self.serial+1
        bootstrap=self.last_sequence is None
        result = super()._outputs(**kwargs)
        self.proof_started['outputs'] = started
        payload,new=result
        if (not self.may_have_run and not kwargs.get('emergency') and self.compact_reads
                and payload.get('hasMore') is False and payload.get('gap') is False
                and self.last_outputs=={i:0 for i in range(1,6)}
                and (bootstrap or not any(entry['pwm']!=0 for entry in new))):
            self._cursor_baseline_count+=1
            name=f'controller-output-history-baseline-{self._cursor_baseline_count:03d}.json'
            value=dict(bootId=self.boot_id,latestSequence=self.last_sequence,lastOutputs=copy.deepcopy(self.last_outputs),
                firstHttpSerial=first_serial,lastHttpSerial=self.serial,complete=True,requestStartedAt=started)
            self._save(name,value)
            self._driver_cursor_baseline=dict(value,evidenceFile=name,evidenceSha256=sha(_read(self.store.root,name)))
            self._cursor_requires_output_read=False
        return result

    def _check_status(self, status, expected, pulse=None, baseline_angle=None):
        super()._check_status(status, expected, pulse, baseline_angle)
        angle = self._shoulder_angle(status)
        self.owner.angles.append(dict(at=self.clock.monotonic(), angleDeg=angle, state=status['state']))
        if self.proving_off: return  # Original OFF proof stays distinct from a local excursion failure.
        scope = self.owner.scope
        if not self.may_have_run:
            require(abs(angle-scope['expectedStartAngleDeg']) <= scope['startAngleToleranceDeg'], 'm4_start_context_changed')
        if self.owner.baseline_angle is not None:
            delta = angle-self.owner.baseline_angle
            require(abs(delta) <= scope['maxObservedDeltaDeg'] and delta >= -.25, 'm4_observed_excursion')


class M4UpCommissioning:
    """One bounded identification, frozen evaluation or model-selected lift attempt.

    Observer: start(deadline), observe(after_off,deadline,tick), latest(), check(),
    close(). observe returns a durable review plus actual native/gripper sources;
    latest/check must be nonblocking and must never capture or perform HTTP.
    Only run() acquires the fixed shared journal and may contact a controller.
    """
    def __init__(self, scope, observer, token, store, *, evidence_root, clock=time, registration=None, model=None):
        self.scope = validate_scope(scope); self.digest = plan_digest(self.scope)
        self.selection_kind = ('manual_lift_fixed_profile' if self.scope['schemaVersion']==LIFT_SCHEMA
                               else 'manual_commissioning_fixed_profile')
        if self.scope['schemaVersion'] == CONTACT_PROBE_SCHEMA:
            self.selection_kind = 'manual_contact_uncertain_probe_fixed_profile'
        if self.scope.get('schemaVersion') == NONCONTACT_SCHEMA:
            self.selection_kind = 'manual_noncontact_fixed_profile'
        self.learning = self.scope['schemaVersion'] in LEARNING_SCHEMAS
        self.model_selection = self.scope['schemaVersion'] in MODEL_SCHEMAS
        self.candidates = (SHORT_CANDIDATES if self.scope['schemaVersion'] in
                           (LEARNING_SHORT_SCHEMA,MODEL_SHORT_SCHEMA) else LEARNING_CANDIDATES)
        self.registration,self.model=copy.deepcopy(registration),model
        self.selected_pulse=copy.deepcopy(self.scope['pulse'])
        self.measurement_records=[];self.model_decision=None;self.before_frames=None
        if self.learning:
            from learning.m4_lift_learning import contract_sha
            require(registration is not None and contract_sha(registration)==self.scope['observationContractSha256'],'m4_observation_contract_changed')
            require((model is None)==(self.scope['phase']=='training'),'m4_frozen_model_required')
            if model is not None:
                require(str(Path(model.path).resolve())==str(Path(self.scope['modelPath']).resolve())
                        and model.sha==self.scope['modelSha256'],'m4_model_binding_changed')
                require(contract_sha(model.registration)==self.scope['observationContractSha256'],'m4_model_registration_changed')
                if self.scope['schemaVersion'] in (LEARNING_SHORT_SCHEMA,MODEL_SHORT_SCHEMA):
                    from learning.m4_lift_learning import SHORT_SCHEMA
                    require(model.model['schema']==SHORT_SCHEMA,'m4_short_model_version_required')
                else:
                    from learning.m4_lift_learning import SCHEMA as LEGACY_MODEL_SCHEMA
                    require(getattr(model,'model',{}).get('schema',LEGACY_MODEL_SCHEMA)==LEGACY_MODEL_SCHEMA,'m4_legacy_model_version_required')
                model.verify()
            self.selection_kind={'training':'manual_training_fixed_profile','evaluation':'fixed_model_evaluation_fixed_profile',
                                 'model_selection':'model_selected_lift'}[self.scope['phase']]
        self.observer, self.store, self.clock = observer, store, clock
        self.root = Path(evidence_root).resolve(strict=True)
        require(self.root.is_relative_to((REPO/'.local').resolve())
                and Path(store.root).resolve().is_relative_to((REPO/'.local').resolve()), 'm4_private_evidence_required')
        self.port = _CommissioningPort(self.scope, token, store, self, clock=clock)
        self.used = self.in_pulse = False
        self.journal = None; self.baseline_angle = None; self.angles = []

    def _guard(self):
        require(self.clock.monotonic() < self.deadline, 'm4_trial_deadline')
        require(plan_digest(self.scope) == self.digest
                and self.scope['implementationSha256'] == implementation_digest(), 'm4_scope_or_code_changed')
        if self.learning:
            from learning.m4_lift_learning import contract_sha
            require(contract_sha(self.registration)==self.scope['observationContractSha256'],'m4_registration_changed')
            if self.model is not None:
                require(str(Path(self.model.path).resolve())==str(Path(self.scope['modelPath']).resolve())
                        and self.model.sha==self.scope['modelSha256'],'m4_model_binding_changed')
                require(contract_sha(self.model.registration)==self.scope['observationContractSha256'],'m4_model_registration_changed')
                self.model.verify()
                if self.scope['schemaVersion'] in (LEARNING_SHORT_SCHEMA,MODEL_SHORT_SCHEMA) and self.model_decision is not None:
                    require(self.model_decision.get('angleEvidenceSha256')==self.model.angle_evidence_sha256,
                            'm4_angle_decision_binding_changed')
            require(self.selected_pulse in self.candidates,'m4_selected_pulse_not_supported')
            if not self.model_selection:require(self.selected_pulse==self.scope['pulse'],'m4_fixed_pulse_changed')
            elif self.model_decision is not None:
                require(all(self.selected_pulse[k]==self.model_decision['pulse'][k]
                            for k in ('joint','direction','percent','durationMs')),'m4_model_selection_changed')
        require(self.journal is not None and self.journal.readiness()['ready'], 'm4_owner_missing')
        rows = self.journal.connection.execute("SELECT session_id,plan_hash,output_path FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
        require(rows == [(self.scope['trialId'], self.digest, str(self.store.root))], 'm4_owner_changed')

    def _observe(self, record, *, after_off=None):
        require(isinstance(record, dict) and set(record) == {
            'hostClockId', 'context', 'external', 'gripper', 'stable', 'onsite', 'evidenceFile', 'evidenceSha256'},
            'm4_dual_observation_required')
        require(record['hostClockId'] == self.scope['hostClockId'] and record['context'] == self.scope['context']
                and record['stable'] is True and _hash(record['evidenceSha256']), 'm4_observation_context_changed')
        raw = _read(self.root, record['evidenceFile'])
        require(len(raw) <= 65536 and sha(raw) == record['evidenceSha256']
                and _decode_json(raw) == {k: v for k, v in record.items() if k not in ('evidenceFile','evidenceSha256')},
                'm4_review_changed')
        onsite = record['onsite']
        require(isinstance(onsite, dict) and set(onsite) == {'operatorPresent', 'clearanceVerified', 'jawContactVerified', 'tableSupportVerified'}
                and all(type(v) is bool for v in onsite.values()) and onsite['operatorPresent'] and onsite['clearanceVerified'],
                'm4_onsite_review_required')
        if self.scope.get('schemaVersion') == NONCONTACT_SCHEMA:
            require(not onsite['jawContactVerified'] and not onsite['tableSupportVerified'],
                    'm4_noncontact_review_required')
        elif after_off is None:
            if self.scope['schemaVersion'] == CONTACT_PROBE_SCHEMA:
                require(onsite['tableSupportVerified'] is True and onsite['jawContactVerified'] is False,
                        'm4_probe_requires_table_support_and_unconfirmed_contact')
            else:
                require(onsite['jawContactVerified'] and onsite['tableSupportVerified'], 'm4_initial_contact_required')
        frames = {'external': _external_frame(self.root, record['external'], self.scope),
                  'gripper': _gripper_frame(self.root, record['gripper'], self.scope, 'before' if after_off is None else 'after')}
        for frame in frames.values():
            times = [frame[k] for k in ('sourceEarliest','sourceLatest','persistedAt')]
            now = self.clock.monotonic()
            require(all(transport.finite_number(v) for v in times) and self.started <= times[0]
                    and times == sorted(times) and times[-1] <= now, 'm4_observation_time_invalid')
            if after_off is None:
                require(now-times[0] <= self.scope['maxObservationAgeSeconds'] and times[-1] <= now-.1,
                        'm4_before_observation_not_ready')
            else:
                require(times[0] >= after_off+self.scope['postOffSeconds'], 'm4_post_off_observation_too_early')
        return frames

    def _tick(self, idle=False):
        self._guard(); self.observer.check(); self._guard()
        if idle: self.port.check_idle()

    def _before_run(self):
        started = self.clock.monotonic(); self._tick()
        self._require_fresh_controller_proof(started)
        latest = self.observer.latest()
        require(latest == self.before, 'm4_input_changed_before_run')
        frames = self._observe(latest)
        if self.learning:
            for record in self.measurement_records:
                for name in ('external','gripper'):
                    source=record[name]
                    require(sha(_read(self.root,source['path']))==source['sha256'],'m4_measured_source_changed')
            if self.model_decision is not None:
                path=Path(self.model_decision['decisionPath'])
                require(path.resolve().is_relative_to(self.root),'m4_prediction_outside_evidence')
                raw=_read(self.root,path.resolve().relative_to(self.root).as_posix())
                require(sha(raw)==self.model_decision['decisionSha256'] and _decode_json(raw)=={
                    k:v for k,v in self.model_decision.items() if k not in ('decisionPath','decisionSha256')},'m4_prediction_changed')
        self._guard()
        self.store.write_json('pre-run-guard.json', dict(scopeSha256=self.digest, frames=frames,
            controllerProofStarted=copy.deepcopy(self.port.proof_started),
            validatedAt=self.clock.monotonic(), selectionKind=self.selection_kind))
        self._guard()
        now = self.clock.monotonic()
        require(now-started <= .1 and now < self.deadline
                and all(now-frame['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                        for frame in frames.values()), 'm4_pre_run_guard_stale')
        self._require_fresh_controller_proof(now)
        if self.learning:
            admission_started = self.clock.monotonic()
            self._require_pinned_native_window(frames['external'], admission_started)
            ready = self.observer.native_pre_window_ready()
            now = self.clock.monotonic()
            require(ready and now-admission_started <= .02, 'm4_native_pre_dispatch_unavailable')
            require(now-started <= .1 and now < self.deadline
                    and all(now-frame['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                            for frame in frames.values()), 'm4_pre_run_guard_stale')
            self._require_fresh_controller_proof(now)

    @staticmethod
    def _require_pinned_native_window(frame, now):
        require(all(at-3. <= frame['sourceEarliest'] and frame['sourceLatest'] <= at-.1
                    and frame['persistedAt'] < now for at in (now, now+.02)),
                'm4_pinned_before_outside_dispatch_window')

    def _prepare_dispatch_inputs(self):
        if not self.learning:
            prepare_controller_proof(self.port,clock=self.clock,guard=self._guard,save=self.store.write_json)
            return
        for cycle in range(4):
            self._require_pinned_native_window(self.before_frames['external'], self.clock.monotonic())
            self.observer.wait_pre_dispatch_window(
                deadline=min(self.deadline, self.clock.monotonic()+1.), tick=self._tick,
                horizon_seconds=.06)
            self._require_pinned_native_window(self.before_frames['external'], self.clock.monotonic())
            # Refresh successful but aged reads, not failed requests. Any read
            # or source error propagates immediately; it does not enter another preparation cycle.
            self.port._refresh_run_proof(require_ready=False)
            self._guard()
            now = self.clock.monotonic()
            proof_ready = not self.port._cursor_requires_output_read and all(0 <= now-at <= .20 for at in self.port.proof_started.values())
            if proof_ready and self.observer.native_pre_window_ready(horizon_seconds=.06):
                self.store.write_json('native-pre-dispatch-admission.json', dict(
                    preparedAt=self.clock.monotonic(), preparationCycles=cycle+1,
                    scopeSha256=self.digest, preparationHorizonSeconds=.06, finalHorizonSeconds=.02,
                    policy='original_pre_window_and_current_availability'))
                self._require_fresh_controller_proof(self.clock.monotonic())
                return
        raise ValueError('m4_native_pre_dispatch_unavailable')

    def _measure(self,label,record,frames):
        from learning.m4_lift_learning import measure, validate_before
        import cv2,numpy as np
        binding=dict(external={k:frames['external'][k] for k in ('configurationSha256','width','height','videoMirrored','videoRotationAngle')},
                     gripper={k:frames['gripper'][k] for k in ('width','height')})
        require(binding==self.registration.get('cameraBinding'),'m4_registered_camera_changed')
        name=f'{label}-external.png';raw=_read(self.root,name)
        pixels=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_UNCHANGED)
        require(pixels is not None and pixels.shape==(frames['external']['height'],frames['external']['width'],4)
                and sha(pixels.tobytes())==frames['external']['pixelSha256'],'m4_preview_not_original_pixels')
        grip=record['gripper'];grip_path=grip['root']+f'/esp32-{label}.jpg'
        source=dict(id=self.scope['hostClockId']+':'+label,context=copy.deepcopy(self.scope['context']),
                    external=dict(path=name,sha256=sha(raw)),gripper=dict(path=grip_path,sha256=grip['imageSha256']))
        features=measure(self.root,source,self.registration)
        if label=='before':validate_before(features,self.registration)
        self.store.write_json(label+'-measurement-source.json',source)
        self.store.write_json(label+'-measurement.json',features)
        self.measurement_records.append(source)
        return source,features

    def _require_fresh_controller_proof(self, now):
        require(not self.port._cursor_requires_output_read and set(self.port.proof_started) == {'status','outputs'}
                and all(0 <= now-at <= MAX_CONTROLLER_PROOF_AGE_SECONDS
                        for at in self.port.proof_started.values()), 'm4_controller_proof_stale')

    def run(self, *, confirmed_scope_sha256):
        require(not self.used and confirmed_scope_sha256 == self.digest, 'm4_exact_unconsumed_scope_required')
        self.used = True; self.started = self.clock.monotonic(); self.deadline = self.started+self.scope['maxSeconds']
        report = dict(schemaVersion=self.scope['schemaVersion'], scopeSha256=self.digest, status='failed', errors=[],
            selectionKind=self.selection_kind, mayHaveRun=False,
            finalIdleOffVerified=False, trainingEligible=False, modelSelected=False, graspVerified=False, liftVerified=False)
        if self.scope['schemaVersion'] == CONTACT_PROBE_SCHEMA:
            report.update(contactStateBefore='unknown', trialKind='contact_uncertain_probe')
        if self.scope.get('schemaVersion') == NONCONTACT_SCHEMA:
            report.update(contactStateBefore='noncontact_empty', trialKind='noncontact_position')
        observer_started = claimed = False; phase = 'journal'
        try:
            self.journal = BoundedActionJournal(str(REPO/'.local/visual-actions/controller-commands.sqlite'), 10000)
            require(self.journal.readiness()['ready'], 'm4_shared_journal_unresolved')
            db = self.journal.connection
            with db:
                db.execute('CREATE TABLE IF NOT EXISTS supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT NOT NULL, output_path TEXT NOT NULL, phase TEXT NOT NULL, result_json TEXT)')
                require(not db.execute("SELECT 1 FROM supervised_blocks WHERE phase IN ('pending','off_unverified') LIMIT 1").fetchone(), 'm4_shared_block_unresolved')
                require(not db.execute('SELECT 1 FROM supervised_blocks WHERE session_id=?', (self.scope['trialId'],)).fetchone(), 'm4_scope_consumed')
                require(db.execute('SELECT COUNT(*) FROM supervised_blocks').fetchone()[0] < 10000, 'm4_scope_capacity')
                db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,NULL)', (self.scope['trialId'],self.digest,str(self.store.root),'pending'))
            claimed = True
            self.store.write_json('scope.json', self.scope)
            phase = 'preflight'; snapshot = self.port.preflight()
            self.baseline_angle = self.port._shoulder_angle(snapshot['status'])
            phase = 'before_observation'; observer_started = True
            self.observer.start(deadline=self.deadline)
            if self.learning:self.observer.wait_pre_window(deadline=self.deadline,tick=lambda:self._tick(True))
            self.before = self.observer.observe(after_off=None, deadline=self.deadline, tick=lambda: self._tick(True))
            before_frames = self._observe(self.before); self._guard()
            self.before_frames = copy.deepcopy(before_frames)
            self.store.write_json('before-observation.json', self.before)
            if self.learning:
                source,features=self._measure('before',self.before,before_frames)
                report['beforeFeatures']=features
                if self.model is not None:
                    self.model_decision=(self.model.select(source) if self.model_selection
                                         else self.model.predict(source,{k:self.scope['pulse'][k] for k in ('joint','direction','percent','durationMs')}))
                    require(self.model_decision.get('modelSha256')==self.scope['modelSha256']
                            and self.model_decision.get('contractSha256')==self.scope['observationContractSha256']
                            and self.model_decision.get('modelSelected') is self.model_selection, 'm4_prediction_binding_changed')
                    if not self.model_selection:
                        require(self.model_decision['pulse']=={k:self.scope['pulse'][k] for k in ('joint','direction','percent','durationMs')},'m4_evaluation_pulse_changed')
                    if self.model_selection:
                        pulse=self.model_decision['pulse'];selected=next((p for p in self.candidates
                            if all(p[k]==pulse[k] for k in ('joint','direction','percent','durationMs'))),None)
                        require(selected is not None,'m4_model_unsupported_selection')
                        self.selected_pulse=copy.deepcopy(selected)
                    report['modelDecision']=copy.deepcopy(self.model_decision)
                report.update(selectedPulse=self.selected_pulse,modelSelected=self.model_selection)
            phase = 'pulse'; self.in_pulse = True
            try: receipt = self.port._execute_validated(copy.deepcopy(self.selected_pulse), self._tick, before_run=self._before_run)
            finally: self.in_pulse = False
            report['receipt'] = receipt
            report['modelSelectedCommandExecuted']=bool(self.learning and self.model_selection and self.port.may_have_run)
            phase = 'after_observation'
            after = self.observer.observe(after_off=receipt['offAt'], deadline=self.deadline, tick=lambda: self._tick(True))
            frames = self._observe(after, after_off=receipt['offAt'])
            require(all(frames[k]['frameId'] != before_frames[k]['frameId'] for k in frames)
                    and all([frames[k]['width'], frames[k]['height']] == [before_frames[k]['width'], before_frames[k]['height']] for k in frames)
                    and frames['external']['configurationSha256'] == before_frames['external']['configurationSha256']
                    and all(frames['external'][k] == before_frames['external'][k] for k in ('videoMirrored','videoRotationAngle'))
                    and frames['external']['frameIndex'] > before_frames['external']['frameIndex']
                    and Fraction(*frames['external']['pts']) > Fraction(*before_frames['external']['pts'])
                    and frames['gripper']['capturedUs'] > before_frames['gripper']['capturedUs'], 'm4_duplicate_or_reversed_after')
            self.store.write_json('after-observation.json', after); self._guard()
            if self.learning:
                _,features=self._measure('after',after,frames);report['afterFeatures']=features
                self.observer.wait_post_window(receipt['offAt'],deadline=self.deadline,tick=lambda:self._tick(True))
            report['status'] = ('probe_completed_contact_unverified'
                                if self.scope['schemaVersion'] == CONTACT_PROBE_SCHEMA else 'completed')
        except (Exception, KeyboardInterrupt) as exc:
            error = dict(phase=phase, type=type(exc).__name__)
            # Only fixed machine codes, never arbitrary exception messages or
            # request headers/credentials, may be added to the private report.
            if isinstance(exc, ValueError) and str(exc) in PUBLIC_FAILURE_CODES:
                error['code'] = str(exc)
            report['errors'].append(error)
        finally:
            if claimed:
                if observer_started:
                    try:
                        report['observerClose'] = self.observer.close()
                        require(isinstance(report['observerClose'], dict) and report['observerClose'].get('status') == 'completed', 'm4_observer_close_failed')
                    except (Exception, KeyboardInterrupt) as exc:
                        report['errors'].append(dict(phase='observer_close', type=type(exc).__name__))
                try:
                    if self.port.off_verified and not self.port.evidence_failed: self.port.check_idle()
                except (Exception, KeyboardInterrupt) as exc:
                    report['errors'].append(dict(phase='final_monitor', type=type(exc).__name__))
                try: report['finalIdleOffVerified'] = self.port.finish() is True
                except (Exception, KeyboardInterrupt) as exc:
                    report['errors'].append(dict(phase='finish', type=type(exc).__name__))
                report.update(mayHaveRun=self.port.may_have_run, angleObservations=self.angles)
                if report['errors'] or not report['finalIdleOffVerified']: report['status'] = 'failed'
                report['commandOutcome'] = ('not_started' if not report['mayHaveRun'] else
                    'receipt_recorded' if 'receipt' in report else 'unknown')
                report['requiresOperatorReview'] = (report['commandOutcome'] == 'unknown'
                                                    or not report['finalIdleOffVerified'])
                # OFF proof does not resolve a lost RUN reply or failed execution.
                # Keep the shared block pending even for a new ID until reviewed.
                terminal_phase = ('off_unverified' if not report['finalIdleOffVerified'] else
                                  'pending' if report['commandOutcome'] == 'unknown' else 'closed')
                try:
                    self.store.write_json('result.json', report)
                    with self.journal.connection:
                        self.journal.connection.execute('UPDATE supervised_blocks SET phase=?,result_json=? WHERE session_id=?',
                            (terminal_phase,json.dumps(report,allow_nan=False),self.scope['trialId']))
                finally: self.journal.close(); self.journal = None
            elif self.journal is not None:
                self.journal.close(); self.journal = None
        return report
