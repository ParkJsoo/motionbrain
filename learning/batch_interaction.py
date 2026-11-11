"""배치의 접촉·보유 검토를 기존 원본 검증 계약에 연결한다. 장치 접근 없음.

판독은 외부에서 제공한 명시적 원본 결합 검토다. 닫기·정상 출력·색상
검출만으로 접촉이나 들림을 생성하지 않는다. 캐시에는 검증한 원본 바이트를
유지하며 RUN 직전에도 기존 접촉 검토와 동일하게 봉인한 원본 해시를 확인한다.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
from types import SimpleNamespace
import time

from learning.artifacts import _read
from learning.batch_observation import digest, number, require
from learning.close_lift_trial import CloseLiftTrial
from learning.model_grasp_trial import ModelGraspTrial
from learning import m4_up_commissioning as m4
from learning.experiment_pipeline import encoded

SCHEMA = 'motionbrain.batch_interaction_review.v1'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _private(repository, value):
    require(isinstance(value, str) and bool(value), 'interaction_private_path_required')
    path = (repository/value).resolve(strict=True)
    require(path.is_relative_to((repository/'.local').resolve()), 'interaction_private_path_required')
    return path


def _read_json(path, expected_hash, *, max_bytes=1024*1024):
    raw = path.read_bytes()
    require(len(raw) <= max_bytes and sha(raw) == expected_hash, 'interaction_review_bytes_changed')
    return json.loads(raw), raw


def validate_binding(plan, condition, binding, repository, *, purpose='dispatch'):
    """파일·원본 계약 선언만 검사한다. 실제 현재 판독·촬영·HTTP를 하지 않는다."""
    repository = Path(repository).resolve()
    require(purpose in ('dispatch','label','declaration'), 'interaction_binding_purpose')
    require(isinstance(binding, dict) and binding.get('interaction') in
            ('noncontact', 'contact_probe', 'grasped'), 'interaction_binding_kind')
    require(binding.get('route', 'generic') in ('generic','m4_up'), 'interaction_binding_route')
    checked = dict(interaction=binding['interaction'], condition=copy.deepcopy(condition),
                   plan=copy.deepcopy(plan), repository=repository, purpose=purpose)
    config = plan['runtime']
    spec_path = _private(repository, config['observerSpecPath'])
    spec, _ = _read_json(spec_path, config['observerSpecSha256'], max_bytes=12*1024*1024)
    generic = binding.get('route', 'generic') == 'generic'
    if purpose in ('dispatch','declaration') and generic and binding['interaction'] == 'contact_probe':
        exact = dict(joint='gripper',direction='close',percent=40,durationMs=350)
        require(condition.get('command') == exact, 'interaction_original_contact_probe_command_required')
        conditions = plan.get('conditions')
        require(isinstance(conditions, list) and any(c.get('id') == condition['id'] for c in conditions),
                'interaction_contact_plan_conditions_required')
        total = 0
        for candidate in conditions:
            other = plan['runtime'].get('conditionBindings', {}).get(candidate['id'], {})
            if candidate['id'] == condition['id']:
                other = binding
            if isinstance(other, dict) and other.get('route', 'generic') == 'generic' and other.get('interaction') == 'contact_probe':
                require(candidate.get('command') == exact and type(candidate.get('maxAttempts')) is int and
                        candidate['maxAttempts'] > 0, 'interaction_original_contact_probe_command_required')
                total += candidate['maxAttempts']
        require(total <= 6, 'interaction_contact_probe_six_command_budget')
    if purpose in ('dispatch','declaration') and generic and binding['interaction'] == 'grasped':
        command = condition.get('command', {})
        require(not (command.get('joint') == 'gripper' and command.get('direction') == 'close'),
                'interaction_no_repeated_close_after_contact')
    if binding['interaction'] == 'noncontact':
        require(spec['load'] == 'empty', 'interaction_noncontact_empty_load_required')
        if spec['object'] == 'none':
            return checked
    if purpose == 'declaration':
        return checked  # Dispatch still requires a complete current source-bound review.
    path = _private(repository, binding.get('reviewPath'))
    envelope, raw = _read_json(path, binding.get('reviewSha256'))
    fields = {'schemaVersion', 'conditionId', 'stage', 'interaction', 'outcome',
              'controllerBootId', 'context', 'outputSequence', 'scope', 'afterOffAt',
              'review', 'registration', 'retentionContext'}
    require(isinstance(envelope, dict) and set(envelope) == fields and
            envelope['schemaVersion'] == SCHEMA, 'interaction_review_schema')
    require(envelope['conditionId'] == condition['id'] and
            envelope['controllerBootId'] == plan['controllerBootId'] and
            envelope['interaction'] == binding['interaction'], 'interaction_plan_binding_changed')
    require(envelope['stage'] in ('before', 'after') and
            envelope['outcome'] in ('unknown', 'failed', 'contact_ready', 'held'),
            'interaction_explicit_outcome_required')
    require(envelope['stage'] == ('before' if purpose == 'dispatch' else 'after'),
            'interaction_before_review_required' if purpose == 'dispatch' else 'interaction_after_review_required')
    require(isinstance(envelope['review'],dict), 'interaction_original_review_required')
    require(type(envelope['outputSequence']) is int and envelope['outputSequence'] >= 0,
            'interaction_output_sequence_required')
    context = envelope['context']
    require(isinstance(context, dict) and set(context) == {'viewId','poseRegime','object','load'} and
            all(isinstance(v,str) and v for v in context.values()), 'interaction_context_required')
    require(context == {k:spec[k] for k in context} and context['viewId'] == plan['viewId'] and
            context['poseRegime'] == config['poseContext'], 'interaction_scene_context_changed')
    scope = envelope['scope']
    require(isinstance(scope, dict) and scope.get('controllerBootId') == plan['controllerBootId'] and
            isinstance(scope.get('hostClockId'), str) and bool(scope['hostClockId']) and
            scope.get('maxObservationAgeSeconds') == 10. and scope.get('postOffSeconds') == 3.,
            'interaction_original_observation_limits_required')
    legacy = scope.get('context')
    require(isinstance(legacy, dict) and set(legacy) == {'physicalView','poseRegime','object','supportMode',
            'externalDeviceId','externalViewId','gripperCameraBootId'} and
            legacy['object'] == context['object'] and legacy['poseRegime'] == context['poseRegime'] and
            legacy['externalViewId'] == context['viewId'] and legacy['externalDeviceId'] == config['deviceId'] and
            legacy['supportMode'] == 'table_supported', 'interaction_original_context_changed')
    camera = config.get('gripper', {})
    require(legacy['gripperCameraBootId'] == camera.get('bootId'), 'interaction_gripper_boot_changed')
    require(envelope['afterOffAt'] is None or number(envelope['afterOffAt']), 'interaction_off_time_required')
    if envelope['outcome'] == 'contact_ready':
        require(binding['interaction'] == 'grasped' and number(envelope['afterOffAt']) and
                isinstance(envelope['registration'], dict) and
                set(envelope['registration']) == {'path','sha256'}, 'interaction_contact_registration_required')
        registration_path = _private(repository, envelope['registration']['path'])
        registration, registration_raw = _read_json(registration_path, envelope['registration']['sha256'])
        checked.update(registration=registration, registrationPath=registration_path,
                       registrationRaw=registration_raw)
    if envelope['outcome'] == 'held':
        require(envelope['stage'] == 'after' and binding['interaction'] == 'grasped' and
                number(envelope['afterOffAt']) and isinstance(envelope['retentionContext'], dict),
                'interaction_retention_scope_required')
        retained_context = envelope['retentionContext']
        require(retained_context.get('object') == context['object'] and
                retained_context.get('poseRegime') == context['poseRegime'] and
                retained_context.get('physicalView') == legacy['physicalView'],
                'interaction_retention_context_changed')
    if binding['interaction'] == 'contact_probe':
        require(envelope['outcome'] in ('unknown','failed'), 'unknown_contact_cannot_claim_grasp')
    if binding['interaction'] == 'noncontact':
        require(envelope['outcome'] == 'unknown', 'noncontact_clearance_cannot_claim_grasp')
    checked.update(envelope=envelope, reviewPath=path, root=path.parent,
                   reviewRaw=raw, reviewSha256=sha(raw))
    return checked


class _ContactChecker:
    """메모리와 원본 파일을 검사하는 뷰. 제어기나 trial 생성자가 아니다."""
    _contact = CloseLiftTrial._contact


class _RecordChecker:
    _observe = m4.M4UpCommissioning._observe


class _RetentionChecker:
    _retention = ModelGraspTrial._retention
    _observe = ModelGraspTrial._observe
    _review_evidence = ModelGraspTrial._review_evidence
    _input_age_limit = ModelGraspTrial._input_age_limit

    def _deadline(self):
        require(self.clock.monotonic() < self.deadline, 'interaction_retention_deadline')

    def _idle_tick(self):
        # Current OFF/output sequence must be supplied and checked by the caller.
        self._deadline()


@dataclass
class VerifiedInteraction:
    report: dict
    snapshots: dict
    snapshotHashes: dict
    reportSha256: str
    expiresAt: float
    outputSequence: int
    verifiedAt: float
    privateRoot: Path | None = None
    deadlineWall: float | None = None


def _seal_legacy_record(root, record, snapshots):
    """Seal every source used by the unchanged M4 dual-frame validators."""
    names = [record['evidenceFile']]
    source = record['external']; prefix = source['root']+'/'
    ingress = json.loads(_read(root, prefix+source['ingressFile']))
    names.extend([prefix+source['ingressFile'], prefix+'host-ingress/clock-before.json',
                  prefix+ingress['metadataFile'], prefix+ingress['imageFile'],
                  record['gripper']['root']+'/esp32-'+record['gripper']['label']+'.json',
                  record['gripper']['root']+'/esp32-'+record['gripper']['label']+'.jpg'])
    for name in names:
        snapshots[str((root/name).resolve())] = _read(root, name)


def verify_review(checked, *, now, output_sequence, after_off=None):
    """Verify saved original evidence, preserving unknown/contact/retention distinctions."""
    require(checked.get('purpose') in ('dispatch','label'), 'interaction_verified_review_required')
    require(number(now) and type(output_sequence) is int, 'interaction_current_state_required')
    if checked['interaction'] == 'noncontact' and 'envelope' not in checked:
        report = dict(interaction='noncontact', contactState='not_reviewed', tableContactReady=False,
                      graspVerified=False, liftVerified=False, retainedLift=False)
        return VerifiedInteraction(report, {}, {}, digest(report), now+10., output_sequence, now)
    envelope = checked['envelope']; root = checked['root']
    require(sha(checked['reviewRaw']) == checked['reviewSha256'] and
            json.loads(checked['reviewRaw']) == envelope, 'interaction_cached_review_changed')
    require(envelope['outputSequence'] == output_sequence, 'interaction_intervening_output')
    require(envelope['stage'] != 'after' or after_off is not None, 'interaction_actual_after_off_required')
    if after_off is not None:
        require(number(after_off) and envelope['afterOffAt'] == after_off and envelope['stage'] == 'after',
                'interaction_after_off_binding_changed')
    scope, proof = envelope['scope'], envelope['review']
    clock = SimpleNamespace(monotonic=lambda:now)
    snapshots = {str(checked['reviewPath']):checked['reviewRaw']}
    frames = []
    if envelope['outcome'] == 'contact_ready':
        checker = _ContactChecker(); checker.root = root; checker.clock = clock
        checker.contact_scope = scope
        checker.lift = SimpleNamespace(registration=checked['registration'])
        checker._contact(proof, envelope['afterOffAt'])
        for name in checker.contact_seal:
            snapshots[str((root/name).resolve())] = _read(root, name)
        snapshots[str(checked['registrationPath'])] = checked['registrationRaw']
        latest = checker.contact_latest_earliest
        for observation in proof['observations']:
            location = (root/observation['root']).resolve(strict=True)
            record = observation['record']
            frames.append(dict(external=m4._external_frame(location,record['external'],scope),
                gripper=m4._gripper_frame(location,record['gripper'],scope,record['gripper']['label'])))
    else:
        # Unknown/failure reviews still require authentic dual-camera source bytes.
        # A held outcome additionally carries the original retention proof below.
        observations = proof.get('dualObservations') if envelope['outcome'] == 'held' else proof.get('observations')
        require(isinstance(observations, list) and bool(observations), 'interaction_dual_observations_required')
        for observation in observations:
            require(isinstance(observation,dict) and set(observation) == {'root','record'} and
                    isinstance(observation['root'],str), 'interaction_observation_source_required')
            location = (root/observation['root']).resolve(strict=True)
            require(location.is_relative_to(root), 'interaction_private_source_required')
            record = observation['record']
            checker = _RecordChecker(); checker.scope = copy.deepcopy(scope)
            checker.root = location; checker.clock = clock; checker.started = 0.
            if envelope['interaction'] in ('contact_probe', 'noncontact'):
                checker.scope['schemaVersion'] = m4.CONTACT_PROBE_SCHEMA
            pair = checker._observe(record, after_off=after_off)
            for frame in pair.values():
                require(0 <= now-frame['sourceEarliest'] <= 10., 'interaction_review_expired')
            frames.append(pair)
            _seal_legacy_record(location, record, snapshots)
        latest = min(f['sourceEarliest'] for f in frames[-1].values())
        if envelope['outcome'] == 'held':
            require(len(frames) >= 2, 'interaction_retention_two_camera_span_required')
            seconds = scope.get('retentionSeconds')
            require(number(seconds) and 1 <= seconds <= 10, 'interaction_retention_seconds_required')
            for name in ('external','gripper'):
                require(len({p[name]['frameId'] for p in frames}) == len(frames) and
                        all(b[name]['sourceEarliest'] > a[name]['sourceLatest'] for a,b in zip(frames,frames[1:])) and
                        frames[-1][name]['sourceEarliest']-frames[0][name]['sourceLatest'] >= seconds,
                        'interaction_retention_two_camera_span_required')
            retention = proof.get('retention')
            require(isinstance(retention, dict), 'interaction_original_retention_required')
            context = envelope['retentionContext']
            verified_frames = {(f['frameId'],f['pixelSha256']) for p in frames for f in p.values()}
            require(all((r['frame']['frameId'],r['frame']['pixelSha256']) in verified_frames
                        for r in retention.get('observations', [])), 'interaction_retention_frame_not_original')
            checker = _RetentionChecker(); checker.clock = clock; checker.root = checker.evidence_root = root
            checker.scope = dict(scope); checker.started = 0.; checker.deadline = now+1.
            checker.bank = SimpleNamespace(context=lambda _:copy.deepcopy(context))
            checker.observer = SimpleNamespace(verify_grasp=lambda *args,**kwargs:copy.deepcopy(retention))
            result = checker._retention(dict(modelId='review-only'), envelope['afterOffAt'])
            require(result['status'] == 'held', 'interaction_retention_not_held')
            snapshots[str((root/retention['evidenceFile']).resolve())] = _read(root,retention['evidenceFile'])
    report = dict(interaction=envelope['interaction'], stage=envelope['stage'],
        contactState='confirmed_table_contact' if envelope['outcome'] == 'contact_ready' else envelope['outcome'],
        tableContactReady=envelope['outcome'] == 'contact_ready',
        graspVerified=envelope['outcome'] == 'held', liftVerified=envelope['outcome'] == 'held',
        retainedLift=envelope['outcome'] == 'held', reviewSha256=checked['reviewSha256'],
        controllerBootId=envelope['controllerBootId'], context=copy.deepcopy(envelope['context']),
        outputSequence=output_sequence, physicalJudgmentGenerated=False,
        frameBindings={name:[{k:p[name][k] for k in ('frameId','pixelSha256')} for p in frames]
                       for name in ('external','gripper')})
    return VerifiedInteraction(report, snapshots, {p:sha(raw) for p,raw in snapshots.items()},
                               digest(report), latest+10., output_sequence, now,
                               privateRoot=(checked['repository']/'.local').resolve())


def before_run_cached(evidence, now, wall_now, output_sequence):
    """Rehash the same sealed originals, retaining the legacy contact guard.

    There is no capture, controller request, inference, or affirmative judgment.
    The caller must retain its RUN guard deadline around this bounded file check.
    """
    started = time.monotonic()
    require(isinstance(evidence, VerifiedInteraction), 'interaction_verified_cache_required')
    require(number(now) and number(wall_now) and evidence.verifiedAt <= now <= evidence.expiresAt and
            (evidence.deadlineWall is None or wall_now < evidence.deadlineWall), 'interaction_cached_review_expired')
    require(output_sequence == evidence.outputSequence, 'interaction_intervening_output')
    require(digest(evidence.report) == evidence.reportSha256 and
            set(evidence.snapshots) == set(evidence.snapshotHashes) and
            all(sha(raw) == evidence.snapshotHashes[p] for p,raw in evidence.snapshots.items()),
            'interaction_cached_evidence_changed')
    if evidence.privateRoot is not None:
        for name, expected in evidence.snapshotHashes.items():
            path = Path(name)
            require(path.is_relative_to(evidence.privateRoot), 'interaction_original_outside_private_root')
            require(sha(_read(evidence.privateRoot, str(path.relative_to(evidence.privateRoot)))) == expected,
                    'interaction_original_evidence_changed')
    elapsed = time.monotonic()-started
    require(elapsed <= .1 and now+elapsed <= evidence.expiresAt and
            (evidence.deadlineWall is None or wall_now+elapsed < evidence.deadlineWall),
            'interaction_run_guard_expired')
    return copy.deepcopy(evidence.report)


def before_command(manager, pair, condition, binding, deadline_wall, *, checked, clock=time):
    require(clock.time() < deadline_wall, 'interaction_deadline')
    require(checked.get('purpose') == 'dispatch', 'interaction_dispatch_review_required')
    require(checked['condition'] == condition and checked['interaction'] == binding['interaction'],
            'interaction_condition_changed')
    require(manager.port is not None and manager.port.off_verified is True, 'interaction_current_off_required')
    evidence = verify_review(checked, now=clock.monotonic(), output_sequence=manager.port.last_sequence)
    evidence.deadlineWall = deadline_wall
    require('envelope' not in checked or checked['envelope']['stage'] == 'before',
            'interaction_before_review_required')
    return evidence


def after_command(checked, *, now, output_sequence, after_off):
    """Post-action targets only. This report must not become a before-state input."""
    require(checked.get('purpose') == 'label', 'interaction_label_review_required')
    evidence = verify_review(checked, now=now, output_sequence=output_sequence, after_off=after_off)
    require('envelope' not in checked or checked['envelope']['stage'] == 'after',
            'interaction_after_review_required')
    return evidence


def validate_after_response(request, response):
    """Bind a newly produced post-action review to its actual immutable source request.

    This parses no image and makes no physical judgment. The returned wrapper
    still must pass validate_binding/after_command and verified frame membership.
    """
    require(isinstance(request,dict) and set(request) == {'schemaVersion','conditionId','source',
            'context','offAt','outputSequence','frames'} and
            request['schemaVersion'] == 'motionbrain.batch_after_review_request.v1', 'after_review_request_schema')
    source = request['source']
    require(isinstance(source,dict) and set(source) == {'sessionId','path','sha256'} and
            all(isinstance(v,str) and v for v in source.values()) and
            re.fullmatch('[0-9a-f]{64}',source['sha256']) is not None, 'after_review_source_required')
    require(isinstance(response,dict) and set(response) == {'requestSha256','review'} and
            len(encoded(response)) <= 1024*1024 and response['requestSha256'] == sha(encoded(request)),
            'after_review_response_request_changed')
    review = response['review']
    require(isinstance(review,dict) and review.get('schemaVersion') == SCHEMA and
            review.get('stage') == 'after' and review.get('conditionId') == request['conditionId'] and
            review.get('context') == request['context'] and review.get('afterOffAt') == request['offAt'] and
            review.get('outputSequence') == request['outputSequence'] and
            isinstance(review.get('scope'),dict) and review['scope'].get('hostClockId') == source['sessionId'],
            'after_review_response_source_changed')
    frames = request['frames']
    require(isinstance(frames,dict) and set(frames) == {'external','gripper'}, 'after_review_frames_required')
    for values in frames.values():
        require(isinstance(values,list) and all(isinstance(f,dict) and set(f) == {'frameId','pixelSha256'} and
                isinstance(f['frameId'],str) and f['frameId'] and
                isinstance(f['pixelSha256'],str) and re.fullmatch('[0-9a-f]{64}',f['pixelSha256']) is not None
                for f in values), 'after_review_frames_required')
    proof = review.get('review',{})
    require(isinstance(proof,dict), 'after_review_original_observations_required')
    observations = proof.get('dualObservations') if review.get('outcome') == 'held' else proof.get('observations')
    require(isinstance(observations,list) and bool(observations), 'after_review_original_observations_required')
    external_ids = {f['frameId'] for f in frames['external']}
    for observation in observations:
        require(isinstance(observation,dict), 'after_review_observation_fields')
        record = observation.get('record',{})
        require(isinstance(record,dict) and record.get('hostClockId') == source['sessionId'],
                'after_review_observation_session_changed')
        external=record.get('external',{})
        require(isinstance(external,dict) and isinstance(external.get('ingressFile'),str), 'after_review_external_source_required')
        ingress = external['ingressFile']
        match = re.fullmatch(r'host-ingress/([0-9]+)\.json',ingress)
        require(match is not None and source['sessionId']+':'+str(int(match.group(1))) in external_ids,
                'after_review_external_frame_not_requested')
    return copy.deepcopy(review)


def verify_after_frame_membership(evidence, request):
    """Call after native/JPEG verification, never trust declarations of pixel hashes."""
    require(isinstance(evidence,VerifiedInteraction), 'interaction_verified_cache_required')
    require(evidence.report.get('stage') == 'after', 'interaction_after_review_required')
    require(digest(evidence.report) == evidence.reportSha256, 'interaction_cached_evidence_changed')
    bindings = evidence.report.get('frameBindings',{})
    for name in ('external','gripper'):
        allowed = {(f['frameId'],f['pixelSha256']) for f in request['frames'][name]}
        actual = bindings.get(name,[])
        require(bool(actual) and all((f['frameId'],f['pixelSha256']) in allowed for f in actual),
                'after_review_verified_frame_not_requested:'+name)
    return copy.deepcopy(evidence.report)
