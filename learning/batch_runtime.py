"""기존 단발 수집에 연결하는 묶음 실행 어댑터. 생성·계획 검사는 장치에 접근하지 않는다."""
from __future__ import annotations

import copy
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import sqlite3
import time
import uuid

from learning.action_journal import read_journal
from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore, StorageError, _read
from learning.batch_observation import observe_frame, validate_spec, digest as observer_digest
from learning.batch_campaign import digest as plan_digest
from learning.batch_learning import evaluate, prospective_evidence, validate_model, model_sha256, command_key
from learning.experiment_pipeline import atomic, encoded
from learning.lossless_frame import decode
from learning.manual_collection import ManualCollection
from learning.pipeline_collection import collect_once
from learning.pipeline_evidence import ingest_session
from learning.supervised_transport import SequencePort, validate_target


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def private_path(repository, value):
    require(isinstance(value, str) and bool(value), 'private_runtime_path_required')
    path = (repository/value).resolve()
    require(path.is_relative_to((repository/'.local').resolve()), 'private_runtime_path_required')
    return path


def validate_runtime(plan, repository):
    """파일·계약만 검사한다. 카메라/HTTP/키체인/프로세스를 시작하지 않는다."""
    repository = Path(repository).resolve()
    config = copy.deepcopy(plan.get('runtime'))
    require(isinstance(config, dict), 'batch_runtime_configuration_required')
    for key in ('controllerUrl', 'deviceId', 'helperPath', 'helperSha256', 'poseContext',
                'observerSpecPath', 'observerSpecSha256', 'journalPath', 'dispatchLockPath'):
        require(isinstance(config.get(key), str) and bool(config[key]), 'runtime_field_required:'+key)
    validate_target(config['controllerUrl'])
    helper = (repository/config['helperPath']).resolve(strict=True)
    require(helper.is_relative_to(repository) and os.access(helper, os.X_OK), 'runtime_helper_invalid')
    require(sha(helper.read_bytes()) == config['helperSha256'], 'runtime_helper_hash_changed')
    spec_path = private_path(repository, config['observerSpecPath'])
    raw = spec_path.read_bytes()
    require(sha(raw) == config['observerSpecSha256'], 'runtime_observer_spec_hash_changed')
    spec = validate_spec(json.loads(raw))
    require(spec['viewId'] == plan['viewId'] and spec['poseRegime'] == config['poseContext']
            and spec['cameraIdentity'] == config['deviceId'], 'runtime_observer_context_mismatch')
    require(spec['load'] == 'empty' and spec['object'] == 'none', 'batch_runtime_noncontact_unloaded_only')
    require('gripper' not in config, 'paired_gripper_runtime_not_supported')
    require(private_path(repository, config['journalPath']) == (repository/'.local/visual-actions/controller-commands.sqlite').resolve(), 'runtime_shared_journal_required')
    lock_name = sha(plan['controllerBootId'].encode())+'.lock'
    require(private_path(repository, config['dispatchLockPath']) == (repository/'.local/batch-locks'/lock_name).resolve(), 'runtime_shared_dispatch_lock_required')
    models = config.get('frozenModels', {})
    require(isinstance(models, dict), 'runtime_frozen_model_mapping_required')
    problems = {}
    for joint in models:
        try:
            _load_model(repository, config, spec, joint)
        except (ValueError, KeyError, OSError, StorageError) as exc:
            problems[joint] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
    unsupported = []
    return dict(status='runtime_checked_no_device_access', helper=helper, spec=spec,
                config=config, unsupported=unsupported, modelProblems=problems)


def _load_model(repository, config, spec, joint):
    binding = config['frozenModels'][joint]
    require(joint in spec['joints'] and isinstance(binding, dict) and set(binding) == {'path','sha256'}, 'runtime_frozen_model_binding')
    raw = private_path(repository, binding['path']).read_bytes()
    require(sha(raw) == binding['sha256'], 'runtime_frozen_model_hash_changed')
    model = validate_model(json.loads(raw))
    require(model['stateFields'] == [f['name'] for f in spec['joints'][joint]['stateFields']], 'runtime_frozen_model_state_fields_changed')
    require(model['joint'] == joint and model['context'] == {
        **{k:spec[k] for k in ('viewId','poseRegime','object','load')},
        'observerSha256':observer_digest(spec)}, 'runtime_frozen_model_context_changed')
    return model


def journal_clear(path):
    """기존 원장의 미해결 명령과 수집 블록을 모두 읽기 전용 확인한다."""
    path = Path(path)
    if not read_journal(path)['ready']:
        return False
    if not path.exists():
        return True
    connection = sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True)
    try:
        table = connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='supervised_blocks'").fetchone()
        return not table or connection.execute("SELECT 1 FROM supervised_blocks WHERE phase IN ('pending','off_unverified') LIMIT 1").fetchone() is None
    finally:
        connection.close()


class BatchRuntime:
    """관측·단발 실행·영속 증거 재대조만 수행한다. 조건 선택은 Campaign 담당."""
    def __init__(self, plan, repository, root, credential, *, clock=time):
        self.plan = copy.deepcopy(plan)
        self.repository, self.root = Path(repository).resolve(), Path(root).resolve()
        require(self.root.is_relative_to((self.repository/'.local').resolve()), 'private_batch_root_required')
        checked = validate_runtime(plan, self.repository)
        self.config, self.spec, self.helper = checked['config'], checked['spec'], checked['helper']
        self.model_problems = checked['modelProblems']
        self.credential, self.clock = credential, clock
        self.journal = private_path(self.repository, self.config['journalPath'])
        self.token = None
        self.manager = self.recorder = self.probe = None
        self.latest_observation = None
        self.latest_before = None
        self.deadline_wall = self.deadline_mono = None
        self.clock_id = 'batch-'+uuid.uuid4().hex
        self.plan_sha = plan_digest(plan)

    def _verify(self):
        checked = validate_runtime(self.plan, self.repository)
        require(checked['config'] == self.config and checked['spec'] == self.spec, 'runtime_binding_changed')

    def _token(self):
        if self.token is None:
            value = self.credential()
            require(isinstance(value, str) and bool(value), 'memory_credential_required')
            self.token = value
        return self.token

    def _context(self):
        return {**{k:self.spec[k] for k in ('viewId','poseRegime','object','load')},
                'observerSha256':observer_digest(self.spec)}

    def _model(self, joint):
        return _load_model(self.repository, self.config, self.spec, joint)

    def _deadline(self):
        require(self.deadline_wall is None or (self.clock.time() < self.deadline_wall and
                self.clock.monotonic() < self.deadline_mono), 'batch_runtime_deadline')

    def _available_frame(self, recorder, *, margin=0):
        end = self.clock.monotonic()+2
        while True:
            recorder.check()
            now = self.clock.monotonic()
            frames = [f for f in copy.deepcopy(recorder.frames)
                      if f['sourceLatest'] <= now-margin and
                      f['stdoutReadAt']+f['persistenceMs']/1000 <= now]
            if frames:
                return frames[-1]
            require(now < end, 'batch_no_available_frame')
            self.clock.sleep(.02)

    def _execution_proof(self, request_id, report):
        manager = self.manager
        source = report.get('sessionPath')
        if not source or not manager.current or manager.current['sessionId'] != report.get('sessionId'):
            return
        raw = _read(source, 'session.json')
        require(sha(raw) == report['sourceSha256'], 'batch_execution_source_changed')
        port = manager.port
        proof = dict(requestId=request_id, planSha256=self.plan_sha, sourceSha256=sha(raw),
                     hasPort=port is not None,
                     mayHaveRun=None if port is None else port.may_have_run,
                     dispatchBefore=None if port is None else port.last_dispatch_before,
                     dispatchAfter=None if port is None else port.last_dispatch_after,
                     finalIdleOffVerified=manager.current.get('finalIdleOffVerified') is True)
        manager.store.write_json('batch-execution-proof.json', proof)

    def _observe_native(self, folder, frame, *, historical=False):
        import cv2
        import numpy as np
        raw = _read(folder, frame['imageFile'])
        require(sha(raw) == frame['imageSha256'], 'batch_native_frame_hash_changed')
        pixels = decode(raw, frame['rawByteLength'], frame['pixelSha256'])
        bgra = np.frombuffer(pixels, dtype=np.uint8).reshape(frame['height'], frame['width'], 4)
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        ok, png = cv2.imencode('.png', bgr)
        require(ok, 'batch_frame_conversion_failed')
        image = png.tobytes()
        # Historical processing keeps its original before/after timestamps; it
        # must never be substituted for the current live observation.
        available = frame['stdoutReadAt']+frame['persistenceMs']/1000
        now = available if historical else self.clock.monotonic()
        metadata = dict(frameId=frame['frameId'], imageSha256=sha(image),
                        pixelSha256=sha(bgr.tobytes()), capturedAt=frame['sourceEarliest'],
                        persistedAt=available, clockId=self.clock_id,
                        cameraIdentity=self.config['deviceId'])
        context = dict(viewId=self.plan['viewId'], poseRegime=self.config['poseContext'],
                       object=self.spec['object'], load=self.spec['load'])
        observed = observe_frame(image, metadata, self.spec, context, now=now, clock_id=self.clock_id)
        return dict(observation=observed, source=dict(folder=str(folder), frame=copy.deepcopy(frame)),
                    derivedMetadata=metadata)

    def observe(self):
        self._verify()
        require(journal_clear(self.journal), 'batch_shared_journal_blocked')
        # Capture stores are immutable and never overwrite an earlier probe.
        sid = 'batch-observe-'+uuid.uuid4().hex
        store = ArtifactStore(self.root/'observations'/sid)
        self.probe = SequencePort(self.config['controllerUrl'], self._token(), self.plan['controllerBootId'],
                                  store, clock=self.clock, compact_reads=True, timeout=2)
        self.probe.preflight()  # GET-only; do not call finish() to invent OFF proof.
        require(self.probe.off_verified, 'batch_probe_off_unverified')
        minimum = 2*1024**3 + 100*(1920*1080*4+65536)
        require(shutil.disk_usage(self.root).free >= minimum, 'batch_storage_reserve_required')
        camera_plan = dict(deviceId=self.config['deviceId'], sessionId=sid, viewId=self.plan['viewId'],
                           seconds=10, maxFrames=100, sampleFps=8, maxBytes=100*(1920*1080*4+65536))
        self.recorder = ActionRecorder(store.root/'native-camera', self.helper, camera_plan, clock=self.clock)
        try:
            self.recorder.start()
            self.recorder.wait_ready(timeout=15)
            self.recorder.check()
            frame = self._available_frame(self.recorder)
            evidence = self._observe_native(self.recorder.root, frame)
            self.probe.preflight()
            result = evidence['observation']
            observation = dict(nodes=[n for n in result.get('nodeMatches', []) if n in self.plan['nodes']] if result.get('status') == 'accepted' else [],
                               bootId=self.probe.boot_id, viewId=self.plan['viewId'],
                               observedAt=frame['sourceEarliest'], offVerified=self.probe.off_verified,
                               journalClear=journal_clear(self.journal), reason=result.get('reason'),
                               evidence=evidence)
            store.write_json('observation.json', observation)
            self.latest_observation = observation
            return observation
        finally:
            self.recorder.close()
            self.recorder = None
            self.probe.token = ''
            self.probe = None

    def _before(self, condition):
        def check(manager, pair):
            manager.camera.check()
            self._deadline()
            frame = self._available_frame(manager.camera, margin=.2)
            evidence = self._observe_native(manager.camera.root, frame)
            result = evidence['observation']
            require(result.get('status') == 'accepted' and
                    bool(set(result.get('nodeMatches', [])) & set(condition['fromNodes'])), 'batch_current_node_not_supported')
            require(self.clock.monotonic()-frame['sourceEarliest'] <= self.plan['maxObservationAgeSeconds'], 'batch_current_frame_stale')
            manager.store.write_json('batch-before-observation.json', evidence)
            self.latest_before = evidence
            if condition['phase'] == 'evaluation':
                model = self._model(condition['command']['joint'])
                joint = condition['command']['joint']
                row = dict(sessionId=manager.current['sessionId'],
                           before=result['states'][joint]['state'], command=condition['command'],
                           context=self._context(), beforePersistedAt=evidence['derivedMetadata']['persistedAt'])
                proof = prospective_evidence(model, row, self.clock.monotonic())
                manager.store.write_json('batch-prospective.json', dict(
                    proof=proof, beforeFrameId=frame['frameId'], beforePixelSha256=frame['pixelSha256']))
            self._deadline()
        return check

    def validate_condition(self, condition):
        if condition['phase'] == 'evaluation' and condition['command']['joint'] not in self.config.get('frozenModels', {}):
            return 'runtime_evaluation_frozen_model_required'
        joint = condition['command']['joint']
        if condition['phase'] == 'evaluation':
            if joint in self.model_problems:
                return 'runtime_evaluation_model_rejected:'+self.model_problems[joint]
            model = self._model(joint)
            if command_key(condition['command']) not in model['models']:
                return 'runtime_evaluation_command_not_supported'
        joint_spec = self.spec['joints'].get(joint)
        if joint_spec is None:
            return 'runtime_joint_observer_not_supported'
        own_nodes = {n['id'] for n in joint_spec['nodes']}
        if not set(condition['fromNodes']) <= own_nodes:
            return 'runtime_condition_nodes_not_bound_to_joint'
        return None

    condition_support = validate_condition

    def execute(self, condition, request_id, *, deadline_wall):
        self._verify()
        self.deadline_wall = deadline_wall
        self.deadline_mono = self.clock.monotonic()+max(0, deadline_wall-self.clock.time())
        self._deadline()
        require(self.validate_condition(condition) is None, 'runtime_condition_not_supported')
        require(journal_clear(self.journal), 'batch_shared_journal_blocked')
        expected_boot = self.plan['controllerBootId']
        from learning.manual_collection import RecordingPort
        class PinnedRecordingPort(RecordingPort):
            def _check_status(port, status, expected, pulse=None, baseline_angle=None):
                super()._check_status(status, expected, pulse, baseline_angle)
                require(port.boot_id == expected_boot, 'batch_controller_boot_changed')
        runtime = self
        class GuardedCollection(ManualCollection):
            def pulse(manager, payload, *, before_run=None):
                def guard():
                    runtime._deadline()
                    manager.camera.check()
                    cached = runtime.latest_before
                    require(cached is not None, 'batch_current_observation_required')
                    at = cached['source']['frame']['sourceEarliest']
                    require(0 <= runtime.clock.monotonic()-at <= runtime.plan['maxObservationAgeSeconds'], 'batch_dispatch_observation_stale')
                    observed = cached['observation']
                    require(observed.get('status') == 'accepted' and
                            bool(set(observed.get('nodeMatches', [])) & set(condition['fromNodes'])), 'batch_dispatch_node_not_supported')
                    if before_run is not None:
                        before_run()
                return super().pulse(payload, before_run=guard)
        self.latest_before = None
        self.manager = GuardedCollection(self.root/'sessions', self.helper, self.config['controllerUrl'],
                                        self._token(), self.config['deviceId'], self.plan['viewId'],
                                        journal_path=self.journal, clock=self.clock, sample_fps=8,
                                        port_factory=PinnedRecordingPort)
        try:
            report = collect_once(self.manager, request_id, condition['command'], self.config['poseContext'],
                                  onsite_confirmed=True, request_root=self.root/'requests',
                                  approval_plan_sha256=self.plan_sha, before_command=self._before(condition))
            self._execution_proof(request_id, report)
            self.manager.close()  # release the shared exclusive journal before read-only audit
            result = self._classify(condition, request_id, report, process=True)
            self._save(self.root/'outcomes'/(request_id+'.json'), result)
            return result
        finally:
            self.manager.close()
            self.manager.token = ''
            if self.manager.port:
                self.manager.port.token = ''
            self.manager = None

    def _classify(self, condition, request_id, report, *, process):
        result = dict(execution='unknown', offVerified=False, journalClear=journal_clear(self.journal),
                      capture='failed', observation='rejected', reason='persisted_execution_not_verified',
                      evidence=dict(requestId=request_id))
        source_name, source_sha = report.get('sessionPath'), report.get('sourceSha256')
        if not source_name or not source_sha:
            return result
        source = Path(source_name).resolve()
        require(source.is_relative_to(self.root/'sessions'), 'batch_source_outside_campaign')
        audit = ingest_session(source, source_sha)
        result['evidence'].update(sessionPath=str(source), sourceSha256=source_sha, audit=audit)
        execution = audit['execution']
        session_identity = json.loads(_read(source, 'session.json'))
        require(session_identity.get('joint') == condition['command']['joint'], 'batch_audited_joint_changed')
        if execution['status'] == 'verified':
            require(execution.get('command') == condition['command'], 'batch_audited_command_changed')
        result['offVerified'] = execution.get('offVerified') is True
        if execution['status'] == 'verified':
            result['execution'] = 'completed'
        elif execution['status'] == 'no_run':
            result['execution'] = 'not_dispatched'
        if result['execution'] == 'unknown' and result['journalClear']:
            proof_path = source/'batch-execution-proof.json'
            if proof_path.is_file():
                proof = json.loads(_read(source, proof_path.name))
                require(proof['requestId'] == request_id and proof['planSha256'] == self.plan_sha
                        and proof['sourceSha256'] == source_sha, 'batch_execution_proof_binding_changed')
                session = json.loads(_read(source, 'session.json'))
                results = [e for e in session.get('events', []) if e.get('kind') == 'result']
                run_responses = [e for e in session.get('events', []) if e.get('kind') == 'http'
                    and e.get('payload') == {'action':'run'}]
                if (proof['hasPort'] is True and proof['mayHaveRun'] is False
                        and proof['dispatchBefore'] is None and proof['dispatchAfter'] is None
                        and proof['finalIdleOffVerified'] is True and
                        session.get('finalIdleOffVerified') is True and not results and not run_responses):
                    result.update(execution='not_dispatched', offVerified=True,
                                  reason='durable_predispatch_off_proof')
        if audit['recording']['status'] == 'verified':
            result['capture'] = 'recorded'
        else:
            result['capture'] = 'partial'
        if process and result['execution'] == 'completed' and audit['visual']['status'] == 'verified':
            try:
                row = audit['visual']['row']
                session = json.loads((source/'session.json').read_bytes())
                pinned = json.loads(_read(source, 'batch-before-observation.json'))
                frame = pinned['source']['frame']
                actual = session['frames'][frame['frameIndex']]
                require(actual == frame and frame['frameIndex'] in row['evidence']['preFrameIds'], 'batch_pinned_input_not_in_audited_window')
                command_at = row['evidence']['commandAt']
                require(command_at-3 <= frame['sourceEarliest'] <= frame['sourceLatest'] <= command_at-.1
                        and frame['stdoutReadAt']+frame['persistenceMs']/1000 < command_at, 'batch_pinned_input_not_causal')
                before = self._observe_native(source/'native-camera', frame, historical=True)
                require(before['derivedMetadata'] == pinned['derivedMetadata'] and
                        before['observation'] == pinned['observation'], 'batch_pinned_observation_changed')
                after = self._observe_native(source/'native-camera', row['targets']['image'], historical=True)
                a, b = before['observation'], after['observation']
                joint = condition['command']['joint']
                if (a.get('status') == b.get('status') == 'accepted' and joint in a['states'] and joint in b['states']
                        and set(a.get('nodeMatches', [])) & set(condition['fromNodes'])):
                    transition = dict(schemaVersion='motionbrain.batch_visual_transition.v1',
                                      conditionId=condition['id'], captureGroup=condition['captureGroup'],
                                      phase=condition['phase'], sourceSha256=source_sha,
                                      command=copy.deepcopy(condition['command']), before=before, after=after,
                                      sessionId=row['sessionId'], episodeId=audit['execution']['episode'],
                                      frameIds=[frame['frameId'], row['targets']['image']['frameId']],
                                      selectionSemantics='pinned_pre_window_frame_not_default_audit_input',
                                      joint=joint, context={k:self.spec[k] for k in ('viewId','poseRegime','object','load')},
                                      stateFields=copy.deepcopy(a['states'][joint]['stateFields']),
                                      beforeState=copy.deepcopy(a['states'][joint]['state']),
                                      afterState=copy.deepcopy(b['states'][joint]['state']),
                                      executionEvidence=audit['execution'], trainingPerformed=False,
                                      controlEligible=False)
                    transition['learningRow'] = dict(
                        sessionId=row['sessionId'], episodeId=':'.join(audit['execution']['episode']),
                        sourceHash=source_sha, frameIds=transition['frameIds'],
                        captureGroup=condition['captureGroup'], joint=joint,
                        context={**transition['context'], 'observerSha256':observer_digest(self.spec)},
                        stateFields=transition['stateFields'], before=transition['beforeState'],
                        after=transition['afterState'], command=transition['command'],
                        commandAt=command_at, beforePersistedAt=before['derivedMetadata']['persistedAt'])
                    if condition['phase'] == 'evaluation':
                        prospective = json.loads(_read(source, 'batch-prospective.json'))
                        require(prospective['beforeFrameId'] == frame['frameId'] and prospective['beforePixelSha256'] == frame['pixelSha256'], 'batch_prospective_frame_changed')
                        transition['evaluation'] = evaluate(self._model(joint), [transition['learningRow']],
                            prospective={row['sessionId']:prospective['proof']})
                        require(transition['evaluation']['prospectiveEvidenceVerified'] is True, 'batch_prospective_evaluation_rejected')
                        result['prospectiveEvidenceVerified'] = True
                    transition_id = sha(encoded(transition))
                    self._save(self.root/'transitions'/(transition_id+'.json'), transition)
                    result.update(observation='eligible', transitionId=transition_id, reason='verified_transition_recorded',
                        dataIdentity={k:copy.deepcopy(transition['learningRow'][k]) for k in ('sourceHash','sessionId','episodeId','frameIds')})
            except (ValueError, KeyError, OSError, StorageError) as exc:
                result['reason'] = 'observation_processing_rejected:'+type(exc).__name__
                if isinstance(exc, (ValueError, KeyError)):
                    result['observationFailureCode'] = str(exc)
        return result

    def reconcile(self, condition, request_id):
        """재시작 시 파일만 읽는다. 토큰 조회·HTTP·카메라·명령 실행 없음."""
        outcome = self.root/'outcomes'/(request_id+'.json')
        request = self.root/'requests'/(request_id+'.json')
        if not request.is_file():
            return None
        report = json.loads(request.read_bytes())
        expected = report.get('request', {})
        require(expected.get('requestId') == request_id and expected.get('command') == condition['command']
                and expected.get('approvalPlanSha256') == self.plan_sha, 'batch_reconcile_request_binding_changed')
        result = self._classify(condition, request_id, report, process=False)
        if outcome.is_file():
            saved = json.loads(outcome.read_bytes())
            if saved.get('transitionId') and result['execution'] == 'completed':
                require(isinstance(saved['transitionId'], str) and re.fullmatch(r'[0-9a-f]{64}', saved['transitionId']), 'batch_transition_id_invalid')
                path = self.root/'transitions'/(saved['transitionId']+'.json')
                raw = path.read_bytes()
                require(sha(raw) == saved['transitionId'], 'batch_transition_hash_changed')
                transition = json.loads(raw)
                require(transition['sourceSha256'] == report['sourceSha256']
                        and transition['conditionId'] == condition['id'], 'batch_transition_source_changed')
                result.update(observation='eligible', transitionId=saved['transitionId'], reason='reconciled_verified_transition',
                    dataIdentity={k:copy.deepcopy(transition['learningRow'][k]) for k in ('sourceHash','sessionId','episodeId','frameIds')})
                if condition['phase'] == 'evaluation':
                    result['prospectiveEvidenceVerified'] = transition.get('evaluation', {}).get('prospectiveEvidenceVerified') is True
        return result

    def _save(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        raw = encoded(value)
        if path.exists():
            require(path.read_bytes() == raw, 'batch_immutable_artifact_changed')
        else:
            atomic(path, raw)

    def close(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None
        if self.manager:
            self.manager.close()
            self.manager.token = ''
            if self.manager.port:
                self.manager.port.token = ''
            self.manager = None
        if self.probe:
            self.probe.token = ''
            self.probe = None
        self.token = None
