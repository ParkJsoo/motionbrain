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
from learning.grasp_observation import GripperObservationSource
from learning.gripper_camera_dataset import bind_gripper_pair, read_frame as read_gripper_frame
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
from learning.registered_gripper_landmarks import RegisteredGripperLandmarks
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


def _require_dispatch_observer(spec):
    # The registered scene adapter has only stationary replay evidence. Its
    # accepted result is not motion-tracking admission for any live route.
    require(not (isinstance(spec,dict) and spec.get('schemaVersion') ==
                 'motionbrain.batch_registered_observer.v1'),
            'registered_observer_stationary_only')


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
    parsed = json.loads(raw)
    admission = None
    if 'motionAdmission' in config:
        from learning.batch_motion_admission import verify_admission
        admission = verify_admission(repository, config['motionAdmission'])
    if admission is None:
        _require_dispatch_observer(parsed)
    spec = validate_spec(parsed)
    require(spec['viewId'] == plan['viewId'] and spec['poseRegime'] == config['poseContext']
            and spec['cameraIdentity'] == config['deviceId'], 'runtime_observer_context_mismatch')
    bindings = config.get('conditionBindings', {})
    require(isinstance(bindings, dict) and set(bindings) <= {c['id'] for c in plan['conditions']}, 'runtime_condition_bindings_required')
    if 'gripper' in config:
        gripper = config['gripper']
        require(isinstance(gripper, dict) and set(gripper) <= {'url','bootId','required','observerSpecPath','observerSpecSha256','retentionCaptures'} and {'url','bootId'} <= set(gripper), 'runtime_gripper_configuration')
        require(type(gripper.get('required', True)) is bool, 'runtime_gripper_required_boolean')
        other = _gripper_spec(repository, config, spec)
        require(admission is not None or other is None or other['kind'] not in ('registered_gripper_points_stationary','registered_gripper_jaws_stationary'),
                'registered_gripper_observer_stationary_only')
        # Reject stationary-only nested observers before even constructing a
        # camera source. This constructor otherwise checks only origin/boot.
        GripperObservationSource(gripper['url'], 'batch-static-check', gripper['bootId'], {}, None)
        if 'retentionCaptures' in gripper:
            retention = gripper['retentionCaptures']
            require(isinstance(retention,dict) and set(retention) == {'count','intervalSeconds'} and
                    type(retention['count']) is int and retention['count'] == 2 and
                    type(retention['intervalSeconds']) in (int,float) and 1 <= retention['intervalSeconds'] <= 3,
                    'runtime_retention_capture_contract')

    if admission is not None:
        from learning.batch_motion_admission import require_scope
        other = _gripper_spec(repository, config, spec)
        for condition in plan['conditions']:
            require(bindings.get(condition['id'], {}).get('route', 'generic') == 'generic',
                    'admission_generic_route_only')
            command = condition['command']
            require_scope(admission, command, _model_context(config, spec, other),
                          _model_fields(spec, other, command['joint']))

    require(private_path(repository, config['journalPath']) == (repository/'.local/visual-actions/controller-commands.sqlite').resolve(), 'runtime_shared_journal_required')
    lock_name = sha(plan['controllerBootId'].encode())+'.lock'
    require(private_path(repository, config['dispatchLockPath']) == (repository/'.local/batch-locks'/lock_name).resolve(), 'runtime_shared_dispatch_lock_required')
    if 'afterReviewer' in config:
        reviewer = config['afterReviewer']
        require(isinstance(reviewer, dict) and set(reviewer) == {'kind','protocolVersion','maxWaitSeconds'} and
                reviewer['kind'] == 'file' and reviewer['protocolVersion'] == 1 and
                type(reviewer['maxWaitSeconds']) in (int,float) and 0 <= reviewer['maxWaitSeconds'] <= 3,
                'runtime_after_reviewer_contract')
    models = config.get('frozenModels', {})
    require(isinstance(models, dict), 'runtime_frozen_model_mapping_required')
    problems = {}
    for joint in models:
        try:
            _load_model(repository, config, spec, joint)
        except (ValueError, KeyError, OSError, StorageError) as exc:
            problems[joint] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
    route_problems, scope_owners = {}, {}
    for condition in plan['conditions']:
        binding = bindings.get(condition['id'], {})
        if isinstance(binding, dict) and binding.get('route') == 'm4_up':
            try:
                from learning.batch_m4 import validate_binding as validate_m4_binding
                route = validate_m4_binding(plan, condition, binding, repository)
                if binding.get('interaction') == 'noncontact':
                    require(spec['object'] == 'none' and spec['load'] == 'empty',
                            'runtime_m4_noncontact_empty_context_required')
                require(route['scope']['context']['object'] == spec['object'] and
                        route['scope']['context']['poseRegime'] == spec['poseRegime'] and
                        route['scope']['context']['externalDeviceId'] == config['deviceId'], 'runtime_m4_observer_context_changed')
                if 'gripper' in config:
                    require(route['scope']['context']['gripperCameraBootId'] == config['gripper']['bootId'], 'runtime_m4_gripper_boot_changed')
                keys = [('trialId',route['scope']['trialId']),('scopeSha256',binding['scopeSha256'])]
                for key in keys:
                    if key in scope_owners:
                        route_problems[condition['id']] = 'runtime_m4_scope_reused'
                        route_problems[scope_owners[key]] = 'runtime_m4_scope_reused'
                    scope_owners[key] = condition['id']
            except (ValueError, KeyError, OSError, StorageError) as exc:
                route_problems[condition['id']] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
    unsupported = []
    return dict(status='runtime_checked_no_device_access', helper=helper, spec=spec,
                config=config, unsupported=unsupported, modelProblems=problems, routeProblems=route_problems,
                motionAdmission=admission)


def _gripper_spec(repository, config, spec):
    gripper = config.get('gripper', {})
    require(('observerSpecPath' in gripper) == ('observerSpecSha256' in gripper), 'runtime_gripper_observer_binding_required')
    if 'observerSpecPath' not in gripper:
        return None
    raw = private_path(repository, gripper['observerSpecPath']).read_bytes()
    require(sha(raw) == gripper['observerSpecSha256'], 'runtime_gripper_observer_hash_changed')
    value = json.loads(raw)
    require(isinstance(value, dict), 'runtime_gripper_observer_spec_required')
    if value.get('kind') == 'camera_local_hsv':
        require(set(value) == {'kind','spec'}, 'runtime_gripper_local_hsv_fields')
        _require_dispatch_observer(value['spec'])
        spec_value = validate_spec(value['spec'])
        require(spec_value['cameraIdentity'] == 'gripper:'+gripper['bootId'] and
                spec_value['viewId'] == spec['viewId']+':gripper' and
                all(spec_value[k] == spec[k] for k in ('poseRegime','object','load')), 'runtime_gripper_observer_context_changed')
        other = dict(kind='camera_local_hsv', spec=spec_value, cameraIdentity=spec_value['cameraIdentity'],
                     maxAgeSeconds=spec_value['maxAgeSeconds'], joints=spec_value['joints'])
    elif value.get('kind') in ('registered_gripper_points_stationary','registered_gripper_jaws_stationary'):
        require(set(value) == {'kind','registrationPath','registrationSha256','referenceImagePath',
                'referenceImageSha256','context','maxAgeSeconds'}, 'runtime_gripper_points_fields')
        registration_raw = private_path(repository,value['registrationPath']).read_bytes()
        reference_raw = private_path(repository,value['referenceImagePath']).read_bytes()
        require(sha(registration_raw) == value['registrationSha256'] and
                sha(reference_raw) == value['referenceImageSha256'], 'runtime_gripper_registration_hash_changed')
        RegisteredGripperLandmarks(json.loads(registration_raw), reference_raw)
        require(value['context'] == {k:spec[k] for k in ('viewId','poseRegime','object','load')},
                'runtime_gripper_registration_context_changed')
        require(type(value['maxAgeSeconds']) in (int,float) and 0 < value['maxAgeSeconds'] <= 10,
                'runtime_gripper_points_age')
        other = dict(value, cameraIdentity='gripper:'+gripper['bootId'],
                     joints={'gripper':dict(stateFields=[dict(name=name) for name in
                         (('jawLeftXpx','jawLeftYpx','jawRightXpx','jawRightYpx','projectedJawSeparationPx')
                          if value['kind'] == 'registered_gripper_jaws_stationary' else
                          ('objectMinusJawMidXpx','objectMinusJawMidYpx','projectedJawSeparationPx'))])})
    elif value.get('kind') == 'registered_gripper_shaft_only':
        require(set(value) == {'kind','registrationPath','registrationSha256','referenceImagePath','referenceImageSha256','context','maxAgeSeconds'}, 'runtime_gripper_shaft_fields')
        registration_raw = private_path(repository,value['registrationPath']).read_bytes()
        reference_raw = private_path(repository,value['referenceImagePath']).read_bytes()
        require(sha(registration_raw) == value['registrationSha256'] and sha(reference_raw) == value['referenceImageSha256'], 'runtime_gripper_registration_hash_changed')
        registration = json.loads(registration_raw)
        tracker = RegisteredGripperCrosssection(registration, reference_raw)
        require(value['context'] == registration['sceneContext'] and
                value['context']['object'] == spec['object'] and
                value['context']['poseRegime'] == spec['poseRegime'], 'runtime_gripper_registration_context_changed')
        require(type(value['maxAgeSeconds']) in (int,float) and 0 < value['maxAgeSeconds'] <= 10, 'runtime_gripper_shaft_age')
        other = dict(value, cameraIdentity='gripper:'+gripper['bootId'],
                     joints={'gripper':dict(stateFields=[dict(name='shaftSeparationPx')])})
    else:
        raise ValueError('runtime_gripper_observer_kind_unsupported')
    require(3.5 < other['maxAgeSeconds'] <= 10, 'runtime_gripper_pre_window_age_incompatible')
    require(gripper.get('required', True), 'runtime_gripper_features_require_pair')
    return other


def _model_context(config, spec, other):
    context = {**{k:spec[k] for k in ('viewId','poseRegime','object','load')}, 'observerSha256':observer_digest(spec)}
    if other is not None:
        context.update(gripperObserverSha256=observer_digest(other), gripperCameraIdentity=other['cameraIdentity'])
    return context


def _model_fields(spec, other, joint):
    external = [f['name'] for f in spec['joints'][joint]['stateFields']]
    if other is None:
        return external
    if joint not in other['joints']:
        return external
    return ['external:'+n for n in external]+['gripper:'+f['name'] for f in other['joints'][joint]['stateFields']]


def _load_model(repository, config, spec, joint):
    binding = config['frozenModels'][joint]
    require(joint in spec['joints'] and isinstance(binding, dict) and set(binding) == {'path','sha256'}, 'runtime_frozen_model_binding')
    raw = private_path(repository, binding['path']).read_bytes()
    require(sha(raw) == binding['sha256'], 'runtime_frozen_model_hash_changed')
    model = validate_model(json.loads(raw))
    other = _gripper_spec(repository, config, spec)
    require(model['stateFields'] == _model_fields(spec, other, joint), 'runtime_frozen_model_state_fields_changed')
    require(model['joint'] == joint and model['context'] == _model_context(config, spec, other), 'runtime_frozen_model_context_changed')
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
    def __init__(self, plan, repository, root, credential, *, clock=time, m4_reviewer_factory=None):
        self.plan = copy.deepcopy(plan)
        self.repository, self.root = Path(repository).resolve(), Path(root).resolve()
        require(self.root.is_relative_to((self.repository/'.local').resolve()), 'private_batch_root_required')
        checked = validate_runtime(plan, self.repository)
        self.config, self.spec, self.helper = checked['config'], checked['spec'], checked['helper']
        self.motion_admission = checked['motionAdmission']
        self.model_problems = checked['modelProblems']
        self.route_problems = checked['routeProblems']
        self.m4_reviewer_factory = m4_reviewer_factory
        self.m4_adapter = None
        self.gripper_spec = _gripper_spec(self.repository, self.config, self.spec)
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
        self.motion_admission = checked['motionAdmission']
        require(checked['config'] == self.config and checked['spec'] == self.spec, 'runtime_binding_changed')

    def _token(self):
        if self.token is None:
            value = self.credential()
            require(isinstance(value, str) and bool(value), 'memory_credential_required')
            self.token = value
        return self.token

    def _context(self):
        return _model_context(self.config, self.spec, self.gripper_spec)

    def _states(self, external, paired, joint):
        names = _model_fields(self.spec, self.gripper_spec, joint)
        values = list(external['states'][joint]['state'])
        if self.gripper_spec is not None and joint in self.gripper_spec['joints']:
            require(paired is not None and paired.get('status') == 'accepted' and joint in paired['states'], 'batch_gripper_state_rejected')
            values += paired['states'][joint]['state']
        return names, values

    def _admission_scope(self, command, fields, state, *, side='before'):
        if 'motionAdmission' in self.config:
            from learning.batch_motion_admission import require_scope, _read
            admission = self.motion_admission
            require(_read(self.repository, self.config['motionAdmission']) == admission,
                    'admission_artifact_changed_after_validation')
            require_scope(admission, command, self._context(), fields, state, side=side)

    def _model(self, joint):
        return _load_model(self.repository, self.config, self.spec, joint)

    def _m4_model(self):
        config = copy.deepcopy(self.config)
        binding = config.get('m4FrozenModel', config.get('frozenModels', {}).get('shoulder'))
        require(binding is not None, 'runtime_m4_evaluation_frozen_model_required')
        config['frozenModels'] = {'shoulder':binding}
        # The dedicated M4 route preserves its original two-camera audit, but
        # its state model is explicitly external-only, separate from generic fusion.
        if 'gripper' in config:
            config['gripper'].pop('observerSpecPath',None)
            config['gripper'].pop('observerSpecSha256',None)
        return _load_model(self.repository, config, self.spec, 'shoulder')

    def _deadline(self):
        require(self.deadline_wall is None or (self.clock.time() < self.deadline_wall and
                self.clock.monotonic() < self.deadline_mono), 'batch_runtime_deadline')

    def _available_frame(self, recorder, *, margin=0, not_before=None):
        end = self.clock.monotonic()+2
        while True:
            recorder.check()
            now = self.clock.monotonic()
            frames = [f for f in copy.deepcopy(recorder.frames)
                      if f['sourceLatest'] <= now-margin and
                      f['stdoutReadAt']+f['persistenceMs']/1000 <= now and
                      (not_before is None or (f['sourceEarliest'] >= not_before and not f.get('qualityFlags',[])))]
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
                     finalIdleOffVerified=manager.current.get('finalIdleOffVerified') is True,
                     outputSequence=None if port is None else port.last_sequence)
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

    def _m4_route(self):
        if self.m4_adapter is None:
            from learning.batch_m4 import BatchM4Route
            model = (lambda:self._m4_model()) if ('m4FrozenModel' in self.config or 'shoulder' in self.config.get('frozenModels', {})) else None
            self.m4_adapter = BatchM4Route(self.plan, self.repository, self.root, self._token,
                self.m4_reviewer_factory, observer_spec=self.spec, frozen_model=model, clock=self.clock,
                after_result=self._m4_after_result)
        return self.m4_adapter

    def _binding(self, condition):
        return self.config.get('conditionBindings', {}).get(condition['id'],
            dict(route='generic', interaction='noncontact'))

    def _gripper_factory(self):
        config = self.config.get('gripper')
        if config is None:
            return None
        def factory(session_id, store):
            scene = dict(physicalView=self.plan['viewId']+':gripper-raw', object=self.spec['object'],
                         poseRegime=self.spec['poseRegime'], landmarks='batch-paired-raw-v1')
            return GripperObservationSource(config['url'], session_id, config['bootId'], scene, store,
                                            max_captures=2+config.get('retentionCaptures',{}).get('count',0), clock=self.clock)
        return factory

    def _after_capture(self, manager, pair, gripper):
        config = self.config['gripper']['retentionCaptures']
        result = dict(status='collecting', intervalSeconds=config['intervalSeconds'], entries=[])
        try:
            require(gripper is not None and pair.get('status') == 'recorded', 'batch_retention_initial_pair_required')
            previous = pair['after']
            initial = self._available_frame(manager.camera,not_before=previous['sourceEarliest'])
            result['initialExternal'] = initial
            previous_external_end = initial['sourceLatest']
            for _ in range(config['count']):
                next_at = previous['persistedAt']+config['intervalSeconds']
                while self.clock.monotonic() < next_at:
                    self._deadline()
                    manager.tick()
                    require(manager.current['phase'] == 'recording', 'batch_retention_recording_ended')
                    self.clock.sleep(min(.05,next_at-self.clock.monotonic()))
                self._deadline()
                manager.port.check_idle()
                record = gripper.capture(deadline=min(self.clock.monotonic()+6,self.deadline_mono))
                gripper.verify(record)
                manager.port.check_idle()
                manager.camera.check()
                frame = self._available_frame(manager.camera,not_before=max(record['sourceEarliest'],previous_external_end+config['intervalSeconds']))
                binding = gripper._bindings[record['frame']['frameId']]
                folder = gripper.store.root/binding['folder']
                snapshot = self._legacy_snapshot(manager.store.root, manager.camera.root, frame, folder, len(result['entries'])+1)
                result['entries'].append(dict(external=frame,gripper=copy.deepcopy(record),
                    gripperFolder=str(folder.relative_to(manager.store.root)), legacySnapshot=snapshot))
                previous = record
                previous_external_end = frame['sourceLatest']
            result['status'] = 'recorded'
        except (ValueError,KeyError,OSError,StorageError) as exc:
            result.update(status='rejected',reason=type(exc).__name__)
        manager.store.write_json('batch-retention-captures.json', result)
        return result

    def _legacy_snapshot(self, source, native, frame, gripper_folder, ordinal):
        """원본 bytes와 메타데이터를 그대로 보존한다. 물리 판독을 생성하지 않는다."""
        import cv2
        import numpy as np
        folder = source/'retention'/('snapshot-'+str(ordinal).zfill(2))
        store = ArtifactStore(folder)
        ingress = 'host-ingress/'+str(frame['frameIndex']).zfill(4)+'.json'
        require(json.loads(_read(native,ingress)) == frame, 'batch_retention_native_ingress_changed')
        copies = {}
        pairs = [(native/name,'external/'+name) for name in (frame['metadataFile'],frame['imageFile'],ingress,'host-ingress/clock-before.json')]
        pairs += [(gripper_folder/'esp32-frame.jpg','gripper/esp32-after.jpg'),
                  (gripper_folder/'esp32-frame.json','gripper/esp32-after.json')]
        for original,target in pairs:
            relative = str(original.relative_to(source))
            raw = _read(source,relative)
            store._write(target,raw)
            copies[target] = dict(source=relative,sha256=sha(raw))
        pixels = decode(_read(native,frame['imageFile']),frame['rawByteLength'],frame['pixelSha256'])
        image = np.frombuffer(pixels,np.uint8).reshape(frame['height'],frame['width'],4)
        ok,png = cv2.imencode('.png',image)
        require(ok,'batch_retention_preview_encoding_failed')
        store._write('after-external.png',png.tobytes())
        store._write('after-gripper.jpg',_read(gripper_folder,'esp32-frame.jpg'))
        sources = dict(external=dict(root='external',ingressFile=ingress,
            ingressSha256=copies['external/'+ingress]['sha256'],clockSha256=copies['external/host-ingress/clock-before.json']['sha256']),
            gripper=dict(root='gripper',label='after',metadataSha256=copies['gripper/esp32-after.json']['sha256'],
                         imageSha256=copies['gripper/esp32-after.jpg']['sha256']))
        store.write_json('source.json',dict(hostClockId=frame['frameId'].rsplit(':',1)[0],sources=sources,
            previews=dict(external='after-external.png',gripper='after-gripper.jpg'),physicalJudgmentGenerated=False))
        return dict(root=str(folder.relative_to(source)), sources=sources, copiedSources=copies,
                    previewSha256=sha(png.tobytes()))

    def _audit_legacy_snapshot(self, source, snapshot, session_id):
        import cv2
        import numpy as np
        from learning import m4_up_commissioning as legacy
        folder = (source/snapshot['root']).resolve()
        require(folder.is_relative_to((source/'retention').resolve()), 'batch_retention_snapshot_outside_session')
        for target,original in snapshot['copiedSources'].items():
            raw = _read(source,original['source'])
            require(sha(raw) == original['sha256'] and _read(folder,target) == raw, 'batch_retention_snapshot_source_changed')
        scope = dict(hostClockId=session_id,context=dict(externalDeviceId=self.config['deviceId'],
            externalViewId=self.plan['viewId'],gripperCameraBootId=self.config['gripper']['bootId']))
        external = legacy._external_frame(folder,snapshot['sources']['external'],scope)
        gripper = legacy._gripper_frame(folder,snapshot['sources']['gripper'],scope,'after')
        preview = _read(folder,'after-external.png')
        require(sha(preview) == snapshot['previewSha256'],'batch_retention_preview_changed')
        pixels = cv2.imdecode(np.frombuffer(preview,np.uint8),cv2.IMREAD_UNCHANGED)
        require(pixels is not None and pixels.shape == (external['height'],external['width'],4) and
                sha(pixels.tobytes()) == external['pixelSha256'],'batch_retention_preview_not_source_pixels')
        require(_read(folder,'after-gripper.jpg') == _read(folder,'gripper/esp32-after.jpg'),'batch_retention_gripper_preview_changed')
        return dict(root=snapshot['root'],sources=copy.deepcopy(snapshot['sources']),external=external,gripper=gripper,
                    sourceHashes={target:original['sha256'] for target,original in snapshot['copiedSources'].items()},
                    physicalJudgmentGenerated=False)

    def _retention_transition(self, source, row):
        if 'retentionCaptures' not in self.config.get('gripper',{}):
            return None
        raw = json.loads(_read(source,'batch-retention-captures.json'))
        require(raw['status'] == 'recorded' and len(raw['entries']) == 2, 'batch_retention_capture_incomplete')
        session = json.loads(_read(source,'session.json'))
        config = self.config['gripper']['retentionCaptures']
        require(raw['intervalSeconds'] == config['intervalSeconds'], 'batch_retention_interval_changed')
        previous, metadata, boot = read_gripper_frame(source/'gripper-pair','after',row['sessionId'])
        last_gripper_end = metadata['persistedAt']
        initial = raw['initialExternal']
        require(initial == session['frames'][initial['frameIndex']] and initial['sourceEarliest'] >= metadata['requestStartedAt'], 'batch_retention_initial_external_changed')
        previous_external_end = initial['sourceLatest']
        external_ids, grip_ids, entries = set(), {previous['frameId']}, []
        for entry in raw['entries']:
            external = entry['external'];index = external['frameIndex']
            require(index < len(session['frames']) and session['frames'][index] == external and
                    external['sourceEarliest'] >= max(row['evidence']['offReceiptAt']+3,previous_external_end+config['intervalSeconds']) and
                    external['frameId'] not in external_ids, 'batch_retention_external_source_changed')
            folder = source/entry['gripperFolder']
            require(folder.resolve().is_relative_to((source/'gripper-camera').resolve()), 'batch_retention_source_outside_session')
            frame,meta,current_boot = read_gripper_frame(folder,'frame',row['sessionId'])
            require(current_boot == boot == self.config['gripper']['bootId'] and
                    frame['frameId'] not in grip_ids and frame['requestedUs'] > previous['capturedUs'], 'batch_retention_gripper_source_changed')
            require(meta['requestStartedAt'] >= last_gripper_end+config['intervalSeconds'] and
                    meta['requestStartedAt'] >= row['evidence']['offReceiptAt']+3 and
                    meta['persistedAt'] <= session['endedAt'], 'batch_retention_timing_changed')
            require(entry['gripper']['frame']['frameId'] == frame['frameId'] and
                    entry['gripper']['frame']['pixelSha256'] == frame['pixelSha256'], 'batch_retention_record_changed')
            snapshot = self._audit_legacy_snapshot(source,entry['legacySnapshot'],row['sessionId'])
            require(snapshot['external']['frameId'] == external['frameId'] and snapshot['gripper']['frameId'] == frame['frameId'], 'batch_retention_snapshot_binding_changed')
            entries.append(dict(external=external,gripper=frame,metadata=meta,gripperFolder=entry['gripperFolder'],
                legacySnapshot=snapshot,
                sourceHashes={name:sha(_read(folder,name)) for name in ('esp32-frame.jpg','esp32-frame.json')}))
            external_ids.add(external['frameId']);grip_ids.add(frame['frameId'])
            previous,last_gripper_end=frame,meta['persistedAt']
            previous_external_end=external['sourceLatest']
        return dict(status='recorded',entries=entries,exposureSynchronizationVerified=False)

    def _observe_gripper(self, folder, side, session_id, *, historical=False):
        if self.gripper_spec is None:
            return None
        import cv2
        import numpy as np
        frame, metadata, boot = read_gripper_frame(folder, side, session_id)
        require(boot == self.config['gripper']['bootId'], 'batch_gripper_boot_changed')
        raw = _read(folder, 'esp32-'+side+'.jpg')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        derived = dict(frameId=frame['frameId'], imageSha256=sha(raw), pixelSha256=sha(image.tobytes()),
            capturedAt=metadata['requestStartedAt'], persistedAt=metadata['persistedAt'], clockId=self.clock_id,
            cameraIdentity='gripper:'+boot)
        now = metadata['persistedAt'] if historical else self.clock.monotonic()
        if self.gripper_spec['kind'] == 'camera_local_hsv':
            local = self.gripper_spec['spec']
            context = {k:local[k] for k in ('viewId','poseRegime','object','load')}
            return observe_frame(raw, derived, local, context, now, self.clock_id)
        require(0 <= now-derived['capturedAt'] <= self.gripper_spec['maxAgeSeconds'], 'batch_gripper_input_stale')
        spec = self.gripper_spec
        registration = json.loads(private_path(self.repository,spec['registrationPath']).read_bytes())
        reference = private_path(self.repository,spec['referenceImagePath']).read_bytes()
        if spec['kind'] == 'registered_gripper_jaws_stationary':
            observed = RegisteredGripperLandmarks(registration,reference).observe_jaws(raw,frame['frameId'])
            require(observed['pixelSha256'] == frame['pixelSha256'], 'batch_gripper_points_pixels_changed')
            derived['pixelSha256'] = frame['pixelSha256']
            return dict(status='accepted',states={'gripper':dict(state=observed['state'],stateFields=observed['stateFields'])},
                nodeMatches=[],frameBinding=derived,specSha256=observer_digest(spec),
                points=observed['points'],quality=observed['quality'],landmarkContract=observed['landmarkContract'],
                semantics=observed['semantics'],motionTrackingVerified=False,trainingEligible=False,controlEligible=False)
        if spec['kind'] == 'registered_gripper_points_stationary':
            observed = RegisteredGripperLandmarks(registration,reference).observe(raw,frame['frameId'])
            require(observed['pixelSha256'] == frame['pixelSha256'], 'batch_gripper_points_pixels_changed')
            # Registered annotations use canonical BGRA pixels; HSV above uses BGR.
            derived['pixelSha256'] = frame['pixelSha256']
            return dict(status='accepted', states={'gripper':dict(state=observed['state'],
                        stateFields=[f['name'] for f in spec['joints']['gripper']['stateFields']])},
                        nodeMatches=[],frameBinding=derived,specSha256=observer_digest(spec),
                        annotation=observed['annotation'],quality=observed['quality'],
                        semantics='projected_surface_relative_to_jaw_corners_not_depth_contact_or_force',
                        motionTrackingVerified=False,trainingEligible=False,controlEligible=False)
        observed = RegisteredGripperCrosssection(registration,reference).observe_shaft(raw,frame['frameId'])
        require(observed['pixelSha256'] == frame['pixelSha256'], 'batch_gripper_shaft_pixels_changed')
        return dict(status='accepted', states={'gripper':dict(state=[observed['shaftSeparationPx']],stateFields=['shaftSeparationPx'])},
                    nodeMatches=[],frameBinding=derived,specSha256=observer_digest(spec),
                    semantics='shaft_separation_not_actual_tips_or_contact',controlEligible=False)

    def _pair_before(self, manager, pair):
        if 'gripper' not in self.config:
            return None
        require(isinstance(pair, dict) and 'before' in pair and 'path' in pair, 'batch_gripper_before_required')
        path = Path(pair['path']).resolve()
        require(path == (manager.store.root/'gripper-pair').resolve(), 'batch_gripper_before_path_changed')
        frame, metadata, boot = read_gripper_frame(path, 'before', manager.current['sessionId'])
        require(boot == self.config['gripper']['bootId'], 'batch_gripper_boot_changed')
        require(frame['frameId'] == pair['before']['frame']['frameId'] and
                frame['pixelSha256'] == pair['before']['frame']['pixelSha256'], 'batch_gripper_before_binding_changed')
        result = dict(frame=frame, metadata=metadata, bootId=boot,
            fileHashes={n:sha(_read(path,n)) for n in ('esp32-before.jpg','esp32-before.json')})
        result['observation'] = self._observe_gripper(path, 'before', manager.current['sessionId'])
        manager.store.write_json('batch-gripper-before.json', result)
        return result

    def _pair_transition(self, source, row):
        if 'gripper' not in self.config:
            return None
        pair = bind_gripper_pair(copy.deepcopy(row), source/'gripper-pair')
        identity = pair['evidence']['cameraIdentity']
        require(identity['cameraBootId'] == self.config['gripper']['bootId'], 'batch_gripper_boot_changed')
        saved = json.loads(_read(source, 'batch-gripper-before.json'))
        require(saved['frame'] == pair['inputs']['image'], 'batch_gripper_before_source_changed')
        return dict(before=pair['inputs']['image'], after=pair['targets']['image'],
                    cameraIdentity=identity, timing=pair['evidence']['gripperFrameEvidence'],
                    fileHashes={n:sha(_read(source/'gripper-pair',n)) for n in (
                        'esp32-before.jpg','esp32-before.json','esp32-after.jpg','esp32-after.json')},
                    observerBinding=copy.deepcopy(self.config['gripper']),
                    externalObserverBinding=dict(path=self.config['observerSpecPath'],sha256=self.config['observerSpecSha256']),
                    sourceKind='paired_gripper_separate_camera_contract',
                    beforeObservation=self._observe_gripper(source/'gripper-pair','before',row['sessionId'],historical=True),
                    afterObservation=self._observe_gripper(source/'gripper-pair','after',row['sessionId'],historical=True),
                    exposureSynchronizationVerified=False)

    def _before(self, condition):
        def check(manager, pair):
            manager.camera.check()
            self._deadline()
            paired = self._pair_before(manager, pair)
            frame = self._available_frame(manager.camera, margin=.2)
            evidence = self._observe_native(manager.camera.root, frame)
            result = evidence['observation']
            require(result.get('status') == 'accepted' and
                    bool(set(result.get('nodeMatches', [])) & set(condition['fromNodes'])), 'batch_current_node_not_supported')
            require(self.clock.monotonic()-frame['sourceEarliest'] <= self.plan['maxObservationAgeSeconds'], 'batch_current_frame_stale')
            manager.store.write_json('batch-before-observation.json', evidence)
            self.latest_before = evidence
            from learning.batch_interaction import validate_binding, before_command
            binding = self._before_binding(manager, pair, condition, evidence)
            checked = validate_binding(self.plan, condition, binding, self.repository)
            self.interaction_token = before_command(manager, pair, condition, binding, self.deadline_wall,
                checked=checked, clock=self.clock)
            manager.store.write_json('batch-interaction-before.json', self.interaction_token.report)
            joint = condition['command']['joint']
            names, values = self._states(result, None if paired is None else paired['observation'], joint)
            self._admission_scope(condition['command'], names, values)
            if condition['phase'] == 'evaluation':
                model = self._model(condition['command']['joint'])
                joint = condition['command']['joint']
                row = dict(sessionId=manager.current['sessionId'],
                           before=values, command=condition['command'],
                           context=self._context(), beforePersistedAt=evidence['derivedMetadata']['persistedAt'])
                proof = prospective_evidence(model, row, self.clock.monotonic())
                manager.store.write_json('batch-prospective.json', dict(
                    proof=proof, beforeFrameId=frame['frameId'], beforePixelSha256=frame['pixelSha256']))
            self._deadline()
        return check

    def _before_binding(self, manager, pair, condition, evidence):
        return self._binding(condition)

    def _validate_interaction_binding(self, condition, binding, route):
        if route == 'm4_up':
            from learning.batch_m4 import validate_binding
        else:
            from learning.batch_interaction import validate_binding
        return validate_binding(self.plan, condition, binding, self.repository)

    def validate_condition(self, condition):
        binding = self.config.get('conditionBindings', {}).get(condition['id'])
        if binding is None:
            if self.spec['load'] != 'empty' or self.spec['object'] != 'none':
                return 'runtime_explicit_interaction_binding_required'
            binding = dict(route='generic', interaction='noncontact')
        if not isinstance(binding, dict):
            return 'runtime_condition_binding_invalid'
        route = binding.get('route', 'generic')
        if route == 'm4_up':
            if condition['id'] in self.route_problems:
                return self.route_problems[condition['id']]
            if not callable(self.m4_reviewer_factory):
                return 'runtime_m4_explicit_reviewer_required'
        elif route != 'generic':
            return 'runtime_route_adapter_not_connected'
        try:
            if route == 'm4_up':
                from learning.batch_m4 import validate_binding
            else:
                from learning.batch_interaction import validate_binding
                require(binding['interaction'] != 'noncontact' or self.spec['load'] == 'empty', 'runtime_interaction_load_mismatch')
            self._validate_interaction_binding(condition, binding, route)
        except (ValueError, KeyError, OSError, StorageError) as exc:
            return 'runtime_interaction_rejected:'+type(exc).__name__
        joint = condition['command']['joint']
        if condition['phase'] == 'evaluation':
            if route == 'm4_up':
                try:
                    model = self._m4_model()
                except (ValueError,KeyError,OSError,StorageError) as exc:
                    return 'runtime_m4_evaluation_model_rejected:'+type(exc).__name__
            else:
                if joint not in self.config.get('frozenModels', {}):
                    return 'runtime_evaluation_frozen_model_required'
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
        from learning.batch_dispatch_budget import reserve
        reserve(self.repository, self.plan, condition, request_id)
        binding = self._binding(condition)
        if binding.get('route') == 'm4_up':
            return self._m4_route().execute(condition, request_id, binding, deadline_wall=deadline_wall)
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
                    if runtime.interaction_token is not None:
                        from learning.batch_interaction import before_run_cached
                        before_run_cached(runtime.interaction_token, runtime.clock.monotonic(), runtime.clock.time(), manager.port.last_sequence)
                    if before_run is not None:
                        before_run()
                return super().pulse(payload, before_run=guard)
        self.latest_before = None
        self.interaction_token = None
        self.manager = GuardedCollection(self.root/'sessions', self.helper, self.config['controllerUrl'],
                                        self._token(), self.config['deviceId'], self.plan['viewId'],
                                        journal_path=self.journal, clock=self.clock, sample_fps=8,
                                        port_factory=PinnedRecordingPort)
        try:
            extras = {'after_capture':self._after_capture} if 'retentionCaptures' in self.config.get('gripper',{}) else {}
            report = collect_once(self.manager, request_id, condition['command'], self.config['poseContext'],
                                  onsite_confirmed=True, request_root=self.root/'requests',
                                  approval_plan_sha256=self.plan_sha, before_command=self._before(condition),
                                  gripper_source_factory=self._gripper_factory(), **extras)
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

    def _live_after_review(self, source, condition, row, output_sequence):
        session = json.loads(_read(source,'session.json'))
        request = dict(schemaVersion='motionbrain.batch_after_review_request.v1', conditionId=condition['id'],
            source=dict(sessionId=session['sessionId'],path=str(source),sha256=sha(_read(source,'session.json'))),
            context={k:self.spec[k] for k in ('viewId','poseRegime','object','load')},
            offAt=row['evidence']['offReceiptAt'],outputSequence=output_sequence,
            frames=dict(external=[{k:f[k] for k in ('frameId','pixelSha256')} for f in session['frames']],gripper=[]))
        if 'gripper' in self.config:
            paired = self._pair_transition(source,row)
            request['frames']['gripper'] = [{k:paired[side][k] for k in ('frameId','pixelSha256')} for side in ('before','after')]
        if 'retentionCaptures' in self.config.get('gripper',{}):
            supplemental = self._retention_transition(source,row)
            for entry in supplemental['entries']:
                for camera in ('external','gripper'):
                    request['frames'][camera].append({k:entry[camera][k] for k in ('frameId','pixelSha256')})
        return self._request_after_review(source,request)

    def _request_after_review(self,source,request):
        self._save(source/'batch-after-review-request.json',request)
        response_path = source/'batch-after-review-response.json'
        limit = self.clock.monotonic()+self.config['afterReviewer']['maxWaitSeconds']
        while not response_path.exists():
            if self.clock.monotonic() >= limit:
                return None
            if self.deadline_wall is not None and (self.clock.time() >= self.deadline_wall or self.clock.monotonic() >= self.deadline_mono):
                return None
            self.clock.sleep(min(.05,max(0,limit-self.clock.monotonic())))
        raw = _read(source,response_path.name)
        require(len(raw) <= 1024*1024, 'runtime_after_review_response_too_large')
        response = json.loads(raw)
        from learning.batch_interaction import validate_after_response
        review = validate_after_response(request,response)
        self._save(source/'batch-after-review-envelope.json',review)
        return dict(reviewPath=str((source/'batch-after-review-envelope.json').relative_to(self.repository)),
                    reviewSha256=sha(_read(source,'batch-after-review-envelope.json')),
                    interaction=review['interaction'], request=request)

    def _interaction_after(self, source, condition, row):
        binding = self._binding(condition)
        report = dict(status='not_reviewed', contactState='unknown', graspVerified=False,
                      liftVerified=False, retainedLift=False)
        has_path, has_sha = 'afterReviewPath' in binding, 'afterReviewSha256' in binding
        live = 'afterReviewer' in self.config
        if not has_path and not has_sha and not live:
            return report
        try:
            proof = json.loads(_read(source, 'batch-execution-proof.json'))
            if live:
                resolved = self._live_after_review(source,condition,row,proof['outputSequence'])
                if resolved is None:
                    return dict(report, reason='after_review_not_supplied')
                request = resolved.pop('request')
                checked_binding = dict(binding,**resolved)
            else:
                # Static hashes only support already-existing offline evidence;
                # actual future after evidence uses the pinned file protocol.
                require(has_path and has_sha, 'runtime_after_review_binding_required')
                checked_binding = dict(binding, reviewPath=binding['afterReviewPath'],reviewSha256=binding['afterReviewSha256'])
                raw_review = private_path(self.repository,binding['afterReviewPath']).read_bytes()
                require(sha(raw_review) == binding['afterReviewSha256'],'runtime_after_review_bytes_changed')
                checked_binding['interaction'] = json.loads(raw_review)['interaction']
            from learning.batch_interaction import validate_binding, after_command
            checked = validate_binding(self.plan, condition, checked_binding, self.repository,purpose='label')
            verified = after_command(checked, now=self.clock.monotonic(),
                output_sequence=proof['outputSequence'], after_off=row['evidence']['offReceiptAt'])
            if live:
                from learning.batch_interaction import verify_after_frame_membership
                verify_after_frame_membership(verified,request)
            return dict(status='reviewed', **verified.report)
        except (ValueError, KeyError, OSError, StorageError) as exc:
            return dict(report, status='rejected', reason=type(exc).__name__)

    def _m4_after_result(self,condition,request_id,folder,result):
        report=dict(status='not_reviewed',contactState='unknown',graspVerified=False,liftVerified=False,retainedLift=False)
        if 'afterReviewer' not in self.config or result.get('execution') != 'completed' or result.get('offVerified') is not True:
            return report
        try:
            from learning.batch_m4 import audit_source,audit_retention_source
            from learning.batch_interaction import validate_binding,after_command,verify_after_frame_membership
            binding=self._binding(condition)
            audited=audit_source(folder,sha(_read(folder,'result.json')),sha(_read(folder,'scope.json')))
            scope=audited['scope']
            sequence=result['evidence']['outputSequence']
            require(type(sequence) is int,'runtime_m4_after_output_sequence_required')
            request=dict(schemaVersion='motionbrain.batch_after_review_request.v1',conditionId=condition['id'],
                source=dict(sessionId=scope['hostClockId'],path=str(folder),sha256=audited['sourceSha256']),
                context={k:self.spec[k] for k in ('viewId','poseRegime','object','load')},
                offAt=audited['result']['receipt']['offAt'],outputSequence=sequence,
                frames={camera:[{k:audited['pairs'][side][camera][k] for k in ('frameId','pixelSha256')}
                    for side in ('before','after')] for camera in ('external','gripper')})
            supplemental=audit_retention_source(folder,audited)
            if supplemental is not None and supplemental.get('status') == 'recorded':
                for entry in supplemental['entries']:
                    for camera in ('external','gripper'):
                        request['frames'][camera].append({k:entry[camera][k] for k in ('frameId','pixelSha256')})
            resolved=self._request_after_review(folder,request)
            if resolved is None:
                return dict(report,reason='after_review_not_supplied')
            resolved.pop('request')
            reviewed_binding=dict(binding,**resolved)
            review_plan=copy.deepcopy(self.plan)
            review_plan['runtime']['gripper']=dict(bootId=scope['context']['gripperCameraBootId'])
            checked=validate_binding(review_plan,condition,reviewed_binding,self.repository,purpose='label')
            token=after_command(checked,now=self.clock.monotonic(),output_sequence=sequence,after_off=request['offAt'])
            verify_after_frame_membership(token,request)
            return dict(status='reviewed',**token.report)
        except (ValueError,KeyError,OSError,StorageError) as exc:
            return dict(report,status='rejected',reason=type(exc).__name__)

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
                    if 'retentionCaptures' in self.config.get('gripper',{}):
                        try:
                            transition['supplementalObservations'] = self._retention_transition(source,row)
                        except (ValueError,KeyError,OSError,StorageError) as exc:
                            transition['supplementalObservations'] = dict(status='rejected',reason=type(exc).__name__)
                    transition['interactionAfter'] = self._interaction_after(source, condition, row)
                    interaction_file = source/'batch-interaction-before.json'
                    if interaction_file.is_file():
                        transition['interactionBefore'] = json.loads(_read(source, interaction_file.name))
                    if 'gripper' in self.config:
                        try:
                            transition['gripperPair'] = self._pair_transition(source, row)
                        except (ValueError, KeyError, OSError, StorageError) as exc:
                            if self.config['gripper'].get('required', True):
                                raise
                            transition['gripperPair'] = dict(status='rejected', reason=type(exc).__name__)
                    paired = transition.get('gripperPair', {})
                    names, values_before = self._states(a, paired.get('beforeObservation'), joint)
                    _, values_after = self._states(b, paired.get('afterObservation'), joint)
                    self._admission_scope(condition['command'], names, values_before)
                    self._admission_scope(condition['command'], names, values_after, side='after')
                    transition['stateFields'] = names
                    transition['beforeState'] = values_before
                    transition['afterState'] = values_after
                    transition['learningRow'] = dict(
                        sessionId=row['sessionId'], episodeId=':'.join(audit['execution']['episode']),
                        sourceHash=source_sha, frameIds=transition['frameIds'],
                        captureGroup=condition['captureGroup'], joint=joint,
                        context=self._context(),
                        stateFields=transition['stateFields'], before=transition['beforeState'],
                        after=transition['afterState'], command=transition['command'],
                        commandAt=command_at, beforePersistedAt=before['derivedMetadata']['persistedAt'])
                    if transition.get('gripperPair', {}).get('before'):
                        transition['learningRow']['pairedFrameIds'] = [
                            transition['gripperPair'][side]['frameId'] for side in ('before','after')]
                        transition['learningRow']['pairedSourceHashes'] = transition['gripperPair']['fileHashes']
                    if transition.get('supplementalObservations',{}).get('status') == 'recorded':
                        transition['learningRow']['supplementalFrameIds'] = [entry[camera]['frameId']
                            for entry in transition['supplementalObservations']['entries'] for camera in ('external','gripper')]
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
                    if 'pairedFrameIds' in transition['learningRow']:
                        result['dataIdentity']['pairedFrameIds'] = copy.deepcopy(transition['learningRow']['pairedFrameIds'])
                    if 'supplementalFrameIds' in transition['learningRow']:
                        result['dataIdentity']['supplementalFrameIds'] = copy.deepcopy(transition['learningRow']['supplementalFrameIds'])
            except (ValueError, KeyError, OSError, StorageError) as exc:
                result['reason'] = 'observation_processing_rejected:'+type(exc).__name__
                if isinstance(exc, (ValueError, KeyError)):
                    result['observationFailureCode'] = str(exc)
        return result

    def reconcile(self, condition, request_id):
        """재시작 시 파일만 읽는다. 토큰 조회·HTTP·카메라·명령 실행 없음."""
        binding = self._binding(condition)
        if binding.get('route') == 'm4_up':
            return self._m4_route().reconcile(condition, request_id, binding)
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
                if transition.get('supplementalObservations',{}).get('status') == 'recorded':
                    supplemental = self._retention_transition(Path(report['sessionPath']),result['evidence']['audit']['visual']['row'])
                    require(supplemental == transition['supplementalObservations'],'batch_retention_transition_changed')
                if transition.get('gripperPair', {}).get('before'):
                    paired = self._pair_transition(Path(report['sessionPath']), result['evidence']['audit']['visual']['row'])
                    require(paired == transition['gripperPair'], 'batch_gripper_pair_changed')
                require(transition['sourceSha256'] == report['sourceSha256']
                        and transition['conditionId'] == condition['id'], 'batch_transition_source_changed')
                result.update(observation='eligible', transitionId=saved['transitionId'], reason='reconciled_verified_transition',
                    dataIdentity={k:copy.deepcopy(transition['learningRow'][k]) for k in ('sourceHash','sessionId','episodeId','frameIds')})
                if 'pairedFrameIds' in transition['learningRow']:
                    result['dataIdentity']['pairedFrameIds'] = copy.deepcopy(transition['learningRow']['pairedFrameIds'])
                if 'supplementalFrameIds' in transition['learningRow']:
                    result['dataIdentity']['supplementalFrameIds'] = copy.deepcopy(transition['learningRow']['supplementalFrameIds'])
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


def observe_gripper_frame_offline(repository, external_spec, gripper_config, folder, side, session_id):
    """저장 프레임만 관측한다. 실행기·인증·카메라를 생성하지 않는다."""
    repository, folder = Path(repository).resolve(), Path(folder).resolve()
    require(folder.is_relative_to((repository/'.local').resolve()), 'private_pair_source_required')
    spec = validate_spec(external_spec)
    config = dict(gripper=copy.deepcopy(gripper_config))
    other = _gripper_spec(repository, config, spec)
    require(other is not None and other['kind'] in ('registered_gripper_points_stationary','registered_gripper_jaws_stationary'),
            'offline_gripper_points_observer_required')
    runtime = object.__new__(BatchRuntime)
    runtime.repository, runtime.config, runtime.spec = repository, config, spec
    runtime.gripper_spec, runtime.clock_id = other, session_id
    return runtime._observe_gripper(folder, side, session_id, historical=True)


def audit_pair_transition(source, row, transition, repository):
    """기존 이중 카메라 원본을 재관측해 결합 상태를 재계산한다. 장치 접근 없음.

    source는 이미 해당 session 원본 감사를 통과해야 한다. 이 함수는 새 runtime을
    생성하지 않고 파일 전용 관측·결합 함수만 사용한다.
    """
    source, repository = Path(source).resolve(), Path(repository).resolve()
    require(source.is_relative_to((repository/'.local').resolve()), 'private_pair_source_required')
    paired = transition.get('gripperPair')
    require(isinstance(paired, dict) and 'before' in paired, 'batch_paired_transition_required')
    binding = paired['externalObserverBinding']
    raw = private_path(repository, binding['path']).read_bytes()
    require(sha(raw) == binding['sha256'], 'batch_external_observer_hash_changed')
    spec = validate_spec(json.loads(raw))
    require(observer_digest(spec) == transition['before']['observation']['specSha256'], 'batch_external_observer_contract_changed')
    runtime = object.__new__(BatchRuntime)
    runtime.repository = repository
    runtime.spec = spec
    runtime.config = dict(gripper=copy.deepcopy(paired['observerBinding']),
                          observerSpecPath=binding['path'], observerSpecSha256=binding['sha256'])
    runtime.gripper_spec = _gripper_spec(repository, runtime.config, spec)
    runtime.clock_id = transition['before']['derivedMetadata']['clockId']
    recomputed = runtime._pair_transition(source, row)
    require(recomputed == paired, 'batch_paired_transition_changed')
    joint = transition['joint']
    fields, before = runtime._states(transition['before']['observation'], recomputed.get('beforeObservation'), joint)
    _, after = runtime._states(transition['after']['observation'], recomputed.get('afterObservation'), joint)
    extra = {k:v for k,v in runtime._context().items() if k not in ('viewId','poseRegime','object','load','observerSha256')}
    return dict(beforeState=before, afterState=after, stateFields=fields, extraContext=extra,
                pairedFrameIds=[recomputed[side]['frameId'] for side in ('before','after')],
                pairedSourceHashes=recomputed['fileHashes'])


def audit_retention_transition(source, row, transition, repository):
    """추가 촬영 원본과 legacy 호환 snapshot을 재감사해 추가 frame ID를 반환한다."""
    source, repository = Path(source).resolve(), Path(repository).resolve()
    require(source.is_relative_to((repository/'.local').resolve()), 'private_retention_source_required')
    paired = transition['gripperPair']
    binding = paired['externalObserverBinding']
    raw = private_path(repository,binding['path']).read_bytes()
    require(sha(raw) == binding['sha256'],'batch_retention_observer_hash_changed')
    spec = validate_spec(json.loads(raw))
    runtime = object.__new__(BatchRuntime)
    runtime.repository = repository
    runtime.spec = spec
    runtime.plan = dict(viewId=spec['viewId'])
    runtime.config = dict(deviceId=spec['cameraIdentity'],gripper=copy.deepcopy(paired['observerBinding']))
    rebuilt = runtime._retention_transition(source,row)
    require(rebuilt == transition.get('supplementalObservations'),'batch_retention_transition_changed')
    return [entry[camera]['frameId'] for entry in rebuilt['entries'] for camera in ('external','gripper')]
