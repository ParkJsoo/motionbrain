"""별도 승인된 M3 UP 한 번으로 원래 비접촉 범위에 복귀하는 수동 실험.

기존 접근 guard의 경계를 바꾸지 않는다. 과거 출력의 관측 delta는 사전 예상
endpoint 검사에만 쓰며 이번 결과나 충돌 여유를 보증하지 않는다.
"""
from __future__ import annotations

import copy

import numpy as np

from learning.approach_external_guard import (CurrentSceneExternalGuard, CURRENT_SCENE_SCHEMA,
    CURRENT_REFERENCE_MEANING)
from learning.artifacts import ArtifactStore
from learning.gripper_camera_dataset import read_frame
from learning.manual_collection_dataset import require
from learning.relative_action_learning import vector
from learning.relative_model_bundle import sha256, validate_model
from learning.supervised_collection import plan_digest
from learning.supervised_transport import ELBOW_DEADLINE_POLICY

SCHEMA = 'motionbrain.one_way_manual_recovery_guard.v1'
EVIDENCE_SCHEMA = 'motionbrain.manual_recovery_delta_evidence.v1'
MEANING = 'one_time_manual_UP_recovery_not_model_selected_success'
COMMAND = dict(joint='elbow', direction='up', percent=60, durationMs=350)
BEFORE_MINIMUM = [3., 24., 124.]
BEFORE_MAXIMUM = [3.5, 24.5, 124.]
AFTER_MINIMUM = [-3., 8., 122.]
AFTER_MAXIMUM = [8., 22., 126.]


def approval_scope_sha256(config):
    """실행 세션 ID·승인 파일 자체를 제외한 정적 계약의 해시."""
    scope = copy.deepcopy(config)
    scope.pop('hostClockId')
    scope['recovery'].pop('approval')
    return plan_digest(scope)


class OneWayManualRecoveryGuard(CurrentSceneExternalGuard):
    """UP 60% 350ms 한 번만 소비한다. 실패 후 새 세션에서도 같은 승인을 재사용하지 않는다.

    첫 검토만 반경 11px와 좁은 gripper 입력을 허용한다. 명령 허용 이후 모든
    관측은 원래 반경 10px·원래 gripper 범위에 복귀해야 한다. 방향 조건과 M4
    monitor·물체/NCC 검사는 기존 현재 구도 guard를 그대로 사용한다.
    첫 검토 또는 명령 검증에서 승인 범위를 선점하고, 거절도 실패로 남긴다.
    실제 명령 intent의 소비는 그 뒤 별도로 기록한다.
    """
    configuration_schema = SCHEMA

    def __init__(self, *args, **kwargs):
        self._consumed = self._failed = False
        self._claim_store = None
        self._failure_recorded = False
        self._review_state = None
        self._expected_endpoints = []
        self._recovery_snapshot = None
        super().__init__(*args, **kwargs)
        self._recovery_snapshot = plan_digest([self._evidence, self._approval, self._deltas])
        self._scope_sha = self._approval['scopeSha256']

    def _configuration_fields(self):
        return super()._configuration_fields() | {'recovery'}

    def _validate_reference(self, config):
        super()._validate_reference(config)
        recovery = config['recovery']
        require(isinstance(recovery, dict) and set(recovery) == {
            'meaning', 'command', 'maxAttempts', 'beforeMinimum', 'beforeMaximum',
            'afterMinimum', 'afterMaximum', 'evidence', 'approval'}, 'manual_recovery_contract_fields')
        require(recovery['meaning'] == MEANING and recovery['command'] == COMMAND and
                type(recovery['maxAttempts']) is int and recovery['maxAttempts'] == 1 and
                recovery['beforeMinimum'] == BEFORE_MINIMUM and recovery['beforeMaximum'] == BEFORE_MAXIMUM and
                recovery['afterMinimum'] == AFTER_MINIMUM and recovery['afterMaximum'] == AFTER_MAXIMUM,
                'manual_recovery_fixed_one_way_contract_required')
        for bounds in ('beforeMinimum', 'beforeMaximum', 'afterMinimum', 'afterMaximum'):
            vector(recovery[bounds])
        require(np.all(vector(recovery['beforeMinimum']) <= vector(recovery['beforeMaximum'])),
                'manual_recovery_before_bounds')

    def _validate_loaded_reference(self):
        super()._validate_loaded_reference()
        self._evidence, self._deltas = self._load_evidence(full=True)
        self._approval = self._load_approval()

    def _load_approval(self):
        approval = self._json(self.config['recovery']['approval'])
        require(isinstance(approval, dict) and set(approval) == {
            'approved', 'scopeSha256', 'fullPlanSha256', 'userReply'} and approval['approved'] is True and
            approval['scopeSha256'] == approval_scope_sha256(self.config) and
            isinstance(approval['fullPlanSha256'], str) and len(approval['fullPlanSha256']) == 64 and
            all(c in '0123456789abcdef' for c in approval['fullPlanSha256']) and
            isinstance(approval['userReply'], str) and bool(approval['userReply'].strip()),
            'manual_recovery_explicit_approval_required')
        return approval

    def _load_evidence(self, *, full):
        evidence = self._json(self.config['recovery']['evidence'])
        require(isinstance(evidence, dict) and set(evidence) == {
            'schemaVersion', 'trainingModel', 'trainingManifest', 'trainingRowsSha256', 'samples'} and
            evidence['schemaVersion'] == EVIDENCE_SCHEMA, 'manual_recovery_delta_evidence_fields')
        model = validate_model(self._json(evidence['trainingModel']), allow_development_observations=True)
        require(model.get('developmentObservationOnly') is True and
                model['context'].get('executionTimingPolicy') == ELBOW_DEADLINE_POLICY and
                model['context']['cameraIdentity']['cameraBootId'] == self.config['gripperCameraBootId'],
                'manual_recovery_training_context_changed')
        manifest = self._json(evidence['trainingManifest'])
        require(isinstance(manifest, dict) and isinstance(manifest.get('rows'), list) and
                manifest.get('controlEligible') is False and manifest.get('independentTest') is False and
                isinstance(manifest.get('model'), dict) and
                manifest['model'].get('sha256') == evidence['trainingModel']['sha256'] and
                plan_digest(manifest['rows']) == evidence['trainingRowsSha256'],
                'manual_recovery_training_manifest_changed')
        rows = {row['sessionId']: row for row in manifest['rows'] if row['command'] == COMMAND}
        samples = evidence['samples']
        require(len(rows) == 3 and isinstance(samples, list) and len(samples) == 3 and
                {s['sessionId'] for s in samples} == set(rows), 'manual_recovery_three_unique_UP_sources_required')
        deltas = []
        for sample in samples:
            require(isinstance(sample, dict) and set(sample) == {
                'sessionId', 'sourceRoot', 'session', 'gripperBefore', 'gripperAfter',
                'externalBefore', 'externalAfter', 'gripperDelta', 'forearmDelta'},
                'manual_recovery_sample_fields')
            row = rows[sample['sessionId']]
            source = self._private(sample['sourceRoot'])
            require(self._private(row['sourceRoot']) == source and row['context'] == model['context'] and
                    sample['session']['sha256'] == row['sourceHash'] and
                    self._private(sample['session']['path']) == source/'session.json',
                    'manual_recovery_session_binding_changed')
            session = self._json(sample['session'])
            commands = [e for e in session.get('events', []) if e.get('kind') == 'command']
            require(session.get('sessionId') == row['sessionId'] and session.get('phase') == 'completed' and
                    session.get('finalIdleOffVerified') is True and len(commands) == 1 and
                    {k: commands[0]['pulse'][k] for k in COMMAND} == COMMAND and
                    commands[0].get('monitorTimingPolicy') == ELBOW_DEADLINE_POLICY,
                    'manual_recovery_original_UP_receipt_required')
            states, external = [], []
            for side, grip_key, ext_key in [('before', 'gripperBefore', 'externalBefore'),
                                           ('after', 'gripperAfter', 'externalAfter')]:
                require(self._private(sample[grip_key]['path']).parent == source and
                        self._private(sample[ext_key]['path']).parent == source,
                        'manual_recovery_source_root_changed')
                observed = self._json(sample[grip_key])
                require(observed.get('state') == row[side] and observed.get('hostClockId') == row['sessionId'] and
                        observed.get('status') == 'complete_development' and
                        observed.get('sourceFreshMetadataVerified') is True and
                        observed.get('referenceImageSelfReplay') is False and
                        observed.get('observationContract') == model['context']['landmarks'] and
                        observed.get('cameraBootId') == self.config['gripperCameraBootId'],
                        'manual_recovery_original_gripper_state_changed')
                if full:
                    frame, metadata, boot = read_frame(source/'gripper-pair', side, row['sessionId'])
                    require(observed['sourceFrame'] == frame and boot == observed['cameraBootId'] and
                            frame['frameId'] == row['frameIds'][0 if side == 'before' else 1] and
                            sha256(_read_pair_metadata(source, side)) == observed['sourceMetadataSha256'] and
                            metadata.get('status') == 200 and metadata.get('method') == 'GET' and
                            metadata.get('path') == '/capture' and metadata.get('query') == {'fresh': '1'},
                            'manual_recovery_original_gripper_frame_changed')
                states.append(vector(observed['state']))
                reviewed = self._json(sample[ext_key])
                require(reviewed.get('schemaVersion') == CURRENT_SCENE_SCHEMA and reviewed.get('accepted') is True and
                        reviewed.get('referenceMeaning') == CURRENT_REFERENCE_MEANING,
                        'manual_recovery_original_external_review_required')
                point = reviewed['external']['forearmCenter']
                require(isinstance(point, list) and len(point) == 2 and
                        all(type(v) in (int, float) and np.isfinite(v) for v in point),
                        'manual_recovery_external_point_required')
                if full:
                    binding = reviewed['externalSource']; frame = binding['frame']
                    external_root = self._private(binding['root'])
                    require(external_root.is_relative_to(source) and
                            sha256(_read_external(external_root, frame['imageFile'])) == frame['imageSha256'] and
                            sha256(_read_external(external_root, frame['metadataFile'])) == frame['metadataSha256'],
                            'manual_recovery_original_external_frame_changed')
                external.append(np.asarray(point, dtype=float))
            delta, forearm = states[1]-states[0], external[1]-external[0]
            require(np.array_equal(delta, vector(sample['gripperDelta'])) and
                    np.array_equal(forearm, sample['forearmDelta']) and
                    -2 <= delta[0] <= -1 and -3.5 <= delta[1] <= -2.5 and delta[2] == 0 and
                    -2 <= forearm[0] <= -1 and forearm[1] == -3,
                    'manual_recovery_observed_delta_changed')
            deltas.append(dict(gripper=delta.tolist(), forearm=forearm.tolist()))
        return evidence, deltas

    def _verify_recovery(self, *, full):
        evidence, deltas = self._load_evidence(full=full)
        approval = self._load_approval()
        require(plan_digest([evidence, approval, deltas]) == self._recovery_snapshot,
                'manual_recovery_frozen_evidence_changed')

    def _claim_scope(self):
        require(not self._failed, 'manual_recovery_scope_failed')
        if self._claim_store is None:
            scope_sha = self._scope_sha
            # 명령 전 검토도 같은 승인 시도를 소비한다. 새 세션 ID는 재시도 근거가 아니다.
            self._claim_store = ArtifactStore(self.repository/'.local/one-way-recovery-attempts'/scope_sha)
            self._claim_store.write_json('claim.json', dict(schemaVersion=SCHEMA, meaning=MEANING,
                configSha256=self.config_binding['sha256'], approvalScopeSha256=scope_sha,
                hostClockId=self.config['hostClockId'], claimedAt=self.clock.monotonic(),
                command=COMMAND, maxAttempts=1, automaticRetry=False, modelSelected=False))

    def _record_failure(self, error):
        self._failed = True
        if self._claim_store is not None and not self._failure_recorded:
            self._failure_recorded = True
            self._claim_store.write_json('failed.json', dict(schemaVersion=SCHEMA,
                failedAt=self.clock.monotonic(), errorType=type(error).__name__,
                commandIntentWritten=self._consumed, automaticRetry=False, modelSelected=False))

    def check(self):
        super().check()
        require(not self._failed, 'manual_recovery_scope_failed')
        if self._recovery_snapshot is not None:
            require(plan_digest([self._evidence, self._approval, self._deltas]) == self._recovery_snapshot,
                    'manual_recovery_memory_contract_changed')

    def _forearm_limit(self):
        return 10 if self._consumed else 11

    def _measure(self, image):
        external = super()._measure(image)
        if not self._consumed:
            require(self._review_state is not None and external['forearmCenter'][1] > self.goal['forearmCenter'][1],
                    'manual_recovery_forearm_must_be_below_reference')
            endpoints = []
            for delta in self._deltas:
                state = vector(self._review_state)+np.asarray(delta['gripper'])
                forearm = np.asarray(external['forearmCenter'])+delta['forearm']
                require(np.all(state >= AFTER_MINIMUM) and np.all(state <= AFTER_MAXIMUM) and
                        np.linalg.norm(forearm-self.goal['forearmCenter']) <= 10,
                        'manual_recovery_expected_endpoint_outside_original_fence')
                endpoints.append(dict(state=state.tolist(), forearmCenter=forearm.tolist()))
            self._expected_endpoints = endpoints
        return external

    def review(self, record, result, *, deadline, tick):
        try:
            self._claim_scope()
            self._verify_recovery(full=True)
            state = vector(result.get('state'))
            lower, upper = ((AFTER_MINIMUM, AFTER_MAXIMUM) if self._consumed else
                            (BEFORE_MINIMUM, BEFORE_MAXIMUM))
            require(np.all(state >= lower) and np.all(state <= upper), 'manual_recovery_state_fence_exceeded')
            self._review_state = state.tolist()
            return super().review(record, result, deadline=deadline, tick=tick)
        except BaseException as error:
            self._record_failure(error)
            raise
        finally:
            self._review_state = None

    def validate_command(self, command):
        try:
            self._claim_scope()
            require(not self._consumed and command == COMMAND, 'manual_recovery_one_UP_attempt_only')
            self._verify_recovery(full=False)
            require(len(self._expected_endpoints) == 3, 'manual_recovery_current_endpoint_checks_required')
            self._consumed = True
            self._claim_store.write_json('intent.json', dict(schemaVersion=SCHEMA, meaning=MEANING,
                configSha256=self.config_binding['sha256'], approvalScopeSha256=self._scope_sha,
                hostClockId=self.config['hostClockId'], command=COMMAND, maxAttempts=1,
                attemptedAt=self.clock.monotonic(), automaticRetry=False, modelSelected=False))
            return super().validate_command(command)
        except BaseException as error:
            self._record_failure(error)
            raise

    def _reference_evidence(self):
        value = super()._reference_evidence()
        value.update(recoveryMeaning=MEANING, manualRecovery=True, modelSelected=False,
            recoveryEvidenceSha256=self.config['recovery']['evidence']['sha256'],
            expectedEndpointsFromHistoricalDeltas=copy.deepcopy(self._expected_endpoints),
            expectedEndpointsAreGuarantees=False,
            recoveryPhase='after_original_fence' if self._consumed else 'before_one_way_input',
            beforeMinimum=BEFORE_MINIMUM, beforeMaximum=BEFORE_MAXIMUM,
            afterMinimum=AFTER_MINIMUM, afterMaximum=AFTER_MAXIMUM)
        value['experimentalFence']['forearmMaximumDistancePx'] = self._forearm_limit()
        return value


def _read_pair_metadata(source, side):
    from learning.artifacts import _read
    return _read(source/'gripper-pair', f'esp32-{side}.json')


def _read_external(root, relative):
    from learning.artifacts import _read
    return _read(root, relative)
