"""현재 M1 구도의 shaft 관측과 고정 자세 보호. 접촉·파지 판정은 하지 않는다.

기존 fresh JPEG·단면 관측·외부 marker 추출·상태 monitor를 재사용한다.
생성자는 파일만 읽으며 카메라·HTTP·인증을 시작하지 않는다.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import time

import numpy as np

from learning.approach_external_guard import CurrentSceneExternalGuard, _same_checker
from learning.artifacts import _decode_json, _read
from learning.gripper_camera_dataset import POLICY, read_frame
from learning.manual_collection_dataset import require
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection, SHAFT_STATE, default_parameters
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort

SCHEMA = 'motionbrain.m1_shaft_scene_guard.v1'
OBSERVATION_SCHEMA = 'motionbrain.bound_M1_shaft_observation.v1'
MONITOR_POLICY = 'm1-default-sequence-monitor-v1'
REFERENCE_MEANING = 'onsite_current_M1_fixed_pose_reference_not_contact_or_grasp_success'
OBSERVED95_LOCAL_COMMAND_POLICY = 'observed95-local450-v1'
OBJECT_FRAGMENT_POLICY = 'fixed_reference_two_fragment_occlusion_v1'


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def scene_context(tracker):
    return dict(tracker.registration['sceneContext'], landmarks=tracker.shaft_landmark_contract)


def observe_bound_shaft_frame(tracker, frame_root, label, *, session_id, camera_boot_id,
                              actual_scene, now, max_age_seconds=3., previous=None):
    """저장 원본을 재검사한다. 시각 갱신만으로 과거 프레임을 현재 관측으로 만들지 않는다."""
    require(actual_scene == scene_context(tracker), 'M1_actual_scene_changed')
    require(_number(now) and _number(max_age_seconds) and 0 < max_age_seconds <= 3,
            'M1_source_age_policy')
    root = Path(frame_root).resolve(strict=True)
    metadata_raw = _read(root, f'esp32-{label}.json')
    frame, metadata, boot = read_frame(root, label, session_id)
    require(metadata == _decode_json(metadata_raw) and boot == camera_boot_id,
            'M1_source_clock_or_boot_changed')
    require(metadata.get('status') == 200 and metadata.get('method') == 'GET' and
            metadata.get('path') == '/capture' and metadata.get('query') == {'fresh': '1'},
            'M1_fresh_capture_required')
    require(metadata['persistedAt'] <= now and 0 <= now-metadata['requestStartedAt'] <= max_age_seconds,
            'M1_source_stale')
    if previous is not None:
        require(previous['cameraBootId'] == boot and previous['hostClockId'] == session_id and
                frame['requestedUs'] > previous['sourceFrame']['capturedUs'] and
                metadata['requestStartedAt'] >= previous['persistedAt'], 'M1_source_reused_or_regressed')
    raw = _read(root, f'esp32-{label}.jpg')
    observed = tracker.observe_shaft(raw, frame['frameId'])
    require(observed['pixelSha256'] == frame['pixelSha256'] and
            _read(root, f'esp32-{label}.json') == metadata_raw and
            sha256(_read(root, f'esp32-{label}.jpg')) == frame['imageSha256'], 'M1_source_bytes_changed')
    context = dict(actual_scene, cameraIdentity=dict(camera='gripper', cameraBootId=boot,
        timestampSemantics='first_dma_buffer', observationPolicy=copy.deepcopy(POLICY)),
        imageShape=[frame['height'], frame['width']], stateFields=SHAFT_STATE.copy())
    return dict(schemaVersion=OBSERVATION_SCHEMA, sourceFrame=frame, context=context,
        sourceBinding=dict(root=str(root), label=label), hostClockId=session_id, cameraBootId=boot,
        sourceHostBracket=[metadata['requestStartedAt'], metadata['receivedAt']],
        persistedAt=metadata['persistedAt'], sourceMetadataSha256=sha256(metadata_raw),
        observation=observed, state=observed['state'], shaftSeparationPx=observed['shaftSeparationPx'],
        sourceFreshMetadataVerified=True, measurementAccuracyVerified=False, contactVerified=False,
        graspVerified=False, controlEligible=False)


def _source_record(result):
    frame = result['sourceFrame']; earliest, latest = result['sourceHostBracket']
    return dict(frame={k: frame[k] for k in ('frameId', 'pixelSha256', 'width', 'height')},
                context=copy.deepcopy(result['context']), hostClockId=result['hostClockId'],
                sourceEarliest=earliest, sourceLatest=latest, receivedAt=latest, persistedAt=result['persistedAt'])


class M1ShaftSceneGuard(CurrentSceneExternalGuard):
    """M1에서 고정해야 하는 전완·물체·M4만 검사한다. 집게 폭은 고정하지 않는다."""
    configuration_schema = SCHEMA
    allowed_policies = (MONITOR_POLICY,)

    def __init__(self, *args, **kwargs):
        self._failed = False
        super().__init__(*args, **kwargs)
        self._tracker_snapshot = plan_digest(self.tracker.registration), sha256(self.tracker.reference_image)

    def _configuration_fields(self):
        fields = super()._configuration_fields() | {'actualScene', 'shaftRegistration', 'shaftReference'}
        if isinstance(self.config, dict) and 'objectObservationPolicy' in self.config:
            fields.add('objectObservationPolicy')
        if isinstance(self.config, dict) and 'commandPolicy' in self.config:
            fields.add('commandPolicy')
        return fields

    def _validate_reference(self, config):
        require(config['referenceMeaning'] == REFERENCE_MEANING and
                isinstance(config['actualScene'], dict) and set(config['actualScene']) ==
                {'physicalView', 'object', 'poseRegime'} and
                all(isinstance(v, str) and bool(v) for v in config['actualScene'].values()),
                'M1_actual_reference_declaration_required')
        if 'commandPolicy' in config:
            require(config['commandPolicy'] == OBSERVED95_LOCAL_COMMAND_POLICY,
                    'M1_explicit_command_policy_required')
        if 'objectObservationPolicy' in config:
            require(config['objectObservationPolicy'] == OBJECT_FRAGMENT_POLICY,
                    'M1_explicit_fixed_object_occlusion_policy_required')

    def _validate_loaded_reference(self):
        super()._validate_loaded_reference()
        self.tracker = RegisteredGripperCrosssection(self._json(self.config['shaftRegistration']),
                                                     self._raw(self.config['shaftReference']))
        require(self.tracker.registration['sceneContext'] == self.config['actualScene'],
                'M1_registration_actual_scene_changed')
        if 'commandPolicy' in self.config:
            require(self.tracker.registration['parameters'] ==
                    default_parameters(preset='current_m1_observed_shaft95'),
                    'M1_command_policy_observed95_registration_required')
        if 'objectObservationPolicy' in self.config:
            area = self.fixed_object['external'].get('objectArea')
            require(_number(area) and area > 0, 'M1_reference_object_area_required')

    @property
    def scene_context(self):
        return scene_context(self.tracker)

    def _forearm_limit(self):
        return 3

    def _measure_object(self, stats):
        if self.config.get('objectObservationPolicy') != OBJECT_FRAGMENT_POLICY:
            return super()._measure_object(stats)
        objects = [s for s in stats[1:] if s[4] >= 500]
        require(len(objects) in (1, 2), 'M1_fixed_object_fragment_count_rejected')
        reference = self.fixed_object['external']
        if len(objects) == 1:
            result = super()._measure_object(stats)
            result.update(objectFragmentCount=1, visibleAreaRatio=result['objectArea']/reference['objectArea'],
                          objectObservationPolicy=OBJECT_FRAGMENT_POLICY, occlusionContractApplied=False)
            return result
        fragments = []
        rx0, ry0, rx1, ry1 = reference['objectBox']
        for component in objects:
            x, y, width, height, area = map(int, component)
            x += 450; y += 550
            require(x > 450 and y > 550 and x+width < 850 and y+height < 950,
                    'M1_fixed_object_fragment_clipped')
            box = [x, y, x+width, y+height]
            require(x >= rx0-3 and y >= ry0-3 and x+width <= rx1+3 and y+height <= ry1+3,
                    'M1_fixed_object_fragment_outside_reference')
            fragments.append(dict(box=box, areaPx=area))
        union = [min(f['box'][0] for f in fragments), min(f['box'][1] for f in fragments),
                 max(f['box'][2] for f in fragments), max(f['box'][3] for f in fragments)]
        area = sum(f['areaPx'] for f in fragments)
        ratio = area/reference['objectArea']
        require(max(abs(a-b) for a,b in zip(union,reference['objectBox'])) <= 3,
                'M1_fixed_object_fragment_extent_changed')
        require(.7 <= ratio <= 1.05, 'M1_fixed_object_fragment_area_rejected')
        return dict(objectBox=union, objectCenter=[(union[0]+union[2])/2,(union[1]+union[3])/2],
            objectArea=area, objectFragmentCount=2, objectFragments=fragments, visibleAreaRatio=ratio,
            objectObservationPolicy=OBJECT_FRAGMENT_POLICY, occlusionContractApplied=True,
            fragmentInterpretation='고정 물체의 가림 가설; 물체 동일성·깊이·접촉·파지 검증 아님')

    def _verify_assets(self):
        super()._verify_assets()
        require(self._json(self.config['shaftRegistration']) == self.tracker.registration and
                self._raw(self.config['shaftReference']) == self.tracker.reference_image,
                'M1_shaft_assets_changed')

    def check(self):
        super().check()
        require(not self._failed, 'M1_observation_scope_failed')
        require((plan_digest(self.tracker.registration), sha256(self.tracker.reference_image)) == self._tracker_snapshot,
                'M1_shaft_memory_contract_changed')

    def reject(self):
        """관측 거절 이후 이전 검토 cache로 새 출력을 허용하지 않는다."""
        self._failed = True

    def review(self, record, result, *, deadline, tick):
        try:
            return self._review_checked(record, result, deadline=deadline, tick=tick)
        except BaseException:
            self.reject()
            raise

    def _review_checked(self, record, result, *, deadline, tick):
        require(not self.active and self.clock.monotonic() < deadline, 'M1_idle_review_required')
        self._verify_assets(); self.check(); tick()
        require(isinstance(result, dict) and result.get('schemaVersion') == OBSERVATION_SCHEMA and
                record.get('hostClockId') == self.config['hostClockId'], 'M1_bound_observation_required')
        binding = result['sourceBinding']; root = self._private(binding['root'])
        checked = observe_bound_shaft_frame(self.tracker, root, binding['label'],
            session_id=self.config['hostClockId'], camera_boot_id=self.config['gripperCameraBootId'],
            actual_scene=self.scene_context, now=self.clock.monotonic(), max_age_seconds=self.config['maxAgeSeconds'])
        require(checked == result and _source_record(checked) == record, 'M1_bound_observation_changed')
        image, frame = self._frame(); external = self._measure(image)
        status = self.port.check_idle(); angle = SequencePort._shoulder_angle(status)
        require(status['shoulderAngle'].get('active') is False and
                abs(angle-self.tracking['expectedM4']) <= self.tracking['maximumM4Difference'], 'M1_fixed_M4_changed')
        require(max(abs(a-b) for a,b in zip(external['objectBox'], self.fixed_object['external']['objectBox'])) <= 3,
                'M1_fixed_object_extent_changed')
        require(np.linalg.norm(np.asarray(external['forearmCenter'])-self.goal['forearmCenter']) <= 3,
                'M1_forearm_fixed_pose_changed')
        self._verify_assets(); self.check(); now = self.clock.monotonic()
        require(now < deadline and 0 <= now-frame['sourceEarliest'] <= self.config['maxAgeSeconds'] and
                0 <= now-record['sourceEarliest'] <= self.config['maxAgeSeconds'], 'M1_review_expired')
        snapshot = plan_digest([record, result]); self._count += 1
        name = f'M1-external-review-{self._count:04d}.json'
        proof = dict(schemaVersion=SCHEMA, accepted=True, observationSha256=snapshot,
            configSha256=self.config_binding['sha256'], externalSource=dict(root=str(self.recorder.root),frame=frame),
            external=external, shaftSeparationPx=checked['shaftSeparationPx'], shoulderAngleDeg=angle,
            reviewedAt=now, referenceMeaning=REFERENCE_MEANING,
            interpretation='고정 자세 보호·shaft 영상 관측. 접촉·힘·파지 성공 판정 아님')
        stored = self.store.write_json(name, proof)
        self._measurement = copy.deepcopy(external)
        self._reviewed_at, self._source_at = now, min(frame['sourceEarliest'],record['sourceEarliest'])
        return dict(accepted=True,observationSha256=snapshot,evidence=dict(path=name,sha256=stored['sha256']))

    def validate_command(self, command):
        require(not self.active and self._status_wrapper is None, 'M1_command_already_active')
        self.check(); self._check_review_age()
        valid_shape = (isinstance(command, dict) and
            set(command) == {'joint','direction','percent','durationMs'} and
            command['joint'] == 'gripper' and type(command['percent']) is int and
            type(command['durationMs']) is int)
        if self.config.get('commandPolicy') == OBSERVED95_LOCAL_COMMAND_POLICY:
            # 새 관측 전용 후보. 기존 넓은 후보를 묵시적으로 합치지 않는다.
            permitted = (valid_shape and
                ((command['direction'] == 'close' and command['percent'] == 60 and command['durationMs'] == 450) or
                 (command['direction'] == 'open' and
                  ((command['percent'] == 60 and command['durationMs'] in (450,550,650)) or
                   (command['percent'] == 40 and command['durationMs'] in (250,350))))))
        else:
            permitted = (valid_shape and
                ((command['direction'] == 'close' and
                  ((command['percent'] == 40 and command['durationMs'] in (200,350)) or
                   (command['percent'] == 60 and command['durationMs'] == 650))) or
                 (command['direction'] == 'open' and
                  ((command['percent'] == 40 and command['durationMs'] == 350) or
                   (command['percent'] == 60 and command['durationMs'] == 650)))))
        require(permitted, 'M1_bounded_command_required')
        original = getattr(self.port,'_check_status',None)
        require(callable(original), 'M1_status_monitor_required')
        try:
            self.active = True
            require(_same_checker(getattr(self.port,'_check_status',None),original),'M1_status_guard_changed')
            def checked(status,expected,pulse=None,baseline_angle=None):
                value = original(status,expected,pulse,baseline_angle)
                if self.active:
                    angle = SequencePort._shoulder_angle(status)
                    require(status['shoulderAngle'].get('active') is False and
                            abs(angle-self.tracking['expectedM4']) <= self.tracking['maximumM4Difference'],
                            'M1_active_fixed_M4_changed')
                return value
            self._status_original,self._status_wrapper=original,checked
            self.port._check_status=checked
        except BaseException:
            self.disarm()
            raise


class M1ShaftSourceObserver:
    """실제 source 검증과 shaft 계측을 연결한다. capture()만 새 영상 요청을 한다."""
    def __init__(self, source, guard, *, clock=time):
        require(isinstance(guard,M1ShaftSceneGuard) and source.boot == guard.config['gripperCameraBootId'] and
                source.session_id == guard.config['hostClockId'] and source._scene() == guard.scene_context,
                'M1_source_guard_contract_changed')
        self.source,self.guard,self.clock=source,guard,clock
        self.previous=None

    def start(self, *, deadline):
        self.source.start(deadline=deadline)

    def check(self):
        self.source.check()
        self.guard.check()

    def close(self):
        return self.source.close()

    def inspect(self, record, *, after_off=None):
        try:
            return self._inspect_checked(record, after_off=after_off)
        except BaseException:
            self.guard.reject()
            raise

    def _inspect_checked(self, record, *, after_off=None):
        self.check()
        self.source.verify(record)
        binding=self.source._bindings[record['frame']['frameId']]
        result=observe_bound_shaft_frame(self.guard.tracker,self.source.store.root/binding['folder'],'frame',
            session_id=self.source.session_id,camera_boot_id=self.source.boot,actual_scene=self.source._scene(),
            now=self.clock.monotonic(),max_age_seconds=self.guard.config['maxAgeSeconds'],previous=self.previous)
        require(_source_record(result)==record,'M1_source_record_changed')
        if after_off is not None:
            require(_number(after_off) and result['sourceHostBracket'][0]>=after_off+3.,'M1_post_OFF_window_not_ready')
        self.previous=copy.deepcopy(result)
        return result

    def capture(self, *, deadline, tick, after_off=None):
        try:
            until=after_off+3. if after_off is not None else self.clock.monotonic()
            while self.clock.monotonic()<until:
                require(self.clock.monotonic()<deadline,'M1_observation_deadline')
                tick();self.clock.sleep(max(0.,min(.05,until-self.clock.monotonic())))
            record=self.source.capture(deadline=deadline)
            result=self.inspect(record,after_off=after_off)
            return record,result
        except BaseException:
            self.guard.reject()
            raise
