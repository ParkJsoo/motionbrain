"""현재 외부 영상으로 유한 M3 접근의 기존 자세·물체·방향 한도를 검사한다.

카메라 시작이나 모터 명령은 포함하지 않는다. 관측기·모델의 정확도나 접촉을
승인하지 않으며, 선택된 명령을 거절할 수 있는 별도 현장 보호 경로다.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path, PurePosixPath
import time

import cv2
import numpy as np

from learning.artifacts import _decode_json, _read
from learning.lossless_frame import decode
from learning.manual_collection_dataset import require
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from learning.supervised_transport import (SequencePort, validate_monitor_policy,
    ELBOW_DEADLINE_POLICY, ELBOW_FINE_POLICY, ELBOW_PLATEAU_POLICY)

SCHEMA = 'motionbrain.approach_external_guard.v1'
CURRENT_SCENE_SCHEMA = 'motionbrain.current_scene_external_guard.v1'
CURRENT_REFERENCE_MEANING = 'onsite_manual_close_ready_stationary_reference_not_model_success'


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _same_checker(left, right):
    # 클래스의 bound method는 getattr할 때마다 새 객체가 되므로 함수·소유자를 비교한다.
    return (left is right or getattr(left, '__func__', None) is not None and
            getattr(left, '__func__', None) is getattr(right, '__func__', None) and
            getattr(left, '__self__', None) is getattr(right, '__self__', None))


class ApproachExternalGuard:
    """실제 수집기·포트에 주입한다. 생성자는 파일·정적 식별자만 검사한다.

    review()에서만 OFF 상태를 읽는다. check()/validate_command()는 HTTP 없이
    이미 검토한 원본과 수집기의 현재 신선도를 검사한다. 실행기는 RUN 직전
    validate_command(), OFF/실패 후 finally에서 disarm()을 호출해야 한다.
    """
    configuration_schema = SCHEMA
    allowed_policies = (ELBOW_DEADLINE_POLICY, ELBOW_FINE_POLICY, ELBOW_PLATEAU_POLICY)

    def _configuration_fields(self):
        return {'schemaVersion', 'tracking', 'goal', 'fixedObject', 'fixedObjectSourceRoot',
                'maxAgeSeconds', 'controllerBootId', 'hostClockId', 'gripperCameraBootId',
                'externalViewId', 'externalDeviceId', 'executionTimingPolicy'}

    def _validate_reference(self, config):
        pass

    def _validate_loaded_reference(self):
        pass

    def _object_box_limit(self):
        return 5

    def _forearm_limit(self):
        return 60

    def _direction_limits(self):
        return {ELBOW_DEADLINE_POLICY: (20, 12), ELBOW_FINE_POLICY: (20, 17),
                ELBOW_PLATEAU_POLICY: (28, 26)}[self.config['executionTimingPolicy']]

    def _reference_evidence(self):
        return {}

    def __init__(self, repository, config_binding, recorder, port, store, *, clock=time):
        self.repository = Path(repository).resolve(strict=True)
        self.recorder, self.port, self.store, self.clock = recorder, port, store, clock
        self.config_binding = copy.deepcopy(config_binding)
        self.config = self._json(config_binding)
        c = self.config
        require(isinstance(c, dict) and set(c) == self._configuration_fields()
                and c['schemaVersion'] == self.configuration_schema,
            'approach_external_configuration')
        require(_number(c['maxAgeSeconds']) and 0 < c['maxAgeSeconds'] <= 3,
                'approach_external_age_limit')
        require(c['executionTimingPolicy'] in self.allowed_policies,
                'approach_external_M3_policy_required')
        self._validate_reference(c)
        self._check_identity()
        tracking = self._json(c['tracking'])
        require(isinstance(tracking, dict) and
                set(tracking) == {'templateFile', 'templateSha256', 'goalTemplateCenter',
                    'searchRoi', 'minimumNcc', 'minimumPeakGap', 'expectedM4',
                    'maximumM4Difference', 'templateRotationsDeg', 'additionalTemplateFile',
                    'additionalTemplateSha256'}, 'approach_external_tracking_fields')
        require(_number(tracking['minimumNcc']) and .9 <= tracking['minimumNcc'] <= 1 and
                _number(tracking['minimumPeakGap']) and .04 <= tracking['minimumPeakGap'] <= 2 and
                tracking['expectedM4'] == 196.17 and
                _number(tracking['maximumM4Difference']) and 0 < tracking['maximumM4Difference'] <= .5,
                'approach_external_policy_relaxation')
        rotations = tracking['templateRotationsDeg']
        require(isinstance(rotations, list) and rotations == [-6, -4, -2, 0, 2, 4, 6] and
                all(type(v) is int for v in rotations), 'approach_external_rotation_policy')
        roi = tracking['searchRoi']
        require(isinstance(roi, list) and len(roi) == 4 and all(type(v) is int for v in roi) and
                0 <= roi[0] < roi[2] <= 1920 and 0 <= roi[1] < roi[3] <= 1080 and
                roi[2]-roi[0] > 57 and roi[3]-roi[1] > 64, 'approach_external_search_roi')
        self.tracking = tracking
        self.goal, self.fixed_object = self._json(c['goal']), self._json(c['fixedObject'])
        require(isinstance(self.goal, dict) and len(self.goal.get('forearmCenter', [])) == 2 and
                len(self.goal.get('objectCenter', [])) == 2 and
                all(_number(v) for k in ('forearmCenter', 'objectCenter') for v in self.goal[k]),
                'approach_external_goal_fields')
        require(isinstance(self.fixed_object, dict) and isinstance(self.fixed_object.get('external'), dict)
                and len(self.fixed_object['external'].get('objectBox', [])) == 4 and
                all(_number(v) for v in self.fixed_object['external']['objectBox']) and
                isinstance(self.fixed_object.get('source'), dict) and
                set(self.fixed_object['source']) == {'image', 'metadata', 'clock', 'admission'},
                'approach_external_fixed_object_fields')
        self._validate_loaded_reference()
        source_root = self._private(c['fixedObjectSourceRoot'])
        self._sources = []
        for binding in self.fixed_object['source'].values():
            self._sources.append((source_root, copy.deepcopy(binding)))
            self._raw_at(source_root, binding)
        self._templates = []
        base = PurePosixPath(c['tracking']['path']).parent
        for name, key in [('templateFile', 'templateSha256'),
                          ('additionalTemplateFile', 'additionalTemplateSha256')]:
            binding = dict(path=str(base / tracking[name]), sha256=tracking[key])
            raw = self._raw(binding)
            template = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
            require(template is not None and template.shape == (64, 57), 'approach_external_template_shape')
            template.flags.writeable = False
            self._templates.append((binding, template))
        self._snapshot = plan_digest([self.config, self.tracking, self.goal, self.fixed_object])
        self.active = False
        self._reviewed_at = self._source_at = None
        self._measurement = None
        self._count = 0

    def _private(self, relative):
        require(isinstance(relative, str) and relative, 'approach_external_private_path')
        path = (self.repository/relative).resolve(strict=True)
        require(path.is_relative_to(self.repository/'.local') and path != self.repository/'.local',
                'approach_external_private_path')
        return path

    def _raw_at(self, root, binding):
        require(isinstance(binding, dict) and set(binding) == {'path', 'sha256'},
                'approach_external_hashed_binding')
        raw = _read(root, binding['path'])
        require(sha256(raw) == binding['sha256'], 'approach_external_asset_changed')
        return raw

    def _raw(self, binding):
        path = self._private(binding['path'])
        private = self.repository/'.local'
        return self._raw_at(private, dict(path=str(path.relative_to(private)), sha256=binding['sha256']))

    def _json(self, binding):
        return _decode_json(self._raw(binding))

    def _check_identity(self):
        c = self.config
        require(self.port.boot_id == c['controllerBootId'] and
                self.recorder.plan.get('sessionId') == c['hostClockId'] and
                self.recorder.plan.get('viewId') == c['externalViewId'] and
                self.recorder.plan.get('deviceId') == c['externalDeviceId'],
                'approach_external_identity_changed')

    def _verify_assets(self):
        require(self._json(self.config_binding) == self.config and
                self._json(self.config['tracking']) == self.tracking and
                self._json(self.config['goal']) == self.goal and
                self._json(self.config['fixedObject']) == self.fixed_object and
                plan_digest([self.config, self.tracking, self.goal, self.fixed_object]) == self._snapshot,
                'approach_external_configuration_changed')
        for root, binding in self._sources:
            self._raw_at(root, binding)
        for binding, template in self._templates:
            current = cv2.imdecode(np.frombuffer(self._raw(binding), np.uint8), cv2.IMREAD_GRAYSCALE)
            require(current is not None and np.array_equal(current, template),
                    'approach_external_template_changed')

    def check(self):
        require(plan_digest([self.config, self.tracking, self.goal, self.fixed_object]) == self._snapshot,
                'approach_external_configuration_changed')
        self._check_identity()
        self.recorder.check()
        if self.active:
            self._check_review_age()

    def _check_review_age(self):
        now = self.clock.monotonic()
        require(self._reviewed_at is not None and self._source_at is not None and
                0 <= now-self._reviewed_at <= self.config['maxAgeSeconds'] and
                0 <= now-self._source_at <= self.config['maxAgeSeconds'],
                'approach_external_review_expired')

    def _frame(self):
        self.recorder.check()
        with self.recorder.lock:
            require(bool(self.recorder.frames), 'approach_external_frame_missing')
            frame = copy.deepcopy(self.recorder.frames[-1])
        now = self.clock.monotonic()
        require(_number(frame.get('sourceEarliest')) and
                0 <= now-frame['sourceEarliest'] <= self.config['maxAgeSeconds'] and
                frame.get('qualityFlags') == [] and frame.get('width') == 1920 and frame.get('height') == 1080,
                'approach_external_frame_quality')
        raw = _read(self.recorder.root, frame['imageFile'])
        metadata_raw = _read(self.recorder.root, frame['metadataFile'])
        require(sha256(raw) == frame['imageSha256'] and
                sha256(metadata_raw) == frame['metadataSha256'], 'approach_external_frame_changed')
        metadata = _decode_json(metadata_raw)
        require(metadata.get('sessionId') == self.config['hostClockId'] and
                metadata.get('viewConfigurationId') == self.config['externalViewId'] and
                metadata.get('deviceUniqueId') == self.config['externalDeviceId'] and
                metadata.get('sourceFrameId') == frame['frameId'] and
                metadata.get('imageSha256') == frame['imageSha256'] and
                metadata.get('pixelSha256') == frame['pixelSha256'], 'approach_external_metadata_identity')
        pixels = decode(raw, frame['rawByteLength'], frame['pixelSha256'])
        require(_read(self.recorder.root, frame['imageFile']) == raw and
                _read(self.recorder.root, frame['metadataFile']) == metadata_raw,
                'approach_external_frame_changed')
        return np.frombuffer(pixels, np.uint8).reshape(1080, 1920, 4)[:, :, :3], frame

    def _measure(self, image):
        require(bool(np.any(image)), 'approach_external_frame_black')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        x0, y0, x1, y1 = self.tracking['searchRoi']
        variants = []
        for _, template in self._templates:
            for angle in self.tracking['templateRotationsDeg']:
                matrix = cv2.getRotationMatrix2D(((template.shape[1]-1)/2, (template.shape[0]-1)/2), angle, 1)
                rotated = cv2.warpAffine(template, matrix, (template.shape[1], template.shape[0]),
                                        borderMode=cv2.BORDER_REFLECT_101)
                variants.append(cv2.matchTemplate(gray[y0:y1, x0:x1], rotated, cv2.TM_CCOEFF_NORMED))
        scores = np.maximum.reduce(variants)
        require(np.isfinite(scores).all(), 'approach_external_ncc_nonfinite')
        _, peak, _, (x, y) = cv2.minMaxLoc(scores)
        rest = scores.copy(); rest[max(0, y-6):y+7, max(0, x-6):x+7] = -2
        gap = float(peak-rest.max())
        require(x not in (0, scores.shape[1]-1) and y not in (0, scores.shape[0]-1) and
                peak >= self.tracking['minimumNcc'] and gap >= self.tracking['minimumPeakGap'],
                'approach_external_forearm_tracking_rejected')
        hsv = cv2.cvtColor(image[550:950, 450:850], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([145, 90, 100]), np.array([179, 255, 255]))
        _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        object_measurement = self._measure_object(stats)
        return dict(forearmCenter=[x0+x+28.5, y0+y+32], ncc=float(peak), peakGap=gap,
                    **object_measurement)

    def _measure_object(self, stats):
        """기본 계약은 원래대로 큰 색 영역 한 개만 허용한다."""
        objects = [s for s in stats[1:] if s[4] >= 500]
        require(len(objects) == 1, 'approach_external_object_ambiguous')
        ox, oy, w, h, area = map(int, objects[0]); ox += 450; oy += 550
        require(ox > 450 and oy > 550 and ox+w < 850 and oy+h < 950,
                'approach_external_object_clipped')
        return dict(objectBox=[ox, oy, ox+w, oy+h], objectCenter=[ox+w/2, oy+h/2], objectArea=area)

    def review(self, record, result, *, deadline, tick):
        require(not self.active, 'approach_external_review_during_output')
        require(self.clock.monotonic() < deadline, 'approach_external_deadline')
        self._verify_assets(); self.check(); tick()
        require(record.get('hostClockId') == self.config['hostClockId'] and
                result.get('cameraBootId') == self.config['gripperCameraBootId'] and
                result.get('sourceFreshMetadataVerified') is True and result.get('state') is not None,
                'approach_external_gripper_identity')
        image, frame = self._frame()
        external = self._measure(image)
        status = self.port.check_idle()
        angle = SequencePort._shoulder_angle(status)
        require(status['shoulderAngle'].get('active') is False and
                abs(angle-self.tracking['expectedM4']) <= self.tracking['maximumM4Difference'],
                'approach_external_fixed_M4_changed')
        require(max(abs(a-b) for a, b in zip(external['objectBox'], self.fixed_object['external']['objectBox']))
                <= self._object_box_limit(),
                'approach_external_fixed_object_extent_changed')
        require(len(result['state']) == 3 and abs(result['state'][2]-123) <= 3,
                'approach_external_jaw_span_changed')
        box = result['object']['box']
        require(isinstance(box, list) and len(box) == 4 and 30 <= box[2]-box[0] <= 52,
                'approach_external_object_scale_changed')
        require(np.linalg.norm(np.array(external['forearmCenter'])-self.goal['forearmCenter']) <= self._forearm_limit(),
                'approach_external_retreat_scope_exceeded')
        self._verify_assets(); self.check()
        now = self.clock.monotonic()
        require(now < deadline and 0 <= now-frame['sourceEarliest'] <= self.config['maxAgeSeconds'],
                'approach_external_review_expired')
        snapshot = plan_digest([record, result])
        evidence = dict(schemaVersion=self.configuration_schema, accepted=True, observationSha256=snapshot,
            configSha256=self.config_binding['sha256'], externalSource=dict(root=str(self.recorder.root), frame=frame),
            external=external, shoulderAngleDeg=angle, reviewedAt=now,
            interpretation='현장범위 보호 검사; 독립 추적정확도·접촉·집기 검증 아님',
            **self._reference_evidence())
        self._count += 1
        name = f'approach-external-review-{self._count:04d}.json'
        descriptor = self.store.write_json(name, evidence)
        self._measurement = copy.deepcopy(external)
        self._reviewed_at, self._source_at = now, frame['sourceEarliest']
        return dict(accepted=True, observationSha256=snapshot,
                    evidence=dict(path=name, sha256=descriptor['sha256']))

    def validate_command(self, command):
        self.check(); self._check_review_age()
        require(isinstance(command, dict) and command.get('joint') == 'elbow' and
                command.get('direction') in ('up', 'down'), 'approach_external_M3_command_required')
        validate_monitor_policy(command, self.config['executionTimingPolicy'])
        external = self._measurement
        delta = ((external['objectCenter'][1]-external['forearmCenter'][1])-
                 (self.goal['objectCenter'][1]-self.goal['forearmCenter'][1]))
        up_limit, down_limit = self._direction_limits()
        require((command['direction'] == 'up' and delta <= up_limit) or
                (command['direction'] == 'down' and delta >= down_limit),
                'approach_external_direction_clearance_rejected')
        self.active = True

    def disarm(self):
        self.active = False


class CurrentSceneExternalGuard(ApproachExternalGuard):
    """사용자가 수동 배치한 현재 정지 자세를 기준으로 한 별도 실험 경계.

    과거 목표의 좌표·방향 한도를 전이한 계약이 아니다. 새 원본과 같은 영상
    좌표계에서 물체 경계 3px·전완 거리 10px·방향별 delta를 제한한다.
    전완 거리·물체 경계는 사후 관측에도 적용한다. UP의 delta<=6과 DOWN의
    delta>=0은 명령 전 방향 조건이며, 사후 delta를 [0,6]으로 강제하지 않는다.
    실행 중 경계 통과를 막는 연속 충돌 검사도 아니다.
    명령 허용 후에는 기존 포트 monitor 응답에도 M4 고정 검사를 적용한다.
    새 HTTP 조회는 만들지 않으며 disarm에서 자기 검사기만 복원한다.
    이 경계는 충돌·접촉·추적 정확도 보증이나 모델 접근 성공 판정이 아니다.
    """
    configuration_schema = CURRENT_SCENE_SCHEMA
    allowed_policies = (ELBOW_DEADLINE_POLICY,)

    def __init__(self, *args, **kwargs):
        self._status_original = self._status_wrapper = None
        super().__init__(*args, **kwargs)

    def check(self):
        super().check()
        if self.active:
            require(self._status_wrapper is not None and
                    getattr(self.port, '_check_status', None) is self._status_wrapper,
                    'current_scene_status_guard_replaced')

    def validate_command(self, command):
        require(not self.active and self._status_wrapper is None,
                'current_scene_command_already_active')
        original = getattr(self.port, '_check_status', None)
        require(callable(original), 'current_scene_status_monitor_required')
        super().validate_command(command)
        try:
            require(_same_checker(getattr(self.port, '_check_status', None), original),
                    'current_scene_status_guard_changed_before_install')

            def checked(status, expected, pulse=None, baseline_angle=None):
                result = original(status, expected, pulse, baseline_angle)
                if self.active:
                    angle = SequencePort._shoulder_angle(status)
                    require(status['shoulderAngle'].get('active') is False and
                            abs(angle-self.tracking['expectedM4']) <= self.tracking['maximumM4Difference'],
                            'current_scene_active_fixed_M4_changed')
                return result

            self._status_original, self._status_wrapper = original, checked
            self.port._check_status = checked
        except BaseException:
            self.disarm()
            raise

    def disarm(self):
        super().disarm()
        if self._status_wrapper is not None and getattr(self.port, '_check_status', None) is self._status_wrapper:
            self.port._check_status = self._status_original
        self._status_original = self._status_wrapper = None

    def _configuration_fields(self):
        return super()._configuration_fields() | {'referenceMeaning'}

    def _validate_reference(self, config):
        require(config['referenceMeaning'] == CURRENT_REFERENCE_MEANING,
                'current_scene_manual_reference_meaning_required')

    def _validate_loaded_reference(self):
        reference = self.fixed_object['external']
        require(self.tracking['goalTemplateCenter'] == self.goal['forearmCenter'] == reference.get('forearmCenter')
                and self.goal['objectCenter'] == reference.get('objectCenter'),
                'current_scene_same_stationary_reference_required')

    def _object_box_limit(self):
        return 3

    def _forearm_limit(self):
        return 10

    def _direction_limits(self):
        # 같은 새 기준에서 측정한 (물체Y-전완Y) 변화다. 과거 12/20px가 아니다.
        return 6, 0

    def _reference_evidence(self):
        return dict(referenceMeaning=CURRENT_REFERENCE_MEANING,
                    experimentalFence=dict(objectBoxMaximumChangePx=3,
                        forearmMaximumDistancePx=10, upMaximumDeltaPx=6, downMinimumDeltaPx=0))
