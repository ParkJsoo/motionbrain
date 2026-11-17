"""병합된 빈 집게도 측정하는 별도 복구 영상 특징. 실제 턱 간격은 측정하지 않는다.

고정 ROI의 검은 실루엣과 분홍 색 성분만 관측한다. 분홍 성분이 없거나
손가락이 합쳐져도 유효할 수 있다. 기존 OBS3 좌표·끝점·가시성·모델 입력을
대신하지 않으며, 접촉·깊이·충돌 여유·복구 성공은 별도 검증해야 한다.
"""
from __future__ import annotations

import hashlib
import json
import math

import cv2
import numpy as np

SCHEMA = 'motionbrain.gripper_recovery_image.v1'
PREFIX = 'gripper-recovery-image-v1:'
STATE_FIELDS = ['darkEnvelopeCentral80WidthFraction', 'darkOccupancyFraction', 'qualifiedPinkPresent']
MEANING = 'fixed_ROI_dark_silhouette_and_color_presence_not_actual_gap_tip_depth_contact_or_grasp'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def observer_policy():
    # 최초 수집·학습 전에 고정한다. 새 동작 결과를 보고 ROI나 한도를 조정하지 않는다.
    return dict(schemaVersion=SCHEMA, imageShape=[240, 320], maximumImageBytes=2_000_000,
                darkRoi=[40, 65, 280, 150], darkMaximumValue=85,
                darkQuantiles=[.1, .9], darkOccupancyRange=[.01, .65],
                envelopeMeaning='central_80_percent_horizontal_dark_pixel_distribution_not_tip_separation',
                pinkRoi=[40, 60, 280, 235], pinkHsvLow=[145, 120, 80], pinkHsvHigh=[179, 255, 255],
                pinkArea=[80, 8000], pinkWidth=[10, 120], pinkHeight=[8, 130], minimumPinkFill=.5,
                pinkPresenceMeaning='one_or_more_qualified_color_components_not_object_identity_or_OBS3_visibility',
                stateMeaning=MEANING, actualGapMeasured=False, occludedPixelsEstimated=False)


def _envelope(histogram, policy):
    width = policy['darkRoi'][2]-policy['darkRoi'][0]
    height = policy['darkRoi'][3]-policy['darkRoi'][1]
    _require(isinstance(histogram, list) and len(histogram) == width
             and all(type(v) is int and 0 <= v <= height for v in histogram),
             'recovery_dark_histogram_invalid')
    count = sum(histogram); fraction = count/(width*height)
    _require(policy['darkOccupancyRange'][0] <= fraction <= policy['darkOccupancyRange'][1],
             'recovery_dark_support_outside_fixed_range')
    xs = np.repeat(np.arange(width), histogram)
    quantiles = np.quantile(xs, policy['darkQuantiles']).tolist()
    span = float(quantiles[1]-quantiles[0])
    return dict(roi=policy['darkRoi'].copy(), xHistogram=histogram.copy(),
                darkPixelCount=count, roiPixelCount=width*height,
                occupancyFraction=fraction, xQuantilesWithinRoiPx=quantiles,
                central80WidthPx=span, central80WidthFraction=span/width,
                meaning=policy['envelopeMeaning'])


def _pink(hsv, policy):
    x0, y0, x1, y1 = policy['pinkRoi']
    mask = cv2.inRange(hsv[y0:y1, x0:x1], np.uint8(policy['pinkHsvLow']), np.uint8(policy['pinkHsvHigh']))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    components = []
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        if (policy['pinkArea'][0] <= area <= policy['pinkArea'][1]
                and policy['pinkWidth'][0] <= width <= policy['pinkWidth'][1]
                and policy['pinkHeight'][0] <= height <= policy['pinkHeight'][1]
                and area/(width*height) >= policy['minimumPinkFill']):
            components.append(dict(bbox=[x+x0, y+y0, width, height], area=area))
    return dict(roi=policy['pinkRoi'].copy(), maskPixelCount=int(cv2.countNonZero(mask)),
                roiPixelCount=(x1-x0)*(y1-y0), qualifiedComponents=components,
                qualifiedComponentCount=len(components), qualifiedPresent=bool(components),
                multipleQualifiedComponents=len(components) > 1,
                meaning=policy['pinkPresenceMeaning'], objectIdentityVerified=False)


class GripperRecoveryObserver:
    def __init__(self):
        self.policy = observer_policy()
        self.binding = digest(self.policy)
        self.landmark_contract = PREFIX+self.binding

    def observe(self, raw):
        _require(self.policy == observer_policy() and digest(self.policy) == self.binding,
                 'recovery_observer_policy_changed')
        _require(isinstance(raw, bytes) and 0 < len(raw) <= self.policy['maximumImageBytes'],
                 'recovery_image_bytes_required')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        _require(image is not None and image.shape == (240, 320, 3), 'recovery_qvga_image_required')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        x0, y0, x1, y1 = self.policy['darkRoi']
        histogram = (hsv[y0:y1, x0:x1, 2] < self.policy['darkMaximumValue']).sum(axis=0).astype(int).tolist()
        jaw = _envelope(histogram, self.policy); pink = _pink(hsv, self.policy)
        return dict(schemaVersion=SCHEMA, observationContractSha256=self.binding, policySha256=self.binding,
                    landmarkContract=self.landmark_contract, imageSha256=hashlib.sha256(raw).hexdigest(),
                    state=[jaw['central80WidthFraction'], jaw['occupancyFraction'], float(pink['qualifiedPresent'])],
                    stateFields=STATE_FIELDS.copy(), stateMeaning=MEANING,
                    jawImage=jaw, pink=pink, actualGapMeasured=False, contactVerified=False,
                    depthVerified=False, forceVerified=False, graspVerified=False, clearanceVerified=False,
                    occludedPixelsEstimated=False, OBS3Compatible=False)


def validate_observation(observation):
    """이미지에서 파생한 상태와 히스토그램·색 성분·고정 계약의 일관성을 검사한다."""
    policy = observer_policy(); binding = digest(policy)
    _require(isinstance(observation, dict) and observation.get('schemaVersion') == SCHEMA
             and observation.get('observationContractSha256') == binding
             and observation.get('policySha256') == binding
             and observation.get('landmarkContract') == PREFIX+binding
             and observation.get('stateFields') == STATE_FIELDS and observation.get('stateMeaning') == MEANING,
             'recovery_observation_contract')
    image_sha = observation.get('imageSha256')
    _require(isinstance(image_sha, str) and len(image_sha) == 64
             and all(ch in '0123456789abcdef' for ch in image_sha), 'recovery_image_binding')
    jaw = observation.get('jawImage')
    _require(isinstance(jaw, dict) and jaw == _envelope(jaw.get('xHistogram'), policy),
             'recovery_jaw_features_changed')
    pink = observation.get('pink')
    fields = {'roi', 'maskPixelCount', 'roiPixelCount', 'qualifiedComponents', 'qualifiedComponentCount',
              'qualifiedPresent', 'multipleQualifiedComponents', 'meaning', 'objectIdentityVerified'}
    _require(isinstance(pink, dict) and set(pink) == fields and pink['roi'] == policy['pinkRoi']
             and pink['roiPixelCount'] == 240*175
             and type(pink['maskPixelCount']) is int and 0 <= pink['maskPixelCount'] <= pink['roiPixelCount']
             and isinstance(pink['qualifiedComponents'], list), 'recovery_pink_features_invalid')
    for component in pink['qualifiedComponents']:
        _require(isinstance(component, dict) and set(component) == {'bbox', 'area'}, 'recovery_pink_component_invalid')
        box = component['bbox']; area = component['area']
        _require(isinstance(box, list) and len(box) == 4 and all(type(v) is int for v in box),
                 'recovery_pink_component_invalid')
        x, y, width, height = box
        _require(policy['pinkRoi'][0] <= x < x+width <= policy['pinkRoi'][2]
                 and policy['pinkRoi'][1] <= y < y+height <= policy['pinkRoi'][3]
                 and policy['pinkWidth'][0] <= width <= policy['pinkWidth'][1]
                 and policy['pinkHeight'][0] <= height <= policy['pinkHeight'][1]
                 and type(area) is int and policy['pinkArea'][0] <= area <= min(policy['pinkArea'][1], width*height)
                 and area/(width*height) >= policy['minimumPinkFill'], 'recovery_pink_component_invalid')
    count = len(pink['qualifiedComponents'])
    _require(type(pink['qualifiedComponentCount']) is int and pink['qualifiedComponentCount'] == count
             and type(pink['qualifiedPresent']) is bool and pink['qualifiedPresent'] == bool(count)
             and type(pink['multipleQualifiedComponents']) is bool and pink['multipleQualifiedComponents'] == (count > 1)
             and pink['meaning'] == policy['pinkPresenceMeaning'] and pink['objectIdentityVerified'] is False
             and sum(c['area'] for c in pink['qualifiedComponents']) <= pink['maskPixelCount'],
             'recovery_pink_presence_changed')
    state = observation.get('state')
    expected = [jaw['central80WidthFraction'], jaw['occupancyFraction'], float(pink['qualifiedPresent'])]
    _require(isinstance(state, list) and len(state) == 3 and all(_number(v) for v in state)
             and state == expected, 'recovery_state_changed')
    _require(all(observation.get(key) is False for key in ('actualGapMeasured', 'contactVerified', 'depthVerified',
             'forceVerified', 'graspVerified', 'clearanceVerified', 'occludedPixelsEstimated', 'OBS3Compatible')),
             'recovery_measurement_meaning_changed')
    return np.array(state, dtype=float)
