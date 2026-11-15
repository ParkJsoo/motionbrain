"""집게 카메라의 보이는 검은 손가락 끝·분홍 물체 관측. 접촉·힘·깊이는 판정하지 않는다."""
from __future__ import annotations
import copy
import hashlib
import json
import re

import cv2
import numpy as np

SCHEMA = 'motionbrain.near_gripper_visible_tips.v1'
PROFILE_SCHEMA = 'motionbrain.near_gripper_visible_tips.v2'
PREFIX = 'near-gripper-visible-tips-v1:'
STATE_FIELDS = ['objectMinusVisibleJawMidXpx', 'objectMinusVisibleJawMidYpx', 'visibleTipHorizontalGapPx']


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def observer_policy(*, version=1):
    _require(type(version) is int and version in (1, 2), 'near_observer_policy_version')
    policy = dict(schemaVersion=SCHEMA, imageShape=[240, 320],
        leftRoi=[20, 35, 160, 170], rightRoi=[160, 35, 300, 170], blackMaximumValue=85,
        componentArea=[80, 6000], componentWidth=[8, 80], componentHeight=[25, 130],
        minimumFill=.2, maximumRootOffsetPx=5, minimumSideMarginPx=1,
        minimumOrphanHeightPx=8, orphanComponentsRejected=True,
        bottomBandHeightPx=8, minimumBottomPixels=20, minimumGapPx=12., maximumTipYDifferencePx=25.,
        objectRoi=[40, 60, 280, 235], objectHsvLow=[145, 120, 80], objectHsvHigh=[179, 255, 255],
        objectArea=[80, 8000], objectWidth=[10, 120], objectHeight=[8, 130], minimumObjectFill=.5,
        depthBandBeforeObjectTopPx=40., depthBandAfterObjectBottomPx=15.,
        tipMeaning='bottom_eight_visible_component_rows_not_fingertip_depth_contact_force_or_clearance',
        gateMeaning='image_visibility_only_external_alignment_and_contact_verification_required')
    if version == 2:
        policy['schemaVersion'] = PROFILE_SCHEMA
        del policy['depthBandBeforeObjectTopPx']; del policy['depthBandAfterObjectBottomPx']
        policy.update(verticalGate='object_overlaps_both_full_visible_finger_component_profiles',
            minimumVerticalProfileOverlapPx=3.,
            gateMeaning='both_finger_profile_image_overlap_not_depth_contact_force_or_clearance')
    return policy


def _gate(policy, left, right, obj):
    l, r = np.array(left['point']), np.array(right['point']); middle = (l+r)/2
    horizontal = bool(l[0] < obj['center'][0] < r[0])
    x, y, width, height = obj['bbox']
    if policy['schemaVersion'] == SCHEMA:
        depth = bool(y-40. <= middle[1] <= y+height+15.)
        return dict(horizontalInside=horizontal, depthBand=depth,
                    passed=bool(horizontal and depth), meaning=policy['gateMeaning'])
    overlaps = {}
    for name, component in [('left', left), ('right', right)]:
        bx, by, w, h = component['bbox']
        overlaps[name] = float(max(0, min(y+height, by+h)-max(y, by)))
    vertical = all(overlap >= policy['minimumVerticalProfileOverlapPx'] for overlap in overlaps.values())
    return dict(horizontalInside=horizontal, verticalProfileOverlapPassed=bool(vertical),
        verticalProfileOverlapPx=overlaps, passed=bool(horizontal and vertical), meaning=policy['gateMeaning'])


def _tip(hsv, roi, policy):
    x, y, xx, yy = roi; patch = hsv[y:yy, x:xx]
    mask = np.uint8(patch[:, :, 2] < policy['blackMaximumValue'])*255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = []; orphan_components = 0
    for index in range(1, count):
        bx, by, width, height, area = map(int, stats[index])
        if (area >= policy['componentArea'][0] and policy['minimumOrphanHeightPx'] <= height <= policy['componentHeight'][1]
                and policy['componentWidth'][0] <= width <= policy['componentWidth'][1]
                and by > policy['maximumRootOffsetPx']): orphan_components += 1
        if not (policy['componentArea'][0] <= area <= policy['componentArea'][1]
                and policy['componentWidth'][0] <= width <= policy['componentWidth'][1]
                and policy['componentHeight'][0] <= height <= policy['componentHeight'][1]
                and area/(width*height) >= policy['minimumFill']
                and by <= policy['maximumRootOffsetPx']
                and bx >= policy['minimumSideMarginPx'] and bx+width < xx-x
                and by+height < yy-y): continue
        ys, xs = np.where(labels == index); bottom_y = int(ys.max())
        bottom = ys >= bottom_y-policy['bottomBandHeightPx']+1
        if int(bottom.sum()) < policy['minimumBottomPixels']: continue
        candidates.append(dict(point=[float(xs[bottom].mean()+x), float(bottom_y+y)],
            bbox=[bx+x, by+y, width, height], area=area, bottomPixels=int(bottom.sum())))
    _require(len(candidates) == 1 and orphan_components == 0, 'near_tip_missing_or_ambiguous')
    return candidates[0]


def _object(hsv, policy):
    x, y, xx, yy = policy['objectRoi']; patch = hsv[y:yy, x:xx]
    mask = cv2.inRange(patch, np.uint8(policy['objectHsvLow']), np.uint8(policy['objectHsvHigh']))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for index in range(1, count):
        bx, by, width, height, area = map(int, stats[index])
        if (policy['objectArea'][0] <= area <= policy['objectArea'][1]
                and policy['objectWidth'][0] <= width <= policy['objectWidth'][1]
                and policy['objectHeight'][0] <= height <= policy['objectHeight'][1]
                and area/(width*height) >= policy['minimumObjectFill']):
            candidates.append(dict(center=(centers[index]+[x, y]).tolist(),
                bbox=[bx+x, by+y, width, height], area=area))
    _require(len(candidates) == 1, 'near_object_missing_or_ambiguous')
    return candidates[0]


class NearGripperTipObserver:
    def __init__(self, *, policy_version=1):
        self.policy = observer_policy(version=policy_version); self.binding = digest(self.policy)
        self.landmark_contract = (PREFIX if policy_version == 1 else 'near-gripper-visible-tips-v2:')+self.binding

    def observe(self, raw):
        _require(isinstance(raw, bytes) and digest(self.policy) == self.binding, 'near_observer_policy_changed')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        _require(image is not None and image.shape == (240, 320, 3), 'near_qvga_image_required')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        left = _tip(hsv, self.policy['leftRoi'], self.policy); right = _tip(hsv, self.policy['rightRoi'], self.policy)
        l, r = np.array(left['point']), np.array(right['point']); gap = float(r[0]-l[0]); middle = (l+r)/2
        _require(gap >= self.policy['minimumGapPx'] and abs(l[1]-r[1]) <= self.policy['maximumTipYDifferencePx'],
                 'near_visible_tips_not_separated')
        obj = _object(hsv, self.policy); x, y, width, height = obj['bbox']
        return dict(schemaVersion=self.policy['schemaVersion'], policySha256=self.binding, observationContractSha256=self.binding,
            landmarkContract=self.landmark_contract, imageSha256=hashlib.sha256(raw).hexdigest(),
            state=[obj['center'][0]-float(middle[0]), obj['center'][1]-float(middle[1]), gap],
            stateFields=STATE_FIELDS.copy(), objectVisibleWidthPx=float(width), object=obj,
            points=dict(left=left['point'], right=right['point'], middle=middle.tolist()),
            components=dict(left=left, right=right),
            nearClosingVisibilityGate=_gate(self.policy, left, right, obj),
            tipMeaning=self.policy['tipMeaning'], contactVerified=False, forceVerified=False,
            depthVerified=False, graspVerified=False, clearanceVerified=False)


def validate_observation(observation):
    """모델 입력의 정책·점·간격·물체·가시성 표기가 서로 같은 관측인지 검사한다."""
    _require(isinstance(observation, dict) and observation.get('schemaVersion') in (SCHEMA, PROFILE_SCHEMA), 'near_observation_contract')
    policy = observer_policy(version=1 if observation['schemaVersion'] == SCHEMA else 2); binding = digest(policy)
    _require(observation.get('schemaVersion') == policy['schemaVersion']
             and observation.get('policySha256') == binding
             and observation.get('observationContractSha256') == binding
             and observation.get('stateFields') == STATE_FIELDS
             and isinstance(observation.get('imageSha256'), str)
             and re.fullmatch('[0-9a-f]{64}', observation['imageSha256']) is not None,
             'near_observation_contract')
    try:
        state = np.array(observation['state'], dtype=float)
        l = np.array(observation['points']['left'], dtype=float); r = np.array(observation['points']['right'], dtype=float)
        middle = (l+r)/2; center = np.array(observation['object']['center'], dtype=float)
        bbox = observation['object']['bbox']; width = observation['objectVisibleWidthPx']
    except (KeyError, TypeError, ValueError): raise ValueError('near_observation_points') from None
    _require(state.shape == (3,) and l.shape == r.shape == center.shape == (2,)
             and np.isfinite(np.concatenate((state, l, r, center))).all(), 'near_observation_points')
    _require(all(0 <= p[0] < 320 and 0 <= p[1] < 240 for p in (l, r, center)), 'near_observation_points')
    expected = [center[0]-middle[0], center[1]-middle[1], r[0]-l[0]]
    _require(np.allclose(state, expected, atol=1e-7) and state[2] >= policy['minimumGapPx']
             and abs(l[1]-r[1]) <= policy['maximumTipYDifferencePx']
             and isinstance(bbox, list) and len(bbox) == 4 and width == bbox[2], 'near_observation_state_changed')
    gate = observation.get('nearClosingVisibilityGate')
    if policy['schemaVersion'] == SCHEMA:
        expected_gate = _gate(policy, dict(point=l.tolist()), dict(point=r.tolist()), observation['object'])
    else:
        components = observation.get('components')
        _require(isinstance(components, dict) and set(components) == {'left', 'right'}, 'near_profile_components_required')
        for name, tip in [('left', l), ('right', r)]:
            component = components[name]
            _require(component.get('point') == tip.tolist() and isinstance(component.get('bbox'), list)
                     and len(component['bbox']) == 4 and all(type(v) is int for v in component['bbox']), 'near_profile_component_changed')
        expected_gate = _gate(policy, components['left'], components['right'], observation['object'])
    _require(isinstance(gate, dict) and gate == expected_gate, 'near_visibility_gate_changed')
    return state
