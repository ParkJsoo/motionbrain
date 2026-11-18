"""집게 카메라의 보이는 검은 손가락 끝·분홍 물체 관측. 접촉·힘·깊이는 판정하지 않는다.

v3만 알려진 단일 분홍 물체의 관측된 두 조각 결합을 허용한다. 각 조각의
밀도는 검사하지만, 가림 사이 빈 행을 포함한 외곽 밀도는 측정값으로만 남긴다.
가려진 픽셀을 복원하거나 두 조각이 같은 물체임을 독립 증명하지 않는다.
"""
from __future__ import annotations
import copy
import hashlib
import json
import re

import cv2
import numpy as np

SCHEMA = 'motionbrain.near_gripper_visible_tips.v1'
PROFILE_SCHEMA = 'motionbrain.near_gripper_visible_tips.v2'
FRAGMENT_SCHEMA = 'motionbrain.near_gripper_visible_tips.v3'
BOUNDARY_SCHEMA = 'motionbrain.near_gripper_visible_tips.v4'
PREFIX = 'near-gripper-visible-tips-v1:'
STATE_FIELDS = ['objectMinusVisibleJawMidXpx', 'objectMinusVisibleJawMidYpx', 'visibleTipHorizontalGapPx']


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def observer_policy(*, version=1):
    _require(type(version) is int and version in (1, 2, 3, 4), 'near_observer_policy_version')
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
    if version in (2, 3, 4):
        policy['schemaVersion'] = PROFILE_SCHEMA
        del policy['depthBandBeforeObjectTopPx']; del policy['depthBandAfterObjectBottomPx']
        policy.update(verticalGate='object_overlaps_both_full_visible_finger_component_profiles',
            minimumVerticalProfileOverlapPx=3.,
            gateMeaning='both_finger_profile_image_overlap_not_depth_contact_force_or_clearance')
    if version in (3, 4):
        policy.update(schemaVersion=FRAGMENT_SCHEMA,
            objectFragmentPolicy=dict(maximumQualifiedComponents=2, maximumSecondaryMainAreaRatio=.35,
                maximumCenterXDifferencePx=5., minimumHorizontalOverlapFraction=.5,
                minimumVerticalNonoverlapGapPx=0., maximumVerticalNonoverlapGapPx=10.,
                minimumFillAppliesToVisibleFragments=True, unionEnvelopeFillNotForegroundAccuracy=True,
                centroidMeaning='observed_pixel_area_weighted', occludedPixelsEstimated=False,
                meaning='known_single_pink_object_visible_fragment_union_not_object_identity_or_grasp_proof'))
    if version == 4:
        policy.update(schemaVersion=BOUNDARY_SCHEMA,
            orphanBoundaryPolicy='ignore_small_side_clipped_orphan_below_and_disjoint_from_unique_finger')
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
    candidates = []; orphan_components = []
    for index in range(1, count):
        bx, by, width, height, area = map(int, stats[index])
        if (area >= policy['componentArea'][0] and policy['minimumOrphanHeightPx'] <= height <= policy['componentHeight'][1]
                and policy['componentWidth'][0] <= width <= policy['componentWidth'][1]
                and by > policy['maximumRootOffsetPx']):
            orphan_components.append(dict(bbox=[bx+x, by+y, width, height], area=area))
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
    _require(len(candidates) == 1, 'near_tip_missing_or_ambiguous')
    ignored = []
    if policy.get('orphanBoundaryPolicy') is not None:
        fx, fy, fw, fh = candidates[0]['bbox']
        for component in orphan_components:
            ox, oy, ow, oh = component['bbox']
            if ((ox == x if roi == policy['leftRoi'] else ox+ow == xx)
                    and oh < policy['componentHeight'][0]
                    and oy >= fy+fh and (ox+ow <= fx or ox >= fx+fw)):
                ignored.append(component)
    _require(len(ignored) == len(orphan_components), 'near_tip_missing_or_ambiguous')
    if policy.get('orphanBoundaryPolicy') is not None:
        candidates[0]['ignoredBoundaryBackground'] = ignored
    return candidates[0]


def _fragment_object(components, policy):
    """관측된 성분의 면적·중심·박스만 결합한다. 빈 픽셀을 복원하지 않는다."""
    _require(isinstance(components, list) and 1 <= len(components) <= 2,
             'near_fragment_count')
    x0, y0, x1, y1 = policy['objectRoi']
    for component in components:
        _require(isinstance(component, dict) and set(component) == {'center', 'bbox', 'area'},
                 'near_fragment_fields')
        box, center, area = component['bbox'], component['center'], component['area']
        _require(isinstance(box, list) and len(box) == 4 and all(type(v) is int for v in box)
                 and isinstance(center, list) and len(center) == 2
                 and all(type(v) in (int, float) and np.isfinite(v) for v in center)
                 and type(area) is int, 'near_fragment_geometry')
        x, y, w, h = box
        _require(x0 <= x < x+w <= x1 and y0 <= y < y+h <= y1
                 and x <= center[0] < x+w and y <= center[1] < y+h
                 and policy['objectArea'][0] <= area <= min(policy['objectArea'][1], w*h)
                 and policy['objectWidth'][0] <= w <= policy['objectWidth'][1]
                 and policy['objectHeight'][0] <= h <= policy['objectHeight'][1]
                 and area/(w*h) >= policy['minimumObjectFill'], 'near_fragment_geometry')
    ordered = sorted(components, key=lambda c: (-c['area'], c['bbox']))
    if len(ordered) == 1:
        return copy.deepcopy(ordered[0])
    main, secondary = ordered
    p = policy['objectFragmentPolicy']
    a, b = main['bbox'], secondary['bbox']
    overlap = max(0, min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0]))
    top, bottom = sorted((a, b), key=lambda box: box[1])
    gap = bottom[1]-(top[1]+top[3])
    _require(secondary['area']/main['area'] <= p['maximumSecondaryMainAreaRatio']
             and abs(main['center'][0]-secondary['center'][0]) <= p['maximumCenterXDifferencePx']
             and overlap/min(a[2], b[2]) >= p['minimumHorizontalOverlapFraction']
             and p['minimumVerticalNonoverlapGapPx'] <= gap <= p['maximumVerticalNonoverlapGapPx'],
             'near_fragment_pair_rejected')
    x, y = min(a[0], b[0]), min(a[1], b[1])
    w, h = max(a[0]+a[2], b[0]+b[2])-x, max(a[1]+a[3], b[1]+b[3])-y
    area = main['area']+secondary['area']
    _require(policy['objectArea'][0] <= area <= policy['objectArea'][1]
             and policy['objectWidth'][0] <= w <= policy['objectWidth'][1]
             and policy['objectHeight'][0] <= h <= policy['objectHeight'][1], 'near_fragment_union_rejected')
    center = [(main['center'][i]*main['area']+secondary['center'][i]*secondary['area'])/area
              for i in range(2)]
    return dict(center=center, bbox=[x, y, w, h], area=area)


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
    if policy['schemaVersion'] in (FRAGMENT_SCHEMA, BOUNDARY_SCHEMA):
        obj = _fragment_object(candidates, policy)
        return obj, dict(
            mode='single_visible_component' if len(candidates) == 1 else 'observed_fragment_union',
            components=copy.deepcopy(candidates), occludedPixelsEstimated=False,
            centroidMeaning='observed_pixel_area_weighted',
            unionEnvelopeFill=obj['area']/(obj['bbox'][2]*obj['bbox'][3]))
    _require(len(candidates) == 1, 'near_object_missing_or_ambiguous')
    return candidates[0]


class NearGripperTipObserver:
    def __init__(self, *, policy_version=1):
        self.policy = observer_policy(version=policy_version); self.binding = digest(self.policy)
        self.landmark_contract = (PREFIX if policy_version == 1 else
                                 f'near-gripper-visible-tips-v{policy_version}:')+self.binding

    def observe(self, raw):
        _require(isinstance(raw, bytes) and digest(self.policy) == self.binding, 'near_observer_policy_changed')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        _require(image is not None and image.shape == (240, 320, 3), 'near_qvga_image_required')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        left = _tip(hsv, self.policy['leftRoi'], self.policy); right = _tip(hsv, self.policy['rightRoi'], self.policy)
        l, r = np.array(left['point']), np.array(right['point']); gap = float(r[0]-l[0]); middle = (l+r)/2
        _require(gap >= self.policy['minimumGapPx'] and abs(l[1]-r[1]) <= self.policy['maximumTipYDifferencePx'],
                 'near_visible_tips_not_separated')
        detected = _object(hsv, self.policy)
        obj, fragments = detected if self.policy['schemaVersion'] in (FRAGMENT_SCHEMA, BOUNDARY_SCHEMA) else (detected, None)
        x, y, width, height = obj['bbox']
        observation = dict(schemaVersion=self.policy['schemaVersion'], policySha256=self.binding, observationContractSha256=self.binding,
            landmarkContract=self.landmark_contract, imageSha256=hashlib.sha256(raw).hexdigest(),
            state=[obj['center'][0]-float(middle[0]), obj['center'][1]-float(middle[1]), gap],
            stateFields=STATE_FIELDS.copy(), objectVisibleWidthPx=float(width), object=obj,
            points=dict(left=left['point'], right=right['point'], middle=middle.tolist()),
            components=dict(left=left, right=right),
            nearClosingVisibilityGate=_gate(self.policy, left, right, obj),
            tipMeaning=self.policy['tipMeaning'], contactVerified=False, forceVerified=False,
            depthVerified=False, graspVerified=False, clearanceVerified=False)
        if fragments is not None:
            observation['objectFragments'] = fragments
        return observation


def validate_observation(observation):
    """모델 입력의 정책·점·간격·물체·가시성 표기가 서로 같은 관측인지 검사한다."""
    versions = {SCHEMA: 1, PROFILE_SCHEMA: 2, FRAGMENT_SCHEMA: 3, BOUNDARY_SCHEMA: 4}
    _require(isinstance(observation, dict) and observation.get('schemaVersion') in versions, 'near_observation_contract')
    version = versions[observation['schemaVersion']]
    policy = observer_policy(version=version); binding = digest(policy)
    _require(observation.get('schemaVersion') == policy['schemaVersion']
             and observation.get('policySha256') == binding
             and observation.get('observationContractSha256') == binding
             and observation.get('stateFields') == STATE_FIELDS
             and isinstance(observation.get('imageSha256'), str)
             and re.fullmatch('[0-9a-f]{64}', observation['imageSha256']) is not None,
             'near_observation_contract')
    if version in (3, 4):
        fragments = observation.get('objectFragments')
        _require(isinstance(fragments, dict) and set(fragments) == {
            'mode', 'components', 'occludedPixelsEstimated', 'centroidMeaning', 'unionEnvelopeFill'}
            and fragments['occludedPixelsEstimated'] is False
            and fragments['centroidMeaning'] == policy['objectFragmentPolicy']['centroidMeaning'],
            'near_fragment_metadata_changed')
        obj = _fragment_object(fragments['components'], policy)
        _require(fragments['mode'] == ('single_visible_component' if len(fragments['components']) == 1
                                     else 'observed_fragment_union') and observation.get('object') == obj
                 and type(fragments['unionEnvelopeFill']) is float
                 and fragments['unionEnvelopeFill'] == obj['area']/(obj['bbox'][2]*obj['bbox'][3])
                 and observation.get('landmarkContract') == f'near-gripper-visible-tips-v{version}:{binding}'
                 and all(observation.get(k) is False for k in
                         ('contactVerified', 'forceVerified', 'depthVerified', 'graspVerified', 'clearanceVerified')),
                 'near_fragment_union_changed')
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
