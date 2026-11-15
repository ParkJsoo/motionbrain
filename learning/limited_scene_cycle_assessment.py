"""고정 외부 구도·분홍 물체의 제한된 영상 단계 전환 정책.

픽셀 판정만 수행한다. 파일·장치 접근, 명령 선택, metric 깊이·접촉력·실제
파지 성공 증명은 없다. 호출자는 관측 원본·고정 구도·부팅·신선도와 시간
간격을 별도로 검증해야 한다. accepted는 이 영상 정책의 통과만 뜻한다.
"""
from __future__ import annotations

import copy
import math
import re

from learning.marker_virtual_observer import SCHEMA as EXTERNAL_SCHEMA, PREFIX
from learning.near_gripper_tip_observer import validate_observation

SCHEMA = 'motionbrain.limited_scene_cycle_image_assessment.v1'
POLICY = dict(minimumUprightAspect=2., maximumCloseGapAboveObjectWidthPx=20.,
              minimumObjectRisePx=20., maximumStableCenterDistancePx=8.,
              maximumGripperRelativeDistancePx=25., maximumRetentionGapChangePx=15.,
              minimumSourceSeparationSeconds=3., maximumTableBottomDifferencePx=6.,
              maximumTableCenterXDifferencePx=12., minimumReleaseGapIncreasePx=20.,
              minimumReleasedJawMarginPx=3.)


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _sha(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _geometry(observation, width, height):
    obj = observation['object']; box = obj['bbox']; center = obj['center']
    _require(isinstance(box, list) and len(box) == 4 and all(_number(x) for x in box)
             and isinstance(center, list) and len(center) == 2 and all(_number(x) for x in center),
             'object_geometry_invalid')
    x, y, w, h = box
    _require(w > 0 and h > 0 and 0 <= x < x+w <= width and 0 <= y < y+h <= height
             and x <= center[0] <= x+w and y <= center[1] <= y+h
             and _number(obj.get('area')) and 0 < obj['area'] <= w*h, 'object_geometry_invalid')
    return dict(center=center.copy(), bbox=box.copy(), bottom=y+h, upright=h >= 2.*w)


def _external(value):
    _require(isinstance(value, dict) and value.get('schemaVersion') == EXTERNAL_SCHEMA
             and _sha(value.get('imageSha256')) and _sha(value.get('registrationSha256'))
             and value.get('landmarkContract') == PREFIX+value['registrationSha256'],
             'external_observation_contract_invalid')
    return _geometry(value, 1920, 1080)


def _near(value, *, closing=True):
    state = validate_observation(value).tolist()
    _geometry(value, 320, 240)
    _require(all(_number(x) for x in value['state'])
             and _number(value['objectVisibleWidthPx']), 'near_observation_numeric_invalid')
    if closing:
        _require(value['nearClosingVisibilityGate']['passed'] is True, 'near_closing_visibility_failed')
    return state


def _records(records, *, closing=True):
    parsed = []
    for record in records:
        _require(isinstance(record, dict), 'dual_observation_required')
        parsed.append((_external(record['external']), _near(record.get('near'), closing=closing)))
    _require(len({r['external']['registrationSha256'] for r in records}) == 1
             and len({r['near']['observationContractSha256'] for r in records}) == 1,
             'observation_contract_changed')
    return parsed


def _span(value):
    _require(_number(value) and value >= POLICY['minimumSourceSeparationSeconds'],
             'source_separation_too_short_or_invalid')


def _distance(left, right):
    return math.hypot(left[0]-right[0], left[1]-right[1])


def _bindings(records):
    return [dict(externalImageSha256=r['external']['imageSha256'],
                 externalRegistrationSha256=r['external']['registrationSha256'],
                 nearImageSha256=r['near']['imageSha256'],
                 nearContractSha256=r['near']['observationContractSha256']) for r in records]


def _result(gate, evaluate):
    evidence = dict(schemaVersion=SCHEMA, gate=gate, policy=copy.deepcopy(POLICY),
                    assessmentMeaning='fixed_view_image_gate_not_metric_depth_force_or_physical_grasp_proof',
                    physicalGraspVerified=False, metricDepthVerified=False, contactForceVerified=False,
                    rawSourceFreshnessVerified=False)
    try:
        metrics = evaluate()
    except (ValueError, TypeError, KeyError, IndexError, OverflowError, AttributeError) as error:
        evidence['reason'] = str(error) if isinstance(error, ValueError) else 'invalid_observation_structure'
        return dict(accepted=False, evidence=evidence)
    evidence.update(metrics)
    return dict(accepted=True, evidence=evidence)


def grasp_candidate(external, near):
    """양턱 사이 영상 가시성과 외부 물체 세로/가로 비율 2 이상만 확인한다."""
    def evaluate():
        ext = _external(external); _near(near)
        _require(ext['upright'], 'external_object_not_upright')
        return dict(externalObject=ext, nearState=near['state'].copy(),
                    externalImageSha256=external['imageSha256'], nearImageSha256=near['imageSha256'])
    return _result('grasp_candidate', evaluate)


def close_candidate(external, near, initial_near):
    """닫힘 후보: 초기보다 작은 gap이며 현재 보이는 폭+20px 이내. 실제 파지 아님."""
    def evaluate():
        ext = _external(external); state = _near(near); initial = _near(initial_near)
        _require(near['observationContractSha256'] == initial_near['observationContractSha256'],
                 'observation_contract_changed')
        _require(near['nearClosingVisibilityGate']['horizontalInside'] is True,
                 'current_object_not_between_jaws')
        _require(state[2] < initial[2], 'gap_did_not_decrease')
        _require(state[2] <= near['objectVisibleWidthPx']+20., 'close_gap_still_too_wide')
        return dict(externalObject=ext, initialGapPx=initial[2], currentGapPx=state[2],
                    currentObjectVisibleWidthPx=near['objectVisibleWidthPx'],
                    sourceImages=_bindings([dict(external=external, near=near)]),
                    initialNearImageSha256=initial_near['imageSha256'])
    return _result('close_candidate', evaluate)


def retained_lift(before, early, late, min_span_seconds):
    """초기보다 물체 하단 20px 이상 상승, 중심 안정 8px, 상대점 25px/gap 15px."""
    def evaluate():
        _span(min_span_seconds)
        (base, bn), (first, en), (last, ln) = _records([before, early, late])
        rises = [base['bottom']-v['bottom'] for v in (first, last)]
        stability = _distance(first['center'], last['center'])
        relative = [_distance(bn[:2], v[:2]) for v in (en, ln)]
        gaps = [abs(v[2]-bn[2]) for v in (en, ln)]
        _require(min(rises) >= 20., 'object_did_not_rise_in_both_observations')
        _require(stability <= 8., 'external_object_not_stable')
        _require(max(relative) <= 25. and max(gaps) <= 15., 'object_not_retained_relative_to_gripper')
        return dict(objectBottomRisePx=rises, externalCenterDistancePx=stability,
                    relativeGripperDistancePx=relative, gapChangePx=gaps,
                    sourceImages=_bindings([before, early, late]),
                    sourceSeparationSecondsAsProvided=min_span_seconds,
                    sourceSeparationIndependentlyVerified=False)
    return _result('retained_lift', evaluate)


def _table(base, current):
    _require(current['upright'], 'external_object_not_upright')
    _require(abs(current['bottom']-base['bottom']) <= 6.
             and abs(current['center'][0]-base['center'][0]) <= 12., 'object_not_at_saved_table_image_position')


def table_return(before_lift, early, late, min_span_seconds):
    """저장 탁상 하단±6px/X±12px, 세로 자세·중심 안정 8px, 원본 간격 3초."""
    def evaluate():
        _span(min_span_seconds)
        (base, _), (first, _), (last, _) = _records([before_lift, early, late])
        _require(base['upright'], 'saved_table_object_not_upright')
        _table(base, first); _table(base, last)
        stability = _distance(first['center'], last['center'])
        _require(stability <= 8., 'external_object_not_stable')
        return dict(savedTableObject=base, earlyObject=first, lateObject=last,
                    externalCenterDistancePx=stability,
                    sourceImages=_bindings([before_lift, early, late]),
                    sourceSeparationSecondsAsProvided=min_span_seconds,
                    sourceSeparationIndependentlyVerified=False)
    return _result('table_return', evaluate)


def released(before_open, after, table_baseline):
    """열기 후 gap 20px 증가·각 턱 3px 여유와 저장 탁상 영상 위치를 확인한다."""
    def evaluate():
        (_, bn), (current, an), (base, _) = _records(
            [before_open, after, table_baseline], closing=False)
        _require(base['upright'], 'saved_table_object_not_upright')
        _table(base, current)
        _require(an[2]-bn[2] >= 20., 'release_gap_did_not_increase')
        near = after['near']; x, _, width, _ = near['object']['bbox']
        left = x-near['points']['left'][0]; right = near['points']['right'][0]-(x+width)
        _require(left >= 3. and right >= 3., 'released_jaws_not_both_clear')
        return dict(gapIncreasePx=an[2]-bn[2], leftJawMarginPx=left, rightJawMarginPx=right,
                    sourceImages=_bindings([before_open, after, table_baseline]),
                    savedTableObject=base, afterExternalObject=current)
    return _result('released', evaluate)
