"""고정 원본 진입의 영상 일치 검사. 깊이·접촉·예측 정확도의 보증이 아니다."""
from learning import limited_scene_cycle_assessment as image

AXIS_BAND_PX = 2.
BOUNDARY_BAND_PX = 1.


def _point(value):
    image._require(isinstance(value, (list, tuple)) and len(value) == 2
                   and all(image._number(x) for x in value), 'entry_point_invalid')
    return value


def _references(records):
    image._require(isinstance(records, list) and len(records) == 2, 'two_entry_references_required')
    image._records(records)
    offsets = []
    for record in records:
        proxy = _point(record['external']['virtualProxyPx'])
        center = record['external']['object']['center']
        offsets.append([proxy[i]-center[i] for i in range(2)])
    image._require(all(abs(offsets[0][i]-offsets[1][i]) <= AXIS_BAND_PX for i in range(2)),
                   'entry_reference_offsets_disagree')
    return records


def prediction(current_xy, predicted_xy, goal_xy):
    """예측의 축별 목표 초과를 거절한다. 근접 영상의 미래 상태는 추정하지 않는다."""
    def evaluate():
        before, after, goal = map(_point, (current_xy, predicted_xy, goal_xy))
        errors = [[p[i]-goal[i] for i in range(2)] for p in (before, after)]
        crossed = [i for i, (a, b) in enumerate(zip(*errors))
                   if (a > AXIS_BAND_PX and b < -AXIS_BAND_PX)
                   or (a < -AXIS_BAND_PX and b > AXIS_BAND_PX)
                   or (abs(a) <= AXIS_BAND_PX and abs(b) > AXIS_BAND_PX)]
        image._require(not crossed, 'predicted_entry_axis_overshoot')
        return dict(signedErrorsPx=errors, axisBandPx=AXIS_BAND_PX, nearScalePredicted=False)
    return image._result('entry_prediction_axis_guard', evaluate)


def assess(records, external, near, goal_xy):
    """첫 CLOSE 직전에만 쓰는 고정 원본의 벌어짐·겉보기 크기 검사."""
    def evaluate():
        refs = _references(records)
        image._records([*refs, dict(external=external, near=near)])
        image._require(image._external(external)['upright'], 'external_object_not_upright')
        proxy, goal = map(_point, (external['virtualProxyPx'], goal_xy))
        errors = [proxy[i]-goal[i] for i in range(2)]
        image._require(max(map(abs, errors)) <= AXIS_BAND_PX, 'entry_axis_alignment_required')
        gaps = [r['near']['state'][2] for r in refs]; gap = near['state'][2]
        delta = 2*BOUNDARY_BAND_PX
        image._require(min(gaps) > delta, 'entry_reference_gap_invalid')
        image._require(min(gaps)-delta <= gap <= max(gaps)+delta, 'entry_opening_mismatch')
        ratios = {}
        for name, index in [('width', 2), ('height', 3)]:
            sizes = [r['near']['object']['bbox'][index] for r in refs]
            low = min((s-delta)/(g+delta) for s, g in zip(sizes, gaps))
            high = max((s+delta)/(g-delta) for s, g in zip(sizes, gaps))
            actual = near['object']['bbox'][index]/gap
            image._require(low <= actual <= high, 'entry_apparent_'+name+'_mismatch')
            ratios[name] = dict(actual=actual, lower=low, upper=high)
        return dict(signedErrorsPx=errors, axisBandPx=AXIS_BAND_PX,
                    boundaryBandPx=BOUNDARY_BAND_PX, apparentRatios=ratios,
                    sourceImages=image._bindings([*refs, dict(external=external, near=near)]),
                    toleranceMeaning='fixed_image_reference_rejection_band_not_calibrated_uncertainty')
    return image._result('first_close_reference_entry', evaluate)
