"""새 외부 구도의 명시적 특징점 등록·평행이동 관측. 장치·파일 접근 없음.

점의 물리적 정체성은 분석가가 확인한다. 고정 패치와 선택적 회전 목록은
2D 이동만 제안하며 깊이·접촉·정확도·옛 모델의 전이 적격을 판단하지 않는다.
회전 후보는 영상 패치의 방향이며 실제 관절의 회전각이 아니다.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math

import cv2
import numpy as np

SCHEMA = 'motionbrain.registered_scene_landmarks.v1'
METHOD = 'registered_scene_template_points'
LANDMARK_PREFIX = 'registered-scene-template-points-v1:'
NAMES = ('object', 'jawLeft', 'jawRight')
MINIMUM_NCC = dict(object=.85, jawLeft=.94, jawRight=.94)


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _digest(value):
    try:
        raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    except (TypeError, ValueError) as exc:
        raise ValueError('scene_contract_not_json') from exc
    return _sha(raw)


def _rectangle(value, shape):
    _require(isinstance(value, list) and len(value) == 4
             and all(type(v) is int for v in value), 'scene_roi_invalid')
    x0, y0, x1, y1 = value
    h, w = shape
    _require(0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h, 'scene_roi_clipped')
    return x0, y0, x1, y1


def _parameters(p):
    required = {
        'imageShape', 'patchRois', 'searchRois', 'minimumNcc', 'minimumPeakGap',
        'peakExclusionRadius', 'minimumTemplateStd'}
    _require(isinstance(p, dict) and required <= set(p)
             and set(p) <= required | {'rotationDegrees', 'jawPair'},
             'scene_parameter_fields')
    shape = p['imageShape']
    _require(isinstance(shape, list) and len(shape) == 2
             and all(type(v) is int for v in shape)
             and 32 <= shape[0] <= 1080 and 32 <= shape[1] <= 1920, 'scene_image_shape_invalid')
    _require(all(isinstance(p[k], dict) and set(p[k]) == set(NAMES)
                 for k in ('patchRois', 'searchRois', 'minimumNcc')), 'scene_three_patch_parameters_required')
    if 'rotationDegrees' in p:
        bank = p['rotationDegrees']
        _require(isinstance(bank, dict) and set(bank) == set(NAMES), 'scene_rotation_names_invalid')
        for angles in bank.values():
            _require(isinstance(angles, list) and 1 <= len(angles) <= 71
                     and all(_number(v) and abs(v) <= 35 for v in angles)
                     and angles == sorted(set(angles)) and 0 in angles, 'scene_rotation_bank_invalid')
    if 'jawPair' in p:
        pair = p['jawPair']
        _require(isinstance(pair, dict) and set(pair) == {'minimumSpanPx', 'maximumSpanPx',
                 'minimumMeanNccGap', 'maxCandidatesPerJaw'}
                 and all(_number(pair.get(k)) for k in
                         ('minimumSpanPx', 'maximumSpanPx', 'minimumMeanNccGap'))
                 and 0 < pair['minimumSpanPx'] < pair['maximumSpanPx'] <= math.hypot(*shape)
                 and .04 <= pair['minimumMeanNccGap'] <= 1.
                 and type(pair['maxCandidatesPerJaw']) is int
                 and 1 <= pair['maxCandidatesPerJaw'] <= 64, 'scene_jaw_pair_parameters_invalid')
    for name in NAMES:
        x0, y0, x1, y1 = _rectangle(p['patchRois'][name], shape)
        a, b, c, d = _rectangle(p['searchRois'][name], shape)
        _require(x1-x0 >= 8 and y1-y0 >= 8
                 and a+2 <= x0 < x1 <= c-2 and b+2 <= y0 < y1 <= d-2,
                 'scene_patch_or_search_margin_invalid')
        threshold = p['minimumNcc'][name]
        _require(_number(threshold) and MINIMUM_NCC[name] <= threshold <= 1.,
                 'scene_ncc_threshold_invalid')
    _require(_number(p['minimumPeakGap']) and .04 <= p['minimumPeakGap'] <= 1.
             and type(p['peakExclusionRadius']) is int and 1 <= p['peakExclusionRadius'] <= 3
             and _number(p['minimumTemplateStd']) and 1.5 <= p['minimumTemplateStd'] <= 128.,
             'scene_quality_parameters_invalid')


def _image(raw, shape):
    _require(isinstance(raw, bytes) and 0 < len(raw) <= 32_000_000, 'scene_image_bytes_required')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    _require(image is not None and list(image.shape[:2]) == shape, 'scene_image_shape_changed')
    pixels = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    return image, _sha(pixels.tobytes())


def _seeds(points, parameters, reference):
    _require(isinstance(points, dict) and set(points) == set(NAMES), 'scene_three_seed_points_required')
    gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    for name in NAMES:
        point = points[name]
        x0, y0, x1, y1 = parameters['patchRois'][name]
        _require(isinstance(point, list) and len(point) == 2 and all(_number(v) for v in point)
                 and x0 <= point[0] < x1 and y0 <= point[1] < y1, 'scene_seed_outside_patch')
        _require(float(gray[y0:y1, x0:x1].std()) >= parameters['minimumTemplateStd'],
                 'scene_template_texture_insufficient')
    _require(points['jawLeft'] != points['jawRight'], 'scene_coincident_jaw_points')


def _template(reference, point, roi, angle):
    x0, y0, x1, y1 = roi
    if angle == 0:
        return reference[y0:y1, x0:x1]
    # 씨앗점을 중심으로 전체 원본을 회전한 뒤 출력 창만 잘라낸다.
    # 작은 템플릿의 빈 모서리를 채워 실제 원본인 것처럼 쓰지 않는다.
    matrix = cv2.getRotationMatrix2D(tuple(map(float, point)), angle, 1.)
    matrix[:, 2] -= [x0, y0]
    width, height = x1-x0, y1-y0
    inverse = cv2.invertAffineTransform(matrix)
    corners = np.array([[0, 0, 1], [width-1, 0, 1],
                        [0, height-1, 1], [width-1, height-1, 1]]) @ inverse.T
    h, w = reference.shape[:2]
    _require(np.all(corners >= 0) and np.all(corners[:, 0] < w-1)
             and np.all(corners[:, 1] < h-1), 'scene_rotated_template_clipped')
    return cv2.warpAffine(reference, matrix, (width, height), flags=cv2.INTER_LINEAR)


def _pair_require(condition, reason, diagnostic):
    if not condition:
        error = ValueError(reason)
        error.diagnostics = copy.deepcopy(diagnostic)
        raise error


def _jaw_candidates(scores, seed, patch, search, floor, radius, cap, diagnostic, name):
    remaining = scores.copy(); candidates = diagnostic['candidates'].setdefault(name, [])
    while True:
        _, peak, _, (ix, iy) = cv2.minMaxLoc(remaining)
        if peak < floor: break
        if len(candidates) >= cap:
            diagnostic['overflow'] = dict(feature=name, nextScorePosition=[ix, iy], nextNcc=float(peak))
            _pair_require(False, 'scene_jaw_candidate_budget_exceeded', diagnostic)
        candidates.append(dict(scorePosition=[ix, iy], ncc=float(peak),
            projectedPoint=[seed[0]+search[0]+ix-patch[0], seed[1]+search[1]+iy-patch[1]]))
        remaining[max(0, iy-radius):iy+radius+1, max(0, ix-radius):ix+radius+1] = -2.
    return candidates


def _jaw_pair(matches, registration):
    p = registration['parameters']; limits = p['jawPair']
    # 각 끝점의 축별 subpixel 이동은 최대 .75px다. 경계 근처의 경쟁 쌍을
    # 정수 위치만으로 배제하지 않는다. 최종 선택은 실제 subpixel span으로 검사한다.
    span_margin = 2*.75*math.sqrt(2.)
    diagnostic = dict(method='registered_jaw_pair_mean_ncc_v1', limits=copy.deepcopy(limits),
                      candidates={}, pairs=[], individualUniquenessEnforced=False,
                      spanIsPhysicalInvariant=False, nmsRadius=p['peakExclusionRadius'],
                      competingPairSubpixelSpanMarginPx=span_margin)
    for name in ('jawLeft', 'jawRight'):
        diagnostic['candidates'][name] = _jaw_candidates(matches[name][0], registration['points'][name],
            p['patchRois'][name], p['searchRois'][name], p['minimumNcc'][name],
            p['peakExclusionRadius'], limits['maxCandidatesPerJaw'], diagnostic, name)
        _pair_require(bool(diagnostic['candidates'][name]), 'scene_jaw_candidates_missing', diagnostic)
    for li, left in enumerate(diagnostic['candidates']['jawLeft']):
        for ri, right in enumerate(diagnostic['candidates']['jawRight']):
            span = float(np.linalg.norm(np.asarray(left['projectedPoint'])-right['projectedPoint']))
            eligible = span+span_margin >= limits['minimumSpanPx'] and span-span_margin <= limits['maximumSpanPx']
            diagnostic['pairs'].append(dict(leftIndex=li, rightIndex=ri, spanPx=span,
                possibleLocalizedSpanPx=[max(0., span-span_margin), span+span_margin],
                meanNcc=(left['ncc']+right['ncc'])/2, eligible=eligible,
                reason=None if eligible else 'outside_registered_span_band'))
    eligible = sorted((x for x in diagnostic['pairs'] if x['eligible']), key=lambda x: -x['meanNcc'])
    _pair_require(bool(eligible), 'scene_jaw_pair_span_not_supported', diagnostic)
    selected = eligible[0]; second = eligible[1] if len(eligible) > 1 else None
    diagnostic.update(selected=copy.deepcopy(selected), secondEligiblePair=copy.deepcopy(second),
        meanNccGap=selected['meanNcc']-second['meanNcc'] if second is not None else None,
        uniquenessBasis='competing_nms_pairs' if second is not None else 'no_second_eligible_pair')
    _pair_require(second is None or diagnostic['meanNccGap'] >= limits['minimumMeanNccGap'],
                  'scene_jaw_pair_ambiguous', diagnostic)
    chosen = {name: diagnostic['candidates'][name][selected[key]]['scorePosition']
              for name, key in [('jawLeft', 'leftIndex'), ('jawRight', 'rightIndex')]}
    return chosen, diagnostic


def _localize(scores, winning_angles, angles, registration, name, position=None, paired=False):
    p = registration['parameters']; x0, y0, _, _ = p['patchRois'][name]
    a, b, _, _ = p['searchRois'][name]
    if position is None: _, _, _, position = cv2.minMaxLoc(scores)
    ix, iy = position; peak = float(scores[iy, ix]); angle = angles[int(winning_angles[iy, ix])]
    _require(0 < ix < scores.shape[1]-1 and 0 < iy < scores.shape[0]-1, 'scene_match_at_search_boundary')
    _require(peak >= p['minimumNcc'][name], 'scene_ncc_below_threshold')
    alternative = scores.copy(); radius = p['peakExclusionRadius']
    alternative[max(0, iy-radius):iy+radius+1, max(0, ix-radius):ix+radius+1] = -2.
    _require(np.any(alternative != -2.), 'scene_search_insufficient_for_uniqueness')
    second = float(alternative.max()); gap = peak-second
    if not paired: _require(gap >= p['minimumPeakGap'], 'scene_match_ambiguous')
    offsets = []
    for before, center, after in ((scores[iy, ix-1], scores[iy, ix], scores[iy, ix+1]),
                                  (scores[iy-1, ix], scores[iy, ix], scores[iy+1, ix])):
        denominator = float(before-2*center+after)
        _require(denominator < -1e-7, 'scene_peak_not_localized')
        offset = .5*float(before-after)/denominator
        _require(_number(offset) and abs(offset) <= .75, 'scene_subpixel_peak_invalid')
        offsets.append(offset)
    shift = [a+ix-x0+offsets[0], b+iy-y0+offsets[1]]
    point = [v+s for v, s in zip(registration['points'][name], shift)]
    quality = dict(ncc=peak, alternativeNcc=second, peakGap=float(gap), shiftPx=shift,
        templateRotationDeg=angle, rotationCandidates=len(angles),
        matching=('fixed_bgr_patch_rotation_bank' if 'rotationDegrees' in p
                  else 'fixed_bgr_patch_translation_only'))
    if paired:
        quality.update(individualUniquenessEnforced=False,
                       individualUniquenessPassed=gap >= p['minimumPeakGap'])
    return point, quality


def register(reference_image, points, parameters, scene_context):
    """동작 전에 정한 구도·점·패치·검색 영역을 원본 바이트에 결합한다."""
    p = copy.deepcopy(parameters)
    _parameters(p)
    reference, pixel_sha = _image(reference_image, p['imageShape'])
    _seeds(points, p, reference)
    _require(isinstance(scene_context, dict)
             and set(scene_context) == {'physicalView', 'object', 'poseRegime'}
             and all(isinstance(v, str) and 0 < len(v) <= 256 for v in scene_context.values()),
             'scene_explicit_context_required')
    registration = dict(schemaVersion=SCHEMA, imageShape=p['imageShape'], points=copy.deepcopy(points),
        parameters=p, parametersSha256=_digest(p), referenceImageSha256=_sha(reference_image),
        referencePixelSha256=pixel_sha, sceneContext=copy.deepcopy(scene_context),
        seedSemantics='analyst_identified_projected_points_not_depth_contact_or_accuracy',
        trainingEligible=False, controlEligible=False)
    registration['registrationSha256'] = _digest(registration)
    # 참조에서도 특징이 유일하지 않으면 등록을 완료하지 않는다.
    RegisteredSceneLandmarks(registration, reference_image).observe(reference_image, 'registration-self-check')
    return registration


class RegisteredSceneLandmarks:
    def __init__(self, registration, reference_image):
        self.registration = copy.deepcopy(registration)
        self.reference_image = reference_image
        self.verify()

    @property
    def landmark_contract(self):
        return LANDMARK_PREFIX + self.registration['registrationSha256']

    def verify(self):
        r = self.registration
        _require(isinstance(r, dict) and set(r) == {
            'schemaVersion', 'imageShape', 'points', 'parameters', 'parametersSha256',
            'referenceImageSha256', 'referencePixelSha256', 'sceneContext', 'seedSemantics',
            'trainingEligible', 'controlEligible', 'registrationSha256'}
            and r['schemaVersion'] == SCHEMA and r['trainingEligible'] is False
            and r['controlEligible'] is False, 'scene_contract_fields')
        _require(_digest({k: v for k, v in r.items() if k != 'registrationSha256'}) == r['registrationSha256']
                 and _digest(r['parameters']) == r['parametersSha256']
                 and _sha(self.reference_image) == r['referenceImageSha256'], 'scene_contract_changed')
        _parameters(r['parameters'])
        _require(r['imageShape'] == r['parameters']['imageShape'], 'scene_shape_binding_changed')
        reference, pixel_sha = _image(self.reference_image, r['imageShape'])
        _require(pixel_sha == r['referencePixelSha256'], 'scene_reference_pixels_changed')
        _require(isinstance(r['sceneContext'], dict)
                 and set(r['sceneContext']) == {'physicalView', 'object', 'poseRegime'}
                 and all(isinstance(v, str) and 0 < len(v) <= 256 for v in r['sceneContext'].values()),
                 'scene_explicit_context_required')
        _seeds(r['points'], r['parameters'], reference)
        return True

    def observe(self, image_bytes, frame_id):
        _require(isinstance(frame_id, str) and 0 < len(frame_id) <= 160, 'scene_frame_id_required')
        self.verify()
        image, pixel_sha = _image(image_bytes, self.registration['imageShape'])
        reference, _ = _image(self.reference_image, self.registration['imageShape'])
        p = self.registration['parameters']
        points, quality, matches = {}, {}, {}
        for name in NAMES:
            x0, y0, x1, y1 = p['patchRois'][name]
            a, b, c, d = p['searchRois'][name]
            angles = p.get('rotationDegrees', {}).get(name, [0])
            scores = None
            winning_angles = None
            for angle_index, angle in enumerate(angles):
                template = _template(reference, self.registration['points'][name],
                                     p['patchRois'][name], angle)
                candidate = cv2.matchTemplate(image[b:d, a:c], template, cv2.TM_CCOEFF_NORMED)
                _require(np.isfinite(candidate).all(), 'scene_nonfinite_match')
                if scores is None:
                    scores = candidate
                    winning_angles = np.zeros(scores.shape, dtype=np.uint8)
                else:
                    better = candidate > scores
                    scores[better] = candidate[better]
                    winning_angles[better] = angle_index
            if 'jawPair' in p and name != 'object':
                matches[name] = (scores, winning_angles, angles)
            else:
                points[name], quality[name] = _localize(scores, winning_angles, angles, self.registration, name)
        if 'jawPair' in p:
            positions, pair_quality = _jaw_pair(matches, self.registration)
            try:
                for name in ('jawLeft', 'jawRight'):
                    points[name], quality[name] = _localize(*matches[name], self.registration, name,
                                                           position=positions[name], paired=True)
                span = float(np.linalg.norm(np.asarray(points['jawLeft'])-points['jawRight']))
                pair_quality['localizedSpanPx'] = span
                _pair_require(p['jawPair']['minimumSpanPx'] <= span <= p['jawPair']['maximumSpanPx'],
                              'scene_localized_jaw_pair_outside_band', pair_quality)
            except ValueError as exc:
                if not hasattr(exc, 'diagnostics'): exc.diagnostics = copy.deepcopy(pair_quality)
                raise
            quality['jawPair'] = pair_quality
        obj, left, right = (np.asarray(points[n], dtype=float) for n in NAMES)
        opening = float(np.linalg.norm(right-left))
        _require(opening > 0, 'scene_coincident_jaw_points')
        annotation = dict(frameId=frame_id, pixelSha256=pixel_sha, points=points, accepted=True,
            inputFrameOnly=True, method=METHOD, registrationSha256=self.registration['registrationSha256'],
            parametersSha256=self.registration['parametersSha256'],
            referenceImageSha256=self.registration['referenceImageSha256'],
            landmarkContract=self.landmark_contract)
        return dict(annotation=annotation, state=[*(obj-(left+right)/2).tolist(), opening],
            quality=quality, imageSha256=_sha(image_bytes), pixelSha256=pixel_sha,
            sceneContext=copy.deepcopy(self.registration['sceneContext']),
            trainingEligible=False, controlEligible=False, measurementAccuracyVerified=False,
            liveMotionTrackingVerified=False, depthVerified=False, contactVerified=False)
