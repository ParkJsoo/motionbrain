"""새 외부 구도의 명시적 특징점 등록·평행이동 관측. 장치·파일 접근 없음.

점의 물리적 정체성은 분석가가 확인한다. 색 영상의 고정 패치 일치는
2D 이동만 제안하며 깊이·접촉·정확도·옛 모델의 전이 적격을 판단하지 않는다.
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
    _require(isinstance(p, dict) and set(p) == {
        'imageShape', 'patchRois', 'searchRois', 'minimumNcc', 'minimumPeakGap',
        'peakExclusionRadius', 'minimumTemplateStd'}, 'scene_parameter_fields')
    shape = p['imageShape']
    _require(isinstance(shape, list) and len(shape) == 2
             and all(type(v) is int for v in shape)
             and 32 <= shape[0] <= 1080 and 32 <= shape[1] <= 1920, 'scene_image_shape_invalid')
    _require(all(isinstance(p[k], dict) and set(p[k]) == set(NAMES)
                 for k in ('patchRois', 'searchRois', 'minimumNcc')), 'scene_three_patch_parameters_required')
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
        points, quality = {}, {}
        for name in NAMES:
            x0, y0, x1, y1 = p['patchRois'][name]
            a, b, c, d = p['searchRois'][name]
            template = reference[y0:y1, x0:x1]
            scores = cv2.matchTemplate(image[b:d, a:c], template, cv2.TM_CCOEFF_NORMED)
            _require(np.isfinite(scores).all(), 'scene_nonfinite_match')
            _, peak, _, (ix, iy) = cv2.minMaxLoc(scores)
            _require(0 < ix < scores.shape[1]-1 and 0 < iy < scores.shape[0]-1,
                     'scene_match_at_search_boundary')
            _require(peak >= p['minimumNcc'][name], 'scene_ncc_below_threshold')
            alternative = scores.copy()
            radius = p['peakExclusionRadius']
            alternative[max(0, iy-radius):iy+radius+1, max(0, ix-radius):ix+radius+1] = -2.
            _require(np.any(alternative != -2.), 'scene_search_insufficient_for_uniqueness')
            second = float(alternative.max())
            _require(peak-second >= p['minimumPeakGap'], 'scene_match_ambiguous')
            offsets = []
            for before, center, after in ((scores[iy, ix-1], scores[iy, ix], scores[iy, ix+1]),
                                          (scores[iy-1, ix], scores[iy, ix], scores[iy+1, ix])):
                denominator = float(before-2*center+after)
                _require(denominator < -1e-7, 'scene_peak_not_localized')
                offset = .5*float(before-after)/denominator
                _require(_number(offset) and abs(offset) <= .75, 'scene_subpixel_peak_invalid')
                offsets.append(offset)
            shift = [a+ix-x0+offsets[0], b+iy-y0+offsets[1]]
            points[name] = [v+s for v, s in zip(self.registration['points'][name], shift)]
            quality[name] = dict(ncc=float(peak), alternativeNcc=second, peakGap=float(peak-second),
                                 shiftPx=shift, matching='fixed_bgr_patch_translation_only')
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
