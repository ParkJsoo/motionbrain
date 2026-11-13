"""집게 shaft의 독립 단면 관측. 턱 끝·접촉·물리 개구폭 관측이 아니다."""
from __future__ import annotations

import copy

import cv2
import numpy as np

from learning.registered_scene_landmarks import _digest, _image, _localize, _number, _require, _sha

SCHEMA = 'motionbrain.registered_gripper_crosssection.v1'
METHOD = 'registered_gripper_crosssection_points'
PREFIX = 'registered-gripper-crosssection-v1:'
LANDMARK_PREFIX = PREFIX
SHAFT_PREFIX = 'registered-gripper-shaft-only-v1:'
SHAFT_METHOD = 'registered_gripper_shaft_only_points'
SHAFT_STATE = ['shaftSeparationPx']
STATE = ['objectMinusJawMidXpx', 'objectMinusJawMidYpx', 'shaftSeparationPx']
SEMANTICS = 'object_top_center_and_independent_shaft_centerline_intersections_at_image_row55_not_tips_contact_or_physical_opening'


def default_parameters(*, preset=None):
    """기본 계약은 보존하고 현재 M1 닫기용 별도 등록에서만 오른쪽 ROI를 넓힌다."""
    _require(preset in (None, 'current_m1_close'), 'crosssection_unknown_parameter_preset')
    parameters = dict(imageShape=[240, 320], objectPatchRoi=[140, 67, 206, 133],
                objectSearchRoi=[116, 43, 230, 157], minimumNcc=.85,
                minimumPeakGap=.04, peakExclusionRadius=3, minimumTemplateStd=1.5,
                shaft=dict(rows=[45, 65], referenceRow=55,
                    rois=dict(jawLeft=[60, 120], jawRight=[205, 260]),
                    darkThreshold=100, runWidthPx=[18, 36], edgeProbePx=2,
                    minimumEdgeContrast=30., minimumRows=16, minimumRowSpan=16,
                    maximumCenterLineResidualPx=1.5, maximumWidthRangePx=8))
    if preset == 'current_m1_close':
        parameters['shaft']['rois']['jawRight'] = [190, 260]
    return parameters


def _parameters(p):
    expected = default_parameters()
    current_close = default_parameters(preset='current_m1_close')
    _require(isinstance(p, dict) and set(p) == set(expected), 'crosssection_parameter_fields')
    _require(p['imageShape'] == expected['imageShape'] and
             p['shaft'] in (expected['shaft'], current_close['shaft']),
             'crosssection_fixed_configuration_changed')
    _require(p['objectPatchRoi'] == expected['objectPatchRoi']
             and p['objectSearchRoi'] == expected['objectSearchRoi'], 'crosssection_object_roi_changed')
    _require(all(_number(p[k]) and p[k] >= expected[k] for k in
                 ('minimumNcc', 'minimumPeakGap', 'minimumTemplateStd'))
             and p['minimumNcc'] <= 1 and p['minimumPeakGap'] <= 1
             and type(p['peakExclusionRadius']) is int
             and p['peakExclusionRadius'] == expected['peakExclusionRadius'],
             'crosssection_quality_configuration_invalid')


def _context(context):
    _require(isinstance(context, dict) and set(context) == {'physicalView', 'object', 'poseRegime'}
             and all(isinstance(v, str) and 0 < len(v) <= 256 for v in context.values()),
             'crosssection_context_required')


def _fail(condition, reason, diagnostic):
    if not condition:
        error = ValueError(reason)
        error.diagnostics = copy.deepcopy(diagnostic)
        raise error


def _shaft(gray, name, p):
    a, b = p['rois'][name]; start, end = p['rows']; probe = p['edgeProbePx']
    diagnostic = dict(method='independent_dark_shaft_crosssection_v1', rows=[],
                      fixedSpanAssumption=False, symmetryAssumption=False)
    accepted = []
    for y in range(start, end+1):
        row = gray[y].astype(float); dark = row[a:b] < p['darkThreshold']
        boundaries = np.flatnonzero(np.diff(np.r_[False, dark, False].astype(int)))
        candidates = []
        for lo, hi in zip(boundaries[::2], boundaries[1::2]):
            left, right = int(a+lo), int(a+hi-1); width = right-left+1
            if not (p['runWidthPx'][0] <= width <= p['runWidthPx'][1]):
                continue
            if not (a+probe <= left and right < b-probe):
                continue
            contrast_left = float(row[left-probe]-row[left+probe])
            contrast_right = float(row[right+probe]-row[right-probe])
            # Each edge interpolates its measured threshold crossing independently.
            left_edge = left-1+(row[left-1]-p['darkThreshold'])/(row[left-1]-row[left])
            right_edge = right+(p['darkThreshold']-row[right])/(row[right+1]-row[right])
            candidates.append(dict(run=[left, right], widthPx=width,
                edgesPx=[float(left_edge), float(right_edge)],
                centerPx=float((left_edge+right_edge)/2),
                edgeContrast=[contrast_left, contrast_right],
                contrastPassed=min(contrast_left, contrast_right) >= p['minimumEdgeContrast']))
        item = dict(row=y, candidates=candidates, accepted=False)
        diagnostic['rows'].append(item)
        # A second plausible-width run is ambiguous even when its contrast is poor.
        _fail(len(candidates) <= 1, 'crosssection_ambiguous_run', diagnostic)
        if len(candidates) == 1 and candidates[0]['contrastPassed']:
            item.update(accepted=True, selected=candidates[0]); accepted.append((y, candidates[0]))
    _fail(len(accepted) >= p['minimumRows'], 'crosssection_rows_insufficient', diagnostic)
    ys = np.array([r[0] for r in accepted], float)
    centers = np.array([r[1]['centerPx'] for r in accepted])
    widths = [r[1]['widthPx'] for r in accepted]
    _fail(float(np.ptp(ys)) >= p['minimumRowSpan'], 'crosssection_row_span_insufficient', diagnostic)
    slope, intercept = np.polyfit(ys-p['referenceRow'], centers, 1)
    residual = float(np.max(np.abs(centers-(slope*(ys-p['referenceRow'])+intercept))))
    diagnostic.update(acceptedRows=len(accepted), slope=float(slope),
        maximumResidualPx=residual, widthRangePx=[min(widths), max(widths)],
        point=[float(intercept), p['referenceRow']])
    _fail(residual <= p['maximumCenterLineResidualPx']
          and max(widths)-min(widths) <= p['maximumWidthRangePx'],
          'crosssection_rows_inconsistent', diagnostic)
    return diagnostic['point'], diagnostic


def register(reference_image, object_point, scene_context, parameters=None):
    p = copy.deepcopy(default_parameters() if parameters is None else parameters)
    _parameters(p); _context(scene_context)
    image, pixel_sha = _image(reference_image, p['imageShape'])
    x0, y0, x1, y1 = p['objectPatchRoi']
    _require(isinstance(object_point, list) and len(object_point) == 2
             and all(_number(v) for v in object_point)
             and x0 <= object_point[0] < x1 and y0 <= object_point[1] < y1,
             'crosssection_object_seed_invalid')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    points = dict(object=copy.deepcopy(object_point))
    for name in ('jawLeft', 'jawRight'):
        points[name], _ = _shaft(gray, name, p['shaft'])
    registration = dict(schemaVersion=SCHEMA, imageShape=p['imageShape'], points=points,
        parameters=p, parametersSha256=_digest(p), referenceImageSha256=_sha(reference_image),
        referencePixelSha256=pixel_sha, sceneContext=copy.deepcopy(scene_context),
        seedSemantics=SEMANTICS, stateFields=STATE.copy(), trainingEligible=False, controlEligible=False)
    registration['registrationSha256'] = _digest(registration)
    RegisteredGripperCrosssection(registration, reference_image).observe(reference_image, 'registration-self-check')
    return registration


class RegisteredGripperCrosssection:
    def __init__(self, registration, reference_image):
        self.registration = copy.deepcopy(registration)
        self.reference_image = reference_image
        self.verify()

    @property
    def landmark_contract(self):
        return PREFIX+self.registration['registrationSha256']

    @property
    def shaft_landmark_contract(self):
        return SHAFT_PREFIX+self.registration['registrationSha256']

    def observe_shaft(self, raw, frame_id):
        """양 shaft만 측정한다. 물체 관측·대체 좌표·3점 상태를 만들지 않는다."""
        _require(isinstance(frame_id, str) and 0 < len(frame_id) <= 160, 'scene_frame_id_required')
        self.verify()
        image, pixel_sha = _image(raw, self.registration['imageShape'])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        points, quality = {}, {}
        try:
            for name in ('jawLeft', 'jawRight'):
                points[name], quality[name] = _shaft(gray, name, self.registration['parameters']['shaft'])
        except ValueError as exc:
            exc.diagnostics = dict(completedQuality=quality, failedJaw=name,
                                   failure=getattr(exc, 'diagnostics', {}))
            raise
        separation = float(points['jawRight'][0]-points['jawLeft'][0])
        _require(separation > 0, 'crosssection_jaw_order_changed')
        annotation = dict(frameId=frame_id, pixelSha256=pixel_sha, points=points, accepted=True,
            inputFrameOnly=True, method=SHAFT_METHOD,
            registrationSha256=self.registration['registrationSha256'],
            parametersSha256=self.registration['parametersSha256'],
            referenceImageSha256=self.registration['referenceImageSha256'],
            landmarkContract=self.shaft_landmark_contract)
        return dict(annotation=annotation, state=[separation], shaftSeparationPx=separation,
            stateFields=SHAFT_STATE.copy(),
            perPoint={name: dict(status='valid', point=points[name]) for name in points},
            objectObservation=dict(status='not_required', observation='not_observed'),
            quality=quality, imageSha256=_sha(raw), pixelSha256=pixel_sha,
            sceneContext=copy.deepcopy(self.registration['sceneContext']),
            trainingEligible=False, controlEligible=False, measurementAccuracyVerified=False,
            liveMotionTrackingVerified=False, depthVerified=False, contactVerified=False)

    def verify(self):
        r = self.registration
        _require(isinstance(r, dict) and set(r) == {
            'schemaVersion', 'imageShape', 'points', 'parameters', 'parametersSha256',
            'referenceImageSha256', 'referencePixelSha256', 'sceneContext', 'seedSemantics',
            'stateFields', 'trainingEligible', 'controlEligible', 'registrationSha256'}
            and r['schemaVersion'] == SCHEMA and r['stateFields'] == STATE
            and r['seedSemantics'] == SEMANTICS and r['trainingEligible'] is False
            and r['controlEligible'] is False, 'crosssection_contract_fields')
        _require(_digest({k:v for k,v in r.items() if k != 'registrationSha256'}) == r['registrationSha256']
             and _digest(r['parameters']) == r['parametersSha256']
             and _sha(self.reference_image) == r['referenceImageSha256'], 'crosssection_contract_changed')
        _parameters(r['parameters']); _context(r['sceneContext'])
        _require(r['imageShape'] == r['parameters']['imageShape'], 'crosssection_shape_binding_changed')
        image, pixel_sha = _image(self.reference_image, r['imageShape'])
        _require(pixel_sha == r['referencePixelSha256'], 'crosssection_reference_pixels_changed')
        points = r['points']; x0,y0,x1,y1 = r['parameters']['objectPatchRoi']
        _require(isinstance(points, dict) and set(points) == {'object','jawLeft','jawRight'}, 'crosssection_points_invalid')
        point = points['object']
        _require(isinstance(point, list) and len(point) == 2 and all(_number(v) for v in point)
             and x0 <= point[0] < x1 and y0 <= point[1] < y1, 'crosssection_object_seed_invalid')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _require(float(gray[y0:y1,x0:x1].std()) >= r['parameters']['minimumTemplateStd'],
                 'scene_template_texture_insufficient')
        for name in ('jawLeft', 'jawRight'):
            measured, _ = _shaft(gray, name, r['parameters']['shaft'])
            _require(points[name] == measured, 'crosssection_reference_shaft_changed')
        return True

    def observe(self, raw, frame_id):
        _require(isinstance(frame_id, str) and 0 < len(frame_id) <= 160, 'scene_frame_id_required')
        self.verify()
        image, pixel_sha = _image(raw, self.registration['imageShape'])
        reference, _ = _image(self.reference_image, self.registration['imageShape'])
        p = self.registration['parameters']; x0,y0,x1,y1 = p['objectPatchRoi']; a,b,c,d = p['objectSearchRoi']
        scores = cv2.matchTemplate(image[b:d,a:c], reference[y0:y1,x0:x1], cv2.TM_CCOEFF_NORMED)
        _require(np.isfinite(scores).all(), 'scene_nonfinite_match')
        adapter = dict(points=self.registration['points'], parameters=dict(
            patchRois=dict(object=p['objectPatchRoi']), searchRois=dict(object=p['objectSearchRoi']),
            minimumNcc=dict(object=p['minimumNcc']), minimumPeakGap=p['minimumPeakGap'],
            peakExclusionRadius=p['peakExclusionRadius']))
        obj, object_quality = _localize(scores, np.zeros(scores.shape, np.uint8), [0], adapter, 'object')
        points, quality = dict(object=obj), dict(object=object_quality)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        try:
            for name in ('jawLeft','jawRight'):
                points[name], quality[name] = _shaft(gray, name, p['shaft'])
        except ValueError as exc:
            exc.diagnostics = dict(completedQuality=quality, failedJaw=name,
                                   failure=getattr(exc, 'diagnostics', {}))
            raise
        left,right = np.array(points['jawLeft']),np.array(points['jawRight'])
        _require(left[0] < right[0], 'crosssection_jaw_order_changed')
        annotation = dict(frameId=frame_id, pixelSha256=pixel_sha, points=points, accepted=True,
            inputFrameOnly=True, method=METHOD, registrationSha256=self.registration['registrationSha256'],
            parametersSha256=self.registration['parametersSha256'],
            referenceImageSha256=self.registration['referenceImageSha256'], landmarkContract=self.landmark_contract)
        return dict(annotation=annotation, state=[*(np.array(obj)-(left+right)/2).tolist(), float(right[0]-left[0])],
            stateFields=STATE.copy(), quality=quality, imageSha256=_sha(raw), pixelSha256=pixel_sha,
            sceneContext=copy.deepcopy(self.registration['sceneContext']),
            trainingEligible=False, controlEligible=False, measurementAccuracyVerified=False,
            liveMotionTrackingVerified=False, depthVerified=False, contactVerified=False)
