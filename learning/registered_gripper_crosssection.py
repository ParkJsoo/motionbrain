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
UNVERIFIED_OBJECT_SEED = 'unverified_seed_only_shaft_registration'
STATE = ['objectMinusJawMidXpx', 'objectMinusJawMidYpx', 'shaftSeparationPx']
SEMANTICS = 'object_top_center_and_independent_shaft_centerline_intersections_at_image_row55_not_tips_contact_or_physical_opening'
OBSERVED95_SEMANTICS = ('object_top_center_and_independent_shaft_unique_direct_observed_image_row95_centers'
                        '_with_local_straight_support_not_global_uniform_width_not_tips_contact_or_physical_opening')
DISTAL_SEMANTICS = ('object_top_center_and_independent_distal_shaft_line_fit_rows70_90_extrapolated_to_image_row55'
                    '_not_direct_row55_pixels_not_tips_contact_or_physical_opening')


def default_parameters(*, preset=None):
    """기본 계약을 보존하고 현재 M1 닫기·직선부 관측을 별도 preset으로 등록한다."""
    _require(preset in (None, 'current_m1_close', 'current_m1_distal_shaft', 'current_m1_observed_shaft95'), 'crosssection_unknown_parameter_preset')
    parameters = dict(imageShape=[240, 320], objectPatchRoi=[140, 67, 206, 133],
                objectSearchRoi=[116, 43, 230, 157], minimumNcc=.85,
                minimumPeakGap=.04, peakExclusionRadius=3, minimumTemplateStd=1.5,
                shaft=dict(rows=[45, 65], referenceRow=55,
                    rois=dict(jawLeft=[60, 120], jawRight=[205, 260]),
                    darkThreshold=100, runWidthPx=[18, 36], edgeProbePx=2,
                    minimumEdgeContrast=30., minimumRows=16, minimumRowSpan=16,
                    maximumCenterLineResidualPx=1.5, maximumWidthRangePx=8))
    if preset in ('current_m1_close', 'current_m1_distal_shaft'):
        parameters['shaft']['rois']['jawRight'] = [190, 260]
    if preset == 'current_m1_distal_shaft':
        parameters['shaft']['rows'] = [70, 90]
        parameters['shaft']['referenceRow'] = 55
    if preset == 'current_m1_observed_shaft95':
        parameters['shaft'].update(rows=[70, 115], referenceRow=95,
            rois=dict(jawLeft=[60, 150], jawRight=[165, 260]))
    return parameters


def _parameters(p):
    expected = default_parameters()
    current_close = default_parameters(preset='current_m1_close')
    current_distal = default_parameters(preset='current_m1_distal_shaft')
    observed95 = default_parameters(preset='current_m1_observed_shaft95')
    _require(isinstance(p, dict) and set(p) == set(expected), 'crosssection_parameter_fields')
    _require(p['imageShape'] == expected['imageShape'] and
             p['shaft'] in (expected['shaft'], current_close['shaft'], current_distal['shaft'], observed95['shaft']),
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


def _seed_semantics(parameters):
    shaft = parameters.get('shaft') if isinstance(parameters, dict) else None
    if isinstance(shaft, dict) and shaft.get('rows') == [70, 115] and shaft.get('referenceRow') == 95:
        return OBSERVED95_SEMANTICS
    if isinstance(shaft, dict) and shaft.get('rows') == [70, 90] and shaft.get('referenceRow') == 55:
        return DISTAL_SEMANTICS
    return SEMANTICS


def _shaft(gray, name, p):
    a, b = p['rois'][name]; start, end = p['rows']; probe = p['edgeProbePx']
    diagnostic = dict(method='independent_dark_shaft_crosssection_v1', rows=[],
                      fixedSpanAssumption=False, symmetryAssumption=False)
    if p['rows'] == [70, 90] and p['referenceRow'] == 55:
        diagnostic.update(method='independent_distal_shaft_line_extrapolation_v1',
            coordinateMeaning='rows70_90_line_fit_extrapolated_to_unobserved_row55',
            observedRowRange=[70, 90], reportedReferenceRow=55,
            extrapolationBeyondNearestObservedRowPx=15, distanceFromWindowCenterPx=25,
            projectionUncertaintyCalibrated=False, measurementAccuracyVerified=False)
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
    if p['rows'] == [70, 115] and p['referenceRow'] == 95:
        return _observed_row95(accepted, p, diagnostic)
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
    if p['rows'] == [70, 90] and p['referenceRow'] == 55:
        diagnostic.update(acceptedRowRange=[int(ys.min()), int(ys.max())],
            extrapolationBeyondNearestObservedRowPx=float(np.min(np.abs(ys-p['referenceRow']))))
    _fail(residual <= p['maximumCenterLineResidualPx']
          and max(widths)-min(widths) <= p['maximumWidthRangePx'],
          'crosssection_rows_inconsistent', diagnostic)
    return diagnostic['point'], diagnostic


def _observed_row95(accepted, p, diagnostic):
    """실측 row95 중심은 고정하며 주변 직선창은 품질 존재 증거로만 사용한다.

    같은 행의 같은 run을 공유하는 복수창은 별도 측정 가설이 아니다.
    내부 유효행을 임의 제거하지 않으며 전체 scan의 폭 균일성을 주장하지 않는다.
    """
    primary = next((candidate for y, candidate in accepted if y == 95), None)
    _fail(primary is not None, 'crosssection_reference_row_not_observed', diagnostic)
    _fail(len(accepted) >= p['minimumRows'], 'crosssection_rows_insufficient', diagnostic)
    witnesses = []
    for i in range(len(accepted)):
        for j in range(i+p['minimumRows']-1, len(accepted)):
            group = accepted[i:j+1]
            ys = np.array([row[0] for row in group], float)
            if not ys[0] <= 95 <= ys[-1] or float(np.ptp(ys)) < p['minimumRowSpan']:
                continue
            widths = [row[1]['widthPx'] for row in group]
            if max(widths)-min(widths) > p['maximumWidthRangePx']:
                continue
            centers = np.array([row[1]['centerPx'] for row in group])
            slope, intercept = np.polyfit(ys-95, centers, 1)
            residual = float(np.max(np.abs(centers-(slope*(ys-95)+intercept))))
            if residual > p['maximumCenterLineResidualPx']:
                continue
            witnesses.append(dict(acceptedRows=len(group), rowRange=[int(ys[0]), int(ys[-1])],
                rowSpan=float(np.ptp(ys)), slope=float(slope), fittedPoint=[float(intercept), 95],
                maximumResidualPx=residual, widthRangePx=[min(widths), max(widths)]))
    diagnostic.update(method='independent_direct_observed_row95_with_local_support_v1',
        coordinateMeaning='unique_threshold_edge_center_directly_observed_at_image_row95',
        point=[primary['centerPx'], 95], primaryRun=copy.deepcopy(primary),
        observedRowRange=[70, 115], reportedReferenceRow=95,
        qualityWitnesses=witnesses, qualityWitnessCount=len(witnesses),
        localStraightSupportOnly=True, globalWidthUniformityRequired=False,
        sameRunWitnessesAreIndependentObservations=False,
        extrapolationUsed=False, measurementAccuracyVerified=False)
    _fail(bool(witnesses), 'crosssection_local_support_insufficient', diagnostic)
    return diagnostic['point'], diagnostic


def register(reference_image, object_point, scene_context, parameters=None):
    return _register(reference_image, object_point, scene_context, parameters, shaft_only=False)


def register_shaft_only(reference_image, object_point, scene_context, parameters=None):
    """물체 점은 미검증 시드로 보관하고 실제 양 shaft 관측만 등록한다.

    기존 물체 ROI·텍스처·원본 해시·shaft 품질 검사는 유지한다. 물체 NCC
    자기검사를 생략한 등록은 전체 물체 관측에 사용할 수 없다.
    """
    return _register(reference_image, object_point, scene_context, parameters, shaft_only=True)


def _register(reference_image, object_point, scene_context, parameters, *, shaft_only):
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
        seedSemantics=_seed_semantics(p), stateFields=STATE.copy(), trainingEligible=False, controlEligible=False)
    if shaft_only:
        registration['objectSeedStatus'] = UNVERIFIED_OBJECT_SEED
    registration['registrationSha256'] = _digest(registration)
    tracker = RegisteredGripperCrosssection(registration, reference_image)
    if shaft_only:
        tracker.observe_shaft(reference_image, 'registration-shaft-only-self-check')
    else:
        tracker.observe(reference_image, 'registration-self-check')
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
        fields = {'schemaVersion', 'imageShape', 'points', 'parameters', 'parametersSha256',
            'referenceImageSha256', 'referencePixelSha256', 'sceneContext', 'seedSemantics',
            'stateFields', 'trainingEligible', 'controlEligible', 'registrationSha256'}
        if isinstance(r, dict) and 'objectSeedStatus' in r:
            fields.add('objectSeedStatus')
            _require(r['objectSeedStatus'] == UNVERIFIED_OBJECT_SEED, 'crosssection_object_seed_status_invalid')
        _require(isinstance(r, dict) and set(r) == fields
            and r['schemaVersion'] == SCHEMA and r['stateFields'] == STATE
            and r['seedSemantics'] == _seed_semantics(r['parameters']) and r['trainingEligible'] is False
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
        _require('objectSeedStatus' not in self.registration,
                 'crosssection_full_observation_requires_object_registration')
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
