"""One registered view's template landmarks; no device access or motor authority.

Analyst seed points are uncalibrated projected coordinates. Matching reports
translation of fixed appearance patches, not grasp depth, contact, or accuracy.
Rejected matches never yield accepted annotations. Parameters are registered
before action data collection and cannot be adapted by observe().
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import time

import cv2
import numpy as np

from learning.manual_collection_dataset import require
from learning import object_arc as arc

SCHEMA = 'motionbrain.registered_gripper_landmarks.v1'
METHOD = 'registered_template_points'
LANDMARK_PREFIX = 'registered-template-points-v1:'
NAMES = ('object', 'jawLeft', 'jawRight')


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _digest(value):
    return _sha(json.dumps(value, sort_keys=True, separators=(',', ':'),
                           allow_nan=False).encode())


def default_parameters(*, object_arc=False):
    """Predeclared for the 320x240 contact view, not a universal detector."""
    parameters = dict(imageShape=[240, 320],
        rois=dict(object=[139, 74, 179, 120], jawLeft=[106, 110, 133, 143],
                  jawRight=[180, 110, 207, 143]),
        searchRadii=dict(object=[4, 4], jawLeft=[24, 14], jawRight=[24, 14]),
        minimumNcc=dict(object=.85, jawLeft=.94, jawRight=.94),
        minimumPeakGap=.04, peakExclusionRadius=2, minimumTemplateStd=1.5,
        # Two predeclared OPEN pulses may move the jaw midpoint vertically by
        # about 8px (old, different-view data suggest ~4px per pulse). This is
        # only a bounded matching hypothesis, not a verified current response.
        maximumJawYDifferencePx=6., maximumJawMidpointDriftPx=12.,
        minimumOpeningPx=64., maximumOpeningPx=124.)
    if object_arc:
        parameters['objectArc'] = dict(method=arc.METHOD,
            parametersSha256=arc.parameter_sha(arc.PARAMETERS), parameters=copy.deepcopy(arc.PARAMETERS))
    return parameters


def _number(x):
    return type(x) in (int, float) and math.isfinite(x)


def _image(jpeg, shape):
    require(isinstance(jpeg, bytes) and 0 < len(jpeg) <= 2_000_000,
            'registered_image_bytes_required')
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape[:2]) == shape,
            'registered_image_shape_changed')
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), _sha(cv2.cvtColor(image, cv2.COLOR_BGR2BGRA).tobytes())


def _parameters(p):
    require(isinstance(p, dict) and set(p) in
            (set(default_parameters()), set(default_parameters()) | {'objectArc'}),
            'registered_parameter_fields')
    if 'objectArc' in p:
        require(p['objectArc'] == default_parameters(object_arc=True)['objectArc']
                and arc.parameter_sha(p['objectArc']['parameters']) == p['objectArc']['parametersSha256'],
                'registered_object_arc_contract_changed')
    require(isinstance(p['imageShape'], list) and len(p['imageShape']) == 2
            and all(type(v) is int and 16 <= v <= 1280 for v in p['imageShape']),
            'registered_image_shape_invalid')
    h, w = p['imageShape']
    require(all(isinstance(p[k], dict) and set(p[k]) == set(NAMES)
                for k in ('rois', 'searchRadii', 'minimumNcc')), 'registered_landmark_parameters')
    for name in NAMES:
        roi, radius = p['rois'][name], p['searchRadii'][name]
        require(isinstance(roi, list) and len(roi) == 4 and all(type(v) is int for v in roi),
                'registered_roi_invalid')
        require(isinstance(radius, list) and len(radius) == 2
                and all(type(v) is int and 1 <= v <= 32 for v in radius),
                'registered_search_invalid')
        x0, y0, x1, y1 = roi; dx, dy = radius
        require(0 <= x0-dx < x0 < x1 < x1+dx <= w
                and 0 <= y0-dy < y0 < y1 < y1+dy <= h
                and x1-x0 >= 8 and y1-y0 >= 8, 'registered_roi_or_search_clipped')
        require(_number(p['minimumNcc'][name]) and .8 <= p['minimumNcc'][name] <= 1.,
                'registered_ncc_threshold_invalid')
    require(type(p['peakExclusionRadius']) is int and 1 <= p['peakExclusionRadius'] <= 3
            and _number(p['minimumPeakGap']) and .01 <= p['minimumPeakGap'] <= .5
            and _number(p['minimumTemplateStd']) and 1 <= p['minimumTemplateStd'] <= 64,
            'registered_quality_parameters_invalid')
    require(all(_number(p[k]) and p[k] > 0 for k in
                ('maximumJawYDifferencePx', 'maximumJawMidpointDriftPx',
                 'minimumOpeningPx', 'maximumOpeningPx'))
            and p['maximumJawYDifferencePx'] <= 10
            and p['maximumJawMidpointDriftPx'] <= 14
            and p['minimumOpeningPx'] < p['maximumOpeningPx'] < w,
            'registered_geometry_parameters_invalid')


def register(reference_jpeg, points, parameters=None):
    p = copy.deepcopy(default_parameters() if parameters is None else parameters)
    _parameters(p)
    gray, pixel_sha = _image(reference_jpeg, p['imageShape'])
    require(isinstance(points, dict) and set(points) == set(NAMES), 'registered_three_seed_points_required')
    for name in NAMES:
        xy = points[name]; x0, y0, x1, y1 = p['rois'][name]
        require(isinstance(xy, list) and len(xy) == 2 and all(_number(v) for v in xy)
                and x0 <= xy[0] < x1 and y0 <= xy[1] < y1,
                'registered_seed_outside_patch')
        if name == 'object' and 'objectArc' in p:
            arc.extract_object_arc(reference_jpeg, p['objectArc']['parameters'])
        else:
            require(float(gray[y0:y1, x0:x1].std()) >= p['minimumTemplateStd'],
                    'registered_template_texture_insufficient')
    reg = dict(schemaVersion=SCHEMA, imageShape=p['imageShape'], points=copy.deepcopy(points),
        parameters=p, parametersSha256=_digest(p), referenceImageSha256=_sha(reference_jpeg),
        referencePixelSha256=pixel_sha,
        seedSemantics='analyst_projected_points_approx_three_px_not_measured_accuracy',
        trainingEligible=False, controlEligible=False)
    reg['registrationSha256'] = _digest(reg)
    RegisteredGripperLandmarks(reg, reference_jpeg).observe(reference_jpeg, 'registration-self-check')
    return reg


class RegisteredGripperLandmarks:
    def __init__(self, registration, reference_jpeg):
        self.registration = copy.deepcopy(registration)
        self.reference_jpeg = reference_jpeg
        self.verify()
        self.gray, _ = _image(reference_jpeg, self.registration['imageShape'])

    @property
    def landmark_contract(self):
        return LANDMARK_PREFIX + self.registration['registrationSha256']

    def verify(self):
        r = self.registration
        require(isinstance(r, dict) and set(r) == {'schemaVersion', 'imageShape', 'points',
            'parameters', 'parametersSha256', 'referenceImageSha256', 'referencePixelSha256',
            'seedSemantics', 'trainingEligible', 'controlEligible', 'registrationSha256'}
            and r['schemaVersion'] == SCHEMA and r['trainingEligible'] is False
            and r['controlEligible'] is False, 'registered_contract_fields')
        require(_digest({k: v for k, v in r.items() if k != 'registrationSha256'}) == r['registrationSha256']
                and _digest(r['parameters']) == r['parametersSha256']
                and _sha(self.reference_jpeg) == r['referenceImageSha256'], 'registered_contract_changed')
        _parameters(r['parameters'])
        require(r['imageShape'] == r['parameters']['imageShape'], 'registered_shape_binding_changed')
        gray, pixel_sha = _image(self.reference_jpeg, r['imageShape'])
        require(pixel_sha == r['referencePixelSha256'], 'registered_reference_pixels_changed')
        require(isinstance(r['points'], dict) and set(r['points']) == set(NAMES),
                'registered_three_seed_points_required')
        for name in NAMES:
            x0, y0, x1, y1 = r['parameters']['rois'][name]
            xy = r['points'][name]
            require(isinstance(xy, list) and len(xy) == 2 and all(_number(v) for v in xy)
                    and x0 <= xy[0] < x1 and y0 <= xy[1] < y1,
                    'registered_seed_outside_patch')
            if name == 'object' and 'objectArc' in r['parameters']:
                arc.extract_object_arc(self.reference_jpeg, r['parameters']['objectArc']['parameters'])
            else:
                require(float(gray[y0:y1, x0:x1].std()) >= r['parameters']['minimumTemplateStd'],
                        'registered_template_texture_insufficient')
        return True

    def observe(self, jpeg, frame_id):
        return self._observe(jpeg, frame_id, jaws_only=False)

    def observe_jaws(self, jpeg, frame_id):
        """등록된 양턱만 관측한다. 물체 누락을 가짜 점으로 채우지 않는다."""
        return self._observe(jpeg, frame_id, jaws_only=True)

    def _observe(self, jpeg, frame_id, *, jaws_only):
        started = time.monotonic()
        require(isinstance(frame_id, str) and 0 < len(frame_id) <= 160, 'registered_frame_id_required')
        self.verify()
        gray, pixel_sha = _image(jpeg, self.registration['imageShape'])
        p = self.registration['parameters']; points, quality, shifts = {}, {}, {}
        for name in (('jawLeft', 'jawRight') if jaws_only else NAMES):
            x0, y0, x1, y1 = p['rois'][name]; dx, dy = p['searchRadii'][name]
            if name == 'object' and 'objectArc' in p:
                reference_arc = arc.extract_object_arc(self.reference_jpeg, p['objectArc']['parameters'])
                current_arc = arc.extract_object_arc(jpeg, p['objectArc']['parameters'])
                shift = (np.asarray(current_arc['centerPx'])-reference_arc['centerPx']).tolist()
                require(abs(shift[0]) < dx and abs(shift[1]) < dy,
                        'registered_object_arc_outside_search')
                shifts[name] = shift
                points[name] = [a+b for a, b in zip(self.registration['points'][name], shift)]
                # No invented NCC/peak score for a different observation method.
                quality[name] = dict(method=arc.METHOD,
                    parametersSha256=p['objectArc']['parametersSha256'],
                    reference=reference_arc, current=current_arc, shiftPx=shift,
                    meaning='registered_seed_plus_arc_center_displacement')
                continue
            template = self.gray[y0:y1, x0:x1]
            search = gray[y0-dy:y1+dy, x0-dx:x1+dx]
            scores = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
            require(np.isfinite(scores).all(), 'registered_nonfinite_match')
            _, peak, _, location = cv2.minMaxLoc(scores)
            ix, iy = location
            require(0 < ix < scores.shape[1]-1 and 0 < iy < scores.shape[0]-1,
                    'registered_match_at_search_boundary')
            require(peak >= p['minimumNcc'][name], 'registered_ncc_below_threshold')
            other = scores.copy(); radius = p['peakExclusionRadius']
            other[max(0, iy-radius):iy+radius+1, max(0, ix-radius):ix+radius+1] = -2.
            second = float(other.max()); gap = peak-second
            require(gap >= p['minimumPeakGap'], 'registered_match_ambiguous')
            # Quadratic interpolation of measured NCC, never state clamping or
            # rounding to fit the action model's training range.
            offsets = []
            for before, center, after in ((scores[iy, ix-1], scores[iy, ix], scores[iy, ix+1]),
                                          (scores[iy-1, ix], scores[iy, ix], scores[iy+1, ix])):
                denominator = float(before-2*center+after)
                require(denominator < -1e-7, 'registered_peak_not_localized')
                offset = .5*float(before-after)/denominator
                require(math.isfinite(offset) and abs(offset) <= .75, 'registered_subpixel_peak_invalid')
                offsets.append(offset)
            shift = [ix-dx+offsets[0], iy-dy+offsets[1]]
            shifts[name] = shift
            points[name] = [a+b for a, b in zip(self.registration['points'][name], shift)]
            quality[name] = dict(ncc=float(peak), alternativeNcc=second, peakGap=float(gap), shiftPx=shift)
        if jaws_only:
            left, right = (np.asarray(points[n], dtype=float) for n in ('jawLeft','jawRight'))
            opening = float(np.linalg.norm(right-left))
            require(left[0] < right[0] and abs(right[1]-left[1]) <= p['maximumJawYDifferencePx']
                    and p['minimumOpeningPx'] <= opening <= p['maximumOpeningPx'],
                    'registered_landmark_geometry_changed')
            return dict(state=[*left.tolist(),*right.tolist(),opening], points=points,
                stateFields=['jawLeftXpx','jawLeftYpx','jawRightXpx','jawRightYpx','projectedJawSeparationPx'],
                frameId=frame_id,pixelSha256=pixel_sha,imageSha256=_sha(jpeg),quality=quality,
                landmarkContract='registered-jaw-points-v1:'+self.registration['registrationSha256'],
                measurementAccuracyVerified=False,liveMotionTrackingVerified=False,
                semantics='projected_jaw_corners_only_not_object_contact_depth_or_force')
        obj, left, right = (np.asarray(points[n], dtype=float) for n in NAMES)
        midpoint = (left+right)/2
        base = self.registration['points']
        base_mid = (np.asarray(base['jawLeft'])+base['jawRight'])/2
        midpoint_drift = np.linalg.norm((midpoint-base_mid)-shifts['object'])
        opening = float(np.linalg.norm(right-left))
        require(left[0] < obj[0] < right[0] and abs(right[1]-left[1]) <= p['maximumJawYDifferencePx']
                and midpoint_drift <= p['maximumJawMidpointDriftPx']
                and p['minimumOpeningPx'] <= opening <= p['maximumOpeningPx'],
                'registered_landmark_geometry_changed')
        annotation = dict(frameId=frame_id, pixelSha256=pixel_sha, points=points,
            accepted=True, inputFrameOnly=True, method=METHOD,
            registrationSha256=self.registration['registrationSha256'],
            parametersSha256=self.registration['parametersSha256'],
            referenceImageSha256=self.registration['referenceImageSha256'],
            landmarkContract=self.landmark_contract)
        return dict(annotation=annotation, state=[*(obj-midpoint).tolist(), opening],
            quality=quality, imageSha256=_sha(jpeg), pixelSha256=pixel_sha,
            elapsedSeconds=time.monotonic()-started,
            measurementAccuracyVerified=False, liveMotionTrackingVerified=False)
