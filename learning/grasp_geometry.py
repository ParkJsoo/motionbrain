"""명시적 수동 주석의 2D 집기 기하 진단. 장치·접촉·깊이 추정 없음."""
from __future__ import annotations

import math
import re

METHOD = 'analyst_manual_actual_tips'
SCHEMA = 'motionbrain.grasp_geometry.v1'
_SHA = re.compile(r'[0-9a-f]{64}\Z')


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _binding(value, frame):
    return (isinstance(value, dict) and value.get('frameId') == frame['frameId']
            and value.get('pixelSha256') == frame['pixelSha256'])


def _uncertainty(item):
    value = item.get('uncertaintyPx')
    _require(_finite(value) and value >= 0, 'explicit_nonnegative_pixel_uncertainty_required')
    return float(value)


def evaluate_geometry(review):
    """bodyBbox는 [x0,y0,x1,y1], point는 [x,y]인 수동 관측을 검사한다.

    uncertaintyPx는 분석가가 선언한 영상 불확실성이며 검증된 정확도가 아니다.
    onsite 확인은 같은 frameId와 pixelSha256에 결합되어야 한다. 반환값은
    명령 권한이나 성공 판정이 아니며, 현장 확인의 존재만 표시한다.
    """
    _require(isinstance(review, dict), 'geometry_review_required')
    frame = review.get('frame')
    _require(isinstance(frame, dict) and set(frame) == {'frameId','pixelSha256','width','height'},
             'explicit_geometry_frame_required')
    _require(isinstance(frame['frameId'], str) and 0 < len(frame['frameId']) <= 160
             and isinstance(frame['pixelSha256'], str) and bool(_SHA.fullmatch(frame['pixelSha256']))
             and all(type(frame[k]) is int and frame[k] > 0 for k in ('width','height')),
             'geometry_frame_invalid')
    annotation = review.get('annotation')
    _require(_binding(annotation, frame), 'geometry_annotation_frame_mismatch')
    _require(annotation.get('method') == METHOD, 'manual_actual_tip_annotation_required')
    obj, tips = annotation.get('object'), annotation.get('tips')
    _require(isinstance(obj, dict) and isinstance(tips, dict)
             and set(tips) == {'jawLeft','jawRight'}, 'body_and_two_actual_tips_required')
    w,h = frame['width'],frame['height']
    _require(type(obj.get('visible')) is bool, 'explicit_body_visibility_required')
    visibility = dict(object=obj['visible'])
    uncertainty = dict(object=_uncertainty(obj))
    bbox = obj.get('bodyBbox')
    if bbox is not None:
        _require(isinstance(bbox, list) and len(bbox) == 4 and all(_finite(v) for v in bbox)
                 and 0 <= bbox[0] < bbox[2] <= w and 0 <= bbox[1] < bbox[3] <= h,
                 'body_bbox_invalid')
    _require(not obj['visible'] or bbox is not None, 'visible_body_bbox_required')
    points = {}
    for name in ('jawLeft','jawRight'):
        item = tips[name]
        _require(isinstance(item, dict) and item.get('kind') == 'actual_tip',
                 'shaft_or_inferred_point_cannot_replace_actual_tip')
        _require(type(item.get('visible')) is bool, 'explicit_tip_visibility_required')
        visibility[name] = item['visible'];uncertainty[name] = _uncertainty(item)
        point = item.get('point')
        if point is not None:
            _require(isinstance(point, list) and len(point) == 2 and all(_finite(v) for v in point)
                     and 0 <= point[0] < w and 0 <= point[1] < h, 'actual_tip_point_invalid')
        _require(not item['visible'] or point is not None, 'visible_actual_tip_point_required')
        if item['visible']:
            points[name] = list(map(float, point))
    diagnostic = {}
    if len(points) == 2:
        left,right = points['jawLeft'],points['jawRight']
        dx,dy = right[0]-left[0],right[1]-left[1]
        diagnostic.update(tipDeltaPx=[dx,dy],tipDistancePx=math.hypot(dx,dy),
                          tipMidpointPx=[(left[0]+right[0])/2,(left[1]+right[1])/2])
        if obj['visible']:
            cx,cy=(bbox[0]+bbox[2])/2,(bbox[1]+bbox[3])/2
            diagnostic.update(bodyCenterMinusTipMidpointPx=[cx-diagnostic['tipMidpointPx'][0],cy-diagnostic['tipMidpointPx'][1]],
                tipHeightWithinBodyProjection={name:bool(bbox[1] <= p[1] <= bbox[3]) for name,p in points.items()},
                bodyCenterXWithinTipXProjection=bool(min(left[0],right[0]) <= cx <= max(left[0],right[0])))
    onsite = review.get('onsite')
    bound = _binding(onsite, frame)
    confirmed = (all(visibility.values()) and bound
                 and all(onsite.get(k) is True for k in ('objectBetweenJaws','depth','clearance')))
    return dict(schemaVersion=SCHEMA,frame=dict(frame),annotationMethod=METHOD,
        status='complete' if all(visibility.values()) else 'partial',visibility=visibility,
        declaredUncertaintyPx=uncertainty,uncertaintyAccuracyVerified=False,
        diagnostic=diagnostic,diagnosticMeaning='image_projection_only_not_depth_contact_clearance_or_grasp',
        onsiteFrameBound=bound,closeEntry='onsite_confirmed' if confirmed else 'needsreview',
        depthInferred=False,contactInferred=False,graspConfirmed=False,controlEligible=False)
