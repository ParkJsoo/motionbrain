"""기준 영상 특징과 HSV를 결합하는 읽기 전용 배치 관측.

기존 visual_registration.register의 점 식별·대비·중복·왕복·12px 제한을
그대로 사용한다. 정지 영상의 관측 연결이며 움직임 추적 검증이 아니다.
"""
from __future__ import annotations

import base64
import binascii
import copy
import hashlib
from pathlib import Path
import struct

import cv2
import numpy as np

from learning import visual_registration
from learning.batch_observation import (_assemble_states, _observe_hsv_landmarks,
    _prepare_frame, _validate_common_spec, _validate_hsv_marker, digest, number, require)

SCHEMA = 'motionbrain.batch_registered_observer.v1'
EXTRA_FIELDS = {'referenceImageBase64', 'referenceImageSha256', 'referencePixelSha256',
                'registrationImplementationSha256', 'maximumReferenceShiftPx'}
SEMANTICS = 'registered_texture_and_declared_hsv_centroids_2d_not_actual_tips_depth_contact_or_joint_angles'
MAX_REFERENCE_BYTES = 8*1024*1024


def registration_implementation_sha256():
    return hashlib.sha256(Path(visual_registration.__file__).read_bytes()).hexdigest()


def _landmark(name, marker, shape, spec):
    require(isinstance(marker,dict) and marker.get('kind') in ('template','hsv'), 'registered_landmark_kind')
    if marker['kind'] == 'hsv':
        _validate_hsv_marker(name, {k:v for k,v in marker.items() if k != 'kind'},shape,spec)
        return
    require(set(marker) == {'kind','point','reference','uncertaintyPx','role'} and
            isinstance(name,str) and bool(name) and isinstance(marker['role'],str) and marker['role'],
            'registered_landmark_fields')
    point = marker['point']
    require(isinstance(point,list) and len(point) == 2 and all(number(v) for v in point) and
            33 <= point[0] < 1887 and 33 <= point[1] < 1047, 'registration_patch_margin_or_role')
    require(type(marker['reference']) is bool and
            marker['reference'] == (name in spec['fixedViewAnchors']), 'registered_reference_role')
    require(number(marker['uncertaintyPx']) and
            0 < marker['uncertaintyPx'] <= spec['maxLandmarkUncertaintyPx'], 'marker_uncertainty_required')


def _reference(spec):
    require(spec['imageShape'] == [1080,1920] and spec['maximumReferenceShiftPx'] == 12 and
            type(spec['maximumReferenceShiftPx']) is int, 'registered_fixed_geometry_required')
    require(spec['registrationImplementationSha256'] == registration_implementation_sha256(),
            'registration_implementation_changed')
    encoded = spec['referenceImageBase64']
    require(isinstance(encoded,str) and 0 < len(encoded) <= ((MAX_REFERENCE_BYTES+2)//3)*4,
            'registered_reference_size')
    try:
        raw = base64.b64decode(encoded,validate=True)
    except (binascii.Error,ValueError) as exc:
        raise ValueError('registered_reference_base64') from exc
    require(len(raw) <= MAX_REFERENCE_BYTES and
            hashlib.sha256(raw).hexdigest() == spec['referenceImageSha256'], 'registered_reference_image_changed')
    require(raw[:8] == b'\x89PNG\r\n\x1a\n' and len(raw) >= 33 and raw[12:16] == b'IHDR' and
            struct.unpack('>II',raw[16:24]) == (1920,1080), 'registered_reference_png_shape')
    image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_UNCHANGED)
    require(image is not None and image.shape == (1080,1920,4) and image.dtype == np.uint8 and
            hashlib.sha256(image.tobytes()).hexdigest() == spec['referencePixelSha256'],
            'registered_reference_pixels_changed')
    return image


def _validate_registered_spec(spec):
    checked = _validate_common_spec(spec,schema=SCHEMA,extra_fields=EXTRA_FIELDS,landmark_validator=_landmark)
    require(all(checked['landmarks'][name]['kind'] == 'template' and
                checked['landmarks'][name]['reference'] for name in checked['fixedViewAnchors']),
            'registered_background_templates_required')
    points=[tuple(marker['point']) for marker in checked['landmarks'].values() if marker['kind']=='template']
    require(len(points)==len(set(points)), 'duplicate_registration_point')
    return checked


def validate_spec(spec):
    checked = _validate_registered_spec(spec)
    _reference(checked)
    return checked


def observe_frame(image_bytes,frame_metadata,spec,context,now,clock_id):
    result = dict(status='rejected',reason=None,states={},nodeMatches=[],landmarks={},
        perLandmarkRejected={},perJointRejected={},frameBinding=copy.deepcopy(frame_metadata),
        semantics=SEMANTICS,controlEligible=False,physicalValidationEstablished=False,
        readonlyStationaryEligible=False,motionTrackingVerified=False,trainingEligible=False)
    try:
        # Common schema checks are shared with the existing color observer.
        spec = _validate_registered_spec(spec)
        reference = _reference(spec)
        image = _prepare_frame(image_bytes,frame_metadata,spec,context,now,clock_id)
        result['specSha256'] = digest(spec)
        current = cv2.cvtColor(image,cv2.COLOR_BGR2BGRA).tobytes()
        seed = reference.tobytes()
        common = set(spec['poseLandmarks']) | set(spec['fixedViewAnchors'])
        templates = {name:marker for name,marker in spec['landmarks'].items() if marker['kind'] == 'template'}
        groups = [[name for name in templates if name in common]]
        groups += [[name] for name in templates if name not in common]
        for names in groups:
            if not names:
                continue
            points = [dict(id=name,x=templates[name]['point'][0],y=templates[name]['point'][1],
                           reference=templates[name]['reference']) for name in names]
            try:
                features,evidence = visual_registration.register(seed,current,points)
                require(len(features) == len(names) and {f['id'] for f in features} == set(names),
                        'registered_point_identity_changed')
                details = {record['id']:record for record in evidence}
                for feature in features:
                    name = feature['id']; marker = templates[name]
                    point = [feature['x'],feature['y']]
                    require(all(number(v) for v in point) and feature['visible'] is True and
                            np.linalg.norm(np.asarray(point)-np.asarray(marker['point'])) <= 12,
                            'registration_shift')
                    result['landmarks'][name] = dict(point=point,uncertaintyPx=marker['uncertaintyPx'],
                        role=marker['role'],registrationEvidence=copy.deepcopy(details[name]),
                        uncertaintyMeaning='declared_pixel_bound_not_calibrated_probability')
            except ValueError as exc:
                for name in names:
                    result['perLandmarkRejected'][name] = str(exc)
                if common.intersection(names):
                    raise
        hsv_spec = dict(spec,landmarks={name:{k:v for k,v in marker.items() if k != 'kind'}
            for name,marker in spec['landmarks'].items() if marker['kind'] == 'hsv'})
        _observe_hsv_landmarks(image,hsv_spec,result)
        _assemble_states(spec,result)
        result['readonlyStationaryEligible'] = result['status'] == 'accepted'
    except (ValueError,TypeError,KeyError,OverflowError,cv2.error) as exc:
        result.update(status='rejected',reason=str(exc),states={},nodeMatches=[],readonlyStationaryEligible=False)
    return result
