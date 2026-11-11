"""고정 구도의 명시적 색 표식을 관측한다. 장치·모터 접근은 없다.

표식 중심의 2D 위치와 선언한 픽셀 불확실성만 반환한다. 노드 일치는
영상 특징 구간과의 호환성이며 실제 관절각·깊이·접촉·안전 보증이 아니다.
실제 표식 설치, 가림 범위, 불확실성의 현장 검증은 별도로 필요하다.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math

import cv2
import numpy as np

SCHEMA = 'motionbrain.batch_color_observer.v1'
JOINTS = {'gripper', 'wrist', 'elbow', 'shoulder', 'base'}
CONTEXT = ('viewId', 'poseRegime', 'object', 'load')
SEMANTICS = 'declared_color_marker_centroids_2d_not_depth_contact_joint_angles_or_pose_certainty'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _validate_hsv_marker(name, marker, shape, spec):
    require(isinstance(name, str) and bool(name) and isinstance(marker, dict) and
            set(marker) == {'hsvLower', 'hsvUpper', 'minAreaPx', 'maxAreaPx',
                            'uncertaintyPx', 'role'}, 'marker_fields')
    require(isinstance(marker['role'], str) and bool(marker['role']), 'marker_role_required')
    lower, upper = marker['hsvLower'], marker['hsvUpper']
    require(all(isinstance(v, list) and len(v) == 3 for v in (lower, upper)) and
            all(type(lo) is int and type(hi) is int and 0 <= lo <= hi <= limit
                for lo, hi, limit in zip(lower, upper, (179, 255, 255))), 'marker_hsv_range')
    require(type(marker['minAreaPx']) is int and type(marker['maxAreaPx']) is int and
            1 <= marker['minAreaPx'] <= marker['maxAreaPx'] <= shape[0]*shape[1],
            'marker_area_range')
    require(number(marker['uncertaintyPx']) and
            0 < marker['uncertaintyPx'] <= spec['maxLandmarkUncertaintyPx'],
            'marker_uncertainty_required')


def _validate_common_spec(spec, *, schema, extra_fields, landmark_validator):
    """설정 전체를 장치 없이 검사한다. 알 수 없는 필드는 거절한다."""
    fields = {'schemaVersion', 'cameraIdentity', 'imageShape', *CONTEXT,
              'maxAgeSeconds', 'maxLandmarkUncertaintyPx', 'landmarks',
              'poseLandmarks', 'fixedViewAnchors', 'joints'} | set(extra_fields)
    require(isinstance(spec, dict) and set(spec) == fields, 'observer_spec_fields')
    require(spec['schemaVersion'] == schema, 'observer_schema')
    require(all(isinstance(spec[k], str) and spec[k] for k in (*CONTEXT, 'cameraIdentity')),
            'observer_context_required')
    shape = spec['imageShape']
    require(isinstance(shape, list) and len(shape) == 2 and
            all(type(v) is int and 1 <= v <= 8192 for v in shape), 'observer_image_shape')
    require(number(spec['maxAgeSeconds']) and 0 < spec['maxAgeSeconds'] <= 60,
            'observer_freshness_limit')
    require(number(spec['maxLandmarkUncertaintyPx']) and spec['maxLandmarkUncertaintyPx'] > 0,
            'observer_uncertainty_limit')
    landmarks = spec['landmarks']
    require(isinstance(landmarks, dict) and bool(landmarks), 'observer_landmarks_required')
    for name, marker in landmarks.items():
        landmark_validator(name, marker, shape, spec)
    pose = spec['poseLandmarks']
    require(isinstance(pose, list) and len(pose) >= 3 and len(set(pose)) == len(pose) and
            all(name in landmarks for name in pose), 'whole_pose_landmarks_required')
    anchors = spec['fixedViewAnchors']
    require(isinstance(anchors, dict) and len(anchors) >= 2, 'fixed_view_anchors_required')
    for name, bounds in anchors.items():
        require(name in landmarks and landmarks[name]['role'] == 'fixed_view_anchor',
                'fixed_view_anchor_role_required')
        require(isinstance(bounds, list) and len(bounds) == 4 and all(number(v) for v in bounds) and
                0 <= bounds[0] <= bounds[1] < shape[1] and
                0 <= bounds[2] <= bounds[3] < shape[0], 'fixed_view_anchor_bounds')
    require(not set(anchors) & set(pose), 'fixed_anchors_not_moving_pose_markers')
    joints = spec['joints']
    require(isinstance(joints, dict) and bool(joints) and set(joints) <= JOINTS,
            'observer_joints_required')
    node_ids = set()
    for joint_spec in joints.values():
        require(isinstance(joint_spec, dict) and set(joint_spec) == {'stateFields', 'nodes'},
                'joint_observer_fields')
        features = joint_spec['stateFields']
        require(isinstance(features, list) and bool(features), 'state_fields_required')
        names, used = set(), set()
        for feature in features:
            require(isinstance(feature, dict) and set(feature) == {'name', 'kind', 'a', 'b'},
                    'state_feature_fields')
            require(isinstance(feature['name'], str) and bool(feature['name']) and
                    feature['name'] not in names, 'unique_state_field_required')
            require(feature['kind'] in ('relative_x', 'relative_y', 'distance') and
                    feature['a'] in landmarks and feature['b'] in landmarks and
                    feature['a'] != feature['b'], 'state_feature_landmarks')
            names.add(feature['name']); used.update((feature['a'], feature['b']))
        require(set(pose) <= used, 'whole_pose_features_required_for_each_joint')
        # Each pose marker must have both coordinate dimensions represented.
        # Merely listing a marker in one distance feature leaves reflection/rotation aliases.
        for name in pose:
            axes = {f['kind'] for f in features if name in (f['a'], f['b'])}
            require({'relative_x', 'relative_y'} <= axes, 'whole_pose_xy_features_required')
        nodes = joint_spec['nodes']
        require(isinstance(nodes, list), 'node_list_required')
        for node in nodes:
            require(isinstance(node, dict) and set(node) == {'id', 'bounds'}, 'node_fields')
            require(isinstance(node['id'], str) and bool(node['id']) and
                    node['id'] not in node_ids, 'campaign_unique_node_id_required')
            node_ids.add(node['id'])
            require(isinstance(node['bounds'], dict) and set(node['bounds']) == names,
                    'node_bounds_all_features_required')
            for bounds in node['bounds'].values():
                require(isinstance(bounds, list) and len(bounds) == 2 and
                        all(number(x) for x in bounds) and bounds[0] <= bounds[1],
                        'node_bounds_invalid')
    return copy.deepcopy(spec)



def validate_spec(spec):
    if isinstance(spec, dict) and spec.get('schemaVersion') == 'motionbrain.batch_background_registered_observer.v1':
        from learning import batch_background_observation
        return batch_background_observation.validate_spec(spec)
    if isinstance(spec, dict) and spec.get('schemaVersion') == 'motionbrain.batch_registered_observer.v1':
        from learning.batch_template_observation import validate_spec as validate_registered
        return validate_registered(spec)
    return _validate_common_spec(spec, schema=SCHEMA, extra_fields=(), landmark_validator=_validate_hsv_marker)


def _prepare_frame(image_bytes, frame_metadata, spec, context, now, clock_id):
    require(isinstance(context, dict) and set(context) == set(CONTEXT) and
            all(context[k] == spec[k] for k in CONTEXT), 'observer_context_changed')
    require(number(now) and isinstance(clock_id, str) and bool(clock_id), 'observer_clock_required')
    frame = frame_metadata
    require(isinstance(frame, dict) and set(frame) == {'frameId', 'imageSha256',
            'pixelSha256', 'capturedAt', 'persistedAt', 'clockId', 'cameraIdentity'},
            'frame_binding_fields')
    require(isinstance(frame['frameId'], str) and bool(frame['frameId']), 'frame_id_required')
    require(frame['clockId'] == clock_id, 'frame_clock_mismatch')
    require(frame['cameraIdentity'] == spec['cameraIdentity'], 'frame_camera_changed')
    require(all(number(frame[k]) for k in ('capturedAt', 'persistedAt')) and
            0 <= frame['capturedAt'] <= frame['persistedAt'] <= now and
            now-frame['capturedAt'] <= spec['maxAgeSeconds'], 'frame_stale_or_noncausal')
    require(isinstance(image_bytes, bytes) and bool(image_bytes), 'frame_bytes_required')
    require(hashlib.sha256(image_bytes).hexdigest() == frame['imageSha256'], 'frame_image_hash_changed')
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape[:2]) == spec['imageShape'], 'frame_shape_changed')
    require(hashlib.sha256(image.tobytes()).hexdigest() == frame['pixelSha256'], 'frame_pixel_hash_changed')
    return image


def _observe_hsv_landmarks(image, spec, result):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = {}
    common = set(spec['poseLandmarks']) | set(spec['fixedViewAnchors'])
    for name, marker in spec['landmarks'].items():
        overlaps = []
        try:
            mask = cv2.inRange(hsv, np.asarray(marker['hsvLower'], np.uint8),
                               np.asarray(marker['hsvUpper'], np.uint8))
            count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
            candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= marker['minAreaPx']]
            require(len(candidates) == 1, 'marker_missing_or_ambiguous:'+name)
            index = candidates[0]
            require(stats[index, cv2.CC_STAT_AREA] <= marker['maxAreaPx'], 'marker_area_exceeded:'+name)
            x, y, width, height = stats[index, :4]
            require(x > 0 and y > 0 and x+width < image.shape[1] and y+height < image.shape[0],
                    'marker_clipped:'+name)
            pixels = labels == index
            overlaps = [other for other, used in masks.items() if np.any(used & pixels)]
            masks[name] = pixels
            if overlaps:
                reason = 'marker_color_identity_overlap:'+name
                require(not (common & set([name, *overlaps])), reason)
                for other in overlaps:
                    result['landmarks'].pop(other, None)
                    result['perLandmarkRejected'][other] = reason
                raise ValueError(reason)
            result['landmarks'][name] = dict(point=centers[index].tolist(),
                areaPx=int(stats[index, cv2.CC_STAT_AREA]), uncertaintyPx=marker['uncertaintyPx'],
                role=marker['role'], uncertaintyMeaning='declared_pixel_bound_not_calibrated_probability')
        except ValueError as exc:
            result['perLandmarkRejected'][name] = str(exc)
            if name in common or common.intersection(overlaps):
                raise


def _assemble_states(spec, result):
    common = set(spec['poseLandmarks']) | set(spec['fixedViewAnchors'])
    require(common <= set(result['landmarks']), 'common_pose_landmarks_unavailable')
    # Absolute background registration rejects displacement that relative
    # features alone would cancel. It is not a calibrated 3D camera pose test.
    for name, bounds in spec['fixedViewAnchors'].items():
        point = result['landmarks'][name]
        x, y = point['point']; error = point['uncertaintyPx']
        require(bounds[0] <= x-error and x+error <= bounds[1] and
                bounds[2] <= y-error and y+error <= bounds[3],
                'fixed_view_anchor_moved:'+name)
    for joint, joint_spec in spec['joints'].items():
        dependencies = {f[k] for f in joint_spec['stateFields'] for k in ('a', 'b')}
        unavailable = sorted(dependencies-set(result['landmarks']))
        if unavailable:
            result['perJointRejected'][joint] = 'joint_landmark_unavailable:'+','.join(unavailable)
            continue
        values, errors = [], []
        names = [f['name'] for f in joint_spec['stateFields']]
        for feature in joint_spec['stateFields']:
            a, b = (result['landmarks'][feature[k]] for k in ('a', 'b'))
            delta = np.asarray(a['point'])-np.asarray(b['point'])
            value = (float(delta[0]) if feature['kind'] == 'relative_x' else
                     float(delta[1]) if feature['kind'] == 'relative_y' else float(np.linalg.norm(delta)))
            # Declared per-point bound applies to each coordinate. Euclidean error
            # can be sqrt(2) larger, unlike the per-axis relative coordinate.
            uncertainty = a['uncertaintyPx']+b['uncertaintyPx']
            if feature['kind'] == 'distance':
                uncertainty *= math.sqrt(2)
            values.append(value); errors.append(uncertainty)
        matches = [node['id'] for node in joint_spec['nodes'] if all(
            node['bounds'][name][0] <= value-error and value+error <= node['bounds'][name][1]
            for name, value, error in zip(names, values, errors))]
        if len(matches) > 1:
            result['perJointRejected'][joint] = 'ambiguous_node:'+joint
            continue
        result['states'][joint] = dict(state=values, stateFields=names,
            uncertaintyPx=errors, nodeMatches=matches)
        result['nodeMatches'].extend(matches)
    if result['states']:
        result.update(status='accepted', reason=None)
    else:
        result.update(status='rejected', reason='no_supported_joint_observations')
    return result


def observe_frame(image_bytes, frame_metadata, spec, context, now, clock_id):
    """Pure observation. Frame times and now must share the caller's clock_id.

    pixelSha256 is SHA256 of the decoded contiguous BGR uint8 bytes.
    A caller may not substitute request completion time for capture time.
    Matching requires every feature's uncertainty interval inside its node bounds.
    All joint matches are returned; no arbitrary winner resolves overlaps.
    """
    if isinstance(spec, dict) and spec.get('schemaVersion') == 'motionbrain.batch_background_registered_observer.v1':
        from learning import batch_background_observation
        return batch_background_observation.observe_frame(image_bytes, frame_metadata, spec, context, now, clock_id)
    if isinstance(spec, dict) and spec.get('schemaVersion') == 'motionbrain.batch_registered_observer.v1':
        from learning.batch_template_observation import observe_frame as observe_registered
        return observe_registered(image_bytes, frame_metadata, spec, context, now, clock_id)
    result = dict(status='rejected', reason=None, states={}, nodeMatches=[], landmarks={},
                  perLandmarkRejected={}, perJointRejected={},
                  frameBinding=copy.deepcopy(frame_metadata), semantics=SEMANTICS,
                  controlEligible=False, physicalValidationEstablished=False)
    try:
        spec = validate_spec(spec)
        result['specSha256'] = digest(spec)
        image = _prepare_frame(image_bytes, frame_metadata, spec, context, now, clock_id)
        _observe_hsv_landmarks(image, spec, result)
        _assemble_states(spec, result)
    except (ValueError, TypeError, KeyError, OverflowError, cv2.error) as exc:
        result.update(status='rejected', reason=str(exc), states={}, nodeMatches=[])
    return result
