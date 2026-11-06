"""Offline projected forearm-marker dynamics; no device or execution access.

toolFront/toolRear name two declared marker patches, not a calibrated tool tip,
3D pose, joint angle, grasp, contact, or clearance. Object presence is recorded
as a scene condition and never becomes a numerical model input.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np

from learning.artifacts import StorageError
from learning.manual_collection_dataset import build_visual_session, digest, require
from learning.relative_action_learning import command_key

MANIFEST = 'motionbrain.projected_arm_manifest.v1'
SCHEMA = 'motionbrain.projected_arm_ridge.v1'
STATE = ['frontMinusBackgroundXpx', 'frontMinusBackgroundYpx',
         'rearMinusFrontXpx', 'rearMinusFrontYpx']
SCENES = ('object_present', 'object_absent')


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def vector(value):
    require(isinstance(value, list) and len(value) == 4 and all(_number(v) for v in value),
            'finite_projected_marker_state_required')
    require(math.hypot(value[2], value[3]) > 0, 'coincident_marker_points')
    return np.asarray(value, dtype=float)


def point_state(annotation, frame):
    require(annotation.get('frameId') == frame['frameId']
            and annotation.get('pixelSha256') == frame['pixelSha256'], 'annotation_frame_mismatch')
    require(annotation.get('accepted') is True and annotation.get('method') == 'analyst_points'
            and annotation.get('inputFrameOnly') is True, 'unaccepted_or_target_assisted_annotation')
    width, height = frame['width'], frame['height']
    require(type(width) is int and type(height) is int and width > 0 and height > 0,
            'frame_dimensions_invalid')
    points = annotation['points']
    require(isinstance(points, dict) and set(points) == {'background', 'toolFront', 'toolRear'},
            'three_marker_landmarks_required')
    parsed = []
    for key in ('background', 'toolFront', 'toolRear'):
        point = points[key]
        require(isinstance(point, list) and len(point) == 2
                and all(_number(v) for v in point)
                and 0 <= point[0] < width and 0 <= point[1] < height, 'landmark_outside_frame')
        parsed.append(np.asarray(point, dtype=float))
    background, front, rear = parsed
    state = [*(front-background).tolist(), *(rear-front).tolist()]
    vector(state)
    return state


def annotated_transition(row, annotation):
    require(row.get('status') == 'eligible', 'source_not_eligible')
    require(annotation['sessionId'] == row['sessionId'], 'annotation_session_mismatch')
    context = annotation['context']
    require(isinstance(context, dict) and set(context) == {'physicalView', 'poseRegime', 'landmarks'}
            and all(isinstance(v, str) and v for v in context.values()), 'observation_context_required')
    require(annotation.get('sceneCondition') in SCENES, 'explicit_scene_condition_required')
    before_frame, after_frame = row['inputs']['image'], row['targets']['image']
    require([before_frame['height'], before_frame['width']]
            == [after_frame['height'], after_frame['width']], 'image_shape_changed')
    before = point_state(annotation['before'], before_frame)
    after = point_state(annotation['after'], after_frame)
    command_key(row['inputs']['command'])
    evidence = row['evidence']
    require(isinstance(evidence.get('cameraIdentity'), dict) and evidence['cameraIdentity'],
            'camera_identity_required')
    require(_number(evidence.get('commandAt')) and evidence['commandAt'] >= 0, 'command_time_required')
    return dict(sessionId=row['sessionId'], captureGroup=annotation['captureGroup'],
        context=dict(context, cameraIdentity=copy.deepcopy(evidence['cameraIdentity']),
                     imageShape=[before_frame['height'], before_frame['width']], stateFields=list(STATE)),
        sceneCondition=annotation['sceneCondition'], command=copy.deepcopy(row['inputs']['command']),
        before=before, after=after, commandAt=evidence['commandAt'],
        episode=[evidence['controllerBootId'], evidence['commandId']],
        frameIds=[before_frame['frameId'], after_frame['frameId']],
        annotationSemantics=annotation.get('annotationSemantics', 'per_frame_analyst_marker_points'),
        originalDatasetEligible=row.get('originalDatasetEligible', False))


def validate_rows(rows):
    require(isinstance(rows, list) and bool(rows), 'transitions_required')
    context = rows[0]['context']
    require(isinstance(context, dict) and set(context) == {
        'physicalView', 'poseRegime', 'landmarks', 'cameraIdentity', 'imageShape', 'stateFields'}
        and context['stateFields'] == STATE and isinstance(context['cameraIdentity'], dict)
        and bool(context['cameraIdentity'])
        and all(isinstance(context[k], str) and context[k] for k in
                ('physicalView', 'poseRegime', 'landmarks')), 'projected_marker_context_required')
    require(isinstance(context['imageShape'], list) and len(context['imageShape']) == 2
            and all(type(v) is int and v > 0 for v in context['imageShape']), 'image_shape_required')
    ids, episodes, frames, joints = set(), set(), set(), set()
    for row in rows:
        require(row['context'] == context, 'observation_context_changed')
        require(isinstance(row['sessionId'], str) and row['sessionId'] and row['sessionId'] not in ids,
                'duplicate_or_missing_session')
        require(isinstance(row['captureGroup'], str) and row['captureGroup'], 'capture_group_required')
        require(row['sceneCondition'] in SCENES, 'explicit_scene_condition_required')
        require(_number(row['commandAt']) and row['commandAt'] >= 0, 'command_time_required')
        episode = row['episode']
        require(isinstance(episode, list) and len(episode) == 2
                and all(isinstance(v, str) and v for v in episode)
                and tuple(episode) not in episodes, 'duplicate_or_missing_episode')
        refs = row['frameIds']
        require(isinstance(refs, list) and len(refs) == 2
                and all(isinstance(v, str) and v for v in refs)
                and refs[0] != refs[1] and not frames.intersection(refs), 'reused_frame')
        ids.add(row['sessionId']); episodes.add(tuple(episode)); frames.update(refs)
        vector(row['before']); vector(row['after'])
        joints.add(command_key(row['command'])[0])
    require(len(joints) == 1, 'joint_contract_changed')
    return context


def fit(rows):
    context = validate_rows(rows)
    models, deficits = [], []
    for key in sorted({command_key(r['command']) for r in rows}):
        selected = [r for r in rows if command_key(r['command']) == key]
        if len(selected) < 2:
            deficits.append(dict(command=copy.deepcopy(selected[0]['command']), count=len(selected),
                                 reason='command_without_two_training_transitions'))
            continue
        x = np.asarray([vector(r['before']) for r in selected])
        delta = np.asarray([vector(r['after'])-vector(r['before']) for r in selected])
        mean = x.mean(axis=0); scale = np.maximum(x.std(axis=0), 1.)
        z = (x-mean)/scale; intercept = delta.mean(axis=0)
        coefficient = np.linalg.solve(z.T@z+np.eye(4), z.T@(delta-intercept))
        require(np.isfinite(coefficient).all(), 'nonfinite_fit')
        models.append(dict(command=copy.deepcopy(selected[0]['command']), count=len(selected),
            mean=mean.tolist(), scale=scale.tolist(), coefficient=coefficient.tolist(),
            deltaMean=intercept.tolist(), minimum=x.min(axis=0).tolist(), maximum=x.max(axis=0).tolist()))
    return dict(schemaVersion=SCHEMA, context=copy.deepcopy(context), joint=rows[0]['command']['joint'],
        models=models, supportDeficits=deficits,
        trainingSessionIds=[r['sessionId'] for r in rows],
        trainingCaptureGroups=sorted({r['captureGroup'] for r in rows}),
        trainingSceneConditions=sorted({r['sceneCondition'] for r in rows}),
        trainingEpisodes=copy.deepcopy([r['episode'] for r in rows]),
        trainingFrameIds=[f for r in rows for f in r['frameIds']],
        trainingCommandTimes=[r['commandAt'] for r in rows], trainingTransitions=len(rows),
        supportedTrainingTransitions=sum(m['count'] for m in models), alpha=1.,
        annotationSemantics='projected_forearm_marker_points_not_tool_tip_3d_pose_or_joint_angle',
        codeSha256=digest(Path(__file__).read_bytes()),
        commandContractCodeSha256=digest(Path(__file__).with_name('relative_action_learning.py').read_bytes()),
        controlEligible=False, independentTest=False)


def _model_contract(model):
    require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
            and model.get('controlEligible') is False and model.get('independentTest') is False
            and _number(model.get('alpha')) and model['alpha'] == 1.
            and isinstance(model.get('context'), dict) and model['context'].get('stateFields') == STATE
            and isinstance(model.get('joint'), str),
            'offline_projected_marker_model_required')


def predict(model, state, command, context, *, baseline=False):
    _model_contract(model)
    require(context == model['context'], 'observation_context_changed')
    key = command_key(command); x = vector(state)
    require(command['joint'] == model['joint'], 'joint_contract_changed')
    candidates = [m for m in model['models'] if command_key(m['command']) == key]
    require(len(candidates) == 1, 'command_without_two_training_transitions')
    m = candidates[0]
    require(type(m.get('count')) is int and m['count'] >= 2, 'invalid_model_support')
    for name in ('minimum', 'maximum', 'mean', 'scale', 'deltaMean'):
        require(isinstance(m.get(name), list) and len(m[name]) == 4
                and all(_number(v) for v in m[name]), 'invalid_model_parameters')
    require(isinstance(m.get('coefficient'), list) and len(m['coefficient']) == 4
            and all(isinstance(row, list) and len(row) == 4 and all(_number(v) for v in row)
                    for row in m['coefficient']), 'invalid_model_parameters')
    arrays = {k: np.asarray(m[k], dtype=float) for k in
              ('minimum', 'maximum', 'mean', 'scale', 'deltaMean', 'coefficient')}
    require(all(a.shape == ((4, 4) if k == 'coefficient' else (4,)) and np.isfinite(a).all()
                for k, a in arrays.items()) and np.all(arrays['scale'] >= 1.)
            and np.all(arrays['minimum'] <= arrays['maximum']), 'invalid_model_parameters')
    require(np.all(x >= arrays['minimum']) and np.all(x <= arrays['maximum']),
            'state_outside_training_range')
    delta = arrays['deltaMean']
    if not baseline:
        delta = delta + ((x-arrays['mean'])/arrays['scale'])@arrays['coefficient']
    out = (x+delta).tolist()
    vector(out)
    return out


def evaluate(model, rows):
    _model_contract(model)
    if not rows:
        return dict(listed=0, evaluated=0, rejected=[], predictions=[], metrics={}, sceneConditions={},
            status='unavailable', reason='no_separate_evaluation_transitions',
            controlEligible=False, independentTest=False)
    validate_rows(rows)
    require(not set(model['trainingCaptureGroups']).intersection(r['captureGroup'] for r in rows),
            'training_evaluation_group_overlap')
    require(not set(model['trainingSessionIds']).intersection(r['sessionId'] for r in rows)
            and not {tuple(e) for e in model['trainingEpisodes']}.intersection(tuple(r['episode']) for r in rows)
            and not set(model['trainingFrameIds']).intersection(f for r in rows for f in r['frameIds']),
            'training_evaluation_overlap')
    predictions, rejected = [], []
    scenes = {scene: sum(r['sceneCondition'] == scene for r in rows) for scene in SCENES}
    for row in rows:
        require(row['context'] == model['context'], 'observation_context_changed')
        require(row['command']['joint'] == model['joint'], 'joint_contract_changed')
        try:
            ridge = predict(model, row['before'], row['command'], row['context'])
            baseline = predict(model, row['before'], row['command'], row['context'], baseline=True)
            predictions.append(dict(sessionId=row['sessionId'], command=copy.deepcopy(row['command']),
                commandAt=row['commandAt'], sceneCondition=row['sceneCondition'],
                sceneConditionShift=row['sceneCondition'] not in model['trainingSceneConditions'],
                actualState=copy.deepcopy(row['after']), stateRidge=ridge, commandMean=baseline,
                noChange=copy.deepcopy(row['before'])))
        except ValueError as exc:
            rejected.append(dict(sessionId=row['sessionId'], sceneCondition=row['sceneCondition'], reason=str(exc)))
    metrics = {}
    if predictions:
        y = np.asarray([p['actualState'] for p in predictions])
        for name in ('stateRidge', 'commandMean', 'noChange'):
            error = np.asarray([p[name] for p in predictions])-y
            metrics[name] = dict(n=len(error), maePerStatePx=np.abs(error).mean(axis=0).tolist(),
                totalStateNormPx=float(np.linalg.norm(error, axis=1).mean()),
                markerPositionErrorPx=float(np.linalg.norm(error[:, :2], axis=1).mean()),
                linkVectorErrorPx=float(np.linalg.norm(error[:, 2:], axis=1).mean()))
    return dict(listed=len(rows), evaluated=len(predictions), rejected=rejected, predictions=predictions,
        metrics=metrics, sceneConditions=scenes, trainingSceneConditions=model['trainingSceneConditions'],
        sceneConditionShiftRows=sum(r['sceneCondition'] not in model['trainingSceneConditions'] for r in rows),
        controlEligible=False, independentTest=False,
        evaluationKind='projected_forearm_marker_analyst_annotations')


def load_manifest(manifest, root):
    """Reaudit external frames and bind each independently reviewed annotation."""
    require(isinstance(manifest, dict) and manifest.get('schemaVersion') == MANIFEST,
            'projected_marker_manifest_required')
    require(isinstance(manifest.get('records'), list), 'manifest_records_required')
    groups = manifest.get('evaluationGroups', [])
    require(isinstance(groups, list) and all(isinstance(g, str) and g for g in groups),
            'evaluation_groups_required')
    root = Path(root).resolve()
    rows, rejected = [], []
    for item in manifest['records']:
        try:
            path = (root/item['sessionRoot']).resolve()
            require(path.is_relative_to(root), 'source_path_outside_manifest_root')
            require(item.get('observationCamera', 'external') == 'external', 'external_camera_required')
            row = build_visual_session(path, item['sessionSha256'])
            rows.append(annotated_transition(row, item['annotation']))
        except (ValueError, KeyError, TypeError, OSError, StorageError) as exc:
            annotation = item.get('annotation') if isinstance(item, dict) else None
            rejected.append(dict(sessionId=annotation.get('sessionId')
                                 if isinstance(annotation, dict) else None, reason=str(exc)))
    if rows:
        validate_rows(rows)
    return rows, rejected
