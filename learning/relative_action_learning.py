"""Offline projected object/jaw dynamics. No transport or execution entry point."""
from __future__ import annotations

import copy
import math
import re
import numpy as np

from learning.manual_collection_dataset import require, build_visual_session
from learning.supervised_collection import PROFILES
from learning.artifacts import StorageError

SCHEMA = 'motionbrain.relative_action_ridge.v1'
STATE = ['objectMinusJawMidXpx', 'objectMinusJawMidYpx', 'jawOpeningPx']
REGISTERED_METHODS = {
    'registered_template_points': 'registered-template-points-v1:',
    'registered_scene_template_points': 'registered-scene-template-points-v1:',
    'registered_scene_rigid_points': 'registered-scene-rigid-points-v1:',
}


def registered_landmarks(value):
    return isinstance(value, str) and value.startswith(tuple(REGISTERED_METHODS.values()))


def validate_motion_scope(context, command):
    if str(context.get('landmarks', '')).startswith(REGISTERED_METHODS['registered_scene_rigid_points']):
        require(command.get('joint') == 'shoulder', 'rigid_observation_requires_M4_only')


def vector(value):
    require(isinstance(value, list) and len(value) == 3
            and all(type(x) in (int, float) and math.isfinite(x) for x in value),
            'finite_relative_state_required')
    require(value[2] >= 0, 'negative_jaw_opening')
    return np.asarray(value, dtype=float)


def command_key(command):
    require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'},
            'single_command_fields')
    limits = PROFILES.get(command['joint'], {}).get('directions', {}).get(command['direction'])
    require(limits is not None, 'joint_direction_not_enabled')
    require(type(command['percent']) is int and 1 <= command['percent'] <= limits[1]
            and type(command['durationMs']) is int and 100 <= command['durationMs'] <= limits[2],
            'joint_output_limit')
    return tuple(command[k] for k in ('joint', 'direction', 'percent', 'durationMs'))


def point_state(annotation, frame):
    require(annotation.get('frameId') == frame['frameId']
            and annotation.get('pixelSha256') == frame['pixelSha256'], 'annotation_frame_mismatch')
    method = annotation.get('method')
    require(annotation.get('accepted') is True and method in ('analyst_points', *REGISTERED_METHODS)
            and annotation.get('inputFrameOnly') is True, 'unaccepted_or_target_assisted_annotation')
    if method in REGISTERED_METHODS:
        require(all(isinstance(annotation.get(k), str)
                    and re.fullmatch('[0-9a-f]{64}', annotation[k]) is not None
                    for k in ('registrationSha256', 'parametersSha256', 'referenceImageSha256'))
                and annotation.get('landmarkContract') ==
                    REGISTERED_METHODS[method]+annotation['registrationSha256'],
                'registered_annotation_contract_required')
    points = annotation['points']
    require(set(points) == {'object', 'jawLeft', 'jawRight'}, 'three_landmarks_required')
    coords = []
    for name in ('object', 'jawLeft', 'jawRight'):
        p = points[name]
        require(isinstance(p, list) and len(p) == 2
                and all(type(x) in (int, float) and math.isfinite(x) for x in p)
                and 0 <= p[0] < frame['width'] and 0 <= p[1] < frame['height'], 'landmark_outside_frame')
        coords.append(np.asarray(p, dtype=float))
    obj, left, right = coords
    require(np.linalg.norm(right-left) > 0, 'coincident_jaw_points')
    return [*(obj-(left+right)/2).tolist(), float(np.linalg.norm(right-left))]


def annotated_transition(row, annotation):
    """Bind analyst landmarks to reaudited frames, never to sensor targets."""
    require(annotation['sessionId'] == row['sessionId'], 'annotation_session_mismatch')
    context = annotation['context']
    require(isinstance(context, dict) and set(context) == {'physicalView', 'object', 'poseRegime', 'landmarks'}
            and all(isinstance(x, str) and x for x in context.values()), 'observation_context_required')
    methods = [annotation[k].get('method') for k in ('before', 'after')]
    require(methods[0] == methods[1], 'annotation_method_changed')
    if methods[0] in REGISTERED_METHODS:
        require(context['landmarks'] == annotation['before'].get('landmarkContract')
                and all(annotation['before'].get(k) == annotation['after'].get(k) for k in
                        ('landmarkContract', 'registrationSha256', 'parametersSha256', 'referenceImageSha256')),
                'registered_annotation_context_changed')
    else:
        require(not registered_landmarks(context['landmarks']),
                'registered_context_requires_registered_annotation')
    before = point_state(annotation['before'], row['inputs']['image'])
    after = point_state(annotation['after'], row['targets']['image'])
    command_key(row['inputs']['command'])
    validate_motion_scope(context, row['inputs']['command'])
    return dict(sessionId=row['sessionId'], captureGroup=annotation['captureGroup'],
                context=dict(context, cameraIdentity=copy.deepcopy(row['evidence']['cameraIdentity']),
                             imageShape=[row['inputs']['image']['height'], row['inputs']['image']['width']],
                             stateFields=STATE),
                command=copy.deepcopy(row['inputs']['command']), before=before, after=after,
                episode=[row['evidence']['controllerBootId'], row['evidence']['commandId']],
                frameIds=[row['inputs']['image']['frameId'], row['targets']['image']['frameId']])


def validate_rows(rows):
    require(isinstance(rows, list) and len(rows) > 0, 'transitions_required')
    ids, episodes, frames = set(), set(), set()
    context = rows[0]['context']
    for row in rows:
        require(row['context'] == context, 'observation_context_changed')
        require(isinstance(row['sessionId'], str) and row['sessionId'] and row['sessionId'] not in ids,
                'duplicate_or_missing_session')
        require(isinstance(row['captureGroup'], str) and row['captureGroup'], 'capture_group_required')
        episode = tuple(row['episode'])
        require(len(episode) == 2 and all(isinstance(x, str) and x for x in episode)
                and episode not in episodes, 'duplicate_or_missing_episode')
        refs = row['frameIds']
        require(len(refs) == 2 and refs[0] != refs[1] and not frames.intersection(refs), 'reused_frame')
        ids.add(row['sessionId']); episodes.add(episode); frames.update(refs)
        vector(row['before']); vector(row['after']); command_key(row['command'])
        validate_motion_scope(context, row['command'])
    return context


def fit(rows):
    context = validate_rows(rows)
    models = []
    for key in sorted({command_key(r['command']) for r in rows}):
        selected = [r for r in rows if command_key(r['command']) == key]
        # One observation is retained as a deficit, never promoted to support.
        if len(selected) < 2:
            continue
        x = np.asarray([vector(r['before']) for r in selected])
        delta = np.asarray([vector(r['after'])-vector(r['before']) for r in selected])
        mean = x.mean(axis=0); scale = np.maximum(x.std(axis=0), 1.)
        z = (x-mean)/scale; intercept = delta.mean(axis=0)
        coefficient = np.linalg.solve(z.T@z+np.eye(3), z.T@(delta-intercept))
        models.append(dict(command=selected[0]['command'], count=len(selected), mean=mean.tolist(),
                           scale=scale.tolist(), coefficient=coefficient.tolist(), deltaMean=intercept.tolist(),
                           minimum=x.min(axis=0).tolist(), maximum=x.max(axis=0).tolist()))
    return dict(schemaVersion=SCHEMA, context=copy.deepcopy(context), models=copy.deepcopy(models),
                trainingSessionIds=[r['sessionId'] for r in rows],
                trainingCaptureGroups=sorted({r['captureGroup'] for r in rows}),
                controlEligible=False, independentTest=False, alpha=1.,
                annotationSemantics=(
                    'registered_feature_translation_of_analyst_seed_points_not_depth_contact_clearance_or_accuracy'
                    if registered_landmarks(context['landmarks']) else
                    'analyst_2d_points_not_grasp_depth_contact_or_clearance'),
                trainingEpisodes=[r['episode'] for r in rows], trainingFrameIds=[f for r in rows for f in r['frameIds']])


def predict(model, state, command, context, *, baseline=False):
    require(model.get('schemaVersion') == SCHEMA and model.get('controlEligible') is False
            and model.get('alpha') == 1., 'offline_model_required')
    require(context == model['context'], 'observation_context_changed')
    validate_motion_scope(context, command)
    key = command_key(command); x = vector(state)
    candidates = [m for m in model['models'] if command_key(m['command']) == key]
    require(len(candidates) == 1, 'command_without_two_training_transitions')
    m = candidates[0]
    require(np.all(x >= np.asarray(m['minimum'])) and np.all(x <= np.asarray(m['maximum'])),
            'state_outside_training_range')
    delta = np.asarray(m['deltaMean'])
    if not baseline:
        delta = delta + ((x-np.asarray(m['mean']))/np.asarray(m['scale']))@np.asarray(m['coefficient'])
    out = x+delta
    require(np.isfinite(out).all() and out[2] >= 0, 'invalid_predicted_state')
    return out.tolist()


def select(model, state, target, commands, context):
    """Compare learned next states with stop; unsupported actions remain visible."""
    x, goal = vector(state), vector(target)
    require(context == model['context'], 'observation_context_changed')
    require(isinstance(commands, list) and bool(commands), 'candidates_required')
    keys = [command_key(c) for c in commands]
    require(len(keys) == len(set(keys)), 'duplicate_candidate')
    ranked = [dict(kind='stop', command=None, predictedState=x.tolist(), errorPx=float(np.linalg.norm(x-goal)))]
    rejected = []
    for command in commands:
        try:
            prediction = predict(model, state, command, context)
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
            continue
        ranked.append(dict(kind='action', command=copy.deepcopy(command), predictedState=prediction,
                           errorPx=float(np.linalg.norm(np.asarray(prediction)-goal))))
    ranked.sort(key=lambda r: r['errorPx'])
    return dict(selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected,
                status='ranked' if len(ranked) > 1 else 'abstained',
                controlEligible=False, counterfactualOutcomesObserved=False, graspVerified=False)


def evaluate(model, rows):
    if not rows:
        return dict(listed=0, evaluated=0, rejected=[], predictions=[], metrics={},
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
    for row in rows:
        require(row['context'] == model['context'], 'observation_context_changed')
        try:
            predictions.append(dict(sessionId=row['sessionId'], actualState=row['after'],
                stateRidge=predict(model, row['before'], row['command'], row['context']),
                commandMean=predict(model, row['before'], row['command'], row['context'], baseline=True),
                noChange=row['before']))
        except ValueError as exc:
            rejected.append(dict(sessionId=row['sessionId'], reason=str(exc)))
    metrics = {}
    if predictions:
        y = np.asarray([p['actualState'] for p in predictions])
        for name in ('stateRidge', 'commandMean', 'noChange'):
            error = np.asarray([p[name] for p in predictions])-y
            metrics[name] = dict(n=len(error), maePerStatePx=np.abs(error).mean(axis=0).tolist(),
                                 meanStateErrorPx=float(np.linalg.norm(error, axis=1).mean()))
    return dict(listed=len(rows), evaluated=len(predictions), rejected=rejected, predictions=predictions,
                metrics=metrics, controlEligible=False, independentTest=False,
                evaluationKind='development_replay_analyst_annotations')


def load_manifest(manifest, root):
    """Reaudit allowed single-command sources with the existing fixed windows."""
    from pathlib import Path
    root = Path(root).resolve()
    rows, rejected = [], []
    for item in manifest['records']:
        path = (root/item['sessionRoot']).resolve()
        require(path.is_relative_to(root), 'source_path_outside_manifest_root')
        # The existing auditor validates hashes, input receipt/persistence,
        # output OFF, and post window. No sensor value becomes an input.
        try:
            row = build_visual_session(path, item['sessionSha256'])
            if item.get('observationCamera', 'external') == 'gripper':
                from learning.gripper_camera_dataset import bind_gripper_pair
                camera_root = (root/item['gripperObservationRoot']).resolve()
                require(camera_root.is_relative_to(root), 'gripper_path_outside_manifest_root')
                row = bind_gripper_pair(row, camera_root)
            else:
                require(item.get('observationCamera', 'external') == 'external', 'unknown_observation_camera')
            rows.append(annotated_transition(row, item['annotation']))
        except (ValueError, KeyError, TypeError, OSError, StorageError) as exc:
            rejected.append(dict(sessionId=item.get('annotation', {}).get('sessionId'), reason=str(exc)))
    if rows:
        validate_rows(rows)
    return rows, rejected
