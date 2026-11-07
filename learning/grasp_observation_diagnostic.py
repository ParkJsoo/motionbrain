"""Fixed-view cylinder/jaw landmark study. No fitting, transport or control."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from learning.artifacts import ArtifactStore, StorageError, _read
from learning.gripper_camera_dataset import read_frame
from learning.relative_action_learning import STATE, command_key, load_manifest, point_state
from learning.relative_model_bundle import load_frozen_model
from learning.manual_collection_dataset import require

METHOD = 'fixed_cylinder_arc_and_distal_jaws_diagnostic.v1'
SHA = lambda raw: hashlib.sha256(raw).hexdigest()


def state_from_points(points):
    obj, left, right = (np.asarray(points[k], dtype=float) for k in ('object', 'jawLeft', 'jawRight'))
    return [*(obj-(left+right)/2).tolist(), float(np.linalg.norm(right-left))]


def extract_points(jpeg, parameters):
    """Parameters are supplied before extraction; models/targets are not inputs."""
    p = parameters
    require(set(p) == {'imageShape', 'grayThreshold', 'objectRightArcRows',
                       'objectRightArcColumns', 'jawRois', 'jawDistalRows'}, 'fixed_parameters_required')
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape[:2]) == p['imageShape'], 'diagnostic_image_shape_changed')
    height, width = image.shape[:2]
    require(type(p['grayThreshold']) is int and 1 <= p['grayThreshold'] <= 254
            and type(p['jawDistalRows']) is int and 1 <= p['jawDistalRows'] <= height,
            'diagnostic_parameters_invalid')
    def pair(values, maximum):
        require(isinstance(values, list) and len(values) == 2
                and all(type(v) is int for v in values) and 0 <= values[0] < values[1] < maximum,
                'diagnostic_roi_invalid')
        return values
    ymin, ymax = pair(p['objectRightArcRows'], height)
    xmin, xmax = pair(p['objectRightArcColumns'], width)
    require(isinstance(p['jawRois'], list) and len(p['jawRois']) == 2, 'two_jaw_rois_required')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    arc, flags = [], []
    for y in range(ymin, ymax+1):  # Object bounds are inclusive.
        xs = np.flatnonzero(gray[y, xmin:xmax+1] < p['grayThreshold'])+xmin
        if len(xs) >= 5:
            arc.append([float(xs[-1]), float(y)])
            if xs[-1] == xmax: flags.append('object_arc_at_right_boundary')
    require(len(arc) >= 20, 'object_arc_insufficient')
    arc = np.asarray(arc)
    design = np.column_stack([arc, np.ones(len(arc))])
    require(np.linalg.matrix_rank(design) == 3, 'object_arc_degenerate')
    coefficient = np.linalg.lstsq(design, (arc**2).sum(axis=1), rcond=None)[0]
    center = coefficient[:2]/2
    radius_sq = coefficient[2]+float((center**2).sum())
    require(np.isfinite(center).all() and np.isfinite(radius_sq) and radius_sq > 0,
            'object_circle_invalid')
    radius = float(np.sqrt(radius_sq))
    residual = float(np.sqrt(np.mean((np.linalg.norm(arc-center, axis=1)-radius)**2)))
    require(0 <= center[0] < width and 0 <= center[1] < height, 'object_center_outside_image')
    if residual > 1.: flags.append('circle_residual_over_one_pixel')
    points = {'object': center.tolist()}
    for name, roi in zip(('jawLeft', 'jawRight'), p['jawRois']):
        require(isinstance(roi, list) and len(roi) == 4 and all(type(v) is int for v in roi),
                'diagnostic_roi_invalid')
        x0, y0, x1, y1 = roi  # Jaw upper bounds are exclusive.
        require(0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height, 'diagnostic_roi_invalid')
        mask = (gray[y0:y1, x0:x1] < p['grayThreshold']).astype('uint8')
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        require(count > 1, 'jaw_component_missing')
        component = 1+int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        ys, xs = np.where(labels == component)
        require(len(xs) >= 16, 'jaw_component_too_small')
        distal = ys >= ys.max()-(p['jawDistalRows']-1)
        points[name] = [float(xs[distal].mean()+x0), float(ys[distal].mean()+y0)]
        if ys.max() == y1-y0-1: flags.append(name+'_distal_roi_clipped')
        if xs.min() == 0 or xs.max() == x1-x0-1:
            flags.append(name+'_lateral_roi_clipped_or_wrong_component')
    rounded = {k: np.rint(v).tolist() for k, v in points.items()}
    return dict(method=METHOD, parametersSha256=SHA(json.dumps(p, sort_keys=True).encode()),
                points=points, relativeStatePx=state_from_points(points),
                nearestIntegerPoints=rounded, nearestIntegerStatePx=state_from_points(rounded),
                objectCircleRadiusPx=radius, objectCircleResidualPx=residual,
                objectArcRows=len(arc), qualityFlags=sorted(set(flags)),
                trainingEligible=False, controlEligible=False)


def input_coverage(training, query):
    """Only before-inputs define ranges; after targets never enlarge support."""
    output = []
    require(all(row['role'] == 'training_before' for row in training)
            and query['role'] == 'rejected_query_input', 'input_only_coverage_required')
    keys = sorted({command_key(row['command']) for row in training if row['command'] is not None})
    if any(row['command'] is None for row in training):
        keys.append(None)
    for key in keys:
        rows = [r for r in training if (command_key(r['command']) if r['command'] is not None else None) == key]
        for variant in ('relativeStatePx', 'nearestIntegerStatePx'):
            supported = [r['measurement'][variant] for r in rows
                         if r.get('measurement') and not r['measurement']['qualityFlags']]
            entry = dict(command=rows[0]['command'], representation=variant,
                         listedTrainingInputs=len(rows), extractedTrainingInputs=len(supported),
                         unknownCommand=key is None, queryCheckAvailable=False, controlEligible=False)
            if supported:
                values = np.asarray(supported); lo, hi = values.min(axis=0), values.max(axis=0)
                entry.update(minimum=lo.tolist(), maximum=hi.tolist())
                if query.get('measurement') and not query['measurement']['qualityFlags']:
                    x = query['measurement'][variant]
                    violations = [dict(field=k, value=v, minimum=a, maximum=b,
                                       outsideByPx=max(a-v, v-b, 0.))
                                  for k, v, a, b in zip(STATE, x, lo.tolist(), hi.tolist()) if not a <= v <= b]
                    entry.update(queryCheckAvailable=True, queryState=x,
                                 insideObservedMinMax=not violations, violations=violations)
            output.append(entry)
    return output


def run_study(config_path, expected_sha256, output_dir, *, root):
    root = Path(root).resolve()
    def private(path):
        value = (root/Path(path)).resolve()
        require(value.is_relative_to(root/'.local'), 'private_diagnostic_path_required')
        return value
    config_path = private(config_path)
    raw = config_path.read_bytes(); require(SHA(raw) == expected_sha256, 'diagnostic_config_changed')
    config = json.loads(raw)
    require(config['schemaVersion'] == 'motionbrain.grasp_observation_study.v1', 'diagnostic_schema')
    if 'parameterOrigin' in config:
        origin = config['parameterOrigin']
        origin_raw = private(origin['path']).read_bytes()
        require(SHA(origin_raw) == origin['sha256'], 'diagnostic_parameter_origin_changed')
        require(json.loads(origin_raw)['parameters'] == {
            k: v for k, v in config['parameters'].items() if k != 'imageShape'},
            'diagnostic_parameters_differ_from_origin')
    baseline = config['frozenModel']; model_path = private(baseline['path'])
    model_raw = model_path.read_bytes(); model = load_frozen_model(model_path, baseline['sha256'])
    manifest_path = private(config['manifestPath']); manifest_raw = manifest_path.read_bytes()
    require(SHA(manifest_raw) == config['manifestSha256'], 'diagnostic_manifest_changed')
    manifest = json.loads(manifest_raw)
    require(manifest.get('evaluationGroups') == [], 'training_sources_only')
    require(len(manifest['records']) == len(model['trainingSessionIds'])
            and {r['annotation']['sessionId'] for r in manifest['records']} == set(model['trainingSessionIds']),
            'same_frozen_training_sources_required')
    rows, rejected = load_manifest(manifest, root)
    by_id = {r['sessionId']: r for r in rows}
    observations, training, pairs = [], [], []
    def observe(source, role, annotation=None):
        entry = dict(role=role, source=copy.deepcopy(source))
        try:
            location = private(source['root'])
            frame, _, boot = read_frame(location, source['label'], source['sessionId'])
            require(boot == config['expectedCameraBootId'], 'diagnostic_camera_boot_changed')
            jpeg = _read(location, 'esp32-'+source['label']+'.jpg')
            require(SHA(jpeg) == frame['imageSha256'], 'diagnostic_verified_image_changed')
            entry.update(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'], imageSha256=SHA(jpeg))
            entry['measurement'] = extract_points(jpeg, config['parameters'])
            if annotation is not None:
                entry['analystStatePx'] = point_state(annotation, frame)
                entry['pointDifferencesFromAnalystPx'] = {
                    k: float(np.linalg.norm(np.asarray(entry['measurement']['points'][k])-annotation['points'][k]))
                    for k in ('object', 'jawLeft', 'jawRight')}
        except (ValueError, KeyError, TypeError, OSError, StorageError) as exc:
            entry['error'] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
            entry.pop('measurement', None)
        observations.append(entry)
        return entry
    for record in manifest['records']:
        ann = record['annotation']; sid = ann['sessionId']
        require(ann['context'] == config['sceneContext'], 'diagnostic_scene_context_changed')
        pair = dict(sessionId=sid, command=by_id[sid]['command'] if sid in by_id else None)
        for label in ('before', 'after'):
            source = dict(root=record['gripperObservationRoot'], label=label, sessionId=sid)
            if sid not in by_id:
                entry = dict(role='training_'+label, source=source, error='original_source_audit_rejected')
                observations.append(entry)
            else:
                entry = observe(source, 'training_'+label, ann[label])
            pair[label] = entry
            if label == 'before':
                training.append(dict(command=pair['command'], **entry))
        pair['completeUnflagged'] = all(
            p.get('measurement') and not p['measurement']['qualityFlags']
            for p in (pair['before'], pair['after']))
        pairs.append(pair)
    query_spec = config['rejectedQuery']
    require(query_spec['source']['label'] == 'before'
            and query_spec['source']['sessionId'] not in model['trainingSessionIds'],
            'new_query_input_required')
    query_annotation_raw = private(query_spec['annotationPath']).read_bytes()
    require(SHA(query_annotation_raw) == query_spec['annotationSha256'], 'diagnostic_query_annotation_changed')
    query = observe(query_spec['source'], 'rejected_query_input', json.loads(query_annotation_raw))
    if query.get('frameId') in model['trainingFrameIds']:
        query['error'] = 'query_frame_overlaps_training'
        query.pop('measurement', None)
    stationary = [observe(source, 'stationary_diagnostic') for source in config.get('stationarySources', [])]
    require(SHA(config_path.read_bytes()) == expected_sha256 and model_path.read_bytes() == model_raw
            and manifest_path.read_bytes() == manifest_raw, 'diagnostic_inputs_changed')
    coverage = input_coverage(training, query)
    paired_inputs = []
    for pair in pairs:
        entry = dict(command=pair['command'], **pair['before'])
        if not pair['completeUnflagged']:
            entry.pop('measurement', None)
        paired_inputs.append(entry)
    old_support = []
    if 'analystStatePx' in query:
        x = query['analystStatePx']
        for part in model['models']:
            old_support.append(dict(command=part['command'], insideObservedMinMax=all(
                a <= v <= b for v, a, b in zip(x, part['minimum'], part['maximum']))))
    repeatability = {}
    for variant in ('relativeStatePx', 'nearestIntegerStatePx'):
        valid = [r['measurement'][variant] for r in stationary
                 if r.get('measurement') and not r['measurement']['qualityFlags']]
        repeatability[variant] = dict(listed=len(stationary), extracted=len(valid),
            rangePerStatePx=np.ptp(valid, axis=0).tolist() if valid else None)
    report = dict(schemaVersion='motionbrain.grasp_observation_study_result.v1', method=METHOD,
        configSha256=expected_sha256, sourceCodeSha256=SHA(Path(__file__).read_bytes()),
        frozenModelSha256=baseline['sha256'], manifestSha256=config['manifestSha256'],
        observations=observations, sourceRejected=rejected,
        listedTrainingRecords=len(manifest['records']), sourceAuditedTrainingRecords=len(rows),
        listedFrames=2*len(manifest['records'])+1+len(stationary),
        extractedFrames=sum('measurement' in r for r in observations),
        failures=sum('error' in r for r in observations),
        flaggedFrames=sum(bool(r.get('measurement', {}).get('qualityFlags')) for r in observations),
        inputCoverage=coverage, observationPairs=[dict(sessionId=p['sessionId'], command=p['command'],
            completeUnflagged=bool(p['completeUnflagged'])) for p in pairs],
        completeUnflaggedPairs=sum(bool(p['completeUnflagged']) for p in pairs),
        pairedInputCoverage=input_coverage(paired_inputs, query), originalAnalystModelQueryCoverage=old_support,
        stationaryRepeatability=repeatability, modelRefit=False, annotationsRewritten=False,
        trainingEligible=False, controlEligible=False, independentTest=False,
        interpretation='저장 원본의 관측 방식·입력 범위 진단. 정확도·독립 평가·실물 제어 허용 근거가 아니다.')
    output = private(output_dir)
    store = ArtifactStore(output)
    store.write_json('locked-config.json', config)
    store.write_json('measurement-audit.json', report)
    return report
