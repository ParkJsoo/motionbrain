"""원본 해시를 검사하는 개발용 집게 관측 재생. 평가자료·실시간 권한을 만들지 않는다."""
from __future__ import annotations

import copy
import math
from pathlib import Path

from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.batch_observation import require
from learning.development_gripper_observer import DevelopmentGripperObserver, decode, sha

SCHEMA = 'motionbrain.development_gripper_replay.v1'
ROOT = Path(__file__).resolve().parents[1]


def _metrics(rows):
    """반환하지 못한 좌표를 0오차로 세지 않고 가림점 오검출도 별도로 센다."""
    object_errors, corner_errors = [], []
    expected_objects = expected_corners = missing_corners = false_corners = 0
    for prediction, label in rows:
        expected_objects += 1
        box = label['visiblePinkRegionBoxXYXY']
        center = [(box[0]+box[2])/2, (box[1]+box[3])/2]
        point = prediction['object']['center']
        if point is not None:
            object_errors.append(math.dist(point, center))
        for name, key in [('jawLeft', 'jawLeftDistalInnerCorner'), ('jawRight', 'jawRightDistalInnerCorner')]:
            expected, actual = label[key]['xy'], prediction['jaws'][name]['point']
            if expected is None:
                missing_corners += 1
                false_corners += int(actual is not None)
            else:
                expected_corners += 1
                if actual is not None:
                    corner_errors.append(math.dist(actual, expected))
    def errors(values):
        return dict(returnedCoordinates=len(values), meanEuclideanPx=sum(values)/len(values) if values else None,
                    maxEuclideanPx=max(values) if values else None)
    return dict(labeledFrames=len(rows), object=dict(expected=expected_objects, **errors(object_errors)),
                corners=dict(expectedVisible=expected_corners, expectedUnavailable=missing_corners,
                             falseVisibleOnUnavailable=false_corners, **errors(corner_errors)),
                meaning='same-source development label agreement; conditional error is not accuracy or holdout')


def run_replay(manifest_path, output_dir, *, root=ROOT):
    private_root = (Path(root)/'.local').resolve()
    output = Path(output_dir).resolve()
    require(output.is_relative_to(private_root) and output != private_root, 'private_output_required')
    manifest_raw = Path(manifest_path).read_bytes()
    spec = _decode_json(manifest_raw)
    require(isinstance(spec, dict) and set(spec) ==
            {'schemaVersion', 'sourceRoot', 'context', 'references', 'frames', 'annotations'}, 'replay_fields')
    require(spec['schemaVersion'] == SCHEMA, 'replay_schema')
    source = Path(spec['sourceRoot']).resolve()
    require(source.is_relative_to(private_root) and source != private_root, 'private_source_required')
    require(isinstance(spec['frames'], list) and 1 <= len(spec['frames']) <= 1000, 'replay_frame_budget')
    require(all(isinstance(f, dict) and set(f) == {'frameId', 'path', 'sha256'} and
                isinstance(f['frameId'], str) and f['frameId'] for f in spec['frames']), 'replay_frame_fields')
    require(len({f['frameId'] for f in spec['frames']}) == len(spec['frames']) and
            len({f['path'] for f in spec['frames']}) == len(spec['frames']), 'replay_duplicate_frame')
    require(isinstance(spec['references'], list) and 1 <= len(spec['references']) <= 8, 'replay_references')
    references = []
    for binding in spec['references']:
        require(isinstance(binding, dict) and set(binding) == {'id', 'path', 'sha256', 'points', 'rois'},
                'replay_reference_fields')
        raw = _read(source, binding['path'])
        require(sha(raw) == binding['sha256'], 'replay_reference_hash')
        references.append(dict(id=binding['id'], imageBytes=raw, imageSha256=binding['sha256'],
                               points=binding['points'], rois=binding['rois']))
    observer = DevelopmentGripperObserver(spec['context'], references)
    binding = spec['annotations']
    require(isinstance(binding, dict) and set(binding) == {'path', 'sha256'}, 'replay_annotation_binding')
    annotation_raw = _read(source, binding['path'])
    require(sha(annotation_raw) == binding['sha256'], 'replay_annotation_hash')
    annotation_document = _decode_json(annotation_raw)
    labels = annotation_document['frames']
    by_path = {}
    for label in labels:
        image_binding = label['imageBinding']
        require(image_binding['path'] not in by_path, 'replay_duplicate_annotation')
        by_path[image_binding['path']] = label
    # 고정 좌표 기준도 명령이나 실제 출력은 참조하지 않는다.
    seed = references[0]
    seed_label = by_path.get(spec['references'][0]['path'])
    require(seed_label is not None, 'replay_baseline_seed_label_required')
    seed_image = decode(seed['imageBytes'], observer.parameters['imageShape'])
    require(seed_label['imageBinding']['sha256'] == seed['imageSha256'] and
            seed_label['imageBinding']['decodedRgbSha256'] == sha(seed_image[:, :, ::-1].tobytes()),
            'replay_baseline_seed_label_binding')
    box = seed_label['visiblePinkRegionBoxXYXY']
    baseline = dict(object=dict(center=[(box[0]+box[2])/2, (box[1]+box[3])/2]),
                    jaws={name: dict(point=copy.deepcopy(seed['points'][name])) for name in ('jawLeft', 'jawRight')})
    predictions, compared, baseline_rows = [], [], []
    for frame in spec['frames']:
        raw = _read(source, frame['path'])
        require(sha(raw) == frame['sha256'], 'replay_image_hash')
        result = observer.observe(raw, frame['frameId'], spec['context'], image_sha256=frame['sha256'])
        result['path'] = frame['path']
        predictions.append(result)
        if frame['path'] in by_path:
            label = by_path[frame['path']]
            require(label['imageBinding']['sha256'] == frame['sha256'] and
                    label['imageBinding']['decodedRgbSha256'] == result['decodedRgbSha256'],
                    'replay_label_image_changed')
            compared.append((result, label))
            baseline_rows.append((baseline, label))
    nonseed = [(prediction, label) for prediction, label in compared if not prediction['referenceImageSelfReplay']]
    nonseed_baseline = [(baseline, label) for _, label in nonseed]
    report = dict(schemaVersion=SCHEMA, manifestSha256=sha(manifest_raw),
        annotationSha256=sha(annotation_raw),
        labelProvenance=annotation_document.get('provenance', 'caller supplied development labels; independence not established'),
        configurationSha256=observer.binding, context=spec['context'],
        kind='development_replay_not_frozen_evaluation', frames=len(predictions),
        uniqueImageHashes=len({p['imageSha256'] for p in predictions}),
        completeFrames=sum(p['status'] == 'complete_development' for p in predictions),
        referenceSelfReplayFrames=sum(p['referenceImageSelfReplay'] for p in predictions),
        boundaryObjectRejections=sum(p['object'].get('reason') == 'color_at_observation_boundary' for p in predictions),
        labelAgreement=_metrics(compared), fixedCoordinateBaseline=_metrics(baseline_rows),
        nonSeedLabelAgreement=_metrics(nonseed), nonSeedFixedCoordinateBaseline=_metrics(nonseed_baseline),
        perFrame=predictions, hardwareRequests=0, motorCommands=0,
        actionModelTrainingPerformed=False, trainingEligible=False, controlEligible=False,
        liveAdmissionVerified=False, measurementAccuracyVerified=False,
        limitations=['same take, manually selected reference patches; no independent fresh-image evaluation',
            'reference self replay is listed separately; nonseed images are still development data',
            'conditional coordinate error must be read with returned/missing/false-visible counts',
            'fixed coordinate baseline is for observation only; command-only baseline belongs to action learning',
            'attached camera/object co-motion does not measure lifting height',
            'no automatic registered-observer integration or hardware dispatch'])
    store = ArtifactStore(output)
    store.write_json('replay-intent.json', dict(manifestSha256=sha(manifest_raw), hardwareRequests=0, motorCommands=0))
    store.write_json('report.json', report)
    return report
