#!/usr/bin/env python3
"""검증된 파이프라인 산출물을 Rerun 기록으로 표시한다. 장치 요청·학습·구동 없음."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import urllib.parse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from learning.artifacts import StorageError, _read
from learning.lossless_frame import decode


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def verified_json(root, reference):
    require(isinstance(reference, dict) and isinstance(reference.get('path'), str)
            and isinstance(reference.get('sha256'), str), 'hashed_pipeline_artifact_required')
    try:
        raw = _read(root, reference['path'])
    except StorageError as exc:
        raise ValueError('unsafe_or_unavailable_pipeline_artifact') from exc
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'pipeline_artifact_hash_changed')
    return json.loads(raw)


def native_image(source, frame):
    import numpy as np
    if 'metadataFile' in frame:
        metadata = _read(source/'native-camera', frame['metadataFile'])
        require(hashlib.sha256(metadata).hexdigest() == frame['metadataSha256'],
                'native_metadata_hash_changed')
    raw = _read(source/'native-camera', frame['imageFile'])
    require(hashlib.sha256(raw).hexdigest() == frame['imageSha256'], 'native_image_hash_changed')
    pixels = decode(raw, frame['rawByteLength'], frame['pixelSha256'])
    require(len(pixels) == frame['height']*frame['width']*4, 'native_image_shape_changed')
    image = np.frombuffer(pixels, np.uint8).reshape(frame['height'], frame['width'], 4)
    return image[:, :, [2, 1, 0]].copy()


def binding_image(binding):
    """관측기가 제공한 원본 위치와 해시만 사용한다. 임의 탐색으로 다른 뷰를 찾지 않는다."""
    import cv2
    import numpy as np
    path = Path(binding['path']).resolve(strict=True)
    raw = _read(path.parent, path.name)
    require(hashlib.sha256(raw).hexdigest() == binding['imageSha256'], 'observation_image_hash_changed')
    if binding['encoding'] == 'native_bgra_lossless':
        pixels = decode(raw, binding['rawByteLength'], binding['pixelSha256'])
        require(len(pixels) == binding['height']*binding['width']*4, 'observation_image_shape_changed')
        image = np.frombuffer(pixels, np.uint8).reshape(binding['height'], binding['width'], 4)
        return image[:, :, [2, 1, 0]].copy()
    require(binding['encoding'] == 'jpeg', 'unsupported_observation_image_encoding')
    try:
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    except cv2.error as exc:
        raise ValueError('observation_image_decode_failed') from exc
    require(image is not None and image.shape[:2] == (binding['height'], binding['width']),
            'observation_image_shape_changed')
    require(hashlib.sha256(cv2.cvtColor(image, cv2.COLOR_BGR2BGRA).tobytes()).hexdigest()
            == binding['pixelSha256'], 'observation_pixels_changed')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _points(observation, frame, width, height):
    if observation is None:
        return []
    require(observation.get('frameId') == frame['frameId']
            and observation.get('pixelSha256') == frame['pixelSha256'],
            'landmarks_image_identity_mismatch')
    result = []
    for name, point in observation.get('perPoint', {}).items():
        if point.get('status') != 'valid':
            continue
        xy = point.get('point')
        require(isinstance(xy, list) and len(xy) == 2
                and all(type(v) in (int, float) and math.isfinite(v) for v in xy)
                and 0 <= xy[0] < width and 0 <= xy[1] < height,
                'invalid_display_landmark')
        result.append(dict(name=name, point=xy))
    return result


def _frame(source, frame, observed):
    result = dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
                  status='unavailable', points=[], image=None)
    try:
        binding = observed.get('imageBinding') if observed else None
        if binding is not None:
            require(binding['pixelSha256'] == observed['pixelSha256'], 'observation_binding_identity_mismatch')
            result.update(frameId=observed['frameId'], pixelSha256=observed['pixelSha256'])
            result['image'] = binding_image(binding)
        else:
            result['image'] = native_image(source, frame)
        h, w = result['image'].shape[:2]
        if observed:
            result['points'] = _points(observed, result, w, h)
        result['status'] = 'verified'
    except (ValueError, OSError, KeyError, TypeError, StorageError) as exc:
        # A point from another camera is never overlaid on the external frame.
        result.update(status='error', reason=str(exc), points=[])
    return result


def load_view(latest_path):
    """확인한 파일만 읽어 표시 계획을 만든다. Rerun SDK나 네트워크가 필요 없다."""
    latest = Path(latest_path).resolve(strict=True)
    reference = json.loads(latest.read_bytes())
    output = latest.parent
    stages = output/'artifacts'
    report = verified_json(stages, reference)
    require(report.get('schemaVersion') == 'motionbrain.experiment_pipeline.v1',
            'experiment_pipeline_report_required')
    records = []
    for item in report['sources']:
        evidence = verified_json(stages, item['evidence']) if item.get('evidence') else None
        observation = verified_json(stages, item['observation']) if item.get('observation') else None
        record = dict(id=item['id'], status=item['status'], reason=item.get('reason'),
                      trainingEligible=item.get('trainingEligible', False), frames={},
                      evidence=evidence, observation=observation)
        if evidence:
            require(evidence.get('schemaVersion') == 'motionbrain.pipeline_evidence.v1',
                    'pipeline_evidence_report_required')
            source = Path(evidence['source']['path']).resolve(strict=True)
            raw = _read(source, 'session.json')
            require(hashlib.sha256(raw).hexdigest() == evidence['source']['sha256'],
                    'pipeline_source_session_changed')
            if observation:
                require(observation.get('schemaVersion') == 'motionbrain.pipeline_observations.v1'
                        and observation.get('sourceHash') == evidence['source']['sha256'],
                        'pipeline_observation_source_changed')
            visual = evidence.get('visual', {})
            row = visual.get('row')
            if visual.get('status') == 'verified' and row:
                record['cameraContext'] = row['evidence'].get('cameraIdentity')
                for side, section in [('before', 'inputs'), ('after', 'targets')]:
                    observed = observation.get('perFrame', {}).get(side) if observation else None
                    record['frames'][side] = _frame(source, row[section]['image'], observed)
        records.append(record)
    for model in report.get('frozenModels', []):
        blob = _read(output, model['path'])
        require(hashlib.sha256(blob).hexdigest() == model['sha256'], 'pipeline_frozen_model_changed')
    return dict(reportSha256=reference['sha256'], records=records,
                counts=report['counts'], learning=report['learning'],
                label='developmentReplay — 저장 원본 재생, 실시간 추종·고정 후 실물 평가 아님',
                hardwareRequests=0, motorCommands=0, controlEligible=False)


def _summary(record):
    value = {k:v for k,v in record.items() if k != 'frames'}
    # Full evidence/observations can be large; show stage outcomes and per-point status.
    evidence = value.get('evidence')
    if evidence:
        value['evidence'] = {k:evidence.get(k) for k in
                             ('source', 'originalOutcome', 'execution', 'recording', 'sensor', 'recoveryDevelopmentOnly')}
    observation = value.get('observation')
    if observation:
        value['observation'] = {k:observation.get(k) for k in
                               ('status', 'reason', 'contractSha256', 'recoveryDevelopmentOnly')}
        value['observation']['perFrame'] = {side:{k:frame.get(k) for k in
            ('frameId','pixelSha256','status','reason','perPoint')} for side,frame in observation.get('perFrame',{}).items()}
    value['frames'] = {side:{k:v for k,v in frame.items() if k != 'image'}
                       for side,frame in record['frames'].items()}
    return value


def export(latest_path, output_path, *, connect=None, rr_module=None):
    output = Path(output_path).resolve()
    require(output.is_relative_to((ROOT/'.local').resolve()) and output.suffix == '.rrd', 'private_rrd_output_required')
    require(not output.exists(), 'new_rrd_output_required')
    if connect is not None:
        url = urllib.parse.urlsplit(connect)
        require(url.scheme == 'rerun+http' and url.hostname in ('127.0.0.1', 'localhost')
                and url.username is None and url.password is None,
                'explicit_local_rerun_connection_required')
    view = load_view(latest_path)
    real_sdk = rr_module is None
    if real_sdk:
        import rerun as rr_module
    rr = rr_module
    output.parent.mkdir(parents=True, exist_ok=True)
    rec = rr.RecordingStream('motionbrain_pipeline_replay', recording_id=view['reportSha256'])
    try:
        if connect is None:
            rec.save(str(output))
        else:
            rec.set_sinks(rr.FileSink(str(output)), rr.GrpcSink(connect))
        if real_sdk:
            import rerun.blueprint as bp
            rec.send_blueprint(bp.Blueprint(bp.Vertical(
                bp.Horizontal(bp.Spatial2DView(origin='/frames/before'), bp.Spatial2DView(origin='/frames/after')),
                bp.Horizontal(bp.TextDocumentView(origin='/status'), bp.TextDocumentView(origin='/learning_comparison')))))
        for index, record in enumerate(view['records']):
            rec.set_time('source_index', sequence=index)
            for side in ('before', 'after'):
                path = 'frames/'+side
                rec.log(path, rr.Clear(recursive=True))
                frame = record['frames'].get(side)
                if frame is None:
                    continue
                if frame['image'] is not None:
                    rec.log(path, rr.Image(frame['image']))
                if frame['points']:
                    rec.log(path+'/valid_landmarks', rr.Points2D(
                        [point['point'] for point in frame['points']],
                        labels=[point['name'] for point in frame['points']]))
            rec.log('status', rr.TextDocument(view['label']+'\n\n'+json.dumps(_summary(record), ensure_ascii=False, indent=2)))
        rec.log('learning_comparison', rr.TextDocument(json.dumps(view['learning'], ensure_ascii=False, indent=2)))
        rec.flush()
    finally:
        rec.disconnect()
    return dict(output=str(output), reportSha256=view['reportSha256'],
                sourceCount=len(view['records']), connected=connect is not None,
                hardwareRequests=0, motorCommands=0, controlEligible=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--latest', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--connect', help='명시적으로 지정한 localhost Rerun gRPC에만 연결')
    args = parser.parse_args(argv)
    try:
        output = args.output or args.latest.parent/'pipeline-replay.rrd'
        result = export(args.latest, output, connect=args.connect)
    except (ValueError, OSError, KeyError, TypeError, StorageError, ImportError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
