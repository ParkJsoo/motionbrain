#!/usr/bin/env python3
"""저장된 M1 shaft campaign 원본·사전 예측을 검증해 Rerun으로 표시한다."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import urllib.parse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from learning import m1_shaft_learning as learning
from learning.artifacts import _read, StorageError
from learning.gripper_camera_dataset import read_frame
from tools.visualization.pipeline import _frame, require


def _load(path): return json.loads(path.read_bytes())
def _sha(raw): return hashlib.sha256(raw).hexdigest()


def _raw_observation(observed):
    annotation = observed['annotation']
    require(annotation['pixelSha256'] == observed['pixelSha256'], 'raw_annotation_pixels_changed')
    return dict(observed, frameId=annotation['frameId'])


def _plan_records(runtime, plan):
    plan_sha = learning.digest(plan); anchored = False; saved = {}
    for path in sorted(runtime.glob('step-*.json')):
        record = _load(path); index = record['step']
        require(type(index) is int and 1 <= index <= len(plan['steps'])
                and path.name == f'step-{index:02d}.json', 'recorded_step_outside_plan')
        stage = plan['steps'][index-1]
        require(record['phase'] == stage['phase'], 'recorded_phase_changed')
        request = record.get('collection', {}).get('request')
        if request:
            require(request['approvalPlanSha256'] == plan_sha, 'executed_plan_hash_changed')
            require(request['command'] == stage['command'], 'planned_command_changed'); anchored = True
        for evidence in (record.get('prospective'), record.get('transition'),
                         record.get('rawAudit', {}).get('visual', {}).get('row', {}).get('inputs')):
            if evidence: require(evidence['command'] == stage['command'], 'recorded_command_changed')
        if record['status'] == 'skipped':
            require(record['phase'] == 'preparation' and stage.get('optional') is True
                    and not record.get('collection'), 'invalid_skipped_preparation')
        elif record['status'] not in ('halted_no_retry',): require(request is not None, 'executed_plan_anchor_missing')
        saved[path.name] = record
    return saved, plan_sha, anchored


def _prospective(record, model, model_sha, frames):
    """저장된 True를 신뢰하지 않고 사전 입력·시간·고정 모델을 재검증한다."""
    p, row, audit = record['prospective'], record['transition'], record['rawAudit']
    before = record['observation']['perFrame']['before']
    callback = _raw_observation(p['observation']); bound = p['frame']['frame']
    require(record['phase'] == 'evaluation' and record['status'] == 'completed', 'completed_evaluation_required')
    require(all(frames[s]['status'] == 'verified' for s in ('before', 'after')), 'verified_pair_required')
    require(audit['visual']['status'] == 'verified' and audit['execution']['status'] == 'verified'
            and audit['execution']['offVerified'] is True, 'verified_raw_execution_required')
    require(p['preparedAt'] < audit['visual']['row']['evidence']['commandAt'], 'prediction_not_before_command')
    require(p['modelSha256'] == model_sha and p['command'] == row['command']
            and p['strictRejection'] is None, 'frozen_prediction_binding_changed')
    require(row['sourceHash'] == audit['source']['sha256'], 'transition_source_changed')
    raw = audit['visual']['row']
    require(row['sessionId'] == raw['sessionId'] and row['command'] == raw['inputs']['command']
            and row['episode'] == [raw['evidence']['controllerBootId'], raw['evidence']['commandId']], 'raw_transition_identity_changed')
    require(before['frameId'] == bound['frameId'] == callback['frameId'] == row['frameIds'][0]
            and before['pixelSha256'] == bound['pixelSha256'] == callback['pixelSha256'], 'before_identity_changed')
    x = row['beforeShaftPx']
    require(x == p['shaftSeparationPx'] == before['shaftSeparationPx'] == callback['state'][0]
            and x == callback['shaftSeparationPx'], 'callback_measurement_changed')
    require(record['observation']['perFrame']['after']['shaftSeparationPx'] == row['afterShaftPx']
            and record['observation']['perFrame']['after']['frameId'] == row['frameIds'][1], 'after_measurement_changed')
    require(p['shaftRidge'] == learning.predict(model, x, row['command'], row['context'])
            and p['commandMean'] == learning.predict(model, x, row['command'], row['context'], baseline=True)
            and p['noChange'] == x, 'frozen_prediction_changed')
    check = learning.evaluate(model, [row])
    require(check['evaluated'] == 1, 'separate_evaluation_required')
    return dict(prospectiveVerified=True, predictions=check['predictions'], metrics=check['metrics'])


def load_view(runtime):
    runtime = Path(runtime).resolve(strict=True)
    require(runtime.is_relative_to((ROOT/'.local').resolve()), 'private_runtime_required')
    plan = _load(runtime/'plan.json'); model_path = runtime/'frozen-model.json'
    saved, plan_sha, anchored = _plan_records(runtime, plan)
    model = learning.validate_model(_load(model_path)) if model_path.exists() else None
    model_sha = _sha(model_path.read_bytes()) if model else None
    if model: require(model_sha == learning.digest(model), 'canonical_frozen_model_required')
    sources = [(p, 'seed_development_reobservation') for p in sorted(runtime.glob('seed-*.json'))]
    sources += [(runtime/f'step-{i:02d}.json', stage['phase']) for i, stage in enumerate(plan['steps'], 1)]
    records = []
    for path, phase in sources:
        record = saved.get(path.name, _load(path) if path.exists() else dict(status='pending_no_record', phase=phase))
        item = dict(file=path.name, phase=phase, status=record.get('status', record.get('observation', {}).get('status')),
            reason=record.get('reason'), frames={}, prospectiveVerified=False, objectObservation='not_required / not_observed',
            sourceArtifactSha256=_sha(path.read_bytes()) if path.exists() else None,
            prospective=record.get('prospective'), evaluation=record.get('evaluation'), modelSha256=model_sha,
            transition=record.get('transition', record.get('observation', {}).get('transition')))
        try:
            audit = record.get('rawAudit', record.get('audit'))
            if audit:
                source = Path(audit['source']['path']).resolve(); sid = audit['visual']['row']['sessionId']
                require(source.is_relative_to((ROOT/'.local').resolve()), 'private_source_required')
                require(_sha(_read(source, 'session.json')) == audit['source']['sha256'], 'session_source_changed')
                observed = record.get('observation', {}).get('perFrame', {})
                if observed: require(record['observation']['sourceHash'] == audit['source']['sha256'], 'observation_source_changed')
                pair = source/'gripper-pair'
                if observed: pair = Path(next(iter(observed.values()))['imageBinding']['path']).parent
                require(pair.resolve().is_relative_to((ROOT/'.local').resolve()), 'private_pair_required')
                for side in ('before', 'after'):
                    frame, _, _ = read_frame(pair, side, sid)
                    observation = copy.deepcopy(observed.get(side))
                    if observation is None:
                        point_source = record.get('prospective', {}).get('observation') if side == 'before' else record.get('afterObservation')
                        observation = _raw_observation(copy.deepcopy(point_source)) if point_source else dict(
                            frameId=frame['frameId'], pixelSha256=frame['pixelSha256'], perPoint={})
                    if observation.get('imageBinding'):
                        require(all(observation['imageBinding'][k] == frame[k] for k in
                                    ('frameId', 'pixelSha256', 'imageSha256', 'width', 'height')), 'original_binding_changed')
                    require(observation['frameId'] == frame['frameId'] and observation['pixelSha256'] == frame['pixelSha256'], 'pair_observation_changed')
                    observation['perPoint'] = {k:v for k,v in observation.get('perPoint', {}).items() if k in ('jawLeft', 'jawRight')}
                    observation['imageBinding'] = dict(frame, path=str(pair/('esp32-'+side+'.jpg')), encoding='jpeg')
                    item['frames'][side] = _frame(source, frame, observation)
            if phase == 'evaluation' and record.get('status') == 'completed':
                require(model is not None, 'frozen_model_missing')
                item.update(_prospective(record, model, model_sha, item['frames']))
                item['evaluationProtocol'] = 'prospective_frozen_model_prediction_verified'
                if item['evaluation']:
                    item['evaluation'] = dict(item['evaluation']); item['evaluation']['offlineScorerLabel'] = item['evaluation'].pop('evaluationKind', None)
        except (ValueError, OSError, KeyError, TypeError, StorageError) as exc:
            item['verificationError'] = str(exc)
        records.append(item)
    evaluations = [r for r in records if r['phase'] == 'evaluation']
    return dict(label='M1 shaft 수동 수집·고정 후 예측 검증; object not_required; 모델 선택·파지·들기 검증 없음',
        records=records, counts=dict(plannedSteps=len(plan['steps']), recordedSteps=sum(r['phase'] != 'seed_development_reobservation' and r['status'] != 'pending_no_record' for r in records),
        listedEvaluation=len(evaluations), pendingEvaluation=sum(r['status'] == 'pending_no_record' for r in evaluations),
        recordedUnverifiedEvaluation=sum(r['status'] != 'pending_no_record' and not r['prospectiveVerified'] for r in evaluations), prospectiveVerified=sum(r['prospectiveVerified'] for r in records)),
        modelSha256=model_sha, planSha256=plan_sha, executedPlanAnchored=anchored,
        executionSourceHashes=plan.get('sourceHashes', {}),
        excludedSources=_load(runtime/'excluded-sources.json') if (runtime/'excluded-sources.json').exists() else [])


def export(runtime, output, *, connect=None, rr_module=None):
    output = Path(output).resolve()
    require(output.is_relative_to((ROOT/'.local').resolve()) and output.suffix == '.rrd', 'private_rrd_output_required')
    require(not output.exists(), 'new_rrd_output_required')
    if connect:
        url = urllib.parse.urlsplit(connect)
        require(url.scheme == 'rerun+http' and url.hostname in ('127.0.0.1', 'localhost')
                and url.username is None and url.password is None, 'explicit_local_rerun_connection_required')
    view = load_view(runtime)
    if rr_module is None:
        import rerun as rr_module
    rr = rr_module; output.parent.mkdir(parents=True, exist_ok=True)
    rec = rr.RecordingStream('motionbrain_m1_shaft_campaign')
    try:
        if connect: rec.set_sinks(rr.FileSink(str(output)), rr.GrpcSink(connect))
        else: rec.save(str(output))
        for index, record in enumerate(view['records']):
            rec.set_time('campaign_record', sequence=index)
            for side in ('before', 'after'):
                path = 'frames/'+side; rec.log(path, rr.Clear(recursive=True)); frame = record['frames'].get(side)
                if frame and frame['status'] == 'verified':
                    rec.log(path, rr.Image(frame['image']))
                    if frame['points']: rec.log(path+'/valid_shaft_points', rr.Points2D([p['point'] for p in frame['points']], labels=[p['name'] for p in frame['points']]))
            summary = dict(record, frames={s:{k:v for k,v in f.items() if k != 'image'} for s,f in record['frames'].items()})
            rec.log('status', rr.TextDocument(view['label']+'\n'+json.dumps(summary, ensure_ascii=False, indent=2)))
        rec.log('campaign', rr.TextDocument(json.dumps({k:v for k,v in view.items() if k != 'records'}, ensure_ascii=False, indent=2)))
        rec.flush()
    finally: rec.disconnect()
    return dict(output=str(output), counts=view['counts'], modelSha256=view['modelSha256'], hardwareRequests=0, motorCommands=0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', required=True, type=Path); parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--connect', help='명시적 localhost Rerun gRPC만 허용')
    args = parser.parse_args(argv)
    print(json.dumps(export(args.runtime, args.output, connect=args.connect), ensure_ascii=False))


if __name__ == '__main__': main()
