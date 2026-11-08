#!/usr/bin/env python3
"""검증된 고정 평가를 재학습·실물 실행·업로드 없이 로컬 Viewer용으로 준비한다."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from learning.robot_visualization import (load_lift_evaluation, public_metrics,
    export_evaluation_rerun, generate_models, restricted_wandb_settings, portal_html)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--experiment', type=Path)
    mode.add_argument('--models-only', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--physics-test-object', action='store_true', help='로봇과 별개인 낙하 진단용 초록 상자 추가')
    args = p.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'.local'):
        p.error('원본 영상이 포함되므로 출력은 .local 아래여야 합니다')
    if args.models_only:
        output.mkdir(parents=True, exist_ok=False)
        generate_models(ROOT/'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf', output, include_test_object=args.physics_test_object)
        print(json.dumps({'models':str(output),'simulationCalibrated':False}))
        return
    data = load_lift_evaluation(args.experiment)
    output.mkdir(parents=True, exist_ok=False)
    generate_models(ROOT/'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf', output, include_test_object=args.physics_test_object)
    export_evaluation_rerun(data, output/'evaluation.rrd')
    # Publishable preview contains allowlisted scalars, never automatic uploads.
    metrics = public_metrics(data)
    (output/'public-metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
    (output/'media').mkdir()
    cases = []
    for i, case in enumerate(data['cases']):
        cleaned = {k: case[k] for k in ('label','status','before','after','predicted','commandOnly','pulse','modelSelected')}
        cleaned['images'] = {}
        for phase, cameras in case['images'].items():
            cleaned['images'][phase] = {}
            for camera, source in cameras.items():
                name = f'{i}-{phase}-{camera}{source.suffix}'
                shutil.copyfile(source, output/'media'/name)
                cleaned['images'][phase][camera] = 'media/'+name
        cases.append(cleaned)
    preview = dict(modelSha256=data['modelSha256'], trainingRows=data['trainingRows'],
                   evaluated=data['evaluated'], metrics=data['metrics'], cases=cases,
                   limitations=data['limitations'], simulationCalibrated=False,
                   replayTimeline='before/after 순번; 실제 시간 간격·동시 촬영을 뜻하지 않음',
                   wandbMode='offline', cloudUploaded=False)
    (output/'experiment.json').write_text(json.dumps(preview, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    os.environ['WANDB_MODE'] = 'offline'
    os.environ['WANDB_ERROR_REPORTING'] = 'false'
    import wandb
    run = wandb.init(project='motionbrain-fixed-evaluation', mode='offline', dir=str(output),
                     config={'model_sha256':data['modelSha256'], 'training_rows':data['trainingRows'],
                             'evaluation_rows':data['evaluated'], 'source':'historical_fixed_evaluation',
                             'retraining':False, 'physical_control_success_established':False},
                     settings=restricted_wandb_settings(wandb))
    try:
        run.log(metrics)
        run.log({'evaluation': wandb.Table(columns=['case','observed_rise_px','predicted_rise_px','absolute_error_px','model_selected'],
            data=[[i+1,c['after'][0],c['predicted'][0],abs(c['after'][0]-c['predicted'][0]),c['modelSelected']] for i,c in enumerate(cases)])})
        (output/'wandb-run.json').write_text(json.dumps({'mode':'offline','runId':run.id,'cloudUploaded':False})+'\n')
    finally:
        run.finish()
    html=portal_html(Path(__file__).with_name('index.html').read_text(),sys.executable,output.relative_to(ROOT)/'model.xml')
    (output/'index.html').write_text(html)
    print(json.dumps({'prepared':str(output),'evaluated':len(cases),'retrained':False,'cloudUploaded':False}))


if __name__ == '__main__':
    main()
