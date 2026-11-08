#!/usr/bin/env python3
"""기본은 게시 지표 미리보기. --publish는 별도 승인·정상 로그인 후에만 사용한다."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from learning.robot_visualization import load_lift_evaluation,public_metrics,restricted_wandb_settings,publication_payload,publication_digest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--experiment',type=Path,required=True)
    p.add_argument('--project',default='motionbrain-fixed-evaluation')
    p.add_argument('--entity')
    p.add_argument('--publish',action='store_true')
    p.add_argument('--receipt',type=Path)
    a=p.parse_args()
    data=load_lift_evaluation(a.experiment)
    metrics=public_metrics(data)
    rows=[[i+1,c['after'][0],c['predicted'][0],abs(c['after'][0]-c['predicted'][0]),c['modelSelected']]
          for i,c in enumerate(data['cases'])]
    config={'model_sha256':data['modelSha256'],'source':'historical_fixed_evaluation',
            'retraining':False,'physical_control_success_established':False}
    if not a.publish:
        print(json.dumps({'upload':False,'project':a.project,'config':config,'metrics':metrics,
                          'evaluationRows':rows,'rawMediaIncluded':False},ensure_ascii=False,indent=2,allow_nan=False))
        return
    if a.receipt and (not a.receipt.resolve().is_relative_to(ROOT/'.local') or a.receipt.exists()):
        p.error('게시 영수증은 .local 아래의 새 파일이어야 합니다')
    if a.receipt:a.receipt.parent.mkdir(parents=True,exist_ok=True)
    import wandb
    out=ROOT/'.local/visual-tools/published-wandb';out.mkdir(parents=True,exist_ok=True)
    run=wandb.init(project=a.project,entity=a.entity,mode='online',dir=str(out),config=config,
                   settings=restricted_wandb_settings(wandb))
    try:
        run.log(metrics)
        run.log({'evaluation':wandb.Table(columns=['case','observed_rise_px','predicted_rise_px',
                                                  'absolute_error_px','model_selected'],data=rows)})
    finally:run.finish()
    remote=wandb.Api().run(f'{run.entity}/{run.project}/{run.id}')
    if not all(abs(remote.summary[k]-v)<1e-9 for k,v in metrics.items()):
        raise ValueError('published_metric_verification_failed')
    payload=publication_payload(data)
    receipt=dict(payload,url=remote.url,runId=remote.id,scalarMetricsVerified=len(metrics),rawMediaIncluded=False,payloadSha256=publication_digest(payload))
    if a.receipt:a.receipt.write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt))


if __name__=='__main__':main()
