#!/usr/bin/env python3
"""Fixed synthetic mathematics smoke only; never writes a physical model artifact."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from learning.artifacts import ArtifactStore
from learning.features import transition_features, FEATURE_VERSION
from learning.models import fit_linear, compare
from learning.splits import assign_splits, validate_splits, LABELS
from learning.visual_dynamics import digest, source_hashes

SEED=20261015


def smoke_data():
    rng=np.random.default_rng(SEED)
    rows,features,targets=[],[],[]
    for i in range(120):
        pwm=int(rng.choice([-90,-60,-30,30,60,90]))
        x,y=float(rng.uniform(70,200)),float(rng.uniform(40,140))
        angle=float(rng.uniform(150,250))
        def observation(phase):
            return {'synthetic':True,'sourceFrameId':f'synthetic-frame-{i}-{phase}',
                    'detectionSourceFrameId':f'synthetic-frame-{i}-{phase}',
                    'detected':True,'liveDetected':True,'held':False,'label':'synthetic_target','classId':0,
                    'width':320,'height':240,'targetBox':{'x':x+phase*pwm/100,'y':y+phase*pwm/200,'width':30,'height':40},
                    'm4':{'motor':4,'sensor':'AS5600','measured':True,'synthetic':True,
                          'angleDegrees':angle+phase*pwm/50}}
        transition={'synthetic':True,'before':observation(0),'after':observation(1),
                    'action':{'request':{'motor':1,'kind':'pulse','pwm':pwm,'durationMs':100,
                                         'targetDegrees':0,'percent':0}}}
        a,b=transition_features(transition); features.append(a); targets.append(b)
        rows.append({'sampleId':f'synthetic-{i}','sessionId':f'synthetic-session-{i}',
                     'episodeId':f'synthetic-episode-{i}','transitionId':f'synthetic-transition-{i}',
                     'actionIdentity':{'bootId':f'synthetic-boot-{i}','commandId':f'synthetic-command-{i}'},
                     'images':[{'sourceFrameId':f'synthetic-frame-{i}-{phase}',
                                'sha256':hashlib.sha256(f'synthetic-identity-{i}-{phase}'.encode()).hexdigest()}
                               for phase in (0,1)]})
    assignments=assign_splits(rows,seed=str(SEED))
    if validate_splits(rows,assignments): raise ValueError('smoke split leakage')
    x,y=np.asarray(features),np.asarray(targets)
    data={}
    for label in LABELS:
        mask=[assignments[r['sampleId']]==label for r in rows]
        data[label]={'x':x[mask],'y':y[mask],'sampleIds':[r['sampleId'] for r,m in zip(rows,mask) if m]}
    return data,assignments


def run_smoke():
    data,assignments=smoke_data()
    model=fit_linear(data['train']['x'],data['train']['y'])
    return {'schemaVersion':'motionbrain.synthetic_dynamics_smoke.v1','evidence':'synthetic_math_only',
            'seed':SEED,'featureVersion':FEATURE_VERSION,'splitSha256':digest(assignments),
            'sourceSha256':source_hashes(),'numpyVersion':np.__version__,
            'fixtureSha256':digest({k:{'x':v['x'].tolist(),'y':v['y'].tolist(),'sampleIds':v['sampleIds']}
                                   for k,v in data.items()}),
            'model':model,'splits':{k:compare(model,v['x'],v['y']) for k,v in data.items()},
            'trainingEligible':False,'controlEligible':False,'physicalManipulationImprovementEstablished':False}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args(argv)
    result=run_smoke()
    ArtifactStore(args.output_dir).write_json('synthetic-smoke.json',result)
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__=='__main__': raise SystemExit(main())
