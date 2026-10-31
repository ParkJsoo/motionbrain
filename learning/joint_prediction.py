"""Small projected-angle predictor; train-only scaling and grouped holdouts.

Pure numerical functions. No physical capture, training CLI, upload or control.
"""
import math
import numpy as np
from learning.joint_collection import conditions, SPLITS

INPUT_NAMES=('angle_deg','velocity_deg_s','direction','duration_s','signed_duration_s','signed_angle_deg')
ALPHAS=(0., .1, 1., 10.)
HORIZON_SECONDS=4.


def features(angle, velocity, direction, duration_ms):
    if direction not in ('up','down') or duration_ms not in (700,900):raise ValueError('unknown_target_command')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (angle,velocity)):raise ValueError('finite_state_required')
    sign=1 if direction=='up' else -1;duration=duration_ms/1000
    return [angle,velocity,sign,duration,sign*duration,sign*angle]


def validate_rows(rows):
    if not isinstance(rows,list) or len(rows)!=32:raise ValueError('32_audited_targets_required')
    session_ids={};seen=set()
    for r in rows:
        i=r['sessionIndex']
        if type(i) is not int or not 0<=i<4 or r['split']!=SPLITS[i]:raise ValueError('locked_split_mismatch')
        if not isinstance(r['sessionId'],str) or not r['sessionId']:raise ValueError('session_id_required')
        if i in session_ids and session_ids[i]!=r['sessionId']:raise ValueError('session_identity_changed')
        session_ids[i]=r['sessionId'];key=(i,r['conditionId'])
        if key in seen or r['conditionId'] not in {c['conditionId'] for c in conditions(i)}:raise ValueError('duplicate_or_unknown_condition')
        seen.add(key)
        if len(r['x'])!=6 or not np.isfinite(np.asarray(r['x'],dtype=float)).all() or not math.isfinite(r['y']):raise ValueError('finite_features_and_label_required')
        c=next(c for c in conditions(i) if c['conditionId']==r['conditionId'])
        if r['x']!=features(r['x'][0],r['x'][1],c['direction'],c['durationMs']):raise ValueError('feature_contract_mismatch')
    if len(set(session_ids.values()))!=4:raise ValueError('session_leakage')
    if any(sum(r['sessionIndex']==i for r in rows)!=8 for i in range(4)):raise ValueError('balanced_sessions_required')


def fit(x,y,alpha):
    x=np.asarray(x,dtype=float);y=np.asarray(y,dtype=float)
    if x.ndim!=2 or x.shape[1]!=6 or y.shape!=(len(x),) or len(x)<7 or not np.isfinite(x).all() or not np.isfinite(y).all() or alpha not in ALPHAS:raise ValueError('invalid_fit_data')
    mean=x.mean(axis=0);scale=x.std(axis=0);scale[scale<1e-9]=1
    z=(x-mean)/scale;centre=float(y.mean());ridge=np.sqrt(alpha)*np.eye(6)
    coef=np.linalg.lstsq(np.vstack([z,ridge]),np.r_[y-centre,np.zeros(6)],rcond=None)[0]
    return dict(mean=mean.tolist(),scale=scale.tolist(),coefficients=coef.tolist(),intercept=centre,alpha=alpha,fitRows=len(x))


def predict(model,x):
    x=np.asarray(x,dtype=float)
    return ((x-np.asarray(model['mean']))/np.asarray(model['scale']))@np.asarray(model['coefficients'])+model['intercept']


def scores(y,p):
    y=np.asarray(y);p=np.asarray(p);e=p-y;moving=np.abs(y)>.1
    return dict(n=len(y),maeDeg=float(np.abs(e).mean()),rmseDeg=float(np.sqrt((e*e).mean())),biasDeg=float(e.mean()),
                movingCount=int(moving.sum()),directionCorrect=int(((np.sign(y)==np.sign(p))&moving).sum()),
                directionAccuracy=float((np.sign(y[moving])==np.sign(p[moving])).mean()) if moving.any() else None)


def train_and_evaluate(rows):
    validate_rows(rows)
    groups={label:[r for r in rows if r['split']==label] for label in ('train','validation','test')}
    xs={s:np.asarray([r['x'] for r in rr]) for s,rr in groups.items()};ys={s:np.asarray([r['y'] for r in rr]) for s,rr in groups.items()}
    candidates=[]
    for alpha in ALPHAS:
        model=fit(xs['train'],ys['train'],alpha)
        candidates.append(dict(model=model,validation=scores(ys['validation'],predict(model,xs['validation']))))
    selected=min(candidates,key=lambda c:(c['validation']['maeDeg'],c['model']['alpha']))['model']
    results={}
    for split in groups:
        y=ys[split];x=xs[split];pred=predict(selected,x)
        results[split]=dict(selected=scores(y,pred),noChange=scores(y,np.zeros(len(y))),constantVelocity=scores(y,x[:,1]*HORIZON_SECONDS),
             predictions=[dict(sessionId=r['sessionId'],conditionId=r['conditionId'],actualDeltaDeg=float(v),predictedDeltaDeg=float(w)) for r,v,w in zip(groups[split],y,pred)])
    return dict(schemaVersion='motionbrain.projected_joint_prediction.v1',inputNames=list(INPUT_NAMES),horizonSeconds=HORIZON_SECONDS,
                selectedModel=selected,candidates=candidates,splits=results,controlEligible=False,
                independentDayOrViewGeneralization=False,physicalJointAngleCalibrated=False)
