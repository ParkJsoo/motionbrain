"""새 복구 코호트의 두 영상 입력→명령별 근접 영상 특징 변화 개발 모델.

근접 silhouette 폭은 실제 gap이 아니다. 누락 OBS3 좌표를 보간하지 않으며,
새 관측 계약을 기존 frozen7에 입력하지 않는다. 독립 평가·실물 성과는 별도다.
"""
import copy
import hashlib
import json
import math
import numpy as np
from learning import limited_scene_cycle_assessment as image
from learning import gripper_recovery_observer as near_observer

SCHEMA='motionbrain.dual_image_gripper_recovery_response.v1'
COMMANDS=[dict(joint='gripper',direction=d,percent=70,durationMs=1000) for d in ('open','close')]
FIELDS=['externalObjectMinusProxyXpx','externalObjectMinusProxyYpx','toolAxisImageAngleDeg','M4Deg',*near_observer.STATE_FIELDS]
SCALES=[30.,30.,5.,.5,.15,.10,1.]
POLICY=dict(minimumRowsPerCommand=2,maximumCommands=1,maximumScaledSupportDistance=1.,ridgeAlpha=1.,
            stateScales=SCALES,objective='fixed_training_reference_near_features_not_physical_grasp',
            nearFeatureCostScales=[.15,.10,1.],calibratedUncertainty=False)

def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def require(ok,reason):
 if not ok:raise ValueError(reason)
def fused(external,near,m4):
 image._external(external);near_observer.validate_observation(near)
 require(image._number(m4) and 0<=m4<360,'recovery_M4_invalid')
 point=external['virtualProxyPx'];axis=external['toolAxisImageAngleDeg']
 require(isinstance(point,list) and len(point)==2 and all(image._number(x) for x in point) and image._number(axis),'recovery_external_point_invalid')
 values=[external['object']['center'][i]-point[i] for i in range(2)]+[axis,float(m4),*near['state']]
 return dict(schemaVersion=SCHEMA+'.input',stateFields=FIELDS.copy(),state=values,external=copy.deepcopy(external),
  near=copy.deepcopy(near),m4Deg=float(m4),externalContractSha256=external['registrationSha256'],nearContractSha256=near['observationContractSha256'])
def validate_input(value):
 require(isinstance(value,dict) and value==fused(value.get('external'),value.get('near'),value.get('m4Deg')),'recovery_fused_input_changed')
 return np.asarray(value['state'],dtype=float)
def _command(command):
 require(isinstance(command,dict) and set(command)=={'joint','direction','percent','durationMs'} and command in COMMANDS
  and type(command['percent']) is type(command['durationMs']) is int,'recovery_exact_command_required')
 return command['direction']
def fit(rows,*,goal,audit_sources):
 require(isinstance(rows,list) and bool(rows) and callable(audit_sources),'recovery_raw_source_audit_required')
 require(audit_sources(copy.deepcopy(rows))==rows,'recovery_source_reaudit_changed')
 goal_state=validate_input(goal);identity=(goal['externalContractSha256'],goal['nearContractSha256'])
 sessions=set();sources=set()
 for row in rows:
  require(isinstance(row,dict) and isinstance(row.get('sessionId'),str) and row['sessionId'] not in sessions
   and image._sha(row.get('sourceSha256')) and row['sourceSha256'] not in sources,'recovery_distinct_sources_required')
  _command(row['command'])
  for name in ('before','after'):
   validate_input(row[name]);require((row[name]['externalContractSha256'],row[name]['nearContractSha256'])==identity,'recovery_source_contract_changed')
  sessions.add(row['sessionId']);sources.add(row['sourceSha256'])
 require(any(row['after']==goal and row['command']['direction']=='open' for row in rows),'recovery_goal_from_training_OPEN_required')
 entries=[];deficits=[]
 for direction in ('open','close'):
  subset=[r for r in rows if r['command']['direction']==direction]
  if len(subset)<2:deficits.append(dict(direction=direction,rows=len(subset)));continue
  before=np.asarray([r['before']['state'] for r in subset]);after=np.asarray([r['after']['state'][-3:] for r in subset])
  mean=before.mean(axis=0);z=(before-mean)/np.asarray(SCALES);delta=after-before[:,-3:];dm=delta.mean(axis=0)
  coefficients=np.linalg.solve(z.T@z+np.eye(len(FIELDS)),z.T@(delta-dm))
  entries.append(dict(command=copy.deepcopy(subset[0]['command']),rows=len(subset),beforeStates=before.tolist(),meanState=mean.tolist(),
    deltaMean=dm.tolist(),coefficients=coefficients.tolist(),sourceSha256s=[r['sourceSha256'] for r in subset]))
 require(any(e['command']['direction']=='open' for e in entries),'recovery_two_OPEN_sources_required')
 return dict(schemaVersion=SCHEMA,policy=copy.deepcopy(POLICY),stateFields=FIELDS.copy(),models=entries,deficits=deficits,
   goal=copy.deepcopy(goal),goalNearState=goal_state[-3:].tolist(),externalContractSha256=identity[0],nearContractSha256=identity[1],
   sourceRows=copy.deepcopy(rows),independentEvaluation=False,controlEligible=False,developmentOnly=True,
   calibratedUncertainty=False,physicalGraspVerified=False,oldModelsChanged=False)
def _validate(model):
 require(isinstance(model,dict) and model.get('schemaVersion')==SCHEMA and model.get('policy')==POLICY
  and model.get('stateFields')==FIELDS and model.get('controlEligible') is False and model.get('developmentOnly') is True,'recovery_frozen_development_model_required')
 require(validate_input(model['goal'])[-3:].tolist()==model['goalNearState'],'recovery_goal_changed')
def predict(model,current,command):
 _validate(model);state=validate_input(current);direction=_command(command)
 require(current['externalContractSha256']==model['externalContractSha256'] and current['nearContractSha256']==model['nearContractSha256'],'recovery_current_contract_changed')
 entries=[e for e in model['models'] if e['command']['direction']==direction];require(len(entries)==1,'recovery_command_not_fitted')
 e=entries[0];distances=np.max(np.abs(np.asarray(e['beforeStates'])-state)/np.asarray(SCALES),axis=1)
 require(float(min(distances))<=1.,'recovery_current_outside_local_support')
 baseline=state[-3:]+np.asarray(e['deltaMean']);predicted=baseline+(state-np.asarray(e['meanState']))/np.asarray(SCALES)@np.asarray(e['coefficients'])
 require(predicted.shape==(3,) and np.isfinite(predicted).all() and np.all((predicted>=0)&(predicted<=1)),'recovery_predicted_features_outside_bounds')
 return dict(predictedNearState=predicted.tolist(),commandMeanNearState=baseline.tolist(),supportDistance=float(min(distances)),
   visibilityScoreMeaning='uncalibrated_continuous_feature_prediction_not_probability',calibratedUncertainty=False)
def cost(state,goal):return float(np.linalg.norm((np.asarray(state)-np.asarray(goal))/np.asarray(POLICY['nearFeatureCostScales'])))
def select(model,current,*,commands_used):
 _validate(model);state=validate_input(current);require(type(commands_used) is int and commands_used in (0,1),'recovery_single_command_budget')
 stop=dict(kind='stop',command=None,error=cost(state[-3:],model['goalNearState']));candidates=[stop];rejected=[]
 if commands_used==0:
  for command in COMMANDS:
   try:
    result=predict(model,current,command);candidates.append(dict(kind='action',command=copy.deepcopy(command),error=cost(result['predictedNearState'],model['goalNearState']),**result))
   except ValueError as e:rejected.append(dict(command=copy.deepcopy(command),reason=str(e)))
 candidates.sort(key=lambda c:c['error'])
 return dict(schemaVersion=SCHEMA+'.decision',modelSha256=digest(model),inputSha256=digest(current),selected=copy.deepcopy(candidates[0]),
  candidates=candidates,rejected=rejected,commandsUsed=commands_used,maximumCommands=1,
  selectionOwner='budget_guard' if commands_used else 'admission_guard' if len(candidates)==1 else 'learned_response_comparison',
  modelPredictionPerformed=len(candidates)>1,
  modelSelectedStop=commands_used==0 and len(candidates)>1 and candidates[0]['kind']=='stop',goalNearState=model['goalNearState'].copy(),
  physicalGraspVerified=False,recoveryCompleted=False)
