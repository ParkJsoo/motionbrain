"""직접 관측한 M3 DOWN의 명령별 국소 도구 영상 반응. 장치 접근 없음.

분홍 물체를 요구하지 않는 별도 표식 투영 계약이다. 가상 영상점은 실제 끝점·
깊이·접촉·충돌 여유가 아니며, 단일 원본 Δ 전이는 검증되지 않은 개발 가설이다.
짧은 명령과 긴 명령은 범주로 분리하고 시간 비례·UP 역함수를 사용하지 않는다.
"""
from __future__ import annotations
import copy
import hashlib
import math

import cv2
import numpy as np

from learning import marker_virtual_observer as marker
from learning.near_gripper_tip_observer import digest

SCHEMA='motionbrain.M3_direct_tool_response.v1'
PROJECTION_SCHEMA='motionbrain.marker_tool_only_projection.v1'
OBSERVATION_SCHEMA=PROJECTION_SCHEMA+'.observation'
COMMANDS=[dict(joint='elbow',direction='down',percent=60,durationMs=350),
          dict(joint='elbow',direction='down',percent=70,durationMs=1000)]
SCALES=np.array([180.,180.,15.,20.])


def require(ok,reason):
    if not ok:raise ValueError(reason)
def number(v):return type(v) in (int,float) and math.isfinite(v)
def sha(v):return isinstance(v,str) and len(v)==64 and all(c in '0123456789abcdef' for c in v)
def difference(a,b):
    result=np.array(a,dtype=float)-np.array(b,dtype=float)
    result[...,3]=(result[...,3]+180.)%360.-180.
    return result

def command_key(command):
    require(isinstance(command,dict) and set(command)=={'joint','direction','percent','durationMs'} and
            type(command['percent']) is int and type(command['durationMs']) is int and command in COMMANDS,
            'direct_M3_categorical_DOWN_command_required')
    return (command['percent'],command['durationMs'])

def policy():
    return dict(estimator='nearest_observed_delta_per_exact_command',scales=SCALES.tolist(),
        maximumStandardDistance=1.,maximumM4DifferenceDeg=15.,maximumImageAxisDifferenceDeg=20.,
        supportMeaning='declared_unverified_existing_M3_local_transfer_limits_not_accuracy_or_safety_bounds',
        minimumSourcesPerCommand=1,singletonAllowedAsDevelopmentHypothesis=True,maximumCommands=1,
        timeScalingAllowed=False,inverseUPTrainingAllowed=False,calibratedUncertainty=False,safetyBound=False)


def projection_contract(registration):
    return dict(schemaVersion=PROJECTION_SCHEMA,sourceFullRegistrationSha256=digest(registration),sourceRegistration=copy.deepcopy(registration),
            seedSha256=registration['seedSha256'],imageShape=[1080,1920],
            detectorPolicy=copy.deepcopy(registration['detectorPolicy']),
            basisCoefficients=copy.deepcopy(registration['basisCoefficients']),objectDetectionRequired=False,
            sourceObjectDetectorRetainedForProvenanceOnly=True,
            stateFields=['absoluteVirtualToolXpx','absoluteVirtualToolYpx','reportedM4Deg','markerImageAxisDeg'],
            meaning='same_observed_marker_basis_virtual_point_without_object_detection_not_physical_tip_depth_grasp_or_clearance')


def validate_projection_contract(contract):
    require(isinstance(contract,dict) and isinstance(contract.get('sourceRegistration'),dict),
            'tool_only_explicit_source_registration_required')
    registration=contract['sourceRegistration']
    require(registration.get('schemaVersion')==marker.SCHEMA and registration.get('detectorPolicy')==marker.detector_policy() and
            contract==projection_contract(registration),'tool_only_projection_basis_or_policy_changed')


class ToolOnlyProjector:
    def __init__(self,registration,seed_bytes):
        original=marker.VirtualMarkerObserver(registration,seed_bytes)
        self.registration=copy.deepcopy(original.registration);self.source_binding=original.binding
        self.contract=projection_contract(self.registration)
        self.binding=digest(self.contract)
    def _record(self,pair,black,image_sha,m4,*,source_kind):
        require(digest(self.registration)==self.source_binding and digest(self.contract)==self.binding,
                'tool_only_projection_memory_changed')
        require(sha(image_sha) and number(m4) and 0<=m4<360,'tool_only_image_sensor_binding')
        p=self.registration['detectorPolicy']
        require(isinstance(pair,dict) and all(isinstance(pair.get(c),dict) for c in ('purple','teal')),
                'tool_only_marker_pair_required')
        centers=[]
        for color in ('purple','teal'):
            center=pair[color].get('center')
            require(isinstance(center,list) and len(center)==2 and all(number(v) for v in center),
                    'tool_only_marker_center_required')
            centers.append(np.array(center))
        distance=float(np.linalg.norm(centers[1]-centers[0]))
        require(p['pairDistancePx'][0]<=distance<=p['pairDistancePx'][1] and
                number(pair.get('distancePx')) and abs(distance-pair['distancePx'])<1e-8,
                'tool_only_marker_pair_distance_changed')
        origin,basis=marker._basis(pair);point=origin+basis@np.array(self.registration['basisCoefficients'])
        radius=p['blackWindowRadiusPx']
        require(radius<=point[0]<1920-radius and radius<=point[1]<1080-radius,
                'tool_only_point_outside_image')
        require(number(black) and p['minimumBlackOccupancyFraction']<=black<=1.,'tool_only_dark_support_missing')
        axis=float(np.degrees(np.arctan2(basis[1,0],basis[0,0])))
        return dict(schemaVersion=OBSERVATION_SCHEMA,projectionContractSha256=self.binding,
            sourceFullRegistrationSha256=self.source_binding,imageSha256=image_sha,
            state=[float(point[0]),float(point[1]),float(m4),axis],markerPair=copy.deepcopy(pair),
            blackOccupancyFraction=float(black),sourceKind=source_kind,objectDetectionRequired=False,
            fullObjectObservationClaimed=False,physicalTipOrClearanceVerified=False)
    def observe(self,raw,m4_deg):
        image=marker._image(raw);p=self.registration['detectorPolicy'];pair,_=marker._pair(image,p)
        origin,basis=marker._basis(pair);point=origin+basis@np.array(self.registration['basisCoefficients'])
        radius=p['blackWindowRadiusPx'];x,y=np.rint(point).astype(int)
        require(radius<=x<1920-radius and radius<=y<1080-radius,'tool_only_point_outside_image')
        patch=image[y-radius:y+radius+1,x-radius:x+radius+1]
        black=float((cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)[:,:,2]<p['blackMaximumValue']).mean())
        return self._record(pair,black,hashlib.sha256(raw).hexdigest(),m4_deg,source_kind='raw_tool_only_projection')
    def from_full(self,full,m4_deg):
        require(isinstance(full,dict) and full.get('schemaVersion')==marker.SCHEMA and
                full.get('registrationSha256')==self.source_binding and
                full.get('quality',{}).get('eligiblePairs')==1,'tool_only_full_source_contract_changed')
        result=self._record(full.get('observedMarkerPair'),full.get('quality',{}).get('blackOccupancyFraction'),
                            full.get('imageSha256'),m4_deg,source_kind='full_observation_projected_to_tool_only')
        require(np.allclose(result['state'][:2],full.get('virtualProxyPx'),atol=1e-8,rtol=0.) and
                np.allclose(result['state'][:2],full.get('toolProxyPx'),atol=1e-8,rtol=0.) and
                number(full.get('toolAxisImageAngleDeg')) and
                abs(difference(result['state'],[0.,0.,0.,full['toolAxisImageAngleDeg']])[3])<1e-8,
                'tool_only_full_proxy_or_axis_changed')
        result['sourceFullObservationSha256']=digest(full)
        return result


def observation(value,contract):
    validate_projection_contract(contract)
    require(isinstance(value,dict) and value.get('schemaVersion')==OBSERVATION_SCHEMA and
            value.get('projectionContractSha256')==digest(contract) and sha(value.get('imageSha256')) and
            value.get('sourceFullRegistrationSha256')==contract['sourceFullRegistrationSha256'] and
            value.get('objectDetectionRequired') is False,'direct_M3_tool_projection_required')
    s=value.get('state')
    require(isinstance(s,list) and len(s)==4 and all(number(v) for v in s) and
            0<=s[0]<1920 and 0<=s[1]<1080 and 0<=s[2]<360 and -180<=s[3]<=180,
            'direct_M3_finite_absolute_tool_state')
    pair=value.get('markerPair',{})
    require(isinstance(pair,dict) and all(isinstance(pair.get(k),dict) and isinstance(pair[k].get('center'),list)
            and len(pair[k]['center'])==2 and all(number(v) for v in pair[k]['center']) for k in ('purple','teal')),
            'direct_M3_marker_evidence_required')
    origin,basis=marker._basis(pair);point=origin+basis@np.array(contract['basisCoefficients'])
    axis=float(np.degrees(np.arctan2(basis[1,0],basis[0,0])))
    distance=float(np.linalg.norm(basis[:,0]));p=contract['detectorPolicy']
    require(p['pairDistancePx'][0]<=distance<=p['pairDistancePx'][1] and number(pair.get('distancePx')) and
            abs(distance-pair['distancePx'])<1e-8 and np.allclose(s[:2],point,atol=1e-8,rtol=0.) and
            abs(difference(s,[0.,0.,0.,axis])[3])<1e-8 and number(value.get('blackOccupancyFraction')) and
            p['minimumBlackOccupancyFraction']<=value['blackOccupancyFraction']<=1.,'direct_M3_tool_evidence_changed')
    return np.array(s,dtype=float)


def _rows(rows,contract):
    require(isinstance(rows,list) and rows,'direct_M3_audited_sources_required')
    sessions,sources=set(),set();identities=[]
    for row in rows:
        require(isinstance(row,dict) and isinstance(row.get('sessionId'),str) and row['sessionId'] and
                row['sessionId'] not in sessions and sha(row.get('sourceSha256')) and row['sourceSha256'] not in sources,
                'direct_M3_unique_original_session_required')
        command_key(row.get('command'));observation(row.get('before'),contract);observation(row.get('after'),contract)
        e=row.get('sourceEvidence',{})
        require(e.get('finalIdleOffVerified') is True and e.get('actualMotor')==3 and
                e.get('recordedCommand')==row['command'] and e.get('sessionJsonSha256')==row['sourceSha256'] and
                all(sha(e.get(k)) for k in ('beforeImageSha256','afterImageSha256','beforePixelSha256','afterPixelSha256',
                    'beforeSensorSha256','afterSensorSha256','commandReceiptSha256','rawAuditSha256')) and
                e['beforeImageSha256']==row['before']['imageSha256'] and e['afterImageSha256']==row['after']['imageSha256'],
                'direct_M3_actual_command_OFF_image_sensor_proof_required')
        identity=e.get('cameraIdentity',{})
        require(isinstance(identity,dict) and isinstance(identity.get('deviceId'),str) and identity['deviceId'] and
                isinstance(identity.get('viewId'),str) and identity['viewId'] and sha(identity.get('helperSha256')) and
                isinstance(identity.get('configuration'),list) and len(identity['configuration'])==5 and
                isinstance(e.get('controllerBootId'),str) and len(e['controllerBootId'])==32 and
                isinstance(e.get('gripperCameraBootId'),str) and len(e['gripperCameraBootId'])==32 and
                all(c in '0123456789abcdef' for c in e['controllerBootId']+e['gripperCameraBootId']),
                'direct_M3_original_camera_boot_cohort_required')
        require(e.get('sourceRole') in ('historical_manual_collection','model_selected_evaluation_reused_for_development') and
                e.get('newModelIndependentEvaluation') is False and
                (e.get('sourceRole')!='historical_manual_collection' or
                 e.get('meetsCurrentManualCollectionPolicy') is False), 'direct_M3_explicit_source_role_required')
        identities.append(identity);sessions.add(row['sessionId']);sources.add(row['sourceSha256'])
    first=identities[0]
    require(all(all(i[k]==first[k] for k in ('deviceId','helperSha256','configuration')) for i in identities),
            'direct_M3_device_helper_native_configuration_changed')


def fit(rows,*,projection_contract,audit_sources):
    require(callable(audit_sources) and audit_sources(copy.deepcopy(rows))==rows,'direct_M3_source_reaudit_changed')
    require(isinstance(projection_contract,dict) and projection_contract.get('schemaVersion')==PROJECTION_SCHEMA and
            projection_contract.get('objectDetectionRequired') is False,'direct_M3_explicit_tool_only_contract_required')
    validate_projection_contract(projection_contract)
    _rows(rows,projection_contract)
    categories=[]
    for command in COMMANDS:
        selected=[r for r in rows if r['command']==command]
        if selected:categories.append(dict(command=copy.deepcopy(command),sourceCount=len(selected),
            singleton=len(selected)==1,sessionIds=[r['sessionId'] for r in selected]))
    return dict(schemaVersion=SCHEMA,projectionContract=copy.deepcopy(projection_contract),
        projectionContractSha256=digest(projection_contract),supportPolicy=policy(),trainingRows=copy.deepcopy(rows),
        categories=categories,sourceCohorts=[dict(sessionId=r['sessionId'],cameraIdentity=copy.deepcopy(r['sourceEvidence']['cameraIdentity']),
            controllerBootId=r['sourceEvidence']['controllerBootId'],gripperCameraBootId=r['sourceEvidence']['gripperCameraBootId'],
            sourceRole=r['sourceEvidence']['sourceRole'],meetsCurrentManualCollectionPolicy=r['sourceEvidence'].get('meetsCurrentManualCollectionPolicy'),
            newModelIndependentEvaluation=False) for r in rows],
        sourceRowsSha256=digest(rows),viewTransferHypothesisUnverified=True,physicalExtrinsicsMatched=False,controlEligible=False,developmentOnly=True,independentTest=False,
        prospectivelyTested=False,baselineSuperiorityVerified=False,physicalAccuracyVerified=False,
        inverseUPTrainingRows=0,originalModelsChanged=False,calibratedUncertainty=False,
        assumptions=['single_nearest_delta_transfer_unverified','source_view_labels_and_boots_preserved_not_runtime_authorization',
                     'same_device_helper_configuration_and_marker_basis_not_general_view_or_load_equivalence'])


def validate(model):
    require(isinstance(model,dict) and model.get('schemaVersion')==SCHEMA and model.get('supportPolicy')==policy() and
            model.get('controlEligible') is False and model.get('developmentOnly') is True and
            model.get('inverseUPTrainingRows')==0 and model.get('prospectivelyTested') is False,
            'direct_M3_development_model_required')
    contract=model.get('projectionContract')
    require(isinstance(contract,dict) and contract.get('schemaVersion')==PROJECTION_SCHEMA and
            contract.get('objectDetectionRequired') is False and model.get('projectionContractSha256')==digest(contract),
            'direct_M3_projection_contract_changed')
    _rows(model.get('trainingRows'),contract)
    require(model.get('sourceRowsSha256')==digest(model['trainingRows']),'direct_M3_source_rows_changed')
    expected=fit(model['trainingRows'],projection_contract=contract,audit_sources=lambda rows:rows)
    require(model==expected,'direct_M3_frozen_model_fields_changed')


def predict(model,current,command):
    validate(model);command_key(command);state=observation(current,model['projectionContract'])
    rows=[r for r in model['trainingRows'] if r['command']==command]
    require(rows,'direct_M3_exact_command_not_trained')
    before=np.array([r['before']['state'] for r in rows]);delta=np.array([difference(r['after']['state'],r['before']['state']) for r in rows])
    diff=difference(before,state);distances=np.linalg.norm(diff/SCALES,axis=1);n=int(np.argmin(distances))
    require(distances[n]<=1. and abs(diff[n,2])<=15. and abs(diff[n,3])<=20.,'direct_M3_current_outside_declared_support')
    predicted=state+delta[n];mean=state+delta.mean(axis=0)
    predicted[3]=(predicted[3]+180.)%360.-180.;mean[3]=(mean[3]+180.)%360.-180.
    require(0<=predicted[0]<1920 and 0<=predicted[1]<1080 and 0<=predicted[2]<360,'direct_M3_prediction_outside_image_or_angle')
    return dict(command=copy.deepcopy(command),predictedState=predicted.tolist(),commandMeanState=mean.tolist(),
        observedSourceDeltaState=delta[n].tolist(),modelSha256=digest(model),inputSha256=digest(current),
        support=dict(nearestSourceSessionId=rows[n]['sessionId'],nearestSourceSha256=rows[n]['sourceSha256'],
            nearestStandardDistance=float(distances[n]),M4DifferenceDeg=float(abs(diff[n,2])),
            imageAxisDifferenceDeg=float(abs(diff[n,3])),categorySourceCount=len(rows),singleton=len(rows)==1,
            declaredTransferHypothesis=True,safetyBound=False),controlEligible=False,prospectivelyTested=False,
        calibratedUncertainty=False,inverseUPHypothesis=False,directDOWNTrainingRows=len(rows),
        viewTransferHypothesisUnverified=True,physicalExtrinsicsMatched=False)


def select(model,current,target_point_px,candidates=None,*,commands_used=0):
    validate(model);state=observation(current,model['projectionContract'])
    require(isinstance(target_point_px,list) and len(target_point_px)==2 and all(number(v) for v in target_point_px) and
            0<=target_point_px[0]<1920 and 0<=target_point_px[1]<1080,'direct_M3_explicit_image_goal_required')
    require(type(commands_used) is int and commands_used in (0,1),'direct_M3_single_command_budget')
    commands=[copy.deepcopy(c['command']) for c in model['categories']] if candidates is None else candidates
    require(isinstance(commands,list) and commands and len({command_key(c) for c in commands})==len(commands),
            'direct_M3_distinct_categorical_candidates_required')
    goal=np.array(target_point_px);binding=dict(pointPx=goal.tolist(),meaning='explicit_2d_tool_goal_not_contact_or_clearance')
    stop=dict(kind='stop',command=None,predictedState=state.tolist(),errorPx=float(np.linalg.norm(state[:2]-goal)))
    ranked,rejected=[stop],[]
    if not commands_used:
        for command in commands:
            try:
                response=predict(model,current,command)
                ranked.append(dict(kind='action',command=copy.deepcopy(command),response=response,
                    predictedState=response['predictedState'],errorPx=float(np.linalg.norm(np.array(response['predictedState'][:2])-goal)),
                    commandMeanErrorPx=float(np.linalg.norm(np.array(response['commandMeanState'][:2])-goal))))
            except ValueError as e:rejected.append(dict(command=copy.deepcopy(command),reason=str(e)))
    ranked.sort(key=lambda r:r['errorPx'])
    return dict(schemaVersion=SCHEMA+'.decision',selected=copy.deepcopy(ranked[0]),candidates=ranked,rejected=rejected,
        modelSha256=digest(model),inputSha256=digest(current),projectionContractSha256=model['projectionContractSha256'],
        objectiveBinding=binding,objectiveBindingSha256=digest(binding),commandsUsed=commands_used,maximumCommands=1,
        selectionOwner='direct_observed_M3_categorical_response_comparison',controlEligible=False,
        modelSelectedAction=ranked[0]['kind']=='action',prospectivelyTested=False,
        selectionReason='command_budget_exhausted' if commands_used else 'minimum_predicted_tool_goal_distance')
