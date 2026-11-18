"""기존 표식 중심을 보존하는 별도 core 품질 관측·직접 DOWN 모델 연결.

청록 성분의 solidity만 실패하면 5px opening core의 품질을 별도로 검사한다.
좌표·면적·박스는 원래 3px 성분을 사용한다. 원 관측 계약으로 위장하지 않으며,
기존 학습 원본 여섯 장의 좌표·전이가 일치하는 감사 후에만 같은 수치 모델을 쓴다.
실제 끝점·깊이·접촉·하중 전이·이동 여유·성능 개선을 검증하지 않는다.
"""
from __future__ import annotations
import copy
import hashlib
import json

import cv2
import numpy as np

from learning import marker_virtual_observer as marker, m3_direct_tool_response as direct
from learning.near_gripper_tip_observer import digest

SCHEMA='motionbrain.marker_core_tool_bridge.v1'
OBSERVATION_SCHEMA=SCHEMA+'.observation'
AUDIT_SCHEMA=SCHEMA+'.source_audit'


def require(ok,reason):
    if not ok:raise ValueError(reason)

def observer_policy(pair_policy='strict'):
    require(pair_policy in ('strict','unique_dark_support_v1'),'core_bridge_pair_policy_required')
    result=dict(schemaVersion=SCHEMA+'.observer_policy',originalDetectorPolicy=marker.detector_policy(),
        fullOpeningKernelSize=3,coreOpeningKernelSize=5,minimumCoreFullAreaFraction=.5,
        fallbackOnly='teal_solidity_failure_after_original_other_checks',
        coreMapping='global_core_pixels_wholly_inside_exactly_one_original_component',
        eligibleCoreCountPerFallbackComponent=1,coreMinimumArea=80,
        outputGeometry='original_3px_component_centroid_bbox_area_unchanged',
        originalSolidityRetained=True,qualityAndCoordinatesSeparated=True)
    if pair_policy=='unique_dark_support_v1':
        result['pairSelectionPolicy']=dict(name=pair_policy,
            candidates='all_original_distance_eligible_pairs',selection='exactly_one_existing_dark_support_pass',
            outOfImageWindowRejected=True,blackWindowRadiusPx=22,blackMaximumValue=85,
            minimumBlackOccupancyFraction=.15,geometry='original_3px_component_unchanged')
    return result


def _component(labels,stats,centers,index,roi):
    x,y,w,h,area=map(int,stats[index])
    contours,_=cv2.findContours(np.uint8(labels==index),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contour=max(contours,key=cv2.contourArea);hull=cv2.contourArea(cv2.convexHull(contour))
    solidity=float(cv2.contourArea(contour)/hull if hull else 0.)
    return dict(center=(centers[index]+np.array(roi[:2])).tolist(),area=area,
        bbox=[x+roi[0],y+roi[1],w,h],fillFraction=area/(w*h),solidity=solidity)


def _geometry(component,kind,policy):
    x,y,w,h=component['bbox'];area=component['area']
    return (policy[kind+'Area'][0]<=area<=policy[kind+'Area'][1]
        and w>0 and h>0 and policy['aspectRatio'][0]<=w/h<=policy['aspectRatio'][1]
        and area/(w*h)>=policy['minimumFillFraction'])


def _core_mapping(full_labels,core_labels,core_stats,minimum_area=80):
    """전체 core를 먼저 대응한다. 교차·배경 포함 core를 골라 무시하지 않는다."""
    require(full_labels.shape==core_labels.shape,'core_bridge_label_shape')
    mapped={}
    for index in range(1,len(core_stats)):
        if int(core_stats[index,cv2.CC_STAT_AREA])<minimum_area:continue
        members=np.unique(full_labels[core_labels==index])
        require(len(members)==1 and members[0]>0,'core_bridge_core_mapping_not_wholly_unique')
        mapped.setdefault(int(members[0]),[]).append(index)
    return mapped


def _teal_candidates(image,policy):
    roi=policy['workingRoi'];patch=marker._roi(image,roi)
    hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.uint8(policy['tealHsvLow']),np.uint8(policy['tealHsvHigh']))
    full=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    core=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    count,labels,stats,centers=cv2.connectedComponentsWithStats(full)
    _,core_labels,core_stats,core_centers=cv2.connectedComponentsWithStats(core)
    mapping=_core_mapping(labels,core_labels,core_stats)
    candidates=[]
    for index in range(1,count):
        component=_component(labels,stats,centers,index,roi)
        if not _geometry(component,'teal',policy):continue
        validation=dict(mode='original_strict_solidity',originalSolidity=component['solidity'])
        if component['solidity']<policy['minimumSolidity']:
            members=mapping.get(index,[])
            if len(members)!=1:continue
            candidate=_component(core_labels,core_stats,core_centers,members[0],roi)
            fraction=candidate['area']/component['area']
            if not (_geometry(candidate,'teal',policy) and candidate['solidity']>=policy['minimumSolidity']
                    and fraction>=.5):continue
            validation=dict(mode='core_quality_original_geometry',originalSolidity=component['solidity'],
                core=candidate,coreFullAreaFraction=fraction,eligibleCoreCount=1,
                mappedFullComponentCount=1,corePixelsInsideFull=True)
        candidates.append((component,validation))
    return candidates


def _distance_pairs(image,policy):
    purple=marker._candidates(image,'purple',policy);teal=_teal_candidates(image,policy);pairs=[]
    for a in purple:
        for b,validation in teal:
            distance=float(np.linalg.norm(np.array(b['center'])-a['center']))
            if policy['pairDistancePx'][0]<=distance<=policy['pairDistancePx'][1]:
                pairs.append((dict(purple=a,teal=b,distancePx=distance),validation))
    return pairs,len(purple),len(teal)


def _pair(image,policy):
    pairs,purple_count,teal_count=_distance_pairs(image,policy)
    require(len(pairs)==1,'core_bridge_marker_pair_missing_or_ambiguous')
    pair,validation=pairs[0]
    return pair,dict(purpleCandidates=purple_count,tealCandidates=teal_count,eligiblePairs=1,tealValidation=validation)


def _projected_support(image,pair,coefficients,policy):
    origin,basis=marker._basis(pair);point=origin+basis@np.array(coefficients)
    radius=policy['blackWindowRadiusPx'];x,y=np.rint(point).astype(int)
    inside=bool(radius<=x<1920-radius and radius<=y<1080-radius)
    dark=None
    if inside:
        patch=image[y-radius:y+radius+1,x-radius:x+radius+1]
        dark=float((cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)[:,:,2]<policy['blackMaximumValue']).mean())
    return point,dark,inside


def _dark_supported_pair(image,policy,coefficients):
    pairs,purple_count,teal_count=_distance_pairs(image,policy);candidates=[]
    for pair,validation in pairs:
        point,dark,inside=_projected_support(image,pair,coefficients,policy)
        candidates.append(dict(markerPair=pair,tealValidation=validation,toolPointPx=point.tolist(),
            blackOccupancyFraction=dark,windowInsideImage=inside,
            accepted=bool(inside and dark>=policy['minimumBlackOccupancyFraction'])))
    accepted=[c for c in candidates if c['accepted']]
    require(len(accepted)==1,'core_bridge_dark_supported_pair_not_unique')
    chosen=accepted[0]
    selection=dict(policy='unique_dark_support_v1',distanceEligiblePairs=len(candidates),
        darkSupportedPairs=1,candidates=candidates)
    return chosen['markerPair'],dict(purpleCandidates=purple_count,tealCandidates=teal_count,
        eligiblePairs=1,tealValidation=chosen['tealValidation'],pairSelection=selection)


def _contract(model,model_file_sha,pair_policy='strict'):
    return dict(schemaVersion=SCHEMA,originalModelSha256=digest(model),originalModelFileSha256=model_file_sha,
        originalProjectionContractSha256=model['projectionContractSha256'],
        originalProjectionContract=copy.deepcopy(model['projectionContract']),
        newObserverSha256=digest(observer_policy(pair_policy)),observerPolicy=observer_policy(pair_policy),
        originalContractClaimed=False,newTrainingPerformed=False,
        sourceAuditRequired=True,stateFields=copy.deepcopy(model['projectionContract']['stateFields']))


class CoreBridgeProjector:
    def __init__(self,model_raw,seed_raw,*,pair_policy='strict'):
        require(isinstance(model_raw,bytes) and isinstance(seed_raw,bytes),'core_bridge_original_bytes_required')
        observer_policy(pair_policy);self.pair_policy=pair_policy;self._pair_policy_seal=pair_policy
        self.model=json.loads(model_raw);direct.validate(self.model)
        self._model_raw=model_raw;self._seed_raw=seed_raw
        self.original_model_file_sha256=hashlib.sha256(model_raw).hexdigest()
        self._model_sha=digest(self.model)
        original=direct.ToolOnlyProjector(self.model['projectionContract']['sourceRegistration'],seed_raw)
        require(original.contract==self.model['projectionContract'],'core_bridge_original_projection_mismatch')
        self.registration=copy.deepcopy(original.registration)
        self.contract=_contract(self.model,self.original_model_file_sha256,self.pair_policy);self.binding=digest(self.contract)
        self.observer_sha256=self.contract['newObserverSha256']
    def verify(self):
        require(hashlib.sha256(self._model_raw).hexdigest()==self.original_model_file_sha256
            and self.pair_policy==self._pair_policy_seal and digest(self.model)==self._model_sha and self.contract==_contract(self.model,self.original_model_file_sha256,self.pair_policy)
            and digest(self.contract)==self.binding and self.observer_sha256==digest(observer_policy(self.pair_policy))
            and self.registration==self.model['projectionContract']['sourceRegistration']
            and hashlib.sha256(self._seed_raw).hexdigest()==self.registration['seedSha256'], 'core_bridge_projector_changed')
        direct.validate(self.model)
    def observe(self,raw,m4_deg):
        self.verify();image=marker._image(raw)
        require(direct.number(m4_deg) and 0<=m4_deg<360,'core_bridge_M4_required')
        policy=self.registration['detectorPolicy']
        if self.pair_policy=='strict':pair,quality=_pair(image,policy)
        else:
            pair,quality=_dark_supported_pair(image,policy,self.registration['basisCoefficients'])
            quality['pairSelection']['sourceImageSha256']=hashlib.sha256(raw).hexdigest()
        origin,basis=marker._basis(pair);point=origin+basis@np.array(self.registration['basisCoefficients'])
        radius=policy['blackWindowRadiusPx'];x,y=np.rint(point).astype(int)
        require(radius<=x<1920-radius and radius<=y<1080-radius,'core_bridge_point_outside_image')
        patch=image[y-radius:y+radius+1,x-radius:x+radius+1]
        dark=float((cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)[:,:,2]<policy['blackMaximumValue']).mean())
        require(dark>=policy['minimumBlackOccupancyFraction'],'core_bridge_dark_support_missing')
        axis=float(np.degrees(np.arctan2(basis[1,0],basis[0,0])))
        return dict(schemaVersion=OBSERVATION_SCHEMA,bridgeContractSha256=self.binding,
            originalModelSha256=self._model_sha,originalModelFileSha256=self.original_model_file_sha256,
            originalProjectionContractSha256=self.model['projectionContractSha256'],newObserverSha256=self.observer_sha256,
            imageSha256=hashlib.sha256(raw).hexdigest(),state=[float(point[0]),float(point[1]),float(m4_deg),axis],
            markerPair=pair,quality=quality,blackOccupancyFraction=dark,originalContractClaimed=False,
            objectDetectionRequired=False,physicalTipOrClearanceVerified=False)


def _validate_candidate(c,kind,policy):
    require(isinstance(c,dict) and set(c)=={'center','area','bbox','fillFraction','solidity'},'core_bridge_component_fields')
    box=c['bbox'];center=c['center']
    require(isinstance(box,list) and len(box)==4 and all(type(v) is int for v in box)
        and isinstance(center,list) and len(center)==2 and all(direct.number(v) for v in center)
        and type(c['area']) is int and direct.number(c['solidity']) and 0<=c['solidity']<=1.
        and direct.number(c['fillFraction']),'core_bridge_component_geometry')
    x,y,w,h=box;roi=policy['workingRoi']
    require(w>0 and h>0 and roi[0]<=x<x+w<=roi[2] and roi[1]<=y<y+h<=roi[3]
        and x<=center[0]<x+w and y<=center[1]<y+h and 0<c['area']<=w*h
        and c['fillFraction']==c['area']/(w*h) and _geometry(c,kind,policy),'core_bridge_component_geometry')


def _validate_teal_quality(pair,v,policy):
    require(isinstance(v,dict) and v.get('originalSolidity')==pair['teal']['solidity'],'core_bridge_quality_metadata_changed')
    if pair['teal']['solidity']>=policy['minimumSolidity']:
        require(v==dict(mode='original_strict_solidity',originalSolidity=pair['teal']['solidity']),'core_bridge_quality_metadata_changed')
    else:
        require(set(v)=={'mode','originalSolidity','core','coreFullAreaFraction','eligibleCoreCount','mappedFullComponentCount','corePixelsInsideFull'}
            and v['mode']=='core_quality_original_geometry' and v['eligibleCoreCount']==v['mappedFullComponentCount']==1
            and v['corePixelsInsideFull'] is True,'core_bridge_core_metadata_changed')
        core=v['core'];_validate_candidate(core,'teal',policy);full=pair['teal'];x,y,w,h=full['bbox'];cx,cy,cw,ch=core['bbox']
        require(core['solidity']>=policy['minimumSolidity'] and x<=cx<cx+cw<=x+w and y<=cy<cy+ch<=y+h
            and v['coreFullAreaFraction']==core['area']/full['area'] and .5<=v['coreFullAreaFraction']<=1.,'core_bridge_core_quality_changed')


def _validate_pair_selection(projector,value):
    quality=value['quality']
    if projector.pair_policy=='strict':
        require('pairSelection' not in quality,'core_bridge_strict_pair_metadata_changed')
        return
    selection=quality.get('pairSelection');policy=projector.registration['detectorPolicy']
    require(isinstance(selection,dict) and set(selection)=={'policy','sourceImageSha256','distanceEligiblePairs','darkSupportedPairs','candidates'}
        and selection['policy']=='unique_dark_support_v1' and selection['sourceImageSha256']==value['imageSha256']
        and type(selection['distanceEligiblePairs']) is int and type(selection['darkSupportedPairs']) is int
        and selection['darkSupportedPairs']==1 and isinstance(selection['candidates'],list)
        and selection['distanceEligiblePairs']==len(selection['candidates'])>0,'core_bridge_dark_selection_metadata_changed')
    accepted=[];seen=set()
    for candidate in selection['candidates']:
        require(isinstance(candidate,dict) and set(candidate)=={'markerPair','tealValidation','toolPointPx','blackOccupancyFraction','windowInsideImage','accepted'},'core_bridge_dark_candidate_fields')
        pair=candidate['markerPair'];require(isinstance(pair,dict) and set(pair)=={'purple','teal','distancePx'},'core_bridge_pair_required')
        for kind in ('purple','teal'):_validate_candidate(pair[kind],kind,policy)
        require(pair['purple']['solidity']>=policy['minimumSolidity'],'core_bridge_purple_strict_required')
        _validate_teal_quality(pair,candidate['tealValidation'],policy)
        pair_sha=digest(pair);require(pair_sha not in seen,'core_bridge_duplicate_dark_pair');seen.add(pair_sha)
        origin,basis=marker._basis(pair);point=origin+basis@np.array(projector.registration['basisCoefficients']);distance=float(np.linalg.norm(basis[:,0]))
        require(direct.number(pair['distancePx']) and abs(pair['distancePx']-distance)<1e-8
            and policy['pairDistancePx'][0]<=distance<=policy['pairDistancePx'][1]
            and isinstance(candidate['toolPointPx'],list) and len(candidate['toolPointPx'])==2
            and all(direct.number(v) for v in candidate['toolPointPx'])
            and np.allclose(candidate['toolPointPx'],point,atol=1e-8,rtol=0.),'core_bridge_dark_candidate_geometry_changed')
        radius=policy['blackWindowRadiusPx'];x,y=np.rint(point).astype(int);inside=bool(radius<=x<1920-radius and radius<=y<1080-radius);dark=candidate['blackOccupancyFraction']
        require(type(candidate['windowInsideImage']) is bool and candidate['windowInsideImage']==inside
            and ((inside and direct.number(dark) and 0<=dark<=1.) or (not inside and dark is None)),'core_bridge_dark_candidate_window_changed')
        passed=bool(inside and dark>=policy['minimumBlackOccupancyFraction'])
        require(type(candidate['accepted']) is bool and candidate['accepted']==passed,'core_bridge_dark_candidate_acceptance_changed')
        if passed:accepted.append(candidate)
    require(len(accepted)==1 and accepted[0]['markerPair']==value['markerPair']
        and accepted[0]['tealValidation']==quality['tealValidation']
        and accepted[0]['blackOccupancyFraction']==value['blackOccupancyFraction'],'core_bridge_dark_selected_pair_changed')


def observation(projector,value):
    projector.verify();require(isinstance(value,dict) and value.get('schemaVersion')==OBSERVATION_SCHEMA
        and value.get('bridgeContractSha256')==projector.binding
        and value.get('originalModelSha256')==projector._model_sha
        and value.get('originalModelFileSha256')==projector.original_model_file_sha256
        and value.get('originalProjectionContractSha256')==projector.model['projectionContractSha256']
        and value.get('newObserverSha256')==projector.observer_sha256 and direct.sha(value.get('imageSha256'))
        and value.get('originalContractClaimed') is False,'core_bridge_new_observation_contract_required')
    pair=value.get('markerPair',{});policy=projector.registration['detectorPolicy']
    require(isinstance(pair,dict) and set(pair)=={'purple','teal','distancePx'},'core_bridge_pair_required')
    for kind in ('purple','teal'):_validate_candidate(pair[kind],kind,policy)
    require(pair['purple']['solidity']>=policy['minimumSolidity'],'core_bridge_purple_strict_required')
    quality=value.get('quality',{});v=quality.get('tealValidation',{})
    require(quality.get('eligiblePairs')==1 and v.get('originalSolidity')==pair['teal']['solidity'],'core_bridge_quality_metadata_changed')
    _validate_teal_quality(pair,v,policy)
    _validate_pair_selection(projector,value)
    state=value.get('state');require(isinstance(state,list) and len(state)==4 and all(direct.number(v) for v in state)
        and 0<=state[0]<1920 and 0<=state[1]<1080 and 0<=state[2]<360 and -180<=state[3]<=180,'core_bridge_state_required')
    origin,basis=marker._basis(pair);point=origin+basis@np.array(projector.registration['basisCoefficients'])
    distance=float(np.linalg.norm(basis[:,0]));axis=float(np.degrees(np.arctan2(basis[1,0],basis[0,0])))
    require(direct.number(pair['distancePx']) and abs(distance-pair['distancePx'])<1e-8
        and policy['pairDistancePx'][0]<=distance<=policy['pairDistancePx'][1]
        and np.allclose(state[:2],point,atol=1e-8,rtol=0.) and abs(direct.difference(state,[0.,0.,0.,axis])[3])<1e-8
        and direct.number(value.get('blackOccupancyFraction')) and policy['minimumBlackOccupancyFraction']<=value['blackOccupancyFraction']<=1.,'core_bridge_geometric_or_dark_evidence_changed')
    return np.array(state,dtype=float)


def audit_sources(projector,sources_raw):
    projector.verify();rows=projector.model['trainingRows']
    require(isinstance(sources_raw,list) and len(sources_raw)==len(rows)==3,'core_bridge_all_three_sources_required')
    require(all(isinstance(s,dict) and set(s)=={'sessionId','before','after'} for s in sources_raw)
        and len({s['sessionId'] for s in sources_raw})==3,'core_bridge_unique_source_bytes_required')
    indexed={s['sessionId']:s for s in sources_raw};audits=[]
    for row in rows:
        require(row['sessionId'] in indexed,'core_bridge_source_missing');source=indexed[row['sessionId']];phases={}
        for phase in ('before','after'):
            raw=source[phase];require(isinstance(raw,bytes) and hashlib.sha256(raw).hexdigest()==row[phase]['imageSha256'],'core_bridge_source_raw_SHA_mismatch')
            current=projector.observe(raw,row[phase]['state'][2]);state=observation(projector,current)
            require(np.allclose(state,row[phase]['state'],atol=1e-8,rtol=0.),'core_bridge_source_coordinates_changed')
            phases[phase]=dict(imageSha256=current['imageSha256'],newObservationSha256=digest(current),newObservation=current,
                originalState=copy.deepcopy(row[phase]['state']),newState=state.tolist())
        old=direct.difference(row['after']['state'],row['before']['state']);new=direct.difference(phases['after']['newState'],phases['before']['newState'])
        require(np.array_equal(old,new),'core_bridge_source_delta_changed')
        audits.append(dict(sessionId=row['sessionId'],sourceSha256=row['sourceSha256'],phases=phases,originalDelta=old.tolist(),newDelta=new.tolist()))
    audit=dict(schemaVersion=AUDIT_SCHEMA,bridgeContractSha256=projector.binding,originalModelSha256=projector._model_sha,
        originalModelFileSha256=projector.original_model_file_sha256,originalProjectionContractSha256=projector.model['projectionContractSha256'],
        newObserverSha256=projector.observer_sha256,rawImageCount=6,sourceCount=3,coordinateAtol=1e-8,deltaExact=True,
        sources=audits,newTrainingPerformed=False,originalModelChanged=False)
    audit['auditSha256']=digest(audit);return audit


def validate_audit(projector,audit):
    projector.verify();require(isinstance(audit,dict),'core_bridge_source_audit_required')
    payload={k:v for k,v in audit.items() if k!='auditSha256'}
    require(audit.get('schemaVersion')==AUDIT_SCHEMA and audit.get('auditSha256')==digest(payload)
        and audit.get('bridgeContractSha256')==projector.binding and audit.get('originalModelSha256')==projector._model_sha
        and audit.get('originalModelFileSha256')==projector.original_model_file_sha256
        and audit.get('originalProjectionContractSha256')==projector.model['projectionContractSha256']
        and audit.get('newObserverSha256')==projector.observer_sha256 and audit.get('sourceCount')==3
        and audit.get('rawImageCount')==6 and audit.get('coordinateAtol')==1e-8 and audit.get('deltaExact') is True
        and audit.get('newTrainingPerformed') is False and audit.get('originalModelChanged') is False,'core_bridge_source_audit_binding_changed')
    sources=audit.get('sources');rows=projector.model['trainingRows']
    require(isinstance(sources,list) and len(sources)==len(rows)==3,'core_bridge_all_three_sources_required')
    for row,source in zip(rows,sources):
        require(source['sessionId']==row['sessionId'] and source['sourceSha256']==row['sourceSha256'],'core_bridge_source_audit_row_changed')
        for phase in ('before','after'):
            value=source['phases'][phase];state=observation(projector,value['newObservation'])
            require(value['imageSha256']==value['newObservation']['imageSha256']==row[phase]['imageSha256']
                and value['newObservationSha256']==digest(value['newObservation']) and value['originalState']==row[phase]['state']
                and value['newState']==state.tolist() and np.allclose(state,row[phase]['state'],atol=1e-8,rtol=0.),'core_bridge_source_audit_coordinates_changed')
        old=direct.difference(row['after']['state'],row['before']['state']).tolist()
        require(source['originalDelta']==source['newDelta']==old,'core_bridge_source_audit_delta_changed')
    return copy.deepcopy(audit)


def predict(projector,audit,current,command):
    validate_audit(projector,audit);state=observation(projector,current)
    response=direct._predict_state(projector.model,state,command,input_sha256=digest(current))
    response.update(bridgeContractSha256=projector.binding,newObserverSha256=projector.observer_sha256,
        originalModelFileSha256=projector.original_model_file_sha256,sourceAuditSha256=audit['auditSha256'],originalContractClaimed=False)
    return response


def select(projector,audit,current,target_point_px,candidates=None,*,commands_used=0):
    validate_audit(projector,audit);state=observation(projector,current)
    require(isinstance(target_point_px,list) and len(target_point_px)==2 and all(direct.number(v) for v in target_point_px)
        and 0<=target_point_px[0]<1920 and 0<=target_point_px[1]<1080,'core_bridge_explicit_image_goal_required')
    require(type(commands_used) is int and commands_used in (0,1),'core_bridge_single_command_budget')
    commands=[copy.deepcopy(c['command']) for c in projector.model['categories']] if candidates is None else candidates
    require(isinstance(commands,list) and commands and len({direct.command_key(c) for c in commands})==len(commands),'core_bridge_distinct_commands_required')
    goal=np.array(target_point_px);stop=dict(kind='stop',command=None,predictedState=state.tolist(),errorPx=float(np.linalg.norm(state[:2]-goal)))
    ranked,rejected=[stop],[]
    if not commands_used:
        for command in commands:
            try:
                response=predict(projector,audit,current,command)
                ranked.append(dict(kind='action',command=copy.deepcopy(command),response=response,predictedState=response['predictedState'],
                    errorPx=float(np.linalg.norm(np.array(response['predictedState'][:2])-goal)),commandMeanErrorPx=float(np.linalg.norm(np.array(response['commandMeanState'][:2])-goal))))
            except ValueError as exc:rejected.append(dict(command=copy.deepcopy(command),reason=str(exc)))
    ranked.sort(key=lambda c:c['errorPx']);performed=any(c['kind']=='action' for c in ranked)
    return dict(schemaVersion=SCHEMA+'.decision',selected=copy.deepcopy(ranked[0]),candidates=ranked,rejected=rejected,
        originalModelSha256=projector._model_sha,originalModelFileSha256=projector.original_model_file_sha256,
        originalProjectionContractSha256=projector.model['projectionContractSha256'],bridgeContractSha256=projector.binding,
        newObserverSha256=projector.observer_sha256,sourceAuditSha256=audit['auditSha256'],inputSha256=digest(current),
        targetVirtualPointPx=copy.deepcopy(target_point_px),commandsUsed=commands_used,maximumCommands=1,
        selectionOwner='budget_guard' if commands_used else 'bridged_direct_M3_response_comparison' if performed else 'admission_guard',
        modelPredictionPerformed=performed,modelSelectedStop=not commands_used and performed and ranked[0]['kind']=='stop',
        modelSelectedAction=ranked[0]['kind']=='action',controlEligible=False,originalContractClaimed=False,newTrainingPerformed=False)
