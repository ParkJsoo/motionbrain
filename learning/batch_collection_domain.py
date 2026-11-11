"""고정 관측기의 유한 수동 수집 영역. 예측·모델 선택의 지원 범위를 바꾸지 않는다."""
import copy
import json
from pathlib import Path

from learning.batch_observation import require, number, digest
from learning.batch_learning import command_key
from learning.batch_motion_admission import _read, build_admission

DOMAIN_SCHEMA='motionbrain.manual_collection_domain.v1'
SCHEMA='motionbrain.qualified_collection_domain.v1'
JAW_FIELDS=['gripper:jawLeftXpx','gripper:jawLeftYpx','gripper:jawRightXpx',
            'gripper:jawRightYpx','gripper:projectedJawSeparationPx']


def validate_domain(domain):
    require(isinstance(domain,dict) and set(domain)=={'schemaVersion','semantics','jawMinimum','jawMaximum',
        'minimumSearchMarginPx','maximumGapChangePx','commands'} and domain['schemaVersion']==DOMAIN_SCHEMA and
        domain['semantics']=='manual_noncontact_collection_not_model_control','collection_domain_schema')
    lo,hi=domain['jawMinimum'],domain['jawMaximum']
    require(all(isinstance(v,list) and len(v)==4 and all(number(x) for x in v) for v in (lo,hi)) and
            all(a<b for a,b in zip(lo,hi)),'collection_domain_jaw_bounds')
    require(number(domain['minimumSearchMarginPx']) and domain['minimumSearchMarginPx']>=2 and
            number(domain['maximumGapChangePx']) and 0<domain['maximumGapChangePx'],
            'collection_domain_motion_limits')
    commands=domain['commands'];require(isinstance(commands,list) and bool(commands),'collection_domain_commands')
    keys=set()
    for entry in commands:
        require(isinstance(entry,dict) and set(entry)=={'command','beforeGap','afterGap'},'collection_domain_command_fields')
        command=entry['command'];key=command_key(command)
        require(key not in keys and command['joint']=='gripper' and command['direction'] in ('open','close') and
                command['percent']==40 and command['durationMs']==350,'collection_domain_fixed_M1_profile')
        keys.add(key)
        for field in ('beforeGap','afterGap'):
            bounds=entry[field]
            require(isinstance(bounds,list) and len(bounds)==2 and all(number(v) for v in bounds) and
                    0<bounds[0]<bounds[1],'collection_domain_gap_bounds')
    return domain


def _registered_limits(repository,observer_binding,domain):
    spec=_read(repository,dict(path=observer_binding['gripper']['observerSpecPath'],
                              sha256=observer_binding['gripper']['observerSpecSha256']))
    require(spec.get('kind')=='registered_gripper_jaws_stationary' and spec['context']['load']=='empty',
            'collection_domain_empty_jaws_required')
    registration=_read(repository,dict(path=spec['registrationPath'],sha256=spec['registrationSha256']))
    params=registration['parameters'];margin=domain['minimumSearchMarginPx']
    for i,name in enumerate(('jawLeft','jawRight')):
        for axis in (0,1):
            center=registration['points'][name][axis];radius=params['searchRadii'][name][axis]
            index=2*i+axis
            require(center-radius+margin<=domain['jawMinimum'][index] and
                    domain['jawMaximum'][index]<=center+radius-margin,'collection_domain_search_margin')
    require(domain['maximumGapChangePx']<=params['maximumOpeningPx']-params['minimumOpeningPx'],
            'collection_domain_original_delta_limits')
    for entry in domain['commands']:
        for side in ('beforeGap','afterGap'):
            require(params['minimumOpeningPx']<=entry[side][0] and
                    entry[side][1]<=params['maximumOpeningPx'],'collection_domain_original_opening_limits')


def require_scope(artifact,command,context,fields,state=None,*,side='before'):
    require(dict(joint=command['joint'],context=context,stateFields=fields)==artifact['contract'],
            'collection_domain_contract_changed')
    domain=validate_domain(artifact['domain'])
    entry=next((e for e in domain['commands'] if e['command']==command),None)
    require(entry is not None,'collection_domain_command_not_supported')
    require(side in ('before','after'),'collection_domain_side')
    if state is None:return
    require(isinstance(state,list) and len(state)==len(fields) and all(number(v) for v in state),
            'collection_domain_state_required')
    values=[state[fields.index(name)] for name in JAW_FIELDS]
    require(all(lo<=v<=hi for v,lo,hi in zip(values[:4],domain['jawMinimum'],domain['jawMaximum'])) and
            entry[side+'Gap'][0]<=values[4]<=entry[side+'Gap'][1],'collection_domain_state_outside_declared_bounds')


def require_transition(artifact,command,fields,before,after):
    change=after[fields.index(JAW_FIELDS[-1])]-before[fields.index(JAW_FIELDS[-1])]
    require(abs(change)<=artifact['domain']['maximumGapChangePx'] and
            (change>0 if command['direction']=='open' else change<0),'collection_domain_delta_rejected')


def build_collection_admission(repository,manifest):
    agreement=build_admission(repository,manifest)
    frozen=_read(repository,manifest['validationScope'])
    domain=validate_domain(frozen.get('collectionDomain'))
    _registered_limits(repository,frozen['observerBinding'],domain)
    artifact=dict(schemaVersion=SCHEMA,manifest=copy.deepcopy(manifest),domain=copy.deepcopy(domain),
        contract=copy.deepcopy(agreement['contract']),observerAgreement=agreement,
        semantics='declared_finite_collection_envelope_with_independent_tracking_samples_not_model_support',
        controlEligible=False,executionAuthorized=False,physicalGraspVerified=False)
    counts={command_key(e['command']):0 for e in domain['commands']}
    from learning.batch_commissioning import load_learning_transition
    for item in manifest['holdout']:
        row,_=load_learning_transition(repository,item['transition'],'evaluation')
        for side in ('before','after'):
            require_scope(artifact,row['command'],row['context'],row['stateFields'],row[side],side=side)
        require_transition(artifact,row['command'],row['stateFields'],row['before'],row['after'])
        labels=_read(repository,item['labels'])
        index=row['stateFields'].index(JAW_FIELDS[-1])
        delta=labels['after'][index]-labels['before'][index]
        require(abs(delta)>2*labels['tolerancePx'] and (delta>0 if row['command']['direction']=='open' else delta<0),
                'collection_domain_manual_gap_motion_unresolved')
        counts[command_key(row['command'])]+=1
    require(all(n>=2 for n in counts.values()),'collection_domain_independent_command_coverage')
    artifact['qualifiedSampleCounts']=counts
    return artifact


def verify_collection_admission(repository,binding):
    stored=_read(repository,binding)
    require(stored.get('schemaVersion')==SCHEMA,'collection_admission_schema')
    require(build_collection_admission(repository,stored['manifest'])==stored,'collection_admission_changed')
    return stored


def validate_manual_authorization(repository,plan,binding):
    """현장 확인 출처가 명시된 고정 수동 수집 범위만 허용한다. 확인을 생성하지 않는다."""
    value=_read(repository,binding)
    require(isinstance(value,dict) and set(value)=={'schemaVersion','campaignId','controllerBootId','viewId',
        'poseContext','commands','budgets','collectionAdmission','observerBinding','authorizationSource',
        'stagingReviewSource','selectionSemantics','currentFramesReviewed','contactVerified'},
        'collection_authorization_schema')
    config=plan['runtime']
    require(value['schemaVersion']=='motionbrain.manual_collection_authorization.v1' and
        all(value[k]==plan[k] for k in ('campaignId','controllerBootId','viewId','budgets')) and
        value['poseContext']==config['poseContext'] and value['collectionAdmission']==config['collectionAdmission'] and
        value['commands']==[c['command'] for c in plan['conditions']] and
        value['observerBinding']=={k:config[k] for k in ('observerSpecPath','observerSpecSha256','gripper')} and
        value['selectionSemantics']=='predeclared_manual_collection_not_model_selected' and
        value['currentFramesReviewed'] is False and value['contactVerified'] is False,
        'collection_authorization_plan_changed')
    source=_read(repository,value['authorizationSource'])
    require(isinstance(source,dict) and source.get('schemaVersion')=='motionbrain.attended_collection_authorization.v1' and
        source.get('campaignId')==plan['campaignId'] and source.get('controllerBootId')==plan['controllerBootId'] and
        source.get('operatorOnSiteConfirmed') is True and source.get('clearanceUnchangedConfirmed') is True and
        source.get('physicalPoseUnchangedConfirmed') is True and
        source.get('commands')==value['commands'] and source.get('budgets')==plan['budgets'] and
        isinstance(source.get('userInstruction'),str) and bool(source['userInstruction']) and
        isinstance(source.get('authorizedAt'),str) and bool(source['authorizedAt']),
        'collection_explicit_onsite_authorization_required')
    staging=_read(repository,value['stagingReviewSource'])
    require(isinstance(staging,dict) and staging.get('campaignId')==plan['campaignId'] and
        staging.get('controllerBootId')==plan['controllerBootId'] and staging.get('observerBinding')==value['observerBinding'] and
        staging.get('objectOutsideJawsReviewed') is True and staging.get('otherJointPoseUnchangedReviewed') is True and
        staging.get('reviewBasis')=='original_camera_staging_visual_review' and
        isinstance(staging.get('sourceFrames'),list) and len(staging['sourceFrames'])>=2,
        'collection_original_staging_review_required')
    # Staging must refer to real private originals. This verifies bytes, not the
    # operator's assertion about clearance, contact, or depth.
    for frame in staging['sourceFrames']:
        require(isinstance(frame,dict) and set(frame)=={'path','sha256'},'collection_staging_frame_binding')
        from learning.batch_runtime import private_path,sha
        require(sha(private_path(Path(repository).resolve(),frame['path']).read_bytes())==frame['sha256'],
                'collection_staging_original_changed')
    return value


def before_command(manager,repository,plan,condition,binding,deadline_wall,clock,*,external,paired,gripper_max_age):
    from learning.batch_interaction import VerifiedInteraction
    from learning.batch_runtime import private_path,sha
    value=validate_manual_authorization(repository,plan,binding)
    require(condition in plan['conditions'] and manager.port is not None and manager.port.off_verified is True and
            clock.time()<deadline_wall,'collection_current_off_and_deadline_required')
    snapshots={}
    for reference in (binding,value['authorizationSource'],value['stagingReviewSource'],*(_read(repository,value['stagingReviewSource'])['sourceFrames'])):
        path=private_path(Path(repository).resolve(),reference['path']);raw=path.read_bytes()
        require(sha(raw)==reference['sha256'],'collection_authorization_original_changed');snapshots[str(path)]=raw
    require(isinstance(paired,dict) and paired.get('observation',{}).get('status')=='accepted',
            'collection_current_gripper_required')
    external_frame=external['source']['frame']
    grip=paired['observation']['frameBinding']
    now=clock.monotonic()
    require(number(gripper_max_age) and gripper_max_age>0 and
            0<=now-grip['capturedAt']<=gripper_max_age and
            0<=now-external_frame['sourceEarliest']<=plan['maxObservationAgeSeconds'],
            'collection_current_sources_stale')
    expires=min(external_frame['sourceEarliest']+plan['maxObservationAgeSeconds'],grip['capturedAt']+gripper_max_age)
    from learning.artifacts import _read as read_original
    for folder,names in ((manager.store.root/'gripper-pair',('esp32-before.jpg','esp32-before.json')),
                         (manager.camera.root,(external_frame['imageFile'],external_frame['metadataFile']))):
        for name in names:
            path=(folder/name).resolve();raw=read_original(folder,name)
            snapshots[str(path)]=raw
    require(sha(snapshots[str((manager.camera.root/external_frame['imageFile']).resolve())])==external_frame['imageSha256'] and
            sha(snapshots[str((manager.camera.root/external_frame['metadataFile']).resolve())])==external_frame['metadataSha256'],
            'collection_current_external_changed')
    require(all(sha(snapshots[str((manager.store.root/'gripper-pair'/name).resolve())])==value
                for name,value in paired['fileHashes'].items()),'collection_current_gripper_changed')
    report=dict(stage='before',interaction='noncontact',contactState='not_reviewed',tableContactReady=False,
        graspVerified=False,liftVerified=False,retainedLift=False,physicalJudgmentGenerated=False,
        currentFramesReviewed=False,reviewBasis='explicit_manual_scope_and_original_staging_not_live_contact_review',
        authorizationSha256=binding['sha256'],outputSequence=manager.port.last_sequence)
    return VerifiedInteraction(report,snapshots,{p:sha(raw) for p,raw in snapshots.items()},digest(report),
        expires,manager.port.last_sequence,now,privateRoot=(Path(repository).resolve()/'.local'),deadlineWall=deadline_wall)
