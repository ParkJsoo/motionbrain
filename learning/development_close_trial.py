"""첫 CLOSE의 명시 개발 시험 진입. 학습 지원·깊이·파지·안전의 보증이 아니다.

원본 bytes와 별도로 고정한 기대 SHA를 받는다. 호출자는 원본 경로, 현재 영상의
신선도·부팅·출력 OFF 및 기존 모델의 실제 명령 선택/출력 제한을 검증해야 한다.
"""
import copy
import hashlib
import json
from learning import limited_scene_cycle_assessment as image
from learning import reference_entry_alignment as strict
from learning import m1_near_action_model as response

SCHEMA = 'motionbrain.development_first_close_trial.v1'
SOURCE_NAMES = {'reference', 'goal', 'model', 'pose_policy', 'native_scope', 'native_result', 'native_proof'}
POLICY = dict(absoluteSizeBandPx=2., externalAxisObjectWidthFraction=.25,
              actualPoseHoldBandDeg=.4, maximumCloseCommands=3,
              meaning='declared_development_image_and_pose_transfer_heuristics_not_calibrated_accuracy_or_safety')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _supported(policy, angle):
    return min(abs(angle-r['beforeM4MedianDeg']) for r in policy['beforeM4Mapping']) <= policy['admissionRadiusDeg']


def _construct(sources, bindings):
    image._require(set(sources) == SOURCE_NAMES and set(bindings) == SOURCE_NAMES
                   and all(image._sha(v) for v in bindings.values()), 'close_trial_source_bindings_invalid')
    ref = sources['reference']; goal = sources['goal']; model = sources['model']
    pose = sources['pose_policy']; scope = sources['native_scope']; result = sources['native_result']; proof = sources['native_proof']
    image._require(digest(ref['contract']) == ref['sha256'] and digest(goal['goal']) == goal['sha256']
                   and goal['goal']['referenceContractSha256'] == ref['sha256'], 'close_trial_reference_goal_binding_changed')
    reference = ref['contract']
    refs = [dict(external=reference['upAfterExternal'], near=reference['upAfterNear']),
            dict(external=reference['closeBeforeExternal'], near=reference['closeBeforeNear'])]
    strict._references(refs)
    response._validate(model)
    image._require(reference['modelFileSha256s']['near'] == bindings['model']
                   and pose['frozenModelFileSha256'] == bindings['model']
                   and reference['nearContractSha256'] == model['observationContractSha256'] == pose['observationContractSha256'],
                   'close_trial_frozen_model_binding_changed')
    image._require(pose['schemaVersion'] == 'motionbrain.M1_reported_angle_cohort_admission.v3'
                   and pose['baseRadiusDeg'] == .5 and pose['reportedAngleOneLsbDeg'] == 360./4096.
                   and pose['admissionRadiusDeg'] == .5+360./4096.
                   and pose['modelWeightsChanged'] is False and pose['imageLocalSupportChanged'] is False,
                   'close_trial_original_pose_policy_changed')
    rows = pose['beforeM4Mapping']
    image._require(len(rows) == len(model['trainingRows']) == 7 and
                   [(r['sessionId'], r['sourceSessionSha256']) for r in rows] ==
                   [(r['sessionId'], r['sourceSha256']) for r in model['trainingRows']]
                   and all(image._number(r['beforeM4MedianDeg']) for r in rows), 'close_trial_pose_source_mapping_changed')
    target = scope['targetAngleDeg']; anchor = result['actualAngleDeg']
    image._require(image._number(target) and image._number(anchor) and _supported(pose, target)
                   and not _supported(pose, anchor), 'close_trial_target_or_actual_pose_role_invalid')
    image._require(result['scopeSha256'] == digest(scope) and result['status'] == 'completed'
                   and result['errors'] == [] and result['commandOutcome'] == 'receipt_recorded'
                   and result['modelSelected'] is True and result['automaticRetry'] is False
                   and all(result[k] is True for k in ('actualAngleWithinNativeTolerance', 'deviceTargetReached',
                       'nativeReportedTargetReached', 'finalIdleOffVerified'))
                   and result['nativeTerminalReason'] == 'TARGET_REACHED' and result['nativeSettledToleranceDeg'] == .4
                   and abs(anchor-target) <= .4 and abs(result['actualAngleErrorDeg']-(anchor-target)) < 1e-9,
                   'close_trial_successful_native_source_required')
    firmware = proof['firmwarePolicy']
    image._require(proof['controllerBootId'] == scope['controllerBootId'] and proof['cameraBootId'] == scope['cameraBootId']
                   and proof['M4Deg'] == anchor and proof['allFiveOff'] is True and proof['currentState'] == 'IDLE'
                   and proof['nativeTerminalReason'] == 'TARGET_REACHED' and firmware['targetDeg'] == target
                   and firmware['angleDeg'] == anchor and firmware['settledSuccessToleranceDeg'] == .4
                   and firmware['active'] is False and firmware['correctionActive'] is False,
                   'close_trial_native_final_proof_changed')
    strict._point(goal['goal']['targetVirtualPointPx'])
    return dict(schemaVersion=SCHEMA, policy=copy.deepcopy(POLICY), sources=copy.deepcopy(sources),
                sourceFileSha256s=copy.deepcopy(bindings), nativeActualAnchorDeg=anchor,
                originalPoseSupported=False, newTrialTransferHypothesis=True,
                learnedM1PoseSupportExpanded=False, sourceNativeTargetOriginalPoseSupported=True,
                developmentOnly=True, controlEligible=False, prospectivelyTested=False,
                physicalGraspVerified=False, metricDepthVerified=False, safetyBound=False)


def build_contract(source_raw, *, expected_source_sha256s):
    """이름별 JSON bytes와 독립적으로 고정한 파일 SHA를 검증하여 시험 계약을 만든다."""
    image._require(set(source_raw) == SOURCE_NAMES and set(expected_source_sha256s) == SOURCE_NAMES,
                   'close_trial_exact_sources_required')
    sources = {}
    for name, raw in source_raw.items():
        image._require(isinstance(raw, bytes) and image._sha(expected_source_sha256s[name])
                       and hashlib.sha256(raw).hexdigest() == expected_source_sha256s[name], 'close_trial_source_file_changed')
        sources[name] = json.loads(raw)
    contract = _construct(sources, expected_source_sha256s)
    return dict(contract=contract, sha256=digest(contract))


def assess(prepared, external, near, *, table_baseline, current_m4_deg,
           controller_boot_id, camera_boot_id, commands_used=0):
    """최초 CLOSE에만 적용한다. 후속 부분닫힘·모델 선택·명령 실행을 대신하지 않는다."""
    old = dict(accepted=False, evidence=dict(reason='trial_contract_not_validated'))
    def evaluate():
        nonlocal old
        contract = prepared['contract']
        image._require(digest(contract) == prepared['sha256']
                       and _construct(contract['sources'], contract['sourceFileSha256s']) == contract,
                       'close_trial_contract_changed')
        src = contract['sources']; ref = src['reference']['contract']; scope = src['native_scope']
        refs = [dict(external=ref['upAfterExternal'], near=ref['upAfterNear']),
                dict(external=ref['closeBeforeExternal'], near=ref['closeBeforeNear'])]
        goal = src['goal']['goal']['targetVirtualPointPx']
        old = strict.assess(refs, external, near, goal)
        image._require(type(commands_used) is int and commands_used == 0, 'close_trial_first_entry_only')
        image._require(controller_boot_id == scope['controllerBootId'] and camera_boot_id == scope['cameraBootId'],
                       'close_trial_boot_changed')
        image._require(image._number(current_m4_deg) and abs(current_m4_deg-contract['nativeActualAnchorDeg']) <= .4,
                       'close_trial_actual_pose_hold_exceeded')
        image._records([*refs, dict(external=external, near=near), dict(external=table_baseline, near=near)])
        ext = image._external(external); image._table(image._external(table_baseline), ext)
        image._require(near['nearClosingVisibilityGate']['horizontalInside'] is True, 'close_trial_object_not_inside_jaws')
        proxy = strict._point(external['virtualProxyPx']); errors = [proxy[i]-goal[i] for i in range(2)]
        axis_band = ext['bbox'][2]*.25
        image._require(max(map(abs, errors)) <= axis_band, 'close_trial_external_axis_mismatch')
        size_bounds = {}
        for name, index in [('width', 2), ('height', 3)]:
            sizes = [r['near']['object']['bbox'][index] for r in refs]
            low, high = min(sizes)-2., max(sizes)+2.
            actual = near['object']['bbox'][index]
            image._require(low <= actual <= high, 'close_trial_absolute_'+name+'_mismatch')
            size_bounds[name] = dict(actual=actual, lower=low, upper=high)
        model = src['model']; gap = near['state'][2]
        image._require(model['minimumGapPx'] <= gap <= model['maximumOpeningGapPx'], 'close_trial_gap_outside_original_model_bounds')
        x, _, width, _ = near['object']['bbox']
        image._require(near['points']['left'][0] <= x and x+width <= near['points']['right'][0], 'close_trial_object_does_not_fit')
        return dict(trialContractSha256=prepared['sha256'], sourceFileSha256s=contract['sourceFileSha256s'],
                    signedErrorsPx=errors, externalAxisBandPx=axis_band, absoluteSizeBoundsPx=size_bounds,
                    gapPx=gap, originalOpeningReferenceRequired=False,
                    originalPoseSupported=_supported(src['pose_policy'], current_m4_deg),
                    learnedM1PoseSupportExpanded=False, newTrialTransferHypothesis=True,
                    actualPoseAnchorDeg=contract['nativeActualAnchorDeg'], actualPoseHoldBandDeg=.4,
                    modelSelectionRequired=True, allowedCommand=dict(joint='gripper', direction='close', percent=70, durationMs=1000),
                    maximumCloseCommands=3, sourceImages=image._bindings([*refs, dict(external=external, near=near)]))
    result = image._result('development_first_close_trial', evaluate)
    result.update(originalStrictAssessment=old, developmentOnly=True, controlEligible=False,
                  learnedM1PoseSupportExpanded=False, newTrialTransferHypothesis=True,
                  assessmentMeaning=POLICY['meaning'])
    return result
