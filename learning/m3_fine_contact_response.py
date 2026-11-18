"""명시 승인된 200/250ms 직접 실측만 쓰는 미세 하강 개발 모델.

접촉 전이·무동작·관측 잡음을 보존하며 기존 350ms 자료를 혼합하거나 시간을
비례 축소하지 않는다. 범주당 한 원본의 응답은 명령 평균과 같으며 독립 성능,
실제 접촉·하중 전이·안전 범위·새 실물 평가 허용을 증명하지 않는다.
"""
from __future__ import annotations
import copy

import numpy as np

from learning import marker_core_tool_bridge as bridge
from learning import m3_direct_tool_response as direct
from learning import attended_collection_profile as profiles
from learning import supervised_transport as transport
from learning.near_gripper_tip_observer import digest

SCHEMA = 'motionbrain.M3_fine_contact_response.v2'
COMMANDS = [dict(joint='elbow', direction='down', percent=60, durationMs=duration)
            for duration in (200, 250)]
SCALES = np.array([10., 10., .5, 2.])


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def command_key(command):
    require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'}
            and type(command['percent']) is int and type(command['durationMs']) is int
            and command in COMMANDS, 'fine_contact_exact_200_or_250_DOWN_required')
    return command['durationMs']


def timing_policy(command):
    return (transport.ELBOW_CONTACT_COMBINED_200_POLICY if command_key(command) == 200
            else transport.ELBOW_CONTACT_COMBINED_250_POLICY)


def policy():
    return dict(estimator='nearest_observed_delta_per_exact_command', scales=SCALES.tolist(),
        maximumStandardDistance=1., maximumM4DifferenceDeg=.5, maximumImageAxisDifferenceDeg=2.,
        minimumSourcesPerCategory=1, maximumSources=2, maximumCommands=1,
        singletonIsDevelopmentOnly=True, timeScalingAllowed=False, old350SourcesMixed=False,
        contactTransitionGeneralizationVerified=False, calibratedUncertainty=False, safetyBound=False)


def source_context(value):
    """촬영·부팅 묶음은 추론 특징이 아닌 명시적 전이 허용 조건이다."""
    require(isinstance(value, dict) and set(value) == {'controllerBootId', 'cameraBootId', 'cameraIdentity'},
            'fine_contact_source_context_required')
    for key in ('controllerBootId', 'cameraBootId'):
        boot = value[key]
        require(isinstance(boot, str) and len(boot) == 32
                and all(c in '0123456789abcdef' for c in boot), 'fine_contact_source_boot_required')
    identity = value['cameraIdentity']
    require(isinstance(identity, dict) and set(identity) == {'deviceId', 'helperSha256', 'configuration', 'viewId'}
            and all(isinstance(identity[k], str) and 0 < len(identity[k]) <= 200 for k in ('deviceId', 'viewId'))
            and direct.sha(identity['helperSha256']), 'fine_contact_camera_identity_required')
    cfg = identity['configuration']
    require(isinstance(cfg, list) and len(cfg) == 5 and direct.sha(cfg[0])
            and type(cfg[1]) is int and cfg[1] == 1920 and type(cfg[2]) is int and cfg[2] == 1080
            and type(cfg[3]) is bool and type(cfg[4]) is int and 0 <= cfg[4] < 360,
            'fine_contact_camera_configuration_required')
    return copy.deepcopy(value)


def _receipt(row):
    command = row['command']; receipt = row['receipt']
    expected = dict(**command, motor=3, expectedPwm=153)
    require(isinstance(receipt, dict) and receipt.get('requestedPulse') == expected
            and receipt.get('offVerified') is True and receipt.get('monitorStatusVerified') is True
            and receipt.get('monitorTimingPolicy') == timing_policy(command)
            and receipt.get('monitorReadBudgetRevision') == transport.ELBOW_CONTACT_READ_BUDGET_REVISION,
            'fine_contact_confirmed_monitor_receipt_required')
    require(all(direct.number(receipt.get(k)) for k in ('dispatchBefore', 'dispatchAfter', 'offAt'))
            and receipt['dispatchBefore'] <= receipt['dispatchAfter'] <= receipt['offAt'],
            'fine_contact_receipt_clock_order')
    trace = receipt.get('outputTrace')
    require(isinstance(trace, list) and bool(trace), 'fine_contact_driver_trace_required')
    for index, entry in enumerate(trace):
        require(isinstance(entry, dict) and all(type(entry.get(k)) is int for k in ('sequence', 'atMs', 'motor', 'pwm'))
                and entry['sequence'] > 0 and entry['atMs'] >= 0 and 1 <= entry['motor'] <= 5
                and (entry['pwm'] == 0 or entry['motor'] == 3 and 0 < entry['pwm'] <= 153),
                'fine_contact_driver_output_outside_scope')
        if index:
            require(entry['sequence'] == trace[index-1]['sequence']+1
                    and entry['atMs'] >= trace[index-1]['atMs'], 'fine_contact_driver_trace_gap')
    positive = [i for i, e in enumerate(trace) if e['motor'] == 3 and e['pwm'] > 0]
    require(bool(positive) and any(e['motor'] == 3 and e['pwm'] == 0 for e in trace[positive[-1]+1:]),
            'fine_contact_positive_output_and_later_OFF_required')


def _rows(rows, projector):
    require(isinstance(rows, list) and 1 <= len(rows) <= 2, 'fine_contact_one_or_two_direct_sources_required')
    sessions, sources, categories = set(), set(), set()
    cohort = None
    for row in rows:
        require(isinstance(row, dict), 'fine_contact_source_row_required')
        context = source_context(row.get('sourceContext'))
        if cohort is None:
            cohort = context
        require(context == cohort, 'fine_contact_source_cohorts_must_not_mix')
        key = command_key(row['command'])
        require(isinstance(row.get('sessionId'), str) and bool(row['sessionId'])
                and row['sessionId'] not in sessions and direct.sha(row.get('sourceSha256'))
                and row['sourceSha256'] not in sources and key not in categories,
                'fine_contact_unique_manual_source_category_required')
        require(row.get('manualCollection') is True and row.get('modelSelected') is False
                and row.get('dataCollection') is True and row.get('sessionPhase') == 'completed'
                and row.get('collectionProfile') == profiles.collection_profile(profiles.ATTENDED_ELBOW_CONTACT_FINE)
                and row.get('monitorTimingPolicy') == timing_policy(row['command']),
                'fine_contact_explicit_manual_profile_provenance')
        require(row.get('beforeSupport') == 'not_supported'
                and row.get('afterSupport') in ('not_supported', 'supported', 'unknown'),
                'fine_contact_support_labels_required')
        before = bridge.observation(projector, row['before']); after = bridge.observation(projector, row['after'])
        require(abs(before[2]-after[2]) <= .5, 'fine_contact_fixed_M4_changed')
        evidence = row.get('sourceEvidence', {})
        require(all(direct.sha(evidence.get(k)) for k in ('beforeExternalImageSha256', 'afterExternalImageSha256',
                'beforeGripperImageSha256', 'afterGripperImageSha256', 'receiptSha256', 'supportAnnotationSha256'))
                and evidence['beforeExternalImageSha256'] == row['before']['imageSha256']
                and evidence['afterExternalImageSha256'] == row['after']['imageSha256']
                and evidence['receiptSha256'] == digest(row['receipt']), 'fine_contact_raw_source_bindings_required')
        _receipt(row)
        sessions.add(row['sessionId']); sources.add(row['sourceSha256']); categories.add(key)


def fit(rows, *, projector, audit_sources):
    projector.verify(); _rows(rows, projector)
    require(callable(audit_sources) and audit_sources(copy.deepcopy(rows)) == rows,
            'fine_contact_actual_source_reaudit_required')
    return dict(schemaVersion=SCHEMA, supportPolicy=policy(), trainingRows=copy.deepcopy(rows),
        sourceContext=copy.deepcopy(rows[0]['sourceContext']), sourceContextSha256=digest(rows[0]['sourceContext']),
        admissionOnlyFields=['sourceContext', 'current_support'],
        observerBridgeContract=copy.deepcopy(projector.contract), observerBridgeContractSha256=projector.binding,
        coordinateSourceOriginalModelFileSha256=projector.original_model_file_sha256,
        dynamicsSource='new_manual_200_250_only', trainingRowsSha256=digest(rows),
        categories=[dict(command=copy.deepcopy(r['command']), sources=1,
            contactTransition=r['afterSupport'] == 'supported', afterSupport=r['afterSupport']) for r in rows],
        controlEligible=False, developmentOnly=True, independentTest=False, calibratedUncertainty=False,
        uncertaintyInterval=None, prospectiveCommands=0, baselineSuperiorityVerified=False,
        contactTransitionGeneralizationVerified=False, actualSupportOrReleaseVerifiedByModel=False,
        old350SourcesMixed=False, timeScalingAllowed=False)


def validate(model, projector):
    projector.verify()
    require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
            and model.get('supportPolicy') == policy() and model.get('observerBridgeContract') == projector.contract
            and model.get('observerBridgeContractSha256') == projector.binding
            and model.get('coordinateSourceOriginalModelFileSha256') == projector.original_model_file_sha256
            and model.get('dynamicsSource') == 'new_manual_200_250_only', 'fine_contact_frozen_contract_changed')
    _rows(model.get('trainingRows'), projector)
    expected = fit(model['trainingRows'], projector=projector, audit_sources=lambda rows: rows)
    require(model == expected, 'fine_contact_frozen_model_changed')


def predict(model, projector, current, command, *, current_support='unknown', current_context=None):
    validate(model, projector); command_key(command)
    require(source_context(current_context) == model['sourceContext'], 'fine_contact_current_context_not_supported')
    require(current_support == 'not_supported', 'fine_contact_current_air_support_confirmation_required')
    state = bridge.observation(projector, current)
    rows = [r for r in model['trainingRows'] if r['command'] == command]
    require(bool(rows), 'fine_contact_exact_category_not_trained')
    before = np.array([r['before']['state'] for r in rows]); delta = np.array([direct.difference(r['after']['state'], r['before']['state']) for r in rows])
    difference = direct.difference(before, state); distances = np.linalg.norm(difference/SCALES, axis=1)
    nearest = int(np.argmin(distances))
    require(distances[nearest] <= 1. and abs(difference[nearest, 2]) <= .5
            and abs(difference[nearest, 3]) <= 2., 'fine_contact_current_outside_local_support')
    predicted = state+delta[nearest]; mean = state+delta.mean(axis=0)
    predicted[3] = (predicted[3]+180.) % 360.-180.; mean[3] = (mean[3]+180.) % 360.-180.
    require(0 <= predicted[0] < 1920 and 0 <= predicted[1] < 1080 and 0 <= predicted[2] < 360,
            'fine_contact_prediction_outside_image')
    return dict(command=copy.deepcopy(command), predictedState=predicted.tolist(), commandMeanState=mean.tolist(),
        noChangeState=state.tolist(), observedSourceDeltaState=delta[nearest].tolist(),
        nearestStandardDistance=float(distances[nearest]), singleton=True,
        nearestAndCommandMeanIdentical=True, contactTransitionSource=rows[nearest]['afterSupport'] == 'supported',
        sourceAfterSupport=rows[nearest]['afterSupport'], sourceContactOutcomeKnown=rows[nearest]['afterSupport'] != 'unknown',
        contactTransitionGeneralizationVerified=False, inputSha256=digest(current), modelSha256=digest(model),
        executionInputFields=['current_observation_state', 'exact_planned_command'],
        admissionOnlyFields=['sourceContext', 'current_support'],
        actualOutputAndAfterSupportUsedAsExecutionInputs=False, controlEligible=False,
        prospectiveCommands=0, calibratedUncertainty=False, baselineSuperiorityVerified=False)


def select(model, projector, current, goal, *, current_support='unknown', current_context=None, commands_used=0):
    validate(model, projector); state = bridge.observation(projector, current)
    require(isinstance(goal, list) and len(goal) == 2 and all(direct.number(v) for v in goal)
            and 0 <= goal[0] < 1920 and 0 <= goal[1] < 1080, 'fine_contact_explicit_image_goal_required')
    require(type(commands_used) is int and commands_used in (0, 1), 'fine_contact_single_command_budget')
    target = np.array(goal); stop = dict(kind='stop', command=None, predictedState=state.tolist(),
        errorPx=float(np.linalg.norm(state[:2]-target)))
    candidates, rejected = [stop], []
    if not commands_used:
        for command in COMMANDS:
            try:
                response = predict(model, projector, current, command, current_support=current_support, current_context=current_context)
                candidates.append(dict(kind='action', command=copy.deepcopy(command), response=response,
                    predictedState=response['predictedState'], errorPx=float(np.linalg.norm(np.array(response['predictedState'][:2])-target)),
                    commandMeanErrorPx=float(np.linalg.norm(np.array(response['commandMeanState'][:2])-target))))
            except ValueError as error:
                rejected.append(dict(command=copy.deepcopy(command), reason=str(error)))
    candidates.sort(key=lambda c: c['errorPx']); performed=len(candidates) > 1
    return dict(schemaVersion=SCHEMA+'.decision', selected=copy.deepcopy(candidates[0]), candidates=candidates,
        rejected=rejected, modelSha256=digest(model), inputSha256=digest(current), targetVirtualPointPx=goal.copy(),
        selectionOwner='budget_guard' if commands_used else 'fine_contact_response_comparison' if performed else 'admission_guard',
        modelPredictionPerformed=performed, modelSelectedStop=not commands_used and performed and candidates[0]['kind'] == 'stop',
        modelSelectedAction=candidates[0]['kind'] == 'action', controlEligible=False,
        currentSupportAdmission=current_support, actualTableSupportVerifiedByModel=False,
        sourceContextSha256=model['sourceContextSha256'], admissionOnlyFields=['sourceContext', 'current_support'],
        additionalPhysicalEvaluationAuthorized=False, baselineSuperiorityVerified=False)
