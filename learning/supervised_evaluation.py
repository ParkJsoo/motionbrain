"""Freeze an existing pre-execution model and evaluation rules; no fitting/devices.

This prepares a prospective evaluation, not a physical collection plan or scorer.
Source fingerprints support a later overlap audit; they do not prove independence.
"""
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import re

import numpy as np

from learning.artifacts import describe_artifact, read_json_artifact
from learning.supervised_measurement import finite, read_bound, sha
from learning.supervised_prediction import ALPHA, CONTRACT, INPUTS, load_index, pre_features

SCHEMA = 'motionbrain.frozen_supervised_evaluation.v1'
TRAINING_SOURCES = (
    'learning/supervised_prediction.py', 'learning/joint_prediction.py',
    'learning/supervised_measurement.py', 'learning/supervised_transport.py',
    'tools/train_supervised_prediction.py',
)


def digest(value):
    return sha(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def _hash(value):
    if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{64}', value):
        raise ValueError('source_sha256_required')
    return value


def _number(value):
    try:
        return finite(value)
    except OverflowError:
        return False


def _validate_model(candidate, rows):
    if (candidate.get('availability') != 'preExecution'
            or candidate.get('inputNames') != INPUTS['preExecution']
            or candidate.get('controlEligible') is not False):
        raise ValueError('pre_execution_model_required')
    model = candidate['model']
    if (type(model.get('fitRows')) is not int or model['fitRows'] != len(rows)
            or not _number(model.get('alpha')) or model['alpha'] != ALPHA
            or not _number(model.get('intercept'))):
        raise ValueError('full_development_model_required')
    for key in ('mean', 'scale', 'coefficients'):
        values = model.get(key)
        if not isinstance(values, list) or len(values) != 6 or not all(map(_number, values)):
            raise ValueError('finite_six_column_model_required')
    if any(s <= 0 for s in model['scale']):
        raise ValueError('positive_model_scale_required')
    # Check existing normalization only. Never solve for or change coefficients.
    x = np.asarray([pre_features(row) for row in rows])
    scale = x.std(axis=0)
    scale[scale < 1e-9] = 1.
    if (not np.allclose(model['mean'], x.mean(axis=0), atol=1e-12, rtol=1e-12)
            or not np.allclose(model['scale'], scale, atol=1e-12, rtol=1e-12)):
        raise ValueError('development_normalization_mismatch')


def freeze(index_path, run_dir):
    """Validate saved receipts, then copy only the frozen pre-execution candidate."""
    index_path, run_dir = Path(index_path).resolve(), Path(run_dir).resolve()
    rows, metadata = load_index(index_path)
    receipt_ref = describe_artifact(run_dir, 'result.json')
    receipt = read_json_artifact(run_dir, receipt_ref)
    if (receipt.get('status') != 'trained_and_explored' or receipt.get('count') != len(rows)
            or receipt.get('independentTest') is not False or receipt.get('controlEligible') is not False):
        raise ValueError('development_training_receipt_required')
    artifact = read_json_artifact(run_dir, receipt['artifact'])
    if (artifact.get('schemaVersion') != 'motionbrain.supervised_prediction.v1'
            or artifact.get('contract') != CONTRACT or artifact.get('alpha') != ALPHA
            or artifact.get('target') != 'settled_projected_angle_delta_deg'
            or artifact.get('controlEligible') is not False
            or artifact.get('legacyDataIncluded') is not False
            or artifact.get('independentTestSetAvailable') is not False
            or artifact.get('physicalJointAngleCalibrated') is not False):
        raise ValueError('settled_development_artifact_required')
    if any(artifact['metadata'].get(k) != v for k, v in metadata.items()):
        raise ValueError('training_index_binding_mismatch')
    root = Path(__file__).resolve().parents[1]
    source_hashes = artifact['metadata']['sourceSha256']
    if set(source_hashes) != set(TRAINING_SOURCES):
        raise ValueError('training_source_inventory_mismatch')
    for name, expected in source_hashes.items():
        read_bound(root, name, _hash(expected))
    candidate = artifact['models']['preExecution']
    _validate_model(candidate, rows)
    requested = {(r['pulse']['direction'], r['pulse']['durationMs'], r['pulse']['expectedPwm']) for r in rows}
    expected = {(d, ms, pwm) for d, pwm in (('up', 160), ('down', -160)) for ms in (450, 550)}
    if requested != expected:
        raise ValueError('four_observed_m2_command_conditions_required')

    reports, originals = set(), set()
    measurement_settings = None
    for ref in metadata['sources']:
        data = json.loads(read_bound(index_path.parent, ref['path'], ref['sha256']))
        config = data['measurementConfig']
        settings = {k: config[k] for k in ('width', 'height', 'representation', 'stabilityThreshold')}
        settings['minLinkLengthPx'] = config.get('minLinkLengthPx', 5.)
        if measurement_settings is not None and settings != measurement_settings:
            raise ValueError('consistent_measurement_settings_required')
        measurement_settings = settings
        reports.add(_hash(data['collectionReportSha256']))
        if not isinstance(data.get('sourceHashes'), dict) or not data['sourceHashes']:
            raise ValueError('original_source_fingerprints_required')
        originals.update(_hash(h) for h in data['sourceHashes'].values())
    if len(reports) != len(metadata['sources']):
        raise ValueError('one_measurement_source_per_collection_report_required')
    conditions = []
    for direction, duration, pwm in sorted(expected):
        selected = [r for r in rows if (r['pulse']['direction'], r['pulse']['durationMs']) == (direction, duration)]
        conditions.append(dict(direction=direction, durationMs=duration, expectedPwm=pwm,
                               motor=2, joint='wrist', percent=63, developmentCount=len(selected),
                               observedAngleDeg=[min(r['before']['value'] for r in selected),
                                                 max(r['before']['value'] for r in selected)],
                               observedVelocityDegS=[min(r['before']['velocityPerSecond'] for r in selected),
                                                     max(r['before']['velocityPerSecond'] for r in selected)]))
    baseline = {d: dict(deltaDeg=float(np.mean([r['delta'] for r in rows if r['pulse']['direction'] == d])),
                        fitRows=sum(r['pulse']['direction'] == d for r in rows)) for d in ('up', 'down')}
    content_fingerprints = []
    for row in rows:
        content_fingerprints.append(digest({k: v for k, v in row.items()
                                           if k not in ('id', 'captureGroup', 'split', 'measurementSource')}))
    bundle = dict(
        schemaVersion=SCHEMA, createdAt=datetime.now(timezone.utc).isoformat(), contract=CONTRACT,
        target=artifact['target'], model=copy.deepcopy(candidate),
        modelSha256=digest(candidate), baselines=dict(noChangeDeltaDeg=0., directionMean=baseline),
        provenance=dict(trainingReceipt=receipt_ref, trainingArtifact=receipt['artifact'],
                        dataset=metadata, trainingSourceSha256=source_hashes,
                        freezeSourceSha256={p: sha((root/p).read_bytes()) for p in (
                            'learning/supervised_evaluation.py', 'tools/freeze_supervised_evaluation.py')},
                        excludedCaptureGroups=metadata['captureGroups'], excludedRowIds=[r['id'] for r in rows],
                        excludedRowContentSha256=sorted(content_fingerprints),
                        excludedCollectionReportSha256=sorted(reports),
                        excludedOriginalSourceSha256=sorted(originals)),
        scope=dict(plannedConditionCount=4, conditions=conditions,
                   observedBoundsAreSafetyLimits=False, boundsPolicy='report_all_eligible_and_flag_out_of_observed_bounds'),
        measurementProtocol=dict(settings=measurement_settings, beforeDispatchWindowSeconds=[-.75, -.10],
                                 afterConfirmedOffWindowSeconds=[2.25, 3.], minimumFrames=3,
                                 maximumFrameGapSeconds=.3, markerRegistration='freeze_before_motion'),
        evaluationPolicy=dict(
            primaryMetric='preExecution_maeDeg_minus_frozenDirectionMean_maeDeg',
            comparisonRows='same_eligible_rows_within_declared_command_scope',
            negativeDifferenceMeans='lower_error_on_this_group_only',
            secondaryMetrics=['maeDeg', 'rmseDeg', 'biasDeg', 'maxAbsoluteErrorDeg', 'directionAccuracy'],
            breakdowns=['captureGroup', 'direction', 'requestedDurationMs', 'observedInputBounds'],
            denominators=['planned', 'attempted', 'executed', 'outcome_unknown', 'not_dispatched',
                          'not_started', 'measurement_rejected', 'eligible', 'within_command_scope'],
            futureMeasurementInput='complete_measurement_rows_and_collection_reports_not_eligible_index_only',
            refitOnEvaluationData=False, updateNormalization=False, updateBaselines=False,
            usePostExecutionFeatures=False, selectCommandsWithModel=False,
            exclusionsBasedOnPredictionError=False, requiredEligibleCount=None,
            newGroupNameAloneProvesIndependence=False,
            requiredEvidenceBeforeScoring=['new_capture_after_freeze', 'no_original_or_row_overlap',
                                           'pre_motion_measurement_config_hash', 'pre_motion_finite_attempt_budget'],
            changedMeasurementConfigAfterOutcomes='reclassify_group_as_development',
            claimsNotEstablished=['live_pre_command_prediction', 'physical_control_improvement',
                                  'other_day_or_view_generalization']),
        collectionReadiness=dict(status='pending_current_field_conditions_and_separate_authorization',
                                 collectionAuthorized=False, attemptBudget=None, conditionCounts=None,
                                 retryUntilEligibleTarget=False, resendFailedAttempts=False),
        newModelFitPerformed=False, evaluationPerformed=False, controlEligible=False,
        physicalJointAngleCalibrated=False,
    )
    return bundle
