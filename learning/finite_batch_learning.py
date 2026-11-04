"""Offline finite endpoint batches: prospective inputs, development-only evaluation.

An index lists all 16 planned slots (4 commissioning + 12 development), including
failures and unstarted slots. No discovery/globbing, legacy import, dispatch or
retry is performed. Cached measurements are bound to original receipts; callers
can additionally recompute frames with reaudit_frames=True before training.
"""
import copy
import json
from pathlib import Path
import platform

import numpy as np

from .finite_profile_measurement import CONTRACT, RESULT_SCHEMA, SCHEMA as REPORT_SCHEMA
from .finite_profile_measurement import digest, execution_identity, measure, validate_output
from .finite_profile_plan import BLOCKS, FEATURES, catalog, prospective_input, validate_prospective_features
from .joint_prediction import predict
from .supervised_measurement import finite, read_bound, sha, validate_config, wrap

INDEX_SCHEMA = 'motionbrain.finite_batch_index.v1'
RESULT = 'motionbrain.finite_batch_learning.v1'
ACQUISITION = dict(kind='fixed_camera_warmup', warmupSeconds=3.0)
SOURCE_KEYS = {'result', 'report', 'measurement', 'beforeReceipt', 'closedEvidence', 'prospectiveInput'}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def same(a, b):
    return json.dumps(a, sort_keys=True, allow_nan=False) == json.dumps(b, sort_keys=True, allow_nan=False)


def bound_directory(base, name):
    require(isinstance(name, str) and name and not Path(name).is_absolute(), 'capture_relative_path_required')
    path = (base / name).resolve()
    require(path.is_relative_to(base.resolve()) and path.is_dir(), 'capture_path_outside_index')
    return path


def load_index(path, *, reaudit_frames=False):
    """Load all fixed slots; authenticate bytes and revalidate finite receipts.

    With reaudit_frames=True, re-run the original endpoint measurement against
    the capture directory. Any mismatch rejects the batch rather than quietly
    excluding a formerly eligible row. Indices are provenance, not signatures.
    """
    path = Path(path).resolve()
    raw = path.read_bytes()
    index = json.loads(raw)
    require(index.get('schemaVersion') == INDEX_SCHEMA and index.get('contract') == CONTRACT
            and index.get('actionContract') == 'finite_profile.v1' and index.get('split') == 'development',
            'finite_development_batch_required')
    group = index.get('captureGroup')
    require(isinstance(group, str) and bool(group), 'capture_group_required')
    require(same(index.get('acquisitionPolicy'), ACQUISITION), 'acquisition_policy_mismatch')
    require(type(reaudit_frames) is bool, 'reaudit_policy')
    sources = {}

    def read_ref(ref):
        require(isinstance(ref, dict) and set(ref) == {'path', 'sha256'}
                and isinstance(ref['path'], str) and isinstance(ref['sha256'], str)
                and len(ref['sha256']) == 64, 'source_reference')
        data = read_bound(path.parent, ref['path'], ref['sha256'])
        sources[ref['path']] = ref['sha256']
        return json.loads(data)

    config = read_ref(index['measurementConfig'])
    validate_config(config)
    require(config['representation']['kind'] == 'relative_angle' and config['stabilityThreshold'] == .5,
            'finite_measurement_config')
    config_hash = digest(config)
    slots = index.get('slots')
    require(isinstance(slots, list) and len(slots) == 16, 'all_16_planned_slots_required')
    orders = BLOCKS['P'] + BLOCKS['D']
    ids, block_ids, file_paths = set(), {}, set()
    attempts, rows = [], []
    halted = False
    for i, slot in enumerate(slots):
        require(isinstance(slot, dict), 'slot_object')
        command, block, condition = (slot.get(k) for k in ('id', 'blockId', 'condition'))
        role = 'commissioning' if i < 4 else 'development'
        require(isinstance(command, str) and command and command not in ids, 'duplicate_or_missing_command')
        ids.add(command)
        require(condition == orders[i // 4][i % 4] and slot.get('phaseRole') == role, 'fixed_slot_order_or_role')
        require(isinstance(block, str) and block, 'block_identity_required')
        if i % 4 == 0:
            require(block not in block_ids.values(), 'block_reused')
            block_ids[i // 4] = block
        require(block_ids[i // 4] == block, 'logical_block_split')
        refs = slot.get('sources')
        require(isinstance(refs, dict) and set(refs) <= SOURCE_KEYS, 'slot_sources')
        require(not halted or not refs, 'attempt_after_batch_stop')
        entry = dict(id=command, blockId=block, condition=condition, phaseRole=role,
                     attempted=False, executionComplete=False, measurementEligible=False,
                     finalIdleOffVerified=False, includedForTraining=False, reason=None)
        if not refs:
            require(isinstance(slot.get('unstartedReason'), str) and bool(slot['unstartedReason']), 'unstarted_reason_required')
            entry.update(status='unstarted', reason=slot['unstartedReason'])
            attempts.append(entry)
            halted = True
            continue
        require('result' in refs and slot.get('unstartedReason') is None, 'attempt_result_required')
        for ref in refs.values():
            require(isinstance(ref, dict) and isinstance(ref.get('path'), str), 'source_reference')
            require(ref['path'] not in file_paths, 'source_reused_across_slots')
            file_paths.add(ref['path'])
        loaded = {key: read_ref(ref) for key, ref in refs.items()}
        outcome = loaded['result']
        require(isinstance(outcome, dict) and all(type(outcome.get(k)) is bool for k in
                ('executionComplete', 'measurementEligible', 'finalIdleOffVerified')),
                'attempt_result_flags')
        entry.update(attempted=True, executionComplete=outcome['executionComplete'],
                     measurementEligible=outcome['measurementEligible'], finalIdleOffVerified=outcome['finalIdleOffVerified'])
        measurement, report = loaded.get('measurement'), loaded.get('report')
        if report is not None:
            require(report.get('schemaVersion') == REPORT_SCHEMA and report.get('actionContract') == 'finite_profile.v1'
                    and report.get('split') == 'development' and report.get('captureGroup') == group
                    and report.get('trial', {}).get('id') == command
                    and same(report.get('requestedProfile'), catalog()[condition]), 'report_contract_or_identity')
        if measurement is not None:
            require(measurement.get('schemaVersion') == RESULT_SCHEMA and measurement.get('contract') == CONTRACT
                    and measurement.get('actionContract') == 'finite_profile.v1'
                    and measurement.get('split') == 'development' and measurement.get('captureGroup') == group
                    and measurement.get('measurementConfigSha256') == config_hash
                    and measurement.get('physicalJointAngleCalibrated') is False
                    and measurement.get('units') == 'projected_degrees', 'measurement_contract')
            require(outcome['measurementEligible'] == (measurement.get('status') == 'eligible'), 'measurement_status_mismatch')
        if not outcome['measurementEligible']:
            entry.update(status='rejected', reason=outcome.get('failure') or (measurement or {}).get('reason') or 'measurement_not_eligible')
            attempts.append(entry)
            halted = True
            continue
        require(set(loaded) == SOURCE_KEYS and outcome['executionComplete'] and outcome['finalIdleOffVerified'],
                'eligible_sources_or_completion_missing')
        before, closed, prospective = (loaded[k] for k in ('beforeReceipt', 'closedEvidence', 'prospectiveInput'))
        trial = report['trial']
        require(same(trial.get('beforeReceipt'), before) and same(trial.get('closedEvidence'), closed), 'original_receipt_changed')
        require(measurement.get('captureReportSha256') == digest(report), 'measurement_report_changed')
        require(measurement.get('beforeReceiptSha256') == before.get('receiptSha256')
                and same(measurement.get('before'), before.get('before')), 'measurement_before_changed')
        require(before.get('measurementConfigSha256') == config_hash, 'before_config_changed')
        for key, name in (('beforeReceipt', 'before-receipt.json'), ('closedEvidence', 'closed-actual-evidence.json')):
            require(measurement.get('sourceHashes', {}).get('../' + name) == refs[key]['sha256'], 'measurement_original_hash_changed')
        identity = execution_identity(report)
        inputs = prospective_input(condition, before, identity, trial['dispatchBefore'])
        require(isinstance(prospective, dict) and set(prospective) == set(inputs), 'prospective_input_shape')
        for key in ('features', 'beforeReceiptSha256', 'executionIdentity', 'requestedProfileSha256'):
            require(same(prospective[key], inputs[key]), 'prospective_input_changed')
        age = prospective.get('beforeAnchorAgeSeconds')
        require(finite(age) and 0 <= age <= inputs['beforeAnchorAgeSeconds'], 'prospective_input_after_dispatch')
        anchor = before['anchorHostMonotonic']
        dispatch, dispatched, off, quiet, post, release = (trial[k] for k in
            ('dispatchBefore', 'dispatchAfter', 'hostOffReceiptAt', 'quietVerifiedUntil', 'postEnd', 'releaseStartedAt'))
        require(all(finite(v) for v in (dispatch, dispatched, off, quiet, post, release))
                and anchor <= dispatch <= dispatched <= off and min(quiet, post, release) >= off + 3
                and release >= quiet, 'host_endpoint_timing')
        summary = validate_output(trial, report['requestedProfile'], identity)
        require(same(summary, measurement.get('actualOutputSummary')), 'actual_summary_changed')
        after, delta = measurement.get('after', {}), measurement.get('delta')
        require(finite(after.get('value')) and finite(delta)
                and abs(delta - wrap(after['value'] - before['before']['value'])) < 1e-9, 'endpoint_delta_changed')
        for endpoint, wanted in ((before['before'], [anchor - .75, anchor - .1]), (after, [off + 2.25, off + 3])):
            require(same(endpoint.get('window'), wanted) and finite(endpoint.get('spread'))
                    and 0 <= endpoint['spread'] <= .5 and isinstance(endpoint.get('frameIds'), list)
                    and len(endpoint['frameIds']) >= 3 and len(set(endpoint['frameIds'])) == len(endpoint['frameIds']),
                    'endpoint_window_quality')
        if reaudit_frames:
            capture = bound_directory(path.parent, slot.get('capturePath'))
            checked = measure(report, config, capture)
            require(same(checked, measurement), 'raw_frame_measurement_mismatch')
        entry.update(status='eligible', includedForTraining=role == 'development')
        attempts.append(entry)
        if role == 'development':
            rows.append(dict(id=command, blockId=block, condition=condition, captureGroup=group,
                             phaseRole=role, features=inputs['features'], delta=delta,
                             beforeReceiptSha256=before['receiptSha256'], actualOutputSummary=summary))
    counts = dict(planned=16, attempted=sum(a['attempted'] for a in attempts),
                  unstarted=sum(a['status'] == 'unstarted' for a in attempts),
                  eligible=sum(a['status'] == 'eligible' for a in attempts),
                  rejected=sum(a['status'] == 'rejected' for a in attempts),
                  commissioningExcluded=sum(a['phaseRole'] == 'commissioning' and a['status'] == 'eligible' for a in attempts),
                  trainingRows=len(rows))
    return dict(rows=rows, attempts=attempts, counts=counts,
                metadata=dict(indexSha256=sha(raw), sourceHashes=sources, captureGroup=group,
                              measurementConfigSha256=config_hash, acquisitionPolicy=copy.deepcopy(ACQUISITION),
                              rawFramesReaudited=reaudit_frames, verification='bound_original_receipts_and_finite_output_validation'))


def pre_features(row):
    """No after endpoint, label or actual output access here."""
    features = validate_prospective_features(row['features'])
    return [features[name] for name in FEATURES]


def fit_ridge(x, y, alpha):
    # Same standardized least-squares kernel as joint_prediction.fit, with the
    # frozen three-input contract and explicit permission for tiny training fits.
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    require(x.ndim == 2 and x.shape[1] == 3 and len(x) >= 1 and y.shape == (len(x),)
            and np.isfinite(x).all() and np.isfinite(y).all() and alpha in (.1, 1., 10.), 'invalid_finite_fit')
    mean, scale = x.mean(axis=0), x.std(axis=0)
    scale[scale < 1e-9] = 1.
    z, centre = (x - mean) / scale, float(y.mean())
    coef = np.linalg.lstsq(np.vstack([z, np.sqrt(alpha) * np.eye(3)]),
                           np.r_[y - centre, np.zeros(3)], rcond=None)[0]
    return dict(mean=mean.tolist(), scale=scale.tolist(), coefficients=coef.tolist(),
                intercept=centre, alpha=alpha, fitRows=len(x))


def metrics(y, p):
    y, p = np.asarray(y, dtype=float), np.asarray(p, dtype=float)
    error = p - y
    return dict(n=len(y), maeDeg=float(np.abs(error).mean()), rmseDeg=float(np.sqrt(np.mean(error ** 2))),
                biasDeg=float(error.mean()), maxAbsoluteErrorDeg=float(np.abs(error).max()),
                residualStdDeg=float(error.std()), uncertaintyInterpretation='descriptive_residuals_not_calibrated_intervals')


def train_and_evaluate(rows):
    """Fit tiny development data, but select/evaluate only 3 complete four-condition blocks."""
    require(isinstance(rows, list), 'rows_required')
    groups, ids = set(), set()
    for row in rows:
        require(row.get('phaseRole') == 'development' and row.get('condition') in catalog()
                and isinstance(row.get('captureGroup'), str) and row['captureGroup']
                and row.get('id') not in ids and finite(row.get('delta')), 'development_row_contract')
        ids.add(row['id']); groups.add(row['captureGroup'])
        features = validate_prospective_features(row['features'])
        expected = catalog()[row['condition']]
        require(features['direction'] == expected['direction'] and
                features['plannedSignedPwmSeconds'] == (8 if row['condition'][1] == '4' else 12.5) * expected['direction'], 'row_condition_features')
    require(len(groups) <= 1, 'one_development_group_required')
    result = dict(schemaVersion=RESULT, contract=CONTRACT, split='development',
                  inputNames=list(FEATURES), target='projected_relative_angle_delta_deg',
                  trainingPerformed=False, trainingRows=len(rows), modelSelectionPerformed=False,
                  models={}, selectedModel=None, fullDevelopmentFitMetrics={},
                  evaluation=dict(status='unavailable', reason='requires_three_complete_four_condition_development_blocks', independentTest=False),
                  outputDiagnostics=[dict(id=r['id'], condition=r['condition'], actual=r.get('actualOutputSummary')) for r in rows],
                  independentTestSetAvailable=False, physicalJointAngleCalibrated=False,
                  legacyDataIncluded=False, controlEligible=False,
                  limitations=['same_group_development_only', 'commissioning_excluded_from_fit',
                               'before_measurement_error_shared_with_delta_label',
                               'actual_output_is_retrospective_only', 'no_calibrated_uncertainty_or_control_validation'])
    if not rows:
        result['evaluation']['reason'] = 'no_eligible_development_rows'
        return result
    x = np.asarray([pre_features(r) for r in rows], dtype=float)
    y = np.asarray([r['delta'] for r in rows], dtype=float)
    labels = np.asarray([r['condition'] for r in rows])
    means = {c: float(y[labels == c].mean()) for c in sorted(set(labels))}
    result['trainingPerformed'] = True
    result['models'] = dict(zeroChange=dict(kind='constant', value=0.),
                            conditionMean=dict(kind='condition_mean', means=means, unsupportedCondition='no_prediction'),
                            ridge1=dict(kind='ridge', model=fit_ridge(x, y, 1.)))
    fit_predictions = dict(zeroChange=np.zeros(len(rows)), conditionMean=np.array([means[c] for c in labels]),
                           ridge1=predict(result['models']['ridge1']['model'], x))
    result['fullDevelopmentFitMetrics'] = {k: metrics(y, p) for k, p in fit_predictions.items()}
    blocks = list(dict.fromkeys(r['blockId'] for r in rows))
    complete = len(rows) == 12 and len(blocks) == 3 and all(
        sorted(r['condition'] for r in rows if r['blockId'] == b) == sorted(catalog()) for b in blocks)
    if not complete:
        result['limitations'].append('tiny_full_fit_is_not_predictive_performance')
        return result
    candidates = ['zeroChange', 'conditionMean', 'ridge10', 'ridge1', 'ridge0.1']
    predictions = {c: np.zeros(len(rows)) for c in candidates}
    folds = []
    for block in blocks:
        held = np.array([r['blockId'] == block for r in rows])
        train = ~held
        fold = dict(blockId=block, fitIds=[r['id'] for r, selected in zip(rows, train) if selected],
                    heldOutIds=[r['id'] for r, selected in zip(rows, held) if selected], models={})
        for condition in catalog():
            selected = labels == condition
            predictions['conditionMean'][held & selected] = y[train & selected].mean()
        for alpha in (10., 1., .1):
            name = 'ridge' + format(alpha, 'g')
            model = fit_ridge(x[train], y[train], alpha)
            predictions[name][held] = predict(model, x[held])
            fold['models'][name] = model
        folds.append(fold)
    all_metrics = {name: metrics(y, p) for name, p in predictions.items()}
    selected = min(candidates, key=lambda name: all_metrics[name]['maeDeg'])
    for alpha in (10., .1):
        name = 'ridge' + format(alpha, 'g')
        result['models'][name] = dict(kind='ridge', model=fit_ridge(x, y, alpha))
    result.update(modelSelectionPerformed=True, selectedModel=selected,
        evaluation=dict(status='available', kind='within_development_leave_logical_block_out', independentTest=False,
                        foldCount=3, folds=folds, metrics=all_metrics,
                        predictions=[dict(id=r['id'], blockId=r['blockId'], actualDeltaDeg=float(y[i]),
                                          **{name: float(p[i]) for name, p in predictions.items()}) for i, r in enumerate(rows)],
                        selection='pooled_MAE_exact_tie_zero_condition_mean_larger_alpha',
                        selectionScoreIsIndependentPerformance=False))
    return result


def run(path, *, reaudit_frames=False):
    dataset = load_index(path, reaudit_frames=reaudit_frames)
    result = train_and_evaluate(dataset['rows'])
    result.update(attempts=dataset['attempts'], counts=dataset['counts'], metadata=dataset['metadata'])
    repo = Path(__file__).resolve().parents[1]
    files = ['learning/finite_batch_learning.py', 'learning/finite_profile_measurement.py',
             'learning/finite_profile_plan.py', 'learning/joint_prediction.py', 'learning/supervised_measurement.py']
    result['metadata'].update(sourceCodeSha256={name: sha((repo / name).read_bytes()) for name in files},
                              numpyVersion=np.__version__, pythonVersion=platform.python_version())
    return result
