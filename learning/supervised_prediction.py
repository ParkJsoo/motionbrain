"""Offline development diagnostics for settled M2 endpoints, never control.

Reuse the six-column numerical ridge kernel, not the legacy dataset or split.
Actual output features are exclusively retrospective, unavailable before RUN.
"""
import json
from pathlib import Path
import platform

import numpy as np

from learning.joint_prediction import fit, predict, scores
from learning.supervised_measurement import (
    DATA_SCHEMA, finite, read_bound, sha, validate_config, validate_execution, wrap,
)
from learning.supervised_transport import validate_pulse

CONTRACT = 'settled_endpoints_after_confirmed_off_v1'
ALPHA = 1.0  # Fixed before evaluation; no search or model selection.
INPUTS = {
    'preExecution': ['angle_deg', 'velocity_deg_s', 'requested_signed_pwm',
                     'requested_duration_s', 'requested_signed_duration_s', 'signed_angle_deg'],
    'postExecution': ['angle_deg', 'velocity_deg_s', 'actual_signed_peak_pwm',
                      'actual_active_duration_s', 'actual_signed_pwm_seconds', 'signed_angle_deg'],
}


def output_summary(row):
    pulse = row['pulse']
    validate_execution(dict(pulse=pulse, requestedPulse=pulse, motor=pulse['motor'],
                            outputTrace=row['outputTrace']), pulse)
    trace = [e for e in row['outputTrace'] if e['motor'] == pulse['motor']]
    active_ms = 0
    integral = 0.
    for a, b in zip(trace, trace[1:]):
        elapsed = (b['atMs'] - a['atMs']) % 2**32
        if a['pwm']:
            active_ms += elapsed
        integral += a['pwm'] * elapsed / 1000.
    if active_ms <= 0 or active_ms > 60000:
        raise ValueError('invalid_active_output_duration')
    return dict(signedPeakPwm=max(trace, key=lambda e: abs(e['pwm']))['pwm'],
                activeDurationMs=active_ms, signedPwmSeconds=integral,
                absolutePwmSeconds=abs(integral))


def pre_features(row):
    """Only pre-command state and requested command; no trace/after/label reads."""
    pulse = validate_pulse(row['pulse'])
    if pulse['motor'] != 2 or pulse['joint'] != 'wrist':
        raise ValueError('m2_only')
    angle, velocity = (row['before'][k] for k in ('value', 'velocityPerSecond'))
    if not all(finite(v) for v in (angle, velocity)):
        raise ValueError('finite_pre_state_required')
    sign = 1 if pulse['direction'] == 'up' else -1
    duration = pulse['durationMs'] / 1000.
    return [angle, velocity, pulse['expectedPwm'], duration, sign*duration, sign*angle]


def post_features(row):
    before = pre_features(row)
    actual = output_summary(row)
    return [*before[:2], actual['signedPeakPwm'], actual['activeDurationMs']/1000.,
            actual['signedPwmSeconds'], before[-1]]


def load_index(path):
    """Bind cached eligible rows to hashed measurements, without re-reading video.

    This verifies existing measurement receipts, not a new raw-frame audit.
    All selected sources must share the one declared development capture group.
    """
    path = Path(path).resolve()
    raw = path.read_bytes()
    index = json.loads(raw)
    if (index.get('schemaVersion') != 'motionbrain.local_supervised_endpoint_index.v1'
            or index.get('contract') != CONTRACT or index.get('split') != 'development'
            or index.get('independentTestSetAvailable') is not False
            or index.get('legacyCommandPlus4SecondsDataIncluded') is not False):
        raise ValueError('development_endpoint_index_required')
    groups = index.get('captureGroups')
    if (not isinstance(groups, list) or len(groups) != 1
            or not isinstance(groups[0], str) or not groups[0]):
        raise ValueError('one_development_group_required')
    sources = {}
    representation = None
    for ref in index['sources']:
        name = ref['path']
        if name in sources:
            raise ValueError('duplicate_measurement_source')
        data = json.loads(read_bound(path.parent, name, ref['sha256']))
        if (data.get('schemaVersion') != DATA_SCHEMA or data.get('contract') != CONTRACT
                or data.get('units') != 'projected_degrees'
                or data.get('velocityUnits') != 'projected_degrees_per_second'
                or data.get('physicalJointAngleCalibrated') is not False
                or data.get('split') != 'development' or data.get('captureGroup') != groups[0]):
            raise ValueError('measurement_contract_or_group_mismatch')
        config = validate_config(data['measurementConfig'])
        config_hash = sha(json.dumps(config, sort_keys=True, separators=(',', ':')).encode())
        if config_hash != ref['configHash'] or config_hash != data['measurementConfigSha256']:
            raise ValueError('measurement_config_hash_mismatch')
        rep = config['representation']
        if rep['kind'] != 'relative_angle' or (representation is not None and rep != representation):
            raise ValueError('measurement_representation_mismatch')
        representation = rep
        source_rows = {r['id']: r for r in data['rows']}
        if len(source_rows) != len(data['rows']):
            raise ValueError('duplicate_source_row')
        sources[name] = source_rows
    rows, seen = [], set()
    for entry in index['entries']:
        row = entry['row']
        name = entry['measurementSource']
        if row['id'] in seen or sources[name].get(row['id']) != row:
            raise ValueError('duplicate_or_changed_index_row')
        seen.add(row['id'])
        if (row.get('status') != 'eligible' or row.get('executionStatus') != 'executed'
                or row.get('split') != 'development' or row.get('captureGroup') != groups[0]):
            raise ValueError('eligible_development_row_required')
        pre_features(row)
        output_summary(row)
        if not all(finite(row[k]) for k in ('delta',)) or not finite(row['after']['value']):
            raise ValueError('finite_label_required')
        if abs(row['delta'] - wrap(row['after']['value'] - row['before']['value'])) > 1e-9:
            raise ValueError('endpoint_delta_mismatch')
        rows.append(dict(row, measurementSource=name))
    eligible = {(name, r['id']) for name, rr in sources.items() for r in rr.values()
                if r.get('status') == 'eligible'}
    if eligible != {(r['measurementSource'], r['id']) for r in rows}:
        raise ValueError('eligible_source_rows_omitted')
    counts = {d: sum(r['pulse']['direction'] == d for r in rows) for d in ('up', 'down')}
    if index['count'] != len(rows) or index['directionCounts'] != counts:
        raise ValueError('index_denominator_mismatch')
    metadata = dict(indexSha256=sha(raw), sources=index['sources'], captureGroups=groups,
                    count=len(rows), directionCounts=counts,
                    verification='hashed_measurement_receipts_and_exact_rows',
                    rawFramesReaudited=False)
    return rows, metadata


def train_and_evaluate(rows):
    """Leave a measurement block out; folds remain within development, not test."""
    y = np.asarray([r['delta'] for r in rows], dtype=float)
    blocks = list(dict.fromkeys(r['measurementSource'] for r in rows))
    if len(blocks) < 2:
        raise ValueError('multiple_measurement_blocks_required')
    x = {'preExecution': np.asarray([pre_features(r) for r in rows]),
         'postExecution': np.asarray([post_features(r) for r in rows])}
    predictions = {name: np.zeros(len(rows)) for name in (*INPUTS, 'noChange', 'directionMean')}
    folds = []
    for block in blocks:
        held = np.asarray([r['measurementSource'] == block for r in rows])
        training = ~held
        if training.sum() < 7:
            raise ValueError('at_least_seven_fit_rows_per_fold_required')
        fold = dict(measurementSource=block, heldOutIds=[r['id'] for r, h in zip(rows, held) if h],
                    fitIds=[r['id'] for r, t in zip(rows, training) if t], models={})
        for name in INPUTS:
            model = fit(x[name][training], y[training], ALPHA)
            predictions[name][held] = predict(model, x[name][held])
            fold['models'][name] = model
        for direction in ('up', 'down'):
            same = np.asarray([r['pulse']['direction'] == direction for r in rows])
            if not (training & same).any():
                raise ValueError('direction_baseline_training_rows_missing')
            predictions['directionMean'][held & same] = y[training & same].mean()
        fold['metrics'] = {name: scores(y[held], p[held]) for name, p in predictions.items()}
        folds.append(fold)
    models = {name: dict(inputNames=INPUTS[name], availability=name,
                        model=fit(values, y, ALPHA), controlEligible=False)
              for name, values in x.items()}
    return dict(schemaVersion='motionbrain.supervised_prediction.v1', contract=CONTRACT,
                target='settled_projected_angle_delta_deg', alpha=ALPHA,
                hyperparameterSelection=False, trainingPerformed=True, models=models,
                evaluation=dict(kind='within_development_leave_measurement_block_out',
                                independentTest=False, foldCount=len(folds), folds=folds,
                                metrics={name: scores(y, p) for name, p in predictions.items()},
                                predictions=[dict(id=r['id'], actualDeltaDeg=float(y[i]),
                                                  **{name: float(p[i]) for name, p in predictions.items()})
                                             for i, r in enumerate(rows)]),
                fullDevelopmentFitMetrics={name: scores(y, predict(m['model'], x[name]))
                                           for name, m in models.items()},
                outputSummaries=[dict(id=r['id'], requestedSignedPwm=r['pulse']['expectedPwm'],
                                      requestedDurationMs=r['pulse']['durationMs'], **output_summary(r))
                                 for r in rows],
                controlEligible=False, independentTestSetAvailable=False,
                physicalJointAngleCalibrated=False, legacyDataIncluded=False,
                limitations=['one_development_capture_group', 'temporally_correlated_blocks',
                             'post_execution_features_unavailable_for_command_selection',
                             'no_independent_test_or_physical_control_validation'])


def run(path):
    rows, metadata = load_index(path)
    root = Path(__file__).resolve().parents[1]
    paths = ['learning/supervised_prediction.py', 'learning/joint_prediction.py',
             'learning/supervised_measurement.py', 'learning/supervised_transport.py',
             'tools/train_supervised_prediction.py']
    metadata.update(sourceSha256={p: sha((root/p).read_bytes()) for p in paths},
                    numpyVersion=np.__version__, pythonVersion=platform.python_version())
    return dict(train_and_evaluate(rows), metadata=metadata)
