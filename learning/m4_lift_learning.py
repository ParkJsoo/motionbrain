"""Fixed-scene lift metrology and local prediction; no transport or actuation.

Registration bounds are a pre-collection operating domain, NOT measured accuracy
or empirical training minima/maxima. Rejection never expands or clamps them.
No contact, retention, calibrated height, or physical safety is inferred here.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
from pathlib import Path
import uuid

import cv2
import numpy as np

STATE = ['objectRisePx', 'toolRisePx', 'slipDyPx']
PULSES = [dict(joint='shoulder', direction='up', percent=16, durationMs=200),
          dict(joint='shoulder', direction='up', percent=50, durationMs=650)]
SCHEMA = 'motionbrain.fixed_scene_lift.v1'
SHORT_SCHEMA = 'motionbrain.fixed_scene_lift.short500.v1'
SHORT_PULSES = [copy.deepcopy(PULSES[0]), dict(joint='shoulder', direction='up', percent=50, durationMs=500)]
ANGLE_SCHEMA = 'motionbrain.lift_angle_evidence.v1'
ANGLE_LIMITS = dict(observedAbortAbsDeg=3., planningAbsLimitDeg=2.5, engineeringReserveDeg=.5,
                    reserveIsValidatedSafetyBound=False, referenceBandDeg=[192.61,197.61])
PARAMETERS = dict(objectRoi=[680, 690, 810, 850], grayThreshold=110, distalRows=8,
                  medianFootWidth=[40, 100], markerRoi=[840, 550, 970, 700],
                  markerHsvLower=[135, 30, 50], markerHsvUpper=[179, 255, 255],
                  templateRoi=[130, 100, 185, 145], searchRoi=[115, 75, 205, 170], minNcc=.8)


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def contract_sha(registration):
    return _sha(_bytes(registration))


def _image(raw):
    result = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    _require(result is not None, 'invalid_image')
    return result


def _landmarks(image):
    _require(image.shape[0] >= 850 and image.shape[1] >= 970, 'external_shape_invalid')
    gray = cv2.cvtColor(image[690:850, 680:810], cv2.COLOR_BGR2GRAY)
    count, labels, stats, centers = cv2.connectedComponentsWithStats((gray < 110).astype('uint8'))
    # The cylinder joins a dark background band; identify its bottom eight-row
    # footprint, not the full connected silhouette or a bounding-box centre.
    choices = []
    for i in range(1, count):
        ys, xs = np.where(labels == i)
        foot = xs[ys >= ys.max()-7]
        widths = [np.ptp(xs[ys == row])+1 for row in range(int(ys.max())-7, int(ys.max())+1)
                  if np.any(ys == row)]
        bottom = xs[ys == ys.max()]
        if (len(widths) == 8 and 40 <= float(np.median(widths)) <= 100 and bottom.min() > 0
                and bottom.max() < 129 and 0 < ys.max() < 159 and len(foot) >= 100):
            choices.append(i)
    _require(bool(choices), 'object_component_missing')
    i = max(choices, key=lambda j: stats[j, cv2.CC_STAT_AREA])
    x, y, w, h, area = stats[i]
    hsv = cv2.cvtColor(image[550:700, 840:970], cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([135, 30, 50]), np.array([179, 255, 255]))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    _require(count > 1, 'tool_marker_missing')
    marker_choices = [i for i in range(1, count) if stats[i, 4] >= 6 and
                      stats[i, 0] > 0 and stats[i, 0]+stats[i, 2] < 130 and
                      stats[i, 1] > 0 and stats[i, 1]+stats[i, 3] < 150]
    _require(bool(marker_choices), 'tool_marker_clipped_or_small')
    i = max(marker_choices, key=lambda j: stats[j, cv2.CC_STAT_AREA])
    x2, y2, w2, h2, area2 = stats[i]
    _require(area2 >= 6 and x2 > 0 and x2+w2 < 130 and y2 > 0 and y2+h2 < 150,
             'tool_marker_clipped_or_small')
    return [float(690+y+h-1), float(550+centers[i][1])]


def register(external_path, gripper_path, context):
    _require(isinstance(context, dict) and bool(context), 'scene_context_required')
    raw = [Path(p).read_bytes() for p in (external_path, gripper_path)]
    ext, grip = map(_image, raw)
    _require(grip.shape[0] >= 170 and grip.shape[1] >= 205, 'gripper_shape_invalid')
    template = cv2.cvtColor(grip[100:145, 130:185], cv2.COLOR_BGR2GRAY)
    _require(float(template.std()) >= 3, 'template_texture_insufficient')
    return dict(schema=SCHEMA, context=copy.deepcopy(context), state=STATE,
                externalShape=list(ext.shape), gripperShape=list(grip.shape),
                baseline=_landmarks(ext), template=base64.b64encode(template.tobytes()).decode(),
                parameters=copy.deepcopy(PARAMETERS),
                referenceSha256=dict(zip(('external', 'gripper'), map(_sha, raw))),
                method='dark_component_distal_8rows_and_magenta_template.v1',
                operatingDomain=dict(objectRisePx=3., toolRisePx=6., slipPx=4.),
                domainMeaning='pre_collection_operating_domain_not_accuracy_or_training_minmax')


def measure(root, record, registration):
    _require(registration['parameters'] == PARAMETERS, 'registered_parameters_changed')
    _require(set(record) == {'id', 'context', 'external', 'gripper'}, 'before_record_fields_invalid')
    _require(record['context'] == registration['context'], 'scene_context_mismatch')
    images, hashes = [], {}
    root = Path(root).resolve()
    for name in ('external', 'gripper'):
        source = record[name]
        _require(set(source) == {'path', 'sha256'}, 'source_fields_invalid')
        path = (root / source['path']).resolve()
        _require(path.is_relative_to(root), 'source_path_escape')
        raw = path.read_bytes()
        hashes[name] = _sha(raw)
        _require(hashes[name] == source['sha256'], 'source_hash_changed')
        image = _image(raw)
        _require(list(image.shape) == registration[name+'Shape'], 'image_shape_changed')
        images.append(image)
    obj, tool = _landmarks(images[0])
    gray = cv2.cvtColor(images[1], cv2.COLOR_BGR2GRAY)
    template = np.frombuffer(base64.b64decode(registration['template']), np.uint8).reshape(45, 55)
    result = cv2.matchTemplate(gray[75:170, 115:205], template, cv2.TM_CCOEFF_NORMED)
    _, score, _, (x, y) = cv2.minMaxLoc(result)
    _require(np.isfinite(score) and score > .8, 'slip_template_quality_insufficient')
    _require(0 < x < result.shape[1]-1 and 0 < y < result.shape[0]-1, 'slip_search_clipped')
    return dict(id=record['id'], context=copy.deepcopy(record['context']),
                contractSha256=contract_sha(registration),
                state=[registration['baseline'][0]-obj, registration['baseline'][1]-tool, float(y+75-100)],
                quality=dict(valid=True, ncc=float(score), slipDxPx=float(x+115-130)),
                sourceSha256=hashes)


def model_pulses(schema):
    _require(schema in (SCHEMA, SHORT_SCHEMA), 'unsupported_lift_model_schema')
    return copy.deepcopy(PULSES if schema == SCHEMA else SHORT_PULSES)


def _pulse_key(pulse, model_schema=SCHEMA):
    pulses = model_pulses(model_schema)
    _require(pulse in pulses, 'unsupported_pulse')
    return str(pulses.index(pulse))


def validate_angle_evidence(report, registration, *, recheck_sources=True):
    _require(report['schemaVersion'] == ANGLE_SCHEMA and report['context'] == registration['context']
             and report['contractSha256'] == contract_sha(registration) and report['limits'] == ANGLE_LIMITS
             and bool(report['responses']), 'angle_evidence_contract_invalid')
    ids = [r['id'] for r in report['responses']]
    _require(len(set(ids)) == len(ids) and all(r['status'] in ('completed','failed') and
             np.isfinite(r['maximumObservedAbsDeltaDeg']) and r['maximumObservedAbsDeltaDeg'] >= 0
             for r in report['responses']), 'angle_responses_invalid')
    if recheck_sources:
        from learning.m4_lift_data import build_angle_evidence
        _require(build_angle_evidence([r['sourceRoot'] for r in report['responses']],registration) == report,
                 'angle_source_evidence_changed')
    return True


def candidate_angle_screen(report, pulse):
    responses = [r for r in report['responses'] if r['pulse'] == pulse]
    maximum = max((r['maximumObservedAbsDeltaDeg'] for r in responses),default=None)
    return dict(admissible=len(responses) >= 2 and maximum <= ANGLE_LIMITS['planningAbsLimitDeg']
                and maximum+ANGLE_LIMITS['engineeringReserveDeg'] <= ANGLE_LIMITS['observedAbortAbsDeg'],
                observations=len(responses), maximumObservedAbsDeltaDeg=maximum,
                planningAbsLimitDeg=2.5, engineeringReserveDeg=.5, reserveIsValidatedSafetyBound=False,
                includesFailedResponses=True, sampleMaximumIsConfidenceBound=False)


def _state(observation, registration, before=False):
    _require(registration['schema'] == SCHEMA and registration['state'] == STATE and
             registration['parameters'] == PARAMETERS and
             registration['operatingDomain'] == dict(objectRisePx=3., toolRisePx=6., slipPx=4.),
             'registered_domain_changed')
    _require(observation['context'] == registration['context'] and
             observation['contractSha256'] == contract_sha(registration), 'observation_contract_mismatch')
    state = np.asarray(observation['state'], dtype=float)
    quality = observation['quality']
    _require(state.shape == (3,) and np.isfinite(state).all() and quality['valid'] is True
             and quality['ncc'] > .8 and np.isfinite(quality['slipDxPx']), 'invalid_measurement')
    if before:
        d = registration['operatingDomain']
        _require(abs(state[0]) <= d['objectRisePx'] and abs(state[1]) <= d['toolRisePx']
                 and abs(state[2]) <= d['slipPx'] and abs(quality['slipDxPx']) <= d['slipPx'],
                 'outside_registered_operating_domain')
    return state


def validate_before(observation, registration):
    """Enforce the pre-registered physical input domain before any command."""
    return _state(observation, registration, before=True).tolist()


def _row_sources(row):
    return [v for side in ('before', 'after') for v in row[side]['sourceSha256'].values()]


def _source_identity(row):
    """Receipt mode consumes ROOT AUDITOR VERIFIED rows, not arbitrary claims.

    The caller must validate native ingress, camera clocks/boot/frame identity,
    source receipts and the actual single RUN against originals before calling.
    These fields alone do not authenticate hardware or raw acquisition evidence.
    """
    keys = ('episodeIdentity', 'sourceEvidenceSha256', 'sourceCaptureIds')
    present = [key in row for key in keys]
    _require(all(present) or not any(present), 'partial_audited_source_identity')
    result = dict(id=row['id'], images=_row_sources(row), mode='image_hashes_v1')
    if all(present):
        episode, receipts, captures = (row[key] for key in keys)
        _require(isinstance(episode, list) and len(episode) == 2 and
                 isinstance(episode[0], str) and bool(episode[0]) and
                 type(episode[1]) is int and episode[1] >= 0, 'invalid_episode_identity')
        _require(isinstance(receipts, list) and len(receipts) == 4 and
                 all(isinstance(v, str) and len(v) == 64 and
                     all(c in '0123456789abcdef' for c in v) for v in receipts) and
                 len(set(receipts)) == 4, 'invalid_source_receipts')
        _require(isinstance(captures, list) and len(captures) == 4 and
                 all(isinstance(v, str) and bool(v) for v in captures) and
                 len(set(captures)) == 4, 'invalid_capture_identity')
        result.update(mode='audited_receipts_v1', episodeIdentity=copy.deepcopy(episode),
                      sourceEvidenceSha256=copy.deepcopy(receipts), sourceCaptureIds=copy.deepcopy(captures))
    return result


def _check_source_disjoint(source, earlier, reason):
    _require(len(source['images']) == 4, reason)
    _require(all(source['mode'] == prior['mode'] for prior in earlier), 'source_identity_mode_mismatch')
    _require(all(source['id'] != prior['id'] for prior in earlier), reason)
    if source['mode'] == 'image_hashes_v1':
        _require(len(set(source['images'])) == 4 and
                 all(not set(source['images']).intersection(prior['images']) for prior in earlier), reason)
    else:
        _require(all(source['episodeIdentity'] != prior['episodeIdentity'] and
                     not set(source['sourceEvidenceSha256']).intersection(prior['sourceEvidenceSha256']) and
                     not set(source['sourceCaptureIds']).intersection(prior['sourceCaptureIds'])
                     for prior in earlier), reason)


def fit(rows, registration, context, *, model_schema=SCHEMA, angle_evidence=None, allow_reaudited_legacy_low=False):
    _require(context == registration['context'] and len(rows) >= 4, 'four_training_rows_required')
    model_pulses(model_schema)
    if model_schema == SHORT_SCHEMA:
        _require(angle_evidence is not None and len(rows) == 4, 'short500_exact_four_audited_rows_required')
        validate_angle_evidence(angle_evidence,registration)
        responses = {r['id']:r for r in angle_evidence['responses']}
        for row in rows:
            _require(row['id'] in responses and responses[row['id']]['resultSha256'] == row['resultSha256']
                     and responses[row['id']]['pulse'] == row['pulse'] and responses[row['id']]['status'] == 'completed'
                     and responses[row['id']]['sourcePhase'] == 'training' and row.get('phase') == 'training',
                     'short500_training_angle_binding')
            if responses[row['id']]['sourceSchema'] != 'motionbrain.m4_up_learning_short500.v1':
                _require(allow_reaudited_legacy_low and row['pulse'] == SHORT_PULSES[0], 'legacy_low_explicit_reaudit_required')
            # All four new-version rows must equal fresh canonical raw audits.
            # A receipt/result hash alone does not authenticate supplied labels.
            from learning.m4_lift_data import build_row
            _require(contract_sha(build_row(row['sourceRoot'],registration)) == contract_sha(row), 'short500_training_reaudit_changed')
    ids, hashes, sources, groups = set(), set(), [], {'0': [], '1': []}
    for row in rows:
        _require(row['id'] not in ids and row['context'] == context and
                 row['contractSha256'] == contract_sha(registration), 'training_identity_or_contract_invalid')
        source = _source_identity(row)
        _check_source_disjoint(source, sources, 'training_source_reused')
        sources.append(source)
        ids.add(row['id']); hashes.update(source['images'])
        a, b = _state(row['before'], registration, True), _state(row['after'], registration)
        groups[_pulse_key(row['pulse'],model_schema)].append((a, b))
    _require(all(len(g) >= 2 for g in groups.values()), 'two_sources_per_candidate_required')
    model = dict(schema=model_schema, kind='local_visual_translation', context=copy.deepcopy(context),
                registration=copy.deepcopy(registration), contractSha256=contract_sha(registration),
                trainingIds=sorted(ids), trainingSourceHashes=sorted(hashes),
                sourceIdentityMode=sources[0]['mode'], trainingSources=sources,
                candidates={k: dict(count=len(g), delta=np.mean([b-a for a,b in g], axis=0).tolist(),
                                    absoluteAfterMean=np.mean([b for _,b in g], axis=0).tolist())
                            for k,g in groups.items()})
    if model_schema == SHORT_SCHEMA:
        model.update(angleEvidence=copy.deepcopy(angle_evidence), angleEvidenceSha256=contract_sha(angle_evidence),
                     legacyLowExplicitlyReaudited=allow_reaudited_legacy_low)
    return model


def save_model(path, model):
    raw = _bytes(model)
    with Path(path).open('xb') as handle:
        handle.write(raw)
    return _sha(raw)


def evaluate(model, rows):
    _require(len(rows) == 2, 'two_held_out_rows_required')
    reg = model['registration']
    sources = copy.deepcopy(model['trainingSources'])
    errors = dict(local_visual_translation=[], commandOnly=[], noChange=[])
    for row in rows:
        source = _source_identity(row)
        _check_source_disjoint(source, sources, 'evaluation_source_reused')
        _require(row['context'] == model['context'] and row['contractSha256'] == model['contractSha256'],
                 'evaluation_contract_mismatch')
        sources.append(source)
        before, after = _state(row['before'], reg, True), _state(row['after'], reg)
        candidate = model['candidates'][_pulse_key(row['pulse'],model['schema'])]
        for key, prediction in [('local_visual_translation', before+candidate['delta']),
                                ('commandOnly', candidate['absoluteAfterMean']), ('noChange', before)]:
            errors[key].append(np.abs(after-prediction).tolist())
    return dict(evaluated=2, meanAbsoluteErrorPx={k: np.mean(v, axis=0).tolist() for k,v in errors.items()})


class FrozenLiftModel:
    def __init__(self, path, sha, registration, evidence_root, source_root=None, *, angle_evidence=None):
        self.path, self.sha = Path(path), sha
        self.registration = copy.deepcopy(registration)
        self.evidence_root = Path(evidence_root)
        self.source_root = Path(source_root or evidence_root)
        self.model = json.loads(self.path.read_bytes())
        self.angle_evidence = copy.deepcopy(angle_evidence if angle_evidence is not None else self.model.get('angleEvidence'))
        self.angle_evidence_sha256 = contract_sha(self.angle_evidence) if self.angle_evidence is not None else None
        self.verify()
        if self.model['schema'] == SHORT_SCHEMA:
            validate_angle_evidence(self.angle_evidence,self.registration)

    def verify(self):
        raw = self.path.read_bytes()
        _require(_sha(raw) == self.sha and json.loads(raw) == self.model, 'frozen_model_changed')
        _require(self.model['schema'] in (SCHEMA,SHORT_SCHEMA) and self.model['kind'] == 'local_visual_translation'
                 and self.model['registration'] == self.registration and
                 self.model['contractSha256'] == contract_sha(self.registration), 'frozen_contract_changed')
        _require(set(self.model['candidates']) == {'0', '1'} and
                 all(c['count'] >= 2 and np.isfinite(c['delta']).all()
                     and len(c['delta']) == 3 and len(c['absoluteAfterMean']) == 3
                     and np.isfinite(c['absoluteAfterMean']).all() for c in self.model['candidates'].values()),
                 'candidate_support_insufficient')
        sources = self.model['trainingSources']
        _require(len(sources) >= 4 and self.model['sourceIdentityMode'] == sources[0]['mode'] and
                 sorted(s['id'] for s in sources) == self.model['trainingIds'] and
                 sorted({h for s in sources for h in s['images']}) == self.model['trainingSourceHashes'],
                 'training_source_manifest_invalid')
        for i, source in enumerate(sources):
            _check_source_disjoint(source, sources[:i], 'training_source_manifest_reused')
        if self.model['schema'] == SHORT_SCHEMA:
            report = self.model['angleEvidence']
            _require(contract_sha(report) == self.model['angleEvidenceSha256'], 'frozen_angle_evidence_changed')
            validate_angle_evidence(report,self.registration,recheck_sources=False)
            _require(contract_sha(self.angle_evidence) == self.angle_evidence_sha256, 'current_angle_evidence_changed')
            validate_angle_evidence(self.angle_evidence,self.registration,recheck_sources=False)
            current = {r['id']:r for r in self.angle_evidence['responses']}
            _require(all(current.get(r['id']) == r for r in report['responses']), 'known_angle_response_removed_or_changed')
        return True

    def _decision(self, record, pulse=None):
        self.verify()
        features = measure(self.source_root, record, self.registration)
        state = _state(features, self.registration, True)
        predictions = [dict(pulse=copy.deepcopy(p), state=(state+self.model['candidates'][str(i)]['delta']).tolist())
                       for i,p in enumerate(model_pulses(self.model['schema']))]
        if self.model['schema'] == SHORT_SCHEMA:
            for prediction in predictions:
                prediction['angleScreen'] = candidate_angle_screen(self.angle_evidence,prediction['pulse'])
        # Pre-collection task eligibility, not measured safety or grasp accuracy.
        selected, rejection = pulse, None
        if pulse is not None and self.model['schema'] == SHORT_SCHEMA:
            key = _pulse_key(pulse,self.model['schema'])
            if not predictions[int(key)]['angleScreen']['admissible']:
                rejection = 'candidate_angle_screen_failed'
        if pulse is None:
            feasible = sorted((p for p in predictions if p['state'][0] >= 8 and abs(p['state'][2]) <= 4
                               and p.get('angleScreen',{}).get('admissible',True)),
                              key=lambda p: abs(p['state'][0]-20))
            if not feasible:
                rejection = 'no_feasible_lift_candidate'
            elif len(feasible) > 1 and abs(abs(feasible[0]['state'][0]-20)-abs(feasible[1]['state'][0]-20)) < 1:
                rejection = 'ambiguous_lift_candidates'
            else:
                selected = feasible[0]['pulse']
        if selected is not None:
            _pulse_key(selected,self.model['schema'])
        result = dict(features=features, predictions=predictions, pulse=copy.deepcopy(selected),
                      modelSha256=self.sha, contractSha256=contract_sha(self.registration),
                      goalObjectRisePx=20, modelSelected=pulse is None and rejection is None,
                      selectionRejection=rejection,
                      selectionEligibility=dict(minimumObjectRisePx=8, maximumAbsSlipDyPx=4,
                                                minimumCostSeparationPx=1))
        if self.model['schema'] == SHORT_SCHEMA:
            result['angleEvidenceSha256'] = self.angle_evidence_sha256
        self.evidence_root.mkdir(parents=True, exist_ok=True)
        path = self.evidence_root / ('decision-'+uuid.uuid4().hex+'.json')
        raw = _bytes(result)
        with path.open('xb') as handle:
            handle.write(raw)
            handle.flush()
            import os
            os.fsync(handle.fileno())
        self.verify()
        _require(rejection is None, rejection)
        return dict(result, decisionPath=str(path), decisionSha256=_sha(raw))

    def select(self, before_record):
        return self._decision(before_record)

    def predict(self, before_record, pulse):
        return self._decision(before_record, pulse)
