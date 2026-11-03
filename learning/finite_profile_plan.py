"""Offline guards for the fixed finite-profile collection plan.

This module performs no I/O, dispatch, retries, model fitting or phase advance.
The operator must persist an append-only attempt journal BEFORE sending each
request. A missing/unfinished/failed attempt blocks continuation; deleting journal
entries cannot be detected here. Physical clearance and current safety checks are
separate requirements, not consequences of passing these pure validators.
"""
import copy
import json
import math
import re

from .finite_profile_measurement import CONTRACT, digest

SCHEMA = 'motionbrain.finite_learning_design.v1'
FEATURES = ('beforeProjectedAngle', 'direction', 'plannedSignedPwmSeconds')
PILOT = ('U4', 'D4', 'U5', 'D5')
BLOCKS = {
    'P': [list(PILOT)],
    'D': [['U4', 'D4', 'D5', 'U5'], ['D4', 'U5', 'U4', 'D5'], ['U5', 'D5', 'D4', 'U4']],
    'H': [['D5', 'U4', 'U5', 'D4'], ['D4', 'U5', 'U4', 'D5']],
}
TIMING = dict(startLeadMs=300, maxLatenessMs=20, offConfirmWindowMs=100, admissionWindowMs=10000)
_IDENTITIES = {
    'U4': ('m2-triangle40-350-v1', '397407eec131923846d17207d9e88504e80ed118fa76716b244ceb593df05a6d'),
    'D4': ('m2-triangle40-down-350-v1', '6ab7f141a57ac34b7d324035251c8e76a04da203aa579bd1c794c2b7badc0c21'),
    'U5': ('m2-triangle50-450-v1', '7cf225f50e4d6c15049a9d05eb28026e78ce97ae4b50394167da7491b23415b5'),
    'D5': ('m2-triangle50-down-450-v1', 'fab591ebd11131047000da4cc1e7ce332c8bb65a580c2a634f643d598b587d4a'),
}


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _equal(actual, expected):
    # JSON comparison distinguishes true from 1 and rejects NaN/Infinity.
    try:
        return json.dumps(actual, sort_keys=True, allow_nan=False) == json.dumps(expected, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError):
        return False


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identifier(value, maximum):
    return isinstance(value, str) and 0 < len(value) <= maximum and re.fullmatch(r'[A-Za-z0-9_.:-]+', value) is not None


def catalog():
    """Return independent copies of the four trusted wire profiles, keyed by condition."""
    profiles = {}
    for condition, (profile_id, catalog_hash) in _IDENTITIES.items():
        n = int(condition[1])
        direction = 1 if condition[0] == 'U' else -1
        values = list(range(10, n * 10 + 1, 10)) + list(range((n - 1) * 10, -1, -10))
        profiles[condition] = dict(contractVersion=1, profileVersion=1, timingPolicyVersion=1,
                                  profileId=profile_id, catalogHash=catalog_hash, motor=2,
                                  direction=direction, **TIMING,
                                  points=[dict(offsetMs=i * 50, pwm=direction * pwm) for i, pwm in enumerate(values)])
    return profiles


def canonical_profile(profile, *, include_catalog_hash=True):
    """C++ canonicalProfile ASCII for a trusted profile only (including final LF)."""
    _require(any(_equal(profile, p) for p in catalog().values()), 'profile_not_trusted')
    keys = ['contractVersion', 'profileVersion', 'timingPolicyVersion']
    if include_catalog_hash:
        keys.append('catalogHash')
    keys += ['profileId', 'motor', 'direction', 'startLeadMs', 'maxLatenessMs',
             'offConfirmWindowMs', 'admissionWindowMs']
    lines = ['motionbrain.finite_profile.v1'] + [f'{key}={profile[key]}' for key in keys]
    lines.append(f"pointCount={len(profile['points'])}")
    lines += [f"point={p['offsetMs']},{p['pwm']}" for p in profile['points']]
    return ('\n'.join(lines) + '\n').encode('ascii')


def planned_signed_integral(condition):
    profiles = catalog()
    _require(condition in profiles, 'unknown_condition')
    points = profiles[condition]['points']
    return sum(a['pwm'] * (b['offsetMs'] - a['offsetMs']) for a, b in zip(points, points[1:])) / 1000


def validate_plan(plan):
    """Validate the fixed operational contract; return its complete JSON fingerprint.

    Metadata may be extended, but it cannot override the fixed profile/order,
    measurement, feature or no-automation policies checked here.
    """
    _require(isinstance(plan, dict) and plan.get('schemaVersion') == SCHEMA, 'plan_schema')
    _require(plan.get('status') == 'design_only' and plan.get('dispatchEnabled') is False, 'plan_not_offline')
    _require(_equal(plan.get('timing'), TIMING), 'plan_timing')
    profiles = plan.get('profiles', {})
    _require(isinstance(profiles, dict) and set(profiles) == set(PILOT), 'plan_conditions')
    for condition, profile in catalog().items():
        expected = dict(direction=profile['direction'], points=profile['points'],
                        peakPwm=int(condition[1]) * 10, activeMs=profile['points'][-1]['offsetMs'],
                        absolutePwmSeconds=abs(planned_signed_integral(condition)),
                        existingPhysicalEvidence=condition == 'U4')
        # Integral 8 and 8.0 are equivalent physical quantities; booleans are not.
        got = copy.deepcopy(profiles[condition])
        _require(isinstance(got, dict) and _finite(got.get('absolutePwmSeconds')), 'plan_integral')
        got['absolutePwmSeconds'] = float(got['absolutePwmSeconds'])
        _require(_equal(got, expected), 'plan_profile')
    phases = plan.get('phases', {})
    _require(isinstance(phases, dict) and set(phases) == set(BLOCKS), 'plan_phases')
    roles = {'P': 'commissioning_excluded_from_model_fit', 'D': 'development_group_one',
             'H': 'future_new_session_heldout_group_one'}
    for phase, blocks in BLOCKS.items():
        expected = dict(role=roles[phase], blocks=blocks, maxAttempts=4 * len(blocks))
        if phase == 'P':
            expected['reviewAfterSlot'] = 2
        _require(_equal(phases[phase], expected), 'plan_phase_order')
    _require(type(plan.get('maximumNewAttempts')) is int and plan['maximumNewAttempts'] == 24, 'plan_attempt_limit')
    for key in ('automaticPhaseAdvance', 'automaticRetry', 'automaticReposition', 'automaticReboot'):
        _require(plan.get(key) is False, 'plan_automation')
    _require(plan.get('skipPolicy') == 'record current slot unstarted and terminate current phase; no replacement', 'plan_skip_policy')
    measurement = dict(contract=CONTRACT, beforeOffsets=[-.75, -.1], afterHostOffReceiptOffsets=[2.25, 3],
                       minimumFrames=3, maxGapSeconds=.3, maximumSpreadDeg=.5, noMotionMaximumIsDetectionThreshold=False)
    _require(_equal(plan.get('measurement'), measurement), 'plan_measurement')
    model = plan.get('model', {})
    required = dict(features=list(FEATURES), intercept=True, ridgeAlphaCandidates=[.1, 1, 10],
                    baselines=['zero_change', 'four_condition_mean'], minimumDevelopmentEligiblePerCondition=3,
                    minimumDevelopmentBlocks=3, allDevelopmentBlocksRequireAllConditions=True,
                    actualOutputAllowedAsProspectiveFeature=False, GP=False, automaticControlEligible=False,
                    selection='leave_one_development_block_out pooled MAE; exact ties zero then condition mean then larger alpha',
                    objective='sum_squared_residuals_plus_alpha_times_squared_coefficient_norm',
                    interceptPenalized=False, scalerFit='union_of_two_training_blocks_per_fold',
                    heldoutReporting='all attempts plus eligible conditional errors; each condition >=1 required for full-condition summary; missing rows are not silently replaced',
                    heldoutScaling='apply_training_fold_mean_and_scale_unchanged',
                    finalFit='refit_scaler_and_selected_model_on_all_12_development_rows_before_H')
    _require(isinstance(model, dict) and all(_equal(model.get(k), v) for k, v in required.items()), 'plan_model_contract')
    contracts = {
        'capture': dict(clipsPerCommand=1, logicalBlockSize=4, maxSeconds=60, maxSampleFps=8, maxFrames=500,
                        maxBytesPerClip=4179968000, freeReserveBytes=2147483648),
        'storage': dict(coreSlots=32, driverSlots=32, driverEntriesPerSlot=64, capacityQueryImplemented=False,
                        capacityCheck='min(32-coreUsed,32-driverUsed) >= phase maximum remaining reservations'),
    }
    for key, expected in contracts.items():
        _require(_equal(plan.get(key), expected), 'plan_' + key)
    return digest(plan)


def validate_device(status, expected_boot, required_slots, *, require_idle=True):
    """Check a fresh pre-prepare GET, including BOTH cumulative record stores.

    Returns current epoch and remaining capacity; never authorizes motion itself.
    required_slots counts all remaining reservations in the current phase.
    """
    _require(isinstance(status, dict), 'device_status')
    _require(_identifier(expected_boot, 32) and status.get('bootId') == expected_boot, 'device_boot')
    _require(type(require_idle) is bool, 'device_idle_policy')
    _require(status.get('enabled') is True and type(status.get('idle')) is bool
             and (not require_idle or status['idle']) and status.get('blocked') is False, 'device_not_ready')
    epoch = status.get('epoch')
    _require(type(epoch) is int and 1 <= epoch <= 2147483647, 'device_epoch')
    _require(type(required_slots) is int and 1 <= required_slots <= 32, 'required_capacity')
    expected = catalog()
    received = status.get('profiles')
    _require(isinstance(received, list) and len(received) == 4, 'device_catalog')
    _require(all(isinstance(p, dict) and isinstance(p.get('profileId'), str) for p in received), 'device_catalog')
    actual = {p['profileId']: p for p in received}
    _require(len(actual) == 4 and all(_equal(actual.get(p['profileId']), p) for p in expected.values()), 'device_catalog')
    _require(_equal(status.get('profile'), expected['U4']), 'device_legacy_profile')
    available = []
    for kind in ('core', 'driver'):
        used, capacity = status.get(kind + 'RecordsUsed'), status.get(kind + 'RecordCapacity')
        _require(type(capacity) is int and capacity == 32 and type(used) is int and 0 <= used <= capacity, 'device_capacity_invalid')
        available.append(capacity - used)
    _require(min(available) >= required_slots, 'device_capacity_insufficient')
    return dict(epoch=epoch, availableSlots=min(available))


def pilot_next(history, expected_boot, *, reviewed_first_pair=False):
    """Return only the next P slot, or None after four successes; never advances to D.

    Each journal entry must be persisted before its one attempt. Incomplete or
    failed entries permanently end this phase; the caller must not remove them.
    Completed flags are caller attestations from validated evidence, not a second
    output/measurement validator. Pair review is explicit, never inferred from time.
    """
    _require(_identifier(expected_boot, 32), 'pilot_boot')
    _require(type(history) is list and len(history) <= 4, 'pilot_attempt_limit')
    _require(type(reviewed_first_pair) is bool, 'pilot_review_flag')
    seen = set()
    for index, entry in enumerate(history):
        _require(isinstance(entry, dict) and type(entry.get('slot')) is int and entry['slot'] == index
                 and entry.get('condition') == PILOT[index], 'pilot_order')
        command = entry.get('commandId')
        _require(_identifier(command, 128) and command not in seen, 'pilot_command_identity')
        seen.add(command)
        _require(entry.get('bootId') == expected_boot, 'pilot_boot')
        _require(entry.get('outcome') == 'Completed' and all(entry.get(k) is True for k in
                 ('measurementEligible', 'closedEvidenceValidated', 'finalIdleZero')), 'pilot_stopped')
    if len(history) == 4:
        return None
    _require(len(history) < 2 or reviewed_first_pair, 'pilot_pair_review_required')
    condition = PILOT[len(history)]
    return dict(slot=len(history), condition=condition, profile=catalog()[condition])


def validate_prospective_features(features):
    """Reject any extra feature, including post-execution actual output fields."""
    _require(isinstance(features, dict) and set(features) == set(FEATURES), 'prospective_feature_allowlist')
    _require(all(_finite(v) for v in features.values()), 'prospective_feature_nonfinite')
    direction, integral = features['direction'], features['plannedSignedPwmSeconds']
    _require(type(direction) is int and direction in (-1, 1)
             and integral in (direction * 8, direction * 12.5), 'prospective_profile_features')
    return copy.deepcopy(features)


def prospective_input(condition, before_receipt, execution_identity, dispatch_before):
    """Construct three prospective features from a sealed preflight receipt.

    The same receipt must remain the target's before endpoint. Host monotonic
    dispatch_before is sampled immediately before transmission, not a device clock.
    """
    _require(condition in catalog(), 'unknown_condition')
    _require(isinstance(before_receipt, dict) and isinstance(execution_identity, dict), 'before_receipt_required')
    _require(set(execution_identity) == {'bootId', 'commandId', 'requestEpoch'}
             and _identifier(execution_identity.get('bootId'), 32)
             and _identifier(execution_identity.get('commandId'), 128)
             and type(execution_identity.get('requestEpoch')) is int
             and 1 <= execution_identity['requestEpoch'] <= 2147483647, 'execution_identity')
    receipt = copy.deepcopy(before_receipt)
    receipt_hash = receipt.pop('receiptSha256', None)
    _require(receipt_hash == digest(receipt), 'before_receipt_hash')
    _require(receipt.get('status') == 'eligible' and receipt.get('contract') == CONTRACT, 'before_receipt_contract')
    _require(_equal(receipt.get('executionIdentity'), execution_identity), 'before_execution_identity')
    profile = catalog()[condition]
    _require(receipt.get('requestedProfileSha256') == digest(profile), 'before_profile_identity')
    anchor = receipt.get('anchorHostMonotonic')
    _require(_finite(anchor) and _finite(dispatch_before) and 0 <= dispatch_before - anchor <= 2,
             'before_anchor_age')
    _require(_equal(receipt.get('maxBeforeAnchorAgeSeconds'), 2.0), 'before_age_contract')
    before = receipt.get('before', {})
    _require(isinstance(before, dict) and _finite(before.get('value')), 'before_angle')
    window = before.get('window')
    _require(isinstance(window, list) and len(window) == 2 and all(_finite(v) for v in window)
             and abs(window[0] - (anchor - .75)) < 1e-9 and abs(window[1] - (anchor - .1)) < 1e-9,
             'before_fixed_window')
    features = validate_prospective_features(dict(beforeProjectedAngle=before['value'],
               direction=profile['direction'], plannedSignedPwmSeconds=planned_signed_integral(condition)))
    return dict(features=features, beforeReceiptSha256=receipt_hash,
                executionIdentity=copy.deepcopy(execution_identity), requestedProfileSha256=digest(profile),
                beforeAnchorAgeSeconds=dispatch_before - anchor)
