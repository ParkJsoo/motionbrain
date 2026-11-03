"""Pure host guards: fixed catalog, no retries, capacity and prospective leakage."""
import copy
import hashlib
import unittest

from learning import finite_profile_plan as fp
from learning.finite_profile_measurement import digest


def plan_fixture():
    profiles = {}
    for condition, profile in fp.catalog().items():
        profiles[condition] = dict(direction=profile['direction'], points=profile['points'],
             peakPwm=int(condition[1]) * 10, activeMs=profile['points'][-1]['offsetMs'],
             absolutePwmSeconds=abs(fp.planned_signed_integral(condition)), existingPhysicalEvidence=condition == 'U4')
    phases = {}
    roles = dict(P='commissioning_excluded_from_model_fit', D='development_group_one', H='future_new_session_heldout_group_one')
    for key, blocks in fp.BLOCKS.items():
        phases[key] = dict(role=roles[key], blocks=copy.deepcopy(blocks), maxAttempts=4 * len(blocks))
    phases['P']['reviewAfterSlot'] = 2
    return dict(schemaVersion=fp.SCHEMA, status='design_only', dispatchEnabled=False,
       profiles=profiles, timing=copy.deepcopy(fp.TIMING), phases=phases, maximumNewAttempts=24,
       automaticPhaseAdvance=False, automaticRetry=False, automaticReposition=False, automaticReboot=False,
       skipPolicy='record current slot unstarted and terminate current phase; no replacement',
       measurement=dict(contract=fp.CONTRACT, beforeOffsets=[-.75, -.1], afterHostOffReceiptOffsets=[2.25, 3],
          minimumFrames=3, maxGapSeconds=.3, maximumSpreadDeg=.5, noMotionMaximumIsDetectionThreshold=False),
       capture=dict(clipsPerCommand=1, logicalBlockSize=4, maxSeconds=60, maxSampleFps=8, maxFrames=500,
          maxBytesPerClip=4179968000, freeReserveBytes=2147483648),
       storage=dict(coreSlots=32, driverSlots=32, driverEntriesPerSlot=64, capacityQueryImplemented=False,
          capacityCheck='min(32-coreUsed,32-driverUsed) >= phase maximum remaining reservations'),
       model=dict(features=list(fp.FEATURES), intercept=True, ridgeAlphaCandidates=[.1, 1, 10],
          baselines=['zero_change', 'four_condition_mean'], minimumDevelopmentEligiblePerCondition=3,
          minimumDevelopmentBlocks=3, allDevelopmentBlocksRequireAllConditions=True,
          actualOutputAllowedAsProspectiveFeature=False, GP=False, automaticControlEligible=False,
          selection='leave_one_development_block_out pooled MAE; exact ties zero then condition mean then larger alpha',
          objective='sum_squared_residuals_plus_alpha_times_squared_coefficient_norm',
          interceptPenalized=False, scalerFit='union_of_two_training_blocks_per_fold',
          heldoutReporting='all attempts plus eligible conditional errors; each condition >=1 required for full-condition summary; missing rows are not silently replaced',
          heldoutScaling='apply_training_fold_mean_and_scale_unchanged',
          finalFit='refit_scaler_and_selected_model_on_all_12_development_rows_before_H'))


def device_fixture():
    profiles = fp.catalog()
    return dict(enabled=True, bootId='boot-current', epoch=7, idle=True, blocked=False,
        profiles=list(profiles.values()), profile=profiles['U4'], coreRecordsUsed=28,
        driverRecordsUsed=27, coreRecordCapacity=32, driverRecordCapacity=32)


def history_fixture(count):
    return [dict(slot=i, condition=fp.PILOT[i], bootId='boot-current', commandId=f'pilot-{i}',
                 outcome='Completed', measurementEligible=True, closedEvidenceValidated=True,
                 finalIdleZero=True) for i in range(count)]


def receipt_fixture(condition='U4'):
    identity = dict(bootId='boot-current', commandId='pilot-0', requestEpoch=7)
    receipt = dict(status='eligible', reason=None, contract=fp.CONTRACT,
        anchorHostMonotonic=10., maxBeforeAnchorAgeSeconds=2., before=dict(value=13.2, window=[9.25, 9.9]),
        executionIdentity=identity, requestedProfileSha256=digest(fp.catalog()[condition]),
        measurementConfigSha256='a' * 64, sourceHashes={'frame': 'b' * 64})
    receipt['receiptSha256'] = digest(receipt)
    return receipt, identity


def rehash(receipt):
    receipt.pop('receiptSha256', None)
    receipt['receiptSha256'] = digest(receipt)


class FiniteProfilePlanTests(unittest.TestCase):
    def test_catalog_vectors_and_integrals(self):
        expected = {'U4': (40, 350, 8.), 'D4': (40, 350, -8.), 'U5': (50, 450, 12.5), 'D5': (50, 450, -12.5)}
        for label, profile in fp.catalog().items():
            peak, duration, integral = expected[label]
            self.assertEqual(max(abs(p['pwm']) for p in profile['points']), peak)
            self.assertEqual(profile['points'][-1], dict(offsetMs=duration, pwm=0))
            self.assertEqual(fp.planned_signed_integral(label), integral)
            self.assertEqual(hashlib.sha256(fp.canonical_profile(profile, include_catalog_hash=False)).hexdigest(), profile['catalogHash'])
        changed = fp.catalog()
        changed['U4']['points'][0]['pwm'] = 200
        self.assertEqual(fp.catalog()['U4']['points'][0]['pwm'], 10)

    def test_canonical_tamper_cannot_borrow_hash(self):
        for key, value in [('motor', 1), ('direction', -1), ('maxLatenessMs', 21), ('profileId', 'unknown')]:
            with self.subTest(key=key):
                profile = fp.catalog()['U4']
                profile[key] = value
                with self.assertRaises(ValueError):
                    fp.canonical_profile(profile)

    def test_plan_valid_and_fingerprinted(self):
        plan = plan_fixture()
        self.assertEqual(fp.validate_plan(plan), digest(plan))
        plan['profiles']['U4']['absolutePwmSeconds'] = 8
        self.assertEqual(fp.validate_plan(plan), digest(plan))

    def test_plan_tampered_profiles_timing_and_order(self):
        for mutate in [lambda p: p['profiles']['U4']['points'][0].update(pwm=11),
                       lambda p: p['profiles'].update(X=copy.deepcopy(p['profiles']['U4'])),
                       lambda p: p['timing'].update(maxLatenessMs=50),
                       lambda p: p['phases']['P']['blocks'][0].reverse(),
                       lambda p: p['phases']['P'].update(maxAttempts=5),
                       lambda p: p['profiles']['U4'].update(direction=True)]:
            with self.subTest(mutate=mutate):
                plan = plan_fixture(); mutate(plan)
                with self.assertRaises(ValueError): fp.validate_plan(plan)

    def test_plan_automation_and_leakage_fail_closed(self):
        for key in ['automaticRetry', 'automaticReposition', 'automaticPhaseAdvance', 'automaticReboot', 'dispatchEnabled']:
            plan = plan_fixture(); plan[key] = True
            with self.subTest(key=key), self.assertRaises(ValueError): fp.validate_plan(plan)
        plan = plan_fixture(); plan['model']['features'].append('actualPwmIntegral')
        with self.assertRaises(ValueError): fp.validate_plan(plan)
        plan = plan_fixture(); plan['measurement']['maximumSpreadDeg'] = 1
        with self.assertRaises(ValueError): fp.validate_plan(plan)

    def test_fixed_storage_and_model_policy(self):
        for section, key, value in [('capture', 'freeReserveBytes', 0), ('storage', 'coreSlots', 64),
                                    ('model', 'interceptPenalized', True), ('model', 'selection', 'random_split')]:
            plan = plan_fixture(); plan[section][key] = value
            with self.subTest(section=section, key=key), self.assertRaises(ValueError): fp.validate_plan(plan)

    def test_capacity_both_stores_boundary(self):
        self.assertEqual(fp.validate_device(device_fixture(), 'boot-current', 4), dict(epoch=7, availableSlots=4))
        for key in ['coreRecordsUsed', 'driverRecordsUsed']:
            device = device_fixture(); device[key] = 29
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'insufficient'):
                fp.validate_device(device, 'boot-current', 4)

    def test_capacity_malformed_not_coerced(self):
        for key, value in [('coreRecordsUsed', -1), ('driverRecordsUsed', 33), ('coreRecordsUsed', True),
                           ('driverRecordsUsed', '0'), ('driverRecordCapacity', 64)]:
            device = device_fixture(); device[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                fp.validate_device(device, 'boot-current', 4)
        for slots in [0, -1, 33, True, 1.5]:
            with self.subTest(slots=slots), self.assertRaises(ValueError):
                fp.validate_device(device_fixture(), 'boot-current', slots)

    def test_prearm_idle_relaxation_preserves_boot_block_catalog(self):
        device = device_fixture(); device['idle'] = False
        fp.validate_device(device, 'boot-current', 4, require_idle=False)
        with self.assertRaises(ValueError): fp.validate_device(device, 'boot-current', 4)
        device['blocked'] = True
        with self.assertRaises(ValueError): fp.validate_device(device, 'boot-current', 4, require_idle=False)
        device['blocked'] = False
        with self.assertRaises(ValueError): fp.validate_device(device, 'old-boot', 4, require_idle=False)

    def test_device_catalog_semantics_duplicates_and_unknown(self):
        mutations = [lambda d: d['profiles'][0]['points'][0].update(pwm=11),
                     lambda d: d['profiles'].__setitem__(3, copy.deepcopy(d['profiles'][0])),
                     lambda d: d['profiles'][3].update(profileId='unknown'),
                     lambda d: d.pop('profiles'), lambda d: d.update(profile=fp.catalog()['D4'])]
        for mutate in mutations:
            device = device_fixture(); mutate(device)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                fp.validate_device(device, 'boot-current', 4)

    def test_pilot_sequence_pause_and_no_phase_advance(self):
        self.assertEqual(fp.pilot_next([], 'boot-current')['condition'], 'U4')
        self.assertEqual(fp.pilot_next(history_fixture(1), 'boot-current')['condition'], 'D4')
        with self.assertRaisesRegex(ValueError, 'review_required'):
            fp.pilot_next(history_fixture(2), 'boot-current')
        self.assertEqual(fp.pilot_next(history_fixture(2), 'boot-current', reviewed_first_pair=True)['condition'], 'U5')
        self.assertEqual(fp.pilot_next(history_fixture(3), 'boot-current', reviewed_first_pair=True)['condition'], 'D5')
        self.assertIsNone(fp.pilot_next(history_fixture(4), 'boot-current', reviewed_first_pair=True))

    def test_failed_unstarted_pending_quality_stop_no_retry(self):
        for outcome in ['Failed', 'Cancelled', 'OutcomeUnknown', 'unstarted', 'pending', 'Expired']:
            history = history_fixture(1); history[0]['outcome'] = outcome
            with self.subTest(outcome=outcome), self.assertRaisesRegex(ValueError, 'pilot_stopped'):
                fp.pilot_next(history, 'boot-current')
        for key in ['measurementEligible', 'closedEvidenceValidated', 'finalIdleZero']:
            history = history_fixture(1); history[0][key] = False
            with self.subTest(key=key), self.assertRaises(ValueError): fp.pilot_next(history, 'boot-current')

    def test_reordered_duplicate_oldboot_overlimit(self):
        for mutate in [lambda h: h.reverse(), lambda h: h[1].update(commandId=h[0]['commandId']),
                       lambda h: h[0].update(bootId='old'), lambda h: h[0].update(slot=True)]:
            history = history_fixture(2); mutate(history)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                fp.pilot_next(history, 'boot-current', reviewed_first_pair=True)
        history = history_fixture(4); history.append(copy.deepcopy(history[0]))
        with self.assertRaises(ValueError): fp.pilot_next(history, 'boot-current', reviewed_first_pair=True)

    def test_features_sealed_receipt_signed_output_and_age_boundary(self):
        receipt, identity = receipt_fixture('D5')
        result = fp.prospective_input('D5', receipt, identity, 12.)
        self.assertEqual(result['features'], dict(beforeProjectedAngle=13.2, direction=-1, plannedSignedPwmSeconds=-12.5))
        self.assertEqual(result['beforeReceiptSha256'], receipt['receiptSha256'])
        self.assertEqual(result['executionIdentity'], identity)
        for at in [9.99, 12.00001, float('nan'), True]:
            with self.subTest(at=at), self.assertRaises(ValueError): fp.prospective_input('D5', receipt, identity, at)

    def test_feature_extra_post_execution_fields_rejected(self):
        features = dict(beforeProjectedAngle=13., direction=1, plannedSignedPwmSeconds=8.)
        for key in ['actualPwmIntegral', 'afterAngle', 'actualDuration', 'stopReason', 'actualPeak']:
            bad = dict(features, **{key: 1})
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'allowlist'):
                fp.validate_prospective_features(bad)
        for key, value in [('beforeProjectedAngle', float('inf')), ('direction', True), ('plannedSignedPwmSeconds', -8.)]:
            bad = dict(features); bad[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): fp.validate_prospective_features(bad)

    def test_receipt_tamper_hash_identity_profile_window(self):
        receipt, identity = receipt_fixture()
        receipt['before']['value'] = 99.
        with self.assertRaisesRegex(ValueError, 'hash'): fp.prospective_input('U4', receipt, identity, 10.1)
        for mutate in [lambda r: r['executionIdentity'].update(bootId='other'),
                       lambda r: r.update(requestedProfileSha256=digest(fp.catalog()['D4'])),
                       lambda r: r.update(status='rejected'),
                       lambda r: r['before'].update(window=[9.0, 9.9])]:
            receipt, identity = receipt_fixture(); identity = copy.deepcopy(identity); mutate(receipt); rehash(receipt)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                fp.prospective_input('U4', receipt, identity, 10.1)
        receipt, identity = receipt_fixture()
        with self.assertRaises(ValueError): fp.prospective_input('U6', receipt, identity, 10.1)


if __name__ == '__main__':
    unittest.main()
