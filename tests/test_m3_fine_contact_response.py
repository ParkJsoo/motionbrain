"""미세 실측 범주·사후 근거·접촉 라벨과 실행 전 입력 분리 검증."""
import copy
import hashlib
import json
import unittest

from learning import m3_fine_contact_response as fine, marker_core_tool_bridge as bridge
from learning import attended_collection_profile as profiles, supervised_transport as transport
from tests import test_m3_direct_tool_response as direct_fixture


def sha(value):
    return hashlib.sha256(value.encode()).hexdigest()


class FineContactTests(unittest.TestCase):
    def setUp(self):
        self.fixture = direct_fixture.DirectToolTests(); self.fixture.setUp()
        original = self.fixture.fit()
        self.projector = bridge.CoreBridgeProjector(json.dumps(original, sort_keys=True).encode(), self.fixture.seed)
        self.before = self.projector.observe(self.fixture.raw(), 195.29)
        self.context = dict(controllerBootId='1'*32, cameraBootId='2'*32,
            cameraIdentity=dict(deviceId='fixture-camera', helperSha256=sha('helper'),
                configuration=[sha('configuration'), 1920, 1080, False, 0], viewId='fixture-view'))

    def row(self, duration=200, dy=1., support='not_supported'):
        command = next(copy.deepcopy(c) for c in fine.COMMANDS if c['durationMs'] == duration)
        after = self.projector.observe(self.fixture.raw(0, dy), 195.29)
        trace = [dict(sequence=1, atMs=1000, motor=3, pwm=10),
                 dict(sequence=2, atMs=1050, motor=3, pwm=20)]
        trace.extend(dict(sequence=i+3, atMs=1100, motor=i+1, pwm=0) for i in range(5))
        receipt = dict(requestedPulse=dict(**command, motor=3, expectedPwm=153), offVerified=True,
            monitorStatusVerified=True, monitorTimingPolicy=fine.timing_policy(command),
            monitorReadBudgetRevision=transport.WRIST_READ_BUDGET_REVISION,
            dispatchBefore=10., dispatchAfter=10.05, offAt=10.3, outputTrace=trace)
        return dict(sessionId='fine-'+str(duration), sourceSha256=sha('session-'+str(duration)), command=command,
            before=copy.deepcopy(self.before), after=after,
            sourceEvidence=dict(beforeExternalImageSha256=self.before['imageSha256'],
                afterExternalImageSha256=after['imageSha256'], beforeGripperImageSha256=sha('before-gripper'),
                afterGripperImageSha256=sha('after-gripper'), receiptSha256=fine.digest(receipt),
                supportAnnotationSha256=sha('support-'+support)), receipt=receipt,
            sourceContext=copy.deepcopy(self.context),
            collectionProfile=profiles.collection_profile(profiles.ATTENDED_ELBOW_CONTACT_FINE),
            manualCollection=True, modelSelected=False, dataCollection=True, sessionPhase='completed',
            monitorTimingPolicy=fine.timing_policy(command), beforeSupport='not_supported', afterSupport=support)

    def fit(self, rows):
        return fine.fit(rows, projector=self.projector, audit_sources=lambda values: values)

    def predict(self, model, command=None, current=None, support='not_supported'):
        return fine.predict(model, self.projector, current or self.before, command or fine.COMMANDS[0], current_support=support, current_context=self.context)

    def test_single_direct_source_freezes_only_observed_category_and_diagnostic_flags(self):
        model = self.fit([self.row()]); fine.validate(model, self.projector)
        self.assertEqual(len(model['categories']), 1)
        response = self.predict(model)
        self.assertEqual(response['predictedState'], response['commandMeanState'])
        self.assertTrue(response['nearestAndCommandMeanIdentical'])
        self.assertFalse(model['independentTest']); self.assertFalse(model['controlEligible'])
        self.assertFalse(model['baselineSuperiorityVerified']); self.assertEqual(model['prospectiveCommands'], 0)
        with self.assertRaisesRegex(ValueError, 'category_not_trained'):
            self.predict(model, fine.COMMANDS[1])

    def test_two_exact_categories_keep_their_observed_delta_without_time_scaling(self):
        model = self.fit([self.row(200, 1), self.row(250, 3)])
        a = self.predict(model); b = self.predict(model, fine.COMMANDS[1])
        self.assertAlmostEqual(a['observedSourceDeltaState'][1], 1.)
        self.assertAlmostEqual(b['observedSourceDeltaState'][1], 3.)
        self.assertNotAlmostEqual(b['observedSourceDeltaState'][1], a['observedSourceDeltaState'][1]*1.25)
        self.assertFalse(model['timeScalingAllowed']); self.assertFalse(model['old350SourcesMixed'])

    def test_unknown_or_supported_current_is_admission_stop_not_model_stop(self):
        model = self.fit([self.row()])
        for support in ('unknown', 'supported'):
            with self.subTest(support=support), self.assertRaisesRegex(ValueError, 'air_support_confirmation'):
                self.predict(model, support=support)
            decision = fine.select(model, self.projector, self.before, [942., 383.], current_support=support, current_context=self.context)
            self.assertEqual(decision['selectionOwner'], 'admission_guard')
            self.assertFalse(decision['modelSelectedStop']); self.assertFalse(decision['modelPredictionPerformed'])
            self.assertFalse(decision['additionalPhysicalEvaluationAuthorized'])

    def test_contact_and_unknown_after_labels_are_retained_not_execution_features(self):
        models = [self.fit([self.row(support=support)]) for support in ('not_supported', 'supported', 'unknown')]
        states = [self.predict(model)['predictedState'] for model in models]
        self.assertEqual(states[0], states[1]); self.assertEqual(states[1], states[2])
        self.assertTrue(models[1]['categories'][0]['contactTransition'])
        self.assertEqual(models[2]['categories'][0]['afterSupport'], 'unknown')
        self.assertFalse(self.predict(models[1])['contactTransitionGeneralizationVerified'])
        self.assertFalse(self.predict(models[1])['actualOutputAndAfterSupportUsedAsExecutionInputs'])

    def test_stationary_response_is_preserved_and_stop_wins_tie(self):
        model = self.fit([self.row(dy=0)])
        response = self.predict(model)
        self.assertEqual(response['observedSourceDeltaState'], [0., 0., 0., 0.])
        decision = fine.select(model, self.projector, self.before, [942., 383.], current_support='not_supported', current_context=self.context)
        self.assertTrue(decision['modelSelectedStop']); self.assertFalse(decision['modelSelectedAction'])
        self.assertFalse(decision['actualTableSupportVerifiedByModel'])

    def test_old_350_other_percent_axis_direction_and_inbetween_duration_rejected(self):
        for update in (dict(durationMs=350), dict(durationMs=225), dict(percent=70), dict(direction='up'), dict(joint='wrist')):
            row = self.row(); row['command'].update(update)
            with self.subTest(update=update), self.assertRaisesRegex(ValueError, 'exact_200_or_250'):
                self.fit([row])

    def test_audit_callback_required_and_cannot_rewrite_original_row(self):
        row = self.row()
        for audit in (None, lambda values: [], lambda values: [dict(values[0], modelSelected=True)]):
            with self.subTest(audit=audit), self.assertRaisesRegex(ValueError, 'source_reaudit_required'):
                fine.fit([row], projector=self.projector, audit_sources=audit)

    def test_manual_profile_phase_or_support_source_provenance_mismatch_rejected(self):
        for update in (dict(manualCollection=False), dict(modelSelected=True), dict(dataCollection=False),
                dict(sessionPhase='failed'), dict(collectionProfile=profiles.collection_profile(profiles.ATTENDED_60_1000)),
                dict(beforeSupport='supported'), dict(afterSupport='invented'), dict(monitorTimingPolicy=transport.ELBOW_DEADLINE_POLICY)):
            row = self.row(); row.update(update)
            with self.subTest(update=update), self.assertRaises(ValueError): self.fit([row])

    def test_raw_image_receipt_and_annotation_SHA_bindings_required(self):
        for key in ('beforeExternalImageSha256', 'afterExternalImageSha256', 'receiptSha256', 'supportAnnotationSha256'):
            row = self.row()
            if key == 'supportAnnotationSha256': row['sourceEvidence'].pop(key)
            else: row['sourceEvidence'][key] = 'f'*64
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'raw_source_bindings'):
                self.fit([row])

    def test_receipt_unknown_unmonitored_unverified_or_wrong_clock_rejected(self):
        for update in (dict(offVerified=False), dict(monitorStatusVerified=False), dict(monitorTimingPolicy='wrong'),
                dict(monitorReadBudgetRevision='old'), dict(offAt=9.)):
            row = self.row(); row['receipt'].update(update); row['sourceEvidence']['receiptSha256']=fine.digest(row['receipt'])
            with self.subTest(update=update), self.assertRaises(ValueError): self.fit([row])

    def test_driver_trace_axis_cap_gap_reverse_and_missing_OFF_rejected(self):
        for mode in ('axis', 'cap', 'reverse', 'gap', 'no_OFF', 'no_ON'):
            row = self.row(); trace=row['receipt']['outputTrace']
            if mode == 'axis': trace[0]['motor']=4
            elif mode == 'cap': trace[0]['pwm']=154
            elif mode == 'reverse': trace[0]['pwm']=-1
            elif mode == 'gap': trace[1]['sequence']=3
            elif mode == 'no_OFF': row['receipt']['outputTrace']=trace[:2]
            else:
                for item in trace: item['pwm']=0
            row['sourceEvidence']['receiptSha256']=fine.digest(row['receipt'])
            with self.subTest(mode=mode), self.assertRaises(ValueError): self.fit([row])

    def test_duplicate_category_or_source_and_more_than_two_rejected(self):
        row = self.row(); second=self.row(250)
        for rows in ([], [row, copy.deepcopy(row)], [row, second, self.row()], [row, dict(second, sourceSha256=row['sourceSha256'])]):
            with self.subTest(count=len(rows)), self.assertRaises(ValueError): self.fit(rows)

    def test_changed_model_or_observer_contract_and_state_rejected(self):
        model = self.fit([self.row()])
        for update in (dict(controlEligible=True), dict(prospectiveCommands=1), dict(observerBridgeContractSha256='a'*64), dict(old350SourcesMixed=True)):
            changed=copy.deepcopy(model); changed.update(update)
            with self.subTest(update=update), self.assertRaises(ValueError): self.predict(changed)
        current=copy.deepcopy(self.before); current['state'][0]+=1
        with self.assertRaises(ValueError): self.predict(model, current=current)

    def test_local_support_and_angular_wrap_and_M4_fixed_checks(self):
        model=self.fit([self.row()])
        outside=self.projector.observe(self.fixture.raw(11, 0),195.29)
        with self.assertRaisesRegex(ValueError, 'outside_local_support'): self.predict(model,current=outside)
        outside=self.projector.observe(self.fixture.raw(),195.80)
        with self.assertRaisesRegex(ValueError, 'outside_local_support'): self.predict(model,current=outside)
        row=self.row(); row['after']=self.projector.observe(self.fixture.raw(),195.80);row['sourceEvidence']['afterExternalImageSha256']=row['after']['imageSha256']
        with self.assertRaisesRegex(ValueError, 'fixed_M4_changed'): self.fit([row])

    def test_distinct_boot_view_configuration_or_helper_cohorts_cannot_mix(self):
        for mode in ('controller', 'camera', 'view', 'helper', 'configuration'):
            first=self.row(); second=self.row(250)
            ctx=second['sourceContext']
            if mode == 'controller': ctx['controllerBootId']='3'*32
            elif mode == 'camera': ctx['cameraBootId']='4'*32
            elif mode == 'view': ctx['cameraIdentity']['viewId']='other-view'
            elif mode == 'helper': ctx['cameraIdentity']['helperSha256']=sha('other-helper')
            else: ctx['cameraIdentity']['configuration'][0]=sha('other-configuration')
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError,'cohorts_must_not_mix'):
                self.fit([first, second])

    def test_source_context_types_hashes_and_numeric_booleans_are_strict(self):
        for mode in ('missing', 'boot_bool', 'boot_case', 'helper_sha', 'config_sha', 'width_bool', 'rotation_bool', 'mirror_int', 'view_bool', 'extra'):
            row=self.row(); ctx=row['sourceContext']
            if mode == 'missing': row.pop('sourceContext')
            elif mode == 'boot_bool': ctx['controllerBootId']=True
            elif mode == 'boot_case': ctx['cameraBootId']='A'*32
            elif mode == 'helper_sha': ctx['cameraIdentity']['helperSha256']='bad'
            elif mode == 'config_sha': ctx['cameraIdentity']['configuration'][0]=False
            elif mode == 'width_bool': ctx['cameraIdentity']['configuration'][1]=True
            elif mode == 'rotation_bool': ctx['cameraIdentity']['configuration'][4]=False
            elif mode == 'mirror_int': ctx['cameraIdentity']['configuration'][3]=0
            elif mode == 'view_bool': ctx['cameraIdentity']['viewId']=False
            else: ctx['unknown']=True
            with self.subTest(mode=mode), self.assertRaises(ValueError): self.fit([row])

    def test_current_context_is_required_exact_and_admission_only(self):
        model=self.fit([self.row()])
        for ctx in (None, {}, dict(self.context, controllerBootId='5'*32),
                dict(self.context, cameraIdentity=dict(self.context['cameraIdentity'], viewId='changed'))):
            with self.subTest(ctx=ctx), self.assertRaises(ValueError):
                fine.predict(model,self.projector,self.before,fine.COMMANDS[0],current_support='not_supported',current_context=ctx)
            decision=fine.select(model,self.projector,self.before,[942.,383.],current_support='not_supported',current_context=ctx)
            self.assertEqual(decision['selectionOwner'],'admission_guard'); self.assertFalse(decision['modelSelectedStop'])
        self.assertEqual(model['sourceContextSha256'],fine.digest(self.context))
        self.assertEqual(self.predict(model)['admissionOnlyFields'],['sourceContext','current_support'])
        changed=copy.deepcopy(model); changed['sourceContext']['cameraIdentity']['viewId']='changed'
        changed['sourceContextSha256']=fine.digest(changed['sourceContext'])
        with self.assertRaises(ValueError): self.predict(changed)

    def test_budget_stop_is_separate_and_unknown_category_explicit(self):
        model=self.fit([self.row()])
        decision=fine.select(model,self.projector,self.before,[942.,383.],current_support='not_supported',current_context=self.context,commands_used=1)
        self.assertEqual(decision['selectionOwner'],'budget_guard');self.assertFalse(decision['modelSelectedStop'])
        self.assertEqual(len(decision['candidates']),1)
        decision=fine.select(model,self.projector,self.before,[942.,383.],current_support='not_supported', current_context=self.context)
        self.assertEqual([c['command']['durationMs'] for c in decision['rejected']],[250])


if __name__ == '__main__':
    unittest.main()
