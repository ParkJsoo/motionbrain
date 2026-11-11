"""별도 비접촉 M4 scope: 실제 장치 없이 기존 고정 출력·원장·정지 검증."""
import copy
import json
import unittest
from unittest.mock import patch

from learning import m4_up_commissioning as core, batch_m4
from learning.batch_learning import fit
from learning.batch_model_trial import BatchModelSequence
from learning.supervised_collection import plan_digest
from tests import test_m4_up_commissioning as fixtures
from tests import test_batch_m4 as batch_fixtures
from tests.test_batch_learning import row


def scope():
    value = fixtures.scope()
    value.update(schemaVersion=core.NONCONTACT_SCHEMA, load='empty')
    value['context'].update(object='none', supportMode='empty_no_object')
    return value


class NoncontactTests(unittest.TestCase):
    def setUp(self):
        f = fixtures.CommissioningTests(); f.setUp(); self.addCleanup(f.doCleanups); self.f = f
        f.scope = scope()
        f.observer.scope = f.scope
        # The supplied reviewer explicitly verifies an empty scene and clearance;
        # neither contact nor an object supported by the table is asserted.
        self.original = f.observer.observe
        def observe(**kwargs):
            record = self.original(**kwargs)
            record['onsite'].update(jawContactVerified=False, tableSupportVerified=False)
            raw = {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            record['evidenceSha256'] = f.observer.save(record['evidenceFile'], raw)
            f.observer.latest_record = copy.deepcopy(record)
            return record
        f.observer.observe = observe
        f.trial = core.M4UpCommissioning(f.scope, f.observer, 'private', f.store,
                                        evidence_root=f.evidence, clock=f.clock)
        f.trial.port.opener = f.device

    def test_empty_noncontact_one_pulse_uses_original_stop_and_consumption(self):
        f = self.f
        result = f.run_trial()
        self.assertEqual(result['status'], 'completed', result)
        self.assertEqual(result['trialKind'], 'noncontact_position')
        self.assertFalse(result['graspVerified']); self.assertFalse(result['liftVerified'])
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(v == {'action':'run'} for _,v in f.posts()), 1)
        self.assertIn(('/sequence', dict(action='add',joint='shoulder',direction='up',speed='16',duration='200')), f.posts())
        with self.assertRaisesRegex(ValueError, 'unconsumed'): f.run_trial()
        self.assertEqual(sum(v == {'action':'run'} for _,v in f.posts()), 1)

    def test_contact_or_uncleared_scene_cannot_dispatch(self):
        f = self.f
        def mark(record):
            record['onsite']['clearanceVerified'] = False
        f.observer.on_before = mark
        result = f.run_trial()
        self.assertNotEqual(result['status'], 'completed')
        self.assertFalse(any(v == {'action':'run'} for _,v in f.posts()))

    def test_explicit_empty_scope_cannot_escalate_or_reinterpret_contact(self):
        for change in ('object','load','supportMode','pulse','limit'):
            value = scope()
            if change == 'object': value['context']['object'] = 'pink-object'
            elif change == 'load': value['load'] = 'held'
            elif change == 'supportMode': value['context']['supportMode'] = 'table_supported'
            elif change == 'pulse': value['pulse'] = copy.deepcopy(core.LIFT_PULSE)
            else: value['maxObservedDeltaDeg'] = 3.
            with self.subTest(change=change), self.assertRaises(ValueError): core.validate_scope(value)
        self.assertEqual(core.MAX_CONTROLLER_PROOF_AGE_SECONDS, .25)

    def test_model_sequence_can_request_position_up_without_close(self):
        rows = [row(1), row(3)]
        command = {k:core.PULSE[k] for k in ('joint','direction','percent','durationMs')}
        for item in rows: item.update(joint='shoulder', command=command)
        model = fit(rows)
        stage = dict(id='up',kind='position',modelId='shoulder',target=[4.]*6,
                     tolerance=[.2]*6,commands=[command],maxCommands=1)
        sequence = BatchModelSequence('up', [stage], {'shoulder':model})
        observation = dict(status='accepted', context=model['context'],stateFields=model['stateFields'],
                           state=[2.]*6,frameIds=['fresh'],observedAt=10.,persistedAt=10.1)
        self.assertEqual(sequence.propose(observation,now=10.2)['request']['command'], command)


class NoncontactBindingTests(unittest.TestCase):
    def setUp(self):
        f = batch_fixtures.BindingTests(); f.setUp(); self.addCleanup(f.doCleanups); self.f = f
        value = scope()
        path = f.local/'scope.json'; path.write_text(json.dumps(value))
        f.binding.update(interaction='noncontact',scopeSha256=batch_m4.sha(path.read_bytes()))

    def test_separate_schema_required_and_retention_is_not_implied(self):
        f = self.f
        result = batch_m4.validate_binding(f.plan,f.condition,f.binding,f.repo)
        self.assertEqual(result['interaction'], 'noncontact')
        f.binding['retentionCaptures'] = dict(count=2,intervalSeconds=1)
        with self.assertRaisesRegex(ValueError, 'noncontact_retention'):
            batch_m4.validate_binding(f.plan,f.condition,f.binding,f.repo)
        f.binding.pop('retentionCaptures'); f.binding['interaction'] = 'grasped'
        with self.assertRaisesRegex(ValueError, 'interaction_scope'):
            batch_m4.validate_binding(f.plan,f.condition,f.binding,f.repo)


class NoncontactSourceTests(unittest.TestCase):
    def test_separate_noncontact_original_audit_preserves_no_contact_claim(self):
        f = batch_fixtures.SourceAuditTests(); f.setUp(); self.addCleanup(f.doCleanups)
        planned = copy.deepcopy(f.planned)
        planned.update(schemaVersion=core.NONCONTACT_SCHEMA, load='empty')
        planned['context'].update(object='none', supportMode='empty_no_object')
        f.fixture.write('scope.json', planned)
        result = f.fixture.read('result.json')
        result.update(schemaVersion=core.NONCONTACT_SCHEMA,scopeSha256=plan_digest(planned))
        f.fixture.write('result.json', result)
        f.result_sha = batch_m4.sha((f.root/'result.json').read_bytes())
        for label in ('before','after'):
            record = f.fixture.read(label+'-observation.json')
            record['context'] = copy.deepcopy(planned['context'])
            record['onsite'].update(jawContactVerified=False, tableSupportVerified=False)
            evidence = {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            f.fixture.write(record['evidenceFile'], evidence)
            record['evidenceSha256'] = batch_m4.sha((f.root/record['evidenceFile']).read_bytes())
            f.fixture.write(label+'-observation.json', record)
        audit = f.audit()
        self.assertEqual(audit['execution']['status'], 'verified')
        self.assertEqual(audit['scope']['schemaVersion'], core.NONCONTACT_SCHEMA)
        self.assertFalse(audit['physicalSuccessClaimed'])
        record = f.fixture.read('after-observation.json')
        record['onsite']['jawContactVerified'] = True
        evidence = {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
        f.fixture.write(record['evidenceFile'], evidence)
        record['evidenceSha256'] = batch_m4.sha((f.root/record['evidenceFile']).read_bytes())
        f.fixture.write('after-observation.json', record)
        with self.assertRaisesRegex(ValueError, 'noncontact_semantics'): f.audit()


class NoncontactModelRouteTests(unittest.TestCase):
    def setUp(self):
        from tests import test_batch_model_runtime as model_fixtures
        self.f = model_fixtures.BatchModelRuntimeTests(); self.f.setUp(); self.addCleanup(self.f.doCleanups)

    def runner(self, interaction):
        from learning.batch_model_runtime import BatchModelRunner
        from learning.batch_campaign import digest
        f = self.f; runtime = f.runtime
        command = {k:core.PULSE[k] for k in ('joint','direction','percent','durationMs')}
        rows = [row(1), row(3)]
        for item in rows: item.update(joint='shoulder', command=command,
            context=f.model['context'],stateFields=f.model['stateFields'],
            before=item['before'][:4],after=item['after'][:4])
        model = fit(rows); runtime.model = model
        condition = dict(f.condition, command=command)
        stage = dict(f.stage, modelId='shoulder', commands=[command])
        runtime.plan['conditions'] = [condition]
        runtime.config['conditionBindings'] = {condition['id']:dict(route='m4_up',interaction=interaction)}
        runtime.plan['runtime'] = runtime.config; runtime.plan_sha = digest(runtime.plan)
        # These tests isolate model/runtime route selection. Exact scope checking
        # and actual core dispatch are exercised above, without this injection.
        runtime.validate_condition = lambda c: None
        runtime._m4_model = lambda: model
        calls = []; original = runtime.observe_model
        def observe(model, *, m4=False):
            calls.append(m4)
            return original(model, m4=m4)
        runtime.observe_model = observe
        runner = BatchModelRunner('noncontact-model',[stage],{'approach':runtime},
            {'approach':[condition]},{'shoulder':model},f.f.private/'noncontact-model')
        return runner, calls

    def test_position_noncontact_uses_m4_observation_for_decision_and_after(self):
        runner, calls = self.runner('noncontact')
        result = runner.run(confirmed_scope_sha256=runner.scope_sha256)
        self.assertEqual(result['status'], 'completed', result)
        self.assertEqual(calls, [True,True,True])

    def test_position_cannot_reinterpret_grasped_scope(self):
        with self.assertRaisesRegex(ValueError, 'position_noncontact_route'):
            self.runner('grasped')
        self.assertEqual(self.f.runtime.executions, 0)


if __name__ == '__main__': unittest.main()
