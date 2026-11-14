"""알려진 이전 완료 원본은 보존하되 라이브 scope 승인은 갱신하지 않는다."""
import copy
import unittest

from learning import m4_up_commissioning as core, batch_m4
from learning.supervised_collection import plan_digest
from tests import test_batch_m4 as fixtures
from tests.test_m4_noncontact import scope as noncontact_scope


class RecordedScopeTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.SourceAuditTests(); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        # NONCONTACT 도입 전 구현을 명시한다. frozenset 순서는 계약이 아니다.
        self.old = 'ed3333408c4662d442c7bd61bec579c1da16dd8e9eae79448f261f57d9b249ba'
        self.scope = copy.deepcopy(self.f.planned)
        self.scope['implementationSha256'] = self.old

    def record(self):
        f = self.f
        f.fixture.write('scope.json', self.scope)
        report = f.fixture.read('result.json'); report['scopeSha256'] = plan_digest(self.scope)
        f.fixture.write('result.json', report)
        f.result_sha = batch_m4.sha((f.root/'result.json').read_bytes())

    def test_known_recorded_digest_audits_without_rewriting_original(self):
        self.record()
        raw = (self.f.root/'scope.json').read_bytes()
        self.assertEqual(self.f.audit()['scope'], self.scope)
        self.assertEqual((self.f.root/'scope.json').read_bytes(), raw)
        with self.assertRaisesRegex(ValueError, 'implementation_changed'):
            core.validate_scope(self.scope)
        with self.assertRaisesRegex(ValueError, 'implementation_changed'):
            core.M4UpCommissioning(self.scope, None, '', None, evidence_root=self.f.root)

    def test_unknown_digest_and_changed_safety_contract_fail_offline(self):
        for key, value in [('implementationSha256','0'*64),('maxCommands',2),
                           ('maxObservedDeltaDeg',2.),('pulse',core.LIFT_PULSE)]:
            changed = copy.deepcopy(self.scope); changed[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                core.validate_recorded_scope(changed)

    def test_old_digest_cannot_claim_new_noncontact_schema(self):
        value = noncontact_scope(); value['implementationSha256'] = self.old
        with self.assertRaisesRegex(ValueError, 'recorded_implementation_unknown'):
            core.validate_recorded_scope(value)

    def test_original_scope_hash_stays_bound_to_result(self):
        self.record()
        changed = copy.deepcopy(self.scope); changed['trialId'] = 'changed'
        self.f.fixture.write('scope.json', changed)
        with self.assertRaisesRegex(ValueError, 'scope_binding_changed'):
            self.f.audit()


class RecordedBindingTests(unittest.TestCase):
    def test_offline_binding_does_not_grant_live_route_execution(self):
        import json
        f = fixtures.BindingTests(); f.setUp(); self.addCleanup(f.doCleanups)
        f.scope['implementationSha256'] = 'ed3333408c4662d442c7bd61bec579c1da16dd8e9eae79448f261f57d9b249ba'
        path = f.local/'scope.json'; path.write_text(json.dumps(f.scope))
        f.binding['scopeSha256'] = batch_m4.sha(path.read_bytes())
        checked = batch_m4.validate_binding(f.plan,f.condition,f.binding,f.repo,recorded=True)
        self.assertEqual(checked['scope'], f.scope)
        with self.assertRaisesRegex(ValueError, 'implementation_changed'):
            batch_m4.validate_binding(f.plan,f.condition,f.binding,f.repo)
        clock = fixtures.WallClock()
        route = batch_m4.BatchM4Route(f.plan,f.repo,f.local/'run',lambda:'unused',
            observer_spec=fixtures.spec_for(f.scope),clock=clock)
        with self.assertRaisesRegex(ValueError, 'implementation_changed'):
            route.execute(f.condition,'old-scope',f.binding,deadline_wall=clock.time()+60)
        self.assertFalse((f.local/'run/m4-scopes').exists())


if __name__ == '__main__': unittest.main()
