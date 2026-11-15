"""실제 M4 actor 최초 GET 실패와 모의 현재 장치를 연결하는 회귀 검증."""
import hashlib
import json
import sqlite3
import unittest
from unittest.mock import patch

from learning import m4_failed_start_recovery as recovery
from learning.supervised_collection import plan_digest
from tests import test_m4_device_guard_integration as fixture


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        fx = fixture.ActorTests()
        fx.setUp()
        self.addCleanup(fx.doCleanups)
        self.fx = fx
        fx.device.open = lambda *a, **k: (_ for _ in ()).throw(TimeoutError('synthetic initial GET'))
        result = fx.run_trial()
        self.assertEqual(result['commandOutcome'], 'not_started', result)
        self.assertFalse(result['finalIdleOffVerified'], result)
        self.root = fx.store.root
        self.journal = fx.repo / '.local/visual-actions/controller-commands.sqlite'
        self.args = dict(repo_root=fx.repo, source_root=self.root,
                         result_sha256=self.sha('result.json'), scope_sha256=self.sha('scope.json'),
                         controller_boot_id=fx.scope['controllerBootId'], controller_url=fx.scope['controllerUrl'])
        self.device = fixture.Device(fx.clock)
        status = self.device.status
        self.device.status = lambda: dict(status(), uptimeMs=int(fx.clock.monotonic() * 1000),
            outputProof=dict(schemaVersion='driver_cursor.v1',bootId=self.device.boot,
                sequenceBefore=self.device.entries[-1]['sequence'],sequenceAfter=self.device.entries[-1]['sequence'],stable=True))
        self.original = {p.name: p.read_bytes() for p in self.root.iterdir() if p.is_file()}
        self.original_row = self.row()
        init = recovery._ReadOnlyPort.__init__
        self.ports = []
        def setup(port, *a, **k):
            init(port, *a, **k)
            port.opener = self.device
            self.ports.append(port)
        p = patch.object(recovery._ReadOnlyPort, '__init__', setup)
        p.start()
        self.addCleanup(p.stop)

    def sha(self, name):
        return hashlib.sha256((self.root / name).read_bytes()).hexdigest()

    def row(self):
        with sqlite3.connect(self.journal) as db:
            return db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',
                              (self.fx.scope['trialId'],)).fetchone()

    def proposal(self):
        return recovery.prepare_failed_m4_start_recovery(**self.args)

    def run_recovery(self, **changes):
        args = dict(self.args, token='private-token', clock=self.fx.clock,
                    confirmed_proposal_sha256=plan_digest(self.proposal()))
        args.update(changes)
        return recovery.recover_failed_m4_start_readonly(**args)

    def test_prepare_is_readonly_and_success_only_updates_phase(self):
        proposal = self.proposal()
        self.assertFalse(proposal['controllerWritesAllowed'])
        self.assertEqual(self.device.requests, [])
        self.assertFalse((self.root / recovery.ATTEMPT).exists())
        result = self.run_recovery()
        self.assertTrue(result['currentOffVerified'])
        self.assertEqual(self.row(), (recovery.PHASE, self.original_row[1]))
        self.assertEqual(self.original, {p: (self.root / p).read_bytes() for p in self.original})
        self.assertTrue(self.device.requests)
        self.assertTrue(all(payload is None for _, payload in self.device.requests))
        proof = json.loads((self.root / recovery.ATTEMPT / 'proof.json').read_text())
        self.assertEqual(len(proof['snapshots']), 2)
        self.assertFalse(proof['originalOutcomeChanged'])
        self.assertFalse(proof['datasetEligible'])
        self.assertFalse(proof['replayAllowed'])
        self.assertEqual(self.ports[0].token, '')

    def test_POST_or_second_intent_rejected_before_device_access(self):
        path = self.root / 'http-000002-intent.json'
        path.write_text(json.dumps(dict(method='POST', path='/sequence', payload={'action': 'run'})))
        with self.assertRaisesRegex(ValueError, 'only_initial_GET'):
            self.run_recovery()
        self.assertEqual(self.device.requests, [])
        self.assertEqual(self.row(), self.original_row)

    def test_forged_single_POST_and_changed_outcome_rejected(self):
        path = self.root / 'http-000001-intent.json'
        intent = json.loads(path.read_text())
        intent.update(method='POST', path='/command', payload={'cmd': 'arm'})
        path.write_text(json.dumps(intent))
        with self.assertRaisesRegex(ValueError, 'initial_GET_proof'):
            self.run_recovery()
        self.assertEqual(self.device.requests, [])

    def test_changed_approval_or_boot_rejected_without_GET(self):
        for change in (dict(confirmed_proposal_sha256='f' * 64), dict(controller_boot_id='f' * 32)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.run_recovery(**change)
        self.assertEqual(self.device.requests, [])
        self.assertEqual(self.row(), self.original_row)

    def test_unrelated_pending_block_preserved(self):
        with sqlite3.connect(self.journal) as db:
            db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)', ('other', 'f' * 64, '/other', 'pending', '{}'))
        with self.assertRaisesRegex(ValueError, 'other_pending'):
            self.run_recovery()
        self.assertEqual(self.device.requests, [])

    def test_live_boot_change_nonzero_output_and_queue_rejected_GET_only(self):
        self.device.boot = 'f' * 32
        with self.assertRaises(ValueError):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)
        self.assertTrue(all(payload is None for _, payload in self.device.requests))

    def test_live_nonzero_output_rejected_without_stop(self):
        self.device.append(4, 35)
        with self.assertRaises(ValueError):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)
        self.assertTrue(all(payload is None for _, payload in self.device.requests))

    def test_live_queue_not_empty_rejected_without_clear(self):
        self.device.queue = [dict(joint='shoulder')]
        with self.assertRaises(ValueError):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)
        self.assertTrue(all(payload is None for _, payload in self.device.requests))

    def test_timeout_preserves_block_and_consumes_sole_recovery_attempt(self):
        self.device.open = lambda *a, **k: (_ for _ in ()).throw(TimeoutError('synthetic recovery'))
        with self.assertRaises(TimeoutError):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)
        with self.assertRaises(Exception):
            self.run_recovery()
        self.assertEqual(len(self.ports), 1)

    def test_source_mutation_during_GET_prevents_update(self):
        opened = self.device.open
        def mutate(*a, **k):
            response = opened(*a, **k)
            (self.root / 'result.json').write_bytes(self.original['result.json'] + b' ')
            return response
        self.device.open = mutate
        with self.assertRaisesRegex(ValueError, 'source_changed'):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)

    def test_output_epoch_change_between_rounds_preserves_block(self):
        opened = self.device.open
        reads = []
        def mutate(*a, **k):
            response = opened(*a, **k)
            if len(self.device.requests) == 4:
                self.device.append(4, 0)
            reads.append(True)
            return response
        self.device.open = mutate
        with self.assertRaisesRegex(ValueError, 'output_epoch_changed'):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)

    def test_new_journal_not_created(self):
        self.journal.unlink()
        with self.assertRaisesRegex(ValueError, 'existing_journal'):
            self.run_recovery()
        self.assertFalse(self.journal.exists())
        self.assertEqual(self.device.requests, [])

    def test_truncated_history_missing_three_motor_OFFs_uses_current_cursor_only(self):
        for _ in range(150):
            self.device.append(4, 50)
            self.device.append(4, 0)
        self.device.entries = self.device.entries[-128:]
        self.assertEqual({e['motor'] for e in self.device.entries}, {4})
        result = self.run_recovery()
        self.assertTrue(result['currentOffVerified'])
        proof = json.loads((self.root / recovery.ATTEMPT / 'proof.json').read_text())
        self.assertFalse(proof['executableTransportBaselineCreated'])
        self.assertFalse(proof['originalOutcomeReevaluated'])

    def test_missing_or_unstable_cursor_cannot_replace_expired_history(self):
        original = self.device.status
        self.device.status = lambda: dict(original(), outputProof=dict(original()['outputProof'], stable=False))
        with self.assertRaisesRegex(ValueError, 'stable_driver_cursor'):
            self.run_recovery()
        self.assertEqual(self.row(), self.original_row)


if __name__ == '__main__':
    unittest.main()
