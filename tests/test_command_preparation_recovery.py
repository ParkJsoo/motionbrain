"""명령 의도 저장 뒤 preflight GET 실패한 실제 수집 경로의 무구동 해소 회귀."""
import hashlib
import json
import unittest
from unittest.mock import patch

from learning import command_preparation_recovery as recovery
from learning.action_journal import BoundedActionJournal
from learning.attended_collection_profile import ATTENDED_60_1000, ATTENDED_SHOULDER_DOWN_12, audit_profile
from learning.manual_collection import ManualCollection
from tests import test_manual_collection as fixtures


class RecoveryTests(unittest.TestCase):
    strong_shoulder_window = False

    def setUp(self):
        owner = fixtures.ManualCollectionTests()
        owner.setUp()
        self.addCleanup(owner.tearDown)
        self.owner = owner
        owner.manager.collection_profile = ATTENDED_SHOULDER_DOWN_12 if self.strong_shoulder_window else ATTENDED_60_1000
        owner.manager.attended_shoulder_down_window = self.strong_shoulder_window
        self.device, self.requests, self.factory = owner.transport()
        joint = 'shoulder' if self.strong_shoulder_window else 'elbow'
        owner.start(joint)
        self.root = owner.manager.store.root
        owner.transport_failed = True
        with self.assertRaises(TimeoutError):
            owner.manager.pulse(dict(joint=joint, direction='down', percent=70, durationMs=1000))
        session = owner.manager.current
        self.assertEqual(session['error'], 'command_failed_no_retry')
        self.assertFalse(session['finalIdleOffVerified'])
        self.assertEqual(sum(e['kind'] == 'command' for e in session['events']), 1)
        self.assertFalse(any(e['kind'] == 'result' for e in session['events']))
        self.assertFalse(any(method == 'POST' for method, _, _ in self.requests))
        self.sid = session['sessionId']
        self.raw = (self.root / 'session.json').read_bytes()
        self.digest = hashlib.sha256(self.raw).hexdigest()
        (self.root / 'native-camera/retained-image.bin').write_bytes(b'original native pixels')
        self.original = {p.relative_to(self.root).as_posix(): p.read_bytes()
                         for p in self.root.rglob('*') if p.is_file()}
        owner.manager.close()
        owner.manager = ManualCollection(owner.root/'sessions', owner.helper, 'http://10.0.0.10',
            'secret', 'cam', journal_path=owner.root/'journal.sqlite', clock=owner.clock)
        self.manager = owner.manager
        self.manager.journal = BoundedActionJournal(str(self.manager.journal_path), 10000)
        self.db = self.manager.journal.connection
        self.row = self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?', (self.sid,)).fetchone()
        owner.transport_failed = False
        self.requests.clear()
        init = recovery._ReadOnlyPort.__init__
        factory = self.factory
        self.ports = []
        def setup(port, *args, **kwargs):
            init(port, *args, **kwargs)
            opened = factory(*args, **kwargs).opener.open
            def open_current(request, timeout):
                from tests.test_supervised_transport import Response
                import urllib.parse
                address = urllib.parse.urlsplit(request.full_url)
                if address.path == '/action_outputs' and not address.query:
                    self.requests.append((request.get_method(), request.full_url, timeout))
                    if owner.transport_failed:
                        raise TimeoutError('synthetic current read')
                    retained = self.device.entries[-128:]
                    self.device.clock.sleep(.01)
                    return Response(dict(bootId=self.device.boot, evidence='driver_write',
                        clockDomain='controller_boot_millis', entries=retained,
                        dropped=retained[0]['sequence']-1))
                return opened(request, timeout)
            port.opener = type('Opener', (), {'open': staticmethod(open_current)})()
            self.ports.append(port)
        p = patch.object(recovery._ReadOnlyPort, '__init__', setup)
        p.start()
        self.addCleanup(p.stop)

    def recover(self, **changes):
        args = dict(original_controller_url='http://127.0.0.1')
        args.update(changes)
        return self.manager.recover_command_preparation_readonly(self.sid, self.digest, 'c'*32, **args)

    def assert_blocked(self):
        self.assertEqual(self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',
                                        (self.sid,)).fetchone(), self.row)

    def test_current_direct_GET_only_retirement_preserves_entire_original(self):
        result = self.recover()
        self.assertTrue(result['currentOffVerified'])
        self.assertEqual(self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',
            (self.sid,)).fetchone(), (recovery.PHASE, self.row[1]))
        self.assertTrue(all((self.root/name).read_bytes() == raw for name, raw in self.original.items()))
        self.assertTrue(self.requests)
        self.assertTrue(all(method == 'GET' and url.startswith('http://10.0.0.10/') for method, url, _ in self.requests))
        self.assertTrue(self.ports[0].full_output_snapshot)
        self.assertEqual(self.ports[0].timeout, 2.)
        self.assertEqual(self.ports[0].token, '')
        proof = json.loads((self.root/recovery.ATTEMPT/'proof.json').read_text())
        self.assertEqual(set(proof['originalSourceHashes']), set(self.original))
        self.assertFalse(proof['executableTransportBaselineCreated'])
        self.assertFalse(proof['datasetEligible'])
        self.assertFalse(proof['replayAllowed'])

    def test_hidden_POST_RUN_or_cleanup_intent_rejected_before_GET(self):
        path = self.root/'http-999999-intent.json'
        for payload in ({'action': 'run'}, {'action': 'clear'}):
            path.write_text(json.dumps(dict(method='POST',path='/sequence',payload=payload)))
            with self.assertRaises(ValueError):
                self.recover()
            self.assertEqual(self.requests, [])
            self.assert_blocked()

    def test_command_result_or_extra_command_rejected(self):
        for name in ('command-001-result.json', 'command-002-intent.json'):
            p = self.root/name
            p.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'single_command'):
                self.recover()
            p.unlink()
        self.assertEqual(self.requests, [])

    def test_command_artifact_must_match_event(self):
        p = self.root/'command-001-intent.json'
        d = json.loads(p.read_text())
        d['pulse']['percent'] = 80
        p.write_text(json.dumps(d))
        with self.assertRaisesRegex(ValueError, 'command_binding'):
            self.recover()
        self.assertEqual(self.requests, [])

    def test_source_URL_and_boot_must_be_explicit_original(self):
        with self.assertRaisesRegex(ValueError, 'owned_source'):
            self.recover(original_controller_url=self.manager.controller_url)
        with self.assertRaises(ValueError):
            self.manager.recover_command_preparation_readonly(self.sid,self.digest,'d'*32,
                original_controller_url='http://127.0.0.1')
        self.assertEqual(self.requests, [])

    def test_current_boot_change_or_nonzero_never_sends_cleanup(self):
        self.device.boot = 'd'*32
        with self.assertRaises(ValueError):
            self.recover()
        self.assert_blocked()
        self.assertTrue(all(method == 'GET' for method, _, _ in self.requests))

    def test_current_nonzero_does_not_retire_or_stop(self):
        self.device.append(3,30)
        with self.assertRaises(ValueError):
            self.recover()
        self.assert_blocked()
        self.assertTrue(all(method == 'GET' for method, _, _ in self.requests))

    def test_read_timeout_consumes_attempt_and_keeps_original(self):
        self.owner.transport_failed = True
        with self.assertRaises(TimeoutError):
            self.recover()
        count = len(self.requests)
        self.assert_blocked()
        self.owner.transport_failed = False
        with self.assertRaisesRegex(ValueError, 'attempt_consumed'):
            self.recover()
        self.assertEqual(len(self.requests), count)

    def test_native_or_HTTP_response_mutation_during_GET_blocks_update(self):
        # owner factory wraps the device's bound method; patch port opener instead.
        init = recovery._ReadOnlyPort.__init__
        def setup(port,*a,**k):
            init(port,*a,**k)
            opened = port.opener.open
            def mutate(*args,**kwargs):
                result = opened(*args,**kwargs)
                (self.root/'native-camera/retained-image.bin').write_bytes(b'changed')
                return result
            port.opener = type('Opener',(),{'open':staticmethod(mutate)})()
        with patch.object(recovery._ReadOnlyPort,'__init__',setup):
            with self.assertRaisesRegex(ValueError,'source_changed'):
                self.recover()
        self.assert_blocked()

    def test_other_pending_block_rejected_before_GET(self):
        self.db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)', ('other','x','x','pending',None))
        self.db.commit()
        with self.assertRaisesRegex(ValueError,'owned_block'):
            self.recover()
        self.assertEqual(self.requests, [])


class StrongShoulderWindowRecoveryTests(RecoveryTests):
    strong_shoulder_window = True

    def test_no_receipt_exception_is_not_available_to_training_audit(self):
        plan=json.loads((self.root/'plan.json').read_text())
        original=json.loads(self.raw)
        pulse=json.loads((self.root/'command-001-intent.json').read_text())['pulse']
        with self.assertRaisesRegex(ValueError,'window_provenance'):
            audit_profile(plan,original,pulse)
        audit_profile(plan,original,pulse,motion_not_started=True)
        original['events'].append(dict(kind='result',receipt={}))
        with self.assertRaisesRegex(ValueError,'not_started_evidence'):
            audit_profile(plan,original,pulse,motion_not_started=True)

    def test_not_started_still_requires_same_explicit_window(self):
        path=self.root/'plan.json'
        plan=json.loads(path.read_text())
        plan['attendedShoulderDownWindow']['maximumDecreaseDeg']=13.
        path.write_text(json.dumps(plan))
        with self.assertRaisesRegex(ValueError,'window_provenance'):
            self.recover()
        self.assertEqual(self.requests,[])
        self.assert_blocked()


if __name__ == '__main__':
    unittest.main()
