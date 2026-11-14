"""실패 RUN 원본을 유지하는 현재 상태 GET 복구 회귀 검사."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from learning.manual_collection import ManualCollection, SCHEMA
from learning.action_journal import BoundedActionJournal


class FailedRunRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name).resolve()
        self.sid = 'manual-owned'
        self.session = self.root/'sessions'/self.sid
        self.session.mkdir(parents=True)
        self.plan = dict(sessionId=self.sid, controllerUrl='http://127.0.0.1', camera=dict(deviceId='camera'))
        self.original = dict(schemaVersion=SCHEMA, sessionId=self.sid, phase='failed',
            error='command_failed_no_retry', finalIdleOffVerified=False,
            events=[dict(kind='http', path='/actions', response=dict(bootId='a'*32)),
                    dict(kind='http', path='/sequence', payload=dict(action='run'), response=dict(success=True)),
                    dict(kind='command')])
        (self.session/'plan.json').write_text(json.dumps(self.plan))
        self.raw = json.dumps(self.original).encode()
        (self.session/'session.json').write_bytes(self.raw)
        self.digest = hashlib.sha256(self.raw).hexdigest()
        self.manager = ManualCollection(self.root/'sessions', self.root/'helper', 'http://127.0.0.1',
            'private-token', 'camera', journal_path=self.root/'journal.sqlite')
        self.manager.journal = BoundedActionJournal(str(self.manager.journal_path), 10000)
        self.db = self.manager.journal.connection
        self.db.execute('CREATE TABLE supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT, output_path TEXT, phase TEXT, result_json TEXT)')
        self.db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)', (self.sid,
            hashlib.sha256(json.dumps(self.plan,sort_keys=True).encode()).hexdigest(), str(self.session),
            'off_unverified', self.raw.decode()))
        self.db.commit()
        self.ports = []
        test = self
        class ReadPort:
            def __init__(self, url, token, boot, store, **kwargs):
                self.token = token
                self.off_verified = True
                self.may_have_run = self.writes_possible = False
                test.ports.append(self)
            def _call(self, path):
                test.assertEqual(path, '/actions')
                return dict(bootId='a'*32)
            def preflight(self): return dict(allFiveOff=True)
        self.port_type = ReadPort
        self.mock = patch('learning.manual_collection.SequencePort', ReadPort)
        self.mock.start()
    def tearDown(self):
        self.manager.close()
        self.mock.stop()
        self.tmp.cleanup()
    def recover(self):
        return self.manager.recover_failed_run_readonly(self.sid, self.digest, 'a'*32)
    def unchanged(self):
        row = self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(self.sid,)).fetchone()
        self.assertEqual(row, ('off_unverified', self.raw.decode()))
        self.assertEqual((self.session/'session.json').read_bytes(), self.raw)
    def test_resolves_only_phase_preserves_original_and_clears_port_token(self):
        self.assertTrue(self.recover()['currentOffVerified'])
        row = self.db.execute('SELECT phase,result_json FROM supervised_blocks').fetchone()
        self.assertEqual(row, ('resolved_current_off_after_failed_run', self.raw.decode()))
        self.assertEqual((self.session/'session.json').read_bytes(), self.raw)
        self.assertEqual(self.ports[0].token, '')
        proof=json.loads((self.session/'readonly-failed-run-recovery/proof.json').read_text())
        self.assertFalse(proof['datasetEligible'])
        self.assertFalse(proof['originalOutcomeChanged'])
        self.assertNotIn('private-token', json.dumps(proof))
        with self.assertRaises(ValueError): self.recover()
    def test_changed_source_and_foreign_path_rejected_before_reads(self):
        with self.assertRaises(ValueError): self.manager.recover_failed_run_readonly(self.sid,'0'*64,'a'*32)
        self.db.execute('UPDATE supervised_blocks SET output_path=?', (str(self.root/'foreign'),))
        self.db.commit()
        with self.assertRaises(ValueError): self.recover()
        self.assertEqual(self.ports, [])
    def test_final_cleanup_failure_resolves_current_off_without_changing_result(self):
        self.original['error']='output_off_unverified'
        self.original['events'].append(dict(kind='result', receipt=dict(offVerified=True)))
        self.raw=json.dumps(self.original).encode()
        self.digest=hashlib.sha256(self.raw).hexdigest()
        (self.session/'session.json').write_bytes(self.raw)
        self.db.execute('UPDATE supervised_blocks SET result_json=?',(self.raw.decode(),))
        self.db.commit()
        self.assertTrue(self.recover()['currentOffVerified'])
        self.assertEqual(self.db.execute('SELECT result_json FROM supervised_blocks').fetchone()[0],self.raw.decode())
        self.assertEqual((self.session/'session.json').read_bytes(),self.raw)
    def test_other_pending_rejected(self):
        self.db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',('other','x','x','pending',None))
        self.db.commit()
        with self.assertRaises(ValueError): self.recover()
        self.unchanged()
        self.assertEqual(self.ports, [])
    def test_reboot_rejected_and_attempt_consumed(self):
        self.port_type._call=lambda port,path: dict(bootId='b'*32)
        with self.assertRaisesRegex(ValueError,'boot_changed'): self.recover()
        self.unchanged()
        self.port_type._call=lambda port,path: dict(bootId='a'*32)
        with self.assertRaises(Exception): self.recover()
        self.assertEqual(len(self.ports),1)
    def test_not_off_or_writes_rejected(self):
        def preflight(port): port.off_verified=False; return {}
        self.port_type.preflight=preflight
        with self.assertRaisesRegex(ValueError,'off_not_proven'): self.recover()
        self.unchanged()
        self.assertEqual(self.ports[0].token,'')
    def test_concurrent_result_change_rejected(self):
        def preflight(port):
            self.db.execute('UPDATE supervised_blocks SET result_json=?', ('{}',)); self.db.commit()
            return {}
        self.port_type.preflight=preflight
        with self.assertRaisesRegex(ValueError,'owned_block'): self.recover()
        self.assertEqual(self.db.execute('SELECT phase FROM supervised_blocks').fetchone()[0], 'off_unverified')

    def test_explicit_reboot_recovery_preserves_original_boot_and_result(self):
        self.port_type._call=lambda port,path: dict(bootId='b'*32)
        self.manager.controller_url='http://127.0.0.1:9000'
        result=self.manager.recover_failed_run_after_reboot_readonly(
            self.sid,self.digest,'a'*32,'b'*32,self.plan['controllerUrl'])
        self.assertTrue(result['currentOffVerified'])
        proof=json.loads((self.session/'readonly-failed-run-recovery/proof.json').read_text())
        self.assertEqual(proof['originalControllerBootId'],'a'*32)
        self.assertEqual(proof['currentControllerBootId'],'b'*32)
        self.assertFalse(proof['originalOutcomeReevaluated'])
        self.assertEqual((self.session/'session.json').read_bytes(),self.raw)
        self.assertEqual(self.db.execute('SELECT result_json FROM supervised_blocks').fetchone()[0],self.raw.decode())

    def test_explicit_reboot_rejects_wrong_source_or_same_boot_before_io(self):
        for source,current,url in [('a'*32,'a'*32,self.plan['controllerUrl']),
                                   ('c'*32,'b'*32,self.plan['controllerUrl']),
                                   ('a'*32,'b'*32,'http://foreign')]:
            with self.assertRaises(ValueError):
                self.manager.recover_failed_run_after_reboot_readonly(self.sid,self.digest,source,current,url)
        self.unchanged()
        self.assertEqual(self.ports,[])

    def test_explicit_reboot_still_rejects_live_boot_change(self):
        self.port_type._call=lambda port,path: dict(bootId='c'*32)
        with self.assertRaisesRegex(ValueError,'boot_changed'):
            self.manager.recover_failed_run_after_reboot_readonly(
                self.sid,self.digest,'a'*32,'b'*32,self.plan['controllerUrl'])
        self.unchanged()
