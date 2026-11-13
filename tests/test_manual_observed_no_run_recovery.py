"""관측 후 무구동 종료 실패의 일회 GET 종결. 실제 소켓·장치 요청은 없다."""
import copy
import hashlib
import json
from unittest.mock import patch
import unittest

from learning.action_journal import BoundedActionJournal
from tests import test_manual_collection as fixtures


class ObservedNoRunRecoveryTests(unittest.TestCase):
    def setUp(self):
        owner=fixtures.ManualCollectionTests();owner.setUp();self.addCleanup(owner.tearDown)
        self.owner=owner
        self.device,self.requests,self.factory=owner.transport()
        owner.start('elbow')
        self.root=owner.manager.store.root
        owner.transport_failed=True
        report=owner.manager.finish()
        self.assertEqual(report['error'],'output_off_unverified')
        self.assertEqual(report['phase'],'failed')
        self.assertGreater(len(report['frames']),0)
        self.assertFalse(any(e['kind'] in ('command','result') for e in report['events']))
        self.sid=report['sessionId'];self.raw=(self.root/'session.json').read_bytes()
        self.digest=hashlib.sha256(self.raw).hexdigest()
        (self.root/'native-camera/retained-image.bin').write_bytes(b'original-observation')
        self.original_files={str(p.relative_to(self.root)):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        owner.manager.close();owner.manager=owner.make();self.manager=owner.manager
        self.manager.journal=BoundedActionJournal(str(self.manager.journal_path),10000)
        self.db=self.manager.journal.connection
        self.result_json=self.db.execute('SELECT result_json FROM supervised_blocks WHERE session_id=?',(self.sid,)).fetchone()[0]
        owner.transport_failed=False;self.requests.clear()
    def recover(self):
        with patch('learning.manual_collection.SequencePort',side_effect=self.factory):
            return self.manager.recover_observed_no_run_readonly(self.sid,self.digest,'c'*32)
    def unchanged(self):
        self.assertEqual((self.root/'session.json').read_bytes(),self.raw)
        self.assertTrue(all((self.root/name).read_bytes()==data for name,data in self.original_files.items()))
        self.assertEqual(self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(self.sid,)).fetchone(),
                         ('off_unverified',self.result_json))
    def update_original(self,mutation):
        original=json.loads(self.raw);mutation(original)
        self.raw=json.dumps(original).encode();self.digest=hashlib.sha256(self.raw).hexdigest()
        (self.root/'session.json').write_bytes(self.raw)
        self.result_json=self.raw.decode()
        self.db.execute('UPDATE supervised_blocks SET result_json=? WHERE session_id=?',(self.result_json,self.sid));self.db.commit()
    def test_only_owned_terminal_phase_changes_and_original_observation_failure_is_preserved(self):
        result=self.recover()
        self.assertTrue(result['currentOffVerified']);self.assertTrue(result['noWrites'])
        self.assertFalse(result['originalOutcomeChanged']);self.assertFalse(result['replayAllowed'])
        self.assertEqual(self.db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(self.sid,)).fetchone(),
                         ('resolved_current_off_no_run_observed',self.result_json))
        self.assertTrue(all((self.root/name).read_bytes()==data for name,data in self.original_files.items()))
        self.assertTrue(self.requests)
        self.assertTrue(all(method=='GET' for method,_,_ in self.requests))
        proof=json.loads((self.root/'readonly-observed-no-run-recovery/proof.json').read_text())
        self.assertEqual(proof['originalSessionSha256'],self.digest)
        self.assertTrue(proof['originalNoMotionRequestVerified'])
        self.assertFalse(proof['originalConfigurationRestoreConfirmed'])
        self.assertFalse(proof['datasetEligible'])
        self.assertEqual(proof['snapshot']['actions']['bootId'],'c'*32)
        self.assertTrue(all(not value['enabled'] and value['speed']==0 for value in proof['snapshot']['status']['motors'].values()))
        self.assertEqual(proof['snapshot']['sequence']['sequence']['totalCount'],0)
        count=len(self.requests)
        with self.assertRaises(ValueError):self.recover()
        self.assertEqual(len(self.requests),count)
    def test_original_safe_cleanup_intents_are_allowed_but_no_cleanup_is_retransmitted(self):
        original=json.loads(self.raw)
        for i,(path,payload) in enumerate([('/command',{'cmd':'stop'}),('/sequence',{'action':'clear'}),
                                        ('/motor',{'action':'default','speed':100})],1000):
            original['events'].append(dict(kind='http_error',path=path,payload=payload,error='URLError'))
            (self.root/f'http-{i:06d}-intent.json').write_text(json.dumps(dict(method='POST',path=path,payload=payload)))
        self.update_original(lambda value:value.update(events=original['events']))
        result=self.recover()
        self.assertTrue(result['currentOffVerified'])
        self.assertTrue(all(method=='GET' for method,_,_ in self.requests))
    def test_any_motion_intent_or_command_artifact_is_rejected_before_fresh_GET(self):
        for route,payload in [('/sequence',{'action':'run'}),('/sequence',{'action':'add'}),
                              ('/command',{'cmd':'arm'}),('/motor',{'action':'default','speed':255}),
                              ('/motor',{'motor':'M3','speed':30})]:
            path=self.root/'http-999999-intent.json'
            path.write_text(json.dumps(dict(method='POST',path=route,payload=payload)))
            with self.subTest(route=route,payload=payload),self.assertRaisesRegex(ValueError,'get_only_required'):
                self.recover()
            path.unlink()
        (self.root/'command-hidden.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'owned_source_required'):self.recover()
        self.assertEqual(self.requests,[])
        self.assertFalse((self.root/'readonly-observed-no-run-recovery').exists())
    def test_command_events_original_boot_and_current_owner_must_match(self):
        for name,value in [('busy',True),('current',{}),('port',object()),('camera',object()),('store',object())]:
            with self.subTest(name=name),patch.object(self.manager,name,value),self.assertRaises(ValueError):self.recover()
        with self.assertRaises(ValueError):
            self.manager.recover_observed_no_run_readonly(self.sid,self.digest,'d'*32)
        self.update_original(lambda original:original['events'].append(dict(kind='command')))
        with self.assertRaisesRegex(ValueError,'owned_source_required'):self.recover()
        self.assertEqual(self.requests,[])
    def test_source_sha_DB_result_plan_and_other_pending_must_match_before_GET(self):
        with self.assertRaisesRegex(ValueError,'source_changed'):
            self.manager.recover_observed_no_run_readonly(self.sid,'0'*64,'c'*32)
        row=self.db.execute('SELECT * FROM supervised_blocks WHERE session_id=?',(self.sid,)).fetchone()
        for field,value in [('plan_hash','other'),('output_path',str(self.root/'foreign')),('result_json','{}')]:
            self.db.execute('UPDATE supervised_blocks SET '+field+'=? WHERE session_id=?',(value,self.sid));self.db.commit()
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'owned_block_required'):self.recover()
            self.db.execute('DELETE FROM supervised_blocks WHERE session_id=?',(self.sid,))
            self.db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',row);self.db.commit()
        self.db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',('other','x','x','pending',None));self.db.commit()
        with self.assertRaisesRegex(ValueError,'owned_block_required'):self.recover()
        self.assertEqual(self.requests,[])
    def test_fresh_boot_change_or_not_OFF_consumes_attempt_and_keeps_blocked(self):
        self.device.boot='d'*32
        with self.assertRaisesRegex(ValueError,'boot_changed'):self.recover()
        self.unchanged()
        count=len(self.requests);self.device.boot='c'*32
        with self.assertRaises(Exception):self.recover()
        self.assertEqual(len(self.requests),count)
    def test_fresh_queue_not_empty_is_not_cleared_and_block_remains(self):
        self.device.queue.append(dict(joint='elbow'))
        with self.assertRaisesRegex(ValueError,'single_step_queue_changed'):self.recover()
        self.assertEqual(len(self.device.queue),1)
        self.assertTrue(all(method=='GET' for method,_,_ in self.requests));self.unchanged()
    def test_fresh_nonzero_motor_is_not_stopped_or_accepted_as_OFF(self):
        self.device.append(3,30)
        with self.assertRaises(ValueError):self.recover()
        self.assertEqual(self.device.speeds[3],30)
        self.assertTrue(all(method=='GET' for method,_,_ in self.requests))
        self.unchanged()

    def test_unready_legacy_and_source_change_during_fresh_read_do_not_retire_block(self):
        with patch.object(self.manager.journal,'readiness',return_value={'ready':False}):
            with self.assertRaisesRegex(ValueError,'owned_block_required'):self.recover()
        self.assertEqual(self.requests,[])
        original_factory=self.factory
        def changed_factory(*args,**kwargs):
            port=original_factory(*args,**kwargs)
            preflight=port.preflight
            def changed():
                result=preflight()
                (self.root/'http-999999-intent.json').write_text(json.dumps(
                    dict(method='POST',path='/command',payload={'cmd':'arm'})))
                return result
            port.preflight=changed
            return port
        self.factory=changed_factory
        with self.assertRaisesRegex(ValueError,'source_changed'):self.recover()
        self.unchanged()
        self.assertFalse((self.root/'readonly-observed-no-run-recovery/proof.json').exists())

    def test_fresh_GET_timeout_is_not_retried_or_used_to_retire_block(self):
        self.owner.transport_failed=True
        with self.assertRaises(TimeoutError):self.recover()
        self.unchanged();count=len(self.requests)
        self.owner.transport_failed=False
        with self.assertRaises(Exception):self.recover()
        self.assertEqual(len(self.requests),count)
        self.assertFalse((self.root/'readonly-observed-no-run-recovery/proof.json').exists())


if __name__=='__main__':unittest.main()
