"""모의 HTTP/임시 원장만 사용한 M4 재부팅 OFF 해소 회귀."""
import copy
import hashlib
import json
import sqlite3
import unittest
from unittest.mock import patch
from learning import m4_failed_run_recovery as recovery
from learning import m4_up_commissioning as core
from learning.supervised_collection import plan_digest
from tests import test_m4_effective as effective_fixture
from tests.test_m4_device_guard_integration import Device

class RecoveryTests(unittest.TestCase):
 def setUp(self):
  fx=effective_fixture.EffectiveActorTests();fx.setUp();self.addCleanup(fx.doCleanups);self.fx=fx
  fx.scope.update(pulse=core.effective.effective_task_pulse('down'),expectedStartAngleDeg=197.75,goalAngleDeg=197.22)
  fx.scope['context']['supportMode']='gripper_supported';fx.device.angle=197.75
  def before(record):
   record['onsite']['tableSupportVerified']=False
   record['evidenceSha256']=fx.observer.save(record['evidenceFile'],{k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
   fx.observer.latest_record=copy.deepcopy(record)
  fx.observer.on_before=before;fx.reset();fx.device.after_run=lambda d:setattr(d,'angle',194.59)
  result=fx.run_trial();self.assertEqual(result['commandOutcome'],'unknown',result);self.assertTrue(result['finalIdleOffVerified'])
  self.root=fx.store.root;self.journal=fx.repo/'.local/visual-actions/controller-commands.sqlite'
  self.device=Device(fx.clock);self.device.boot='c'*32;self.calls=[]
  original=recovery._ReadOnlyPort.__init__
  def init(port,*args,**kwargs):original(port,*args,**kwargs);port.opener=self.device;self.calls.append(port)
  self.patcher=patch.object(recovery._ReadOnlyPort,'__init__',init);self.patcher.start();self.addCleanup(self.patcher.stop)
  self.args=dict(repo_root=fx.repo,source_root=self.root,result_sha256=self.sha('result.json'),scope_sha256=self.sha('scope.json'),
   original_boot_id='a'*32,current_boot_id='c'*32,original_controller_url=fx.scope['controllerUrl'],current_controller_url='http://controller.invalid')
  self.original={p.name:p.read_bytes() for p in self.root.iterdir() if p.is_file()}
  self.original_row=self.row()
 def sha(self,name):return hashlib.sha256((self.root/name).read_bytes()).hexdigest()
 def row(self):
  with sqlite3.connect(self.journal) as db:return db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(self.fx.scope['trialId'],)).fetchone()
 def proposal(self):return recovery.prepare_failed_m4_reboot_recovery(**self.args)
 def run_recovery(self,**changes):
  args=dict(self.args,token='private-token',clock=self.fx.clock,confirmed_proposal_sha256=plan_digest(self.proposal()));args.update(changes)
  return recovery.recover_failed_m4_after_reboot_readonly(**args)
 def test_prepare_has_no_device_or_file_writes(self):
  proposal=self.proposal();self.assertEqual(proposal['fromPhase'],'pending');self.assertEqual(self.device.requests,[])
  self.assertFalse((self.root/recovery.ATTEMPT).exists());self.assertEqual(self.row(),self.original_row)
 def test_only_phase_changes_original_unknown_and_raw_preserved(self):
  result=self.run_recovery();self.assertEqual(result['phase'],recovery.PHASE)
  self.assertEqual(self.row(),(recovery.PHASE,self.original_row[1]))
  self.assertEqual(self.original,{name:(self.root/name).read_bytes() for name in self.original})
  self.assertTrue(self.device.requests);self.assertTrue(all(v is None for _,v in self.device.requests))
  proof=json.loads((self.root/recovery.ATTEMPT/'proof.json').read_text())
  self.assertFalse(proof['originalOutcomeReevaluated']);self.assertFalse(proof['datasetEligible']);self.assertFalse(proof['replayAllowed'])
 def test_same_boot_missing_approval_and_wrong_original_reject_before_GET(self):
  for changes in (dict(current_boot_id='a'*32),dict(confirmed_proposal_sha256='f'*64),dict(original_boot_id='d'*32)):
   with self.subTest(changes=changes),self.assertRaises(ValueError):self.run_recovery(**changes)
  self.assertEqual(self.device.requests,[]);self.assertEqual(self.row(),self.original_row)
 def test_other_pending_and_changed_DB_source_reject_before_GET(self):
  with sqlite3.connect(self.journal) as db:db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',('other','f'*64,'/other','pending','{}'))
  with self.assertRaises(ValueError):self.run_recovery()
  self.assertEqual(self.device.requests,[])
 def test_nonzero_output_or_guard_used_preserves_block_consumes_attempt(self):
  self.device.record=dict(nonce='f'*32,state='terminal',offVerified=True)
  with self.assertRaisesRegex(ValueError,'clean_guard_boot'):self.run_recovery()
  self.assertEqual(self.row(),self.original_row);count=len(self.device.requests)
  with self.assertRaises(Exception):self.run_recovery()
  self.assertEqual(len(self.device.requests),count)
 def test_boot_changed_or_queue_not_empty_never_cleanup_POST(self):
  self.device.boot='d'*32
  with self.assertRaises(ValueError):self.run_recovery()
  self.assertEqual(self.row(),self.original_row);self.assertTrue(all(v is None for _,v in self.device.requests))
 def test_timeout_not_retried_and_attempt_consumed(self):
  self.device.open=lambda *a,**k:(_ for _ in ()).throw(TimeoutError('synthetic'))
  with self.assertRaises(TimeoutError):self.run_recovery()
  self.assertEqual(len(self.calls),1);self.assertEqual(self.row(),self.original_row)
  with self.assertRaises(Exception):self.run_recovery()
  self.assertEqual(len(self.calls),1)
 def test_source_mutation_during_GET_prevents_update(self):
  original=self.device.open
  def opened(*a,**k):
   result=original(*a,**k)
   (self.root/'scope.json').write_bytes(self.original['scope.json']+b' ')
   return result
  self.device.open=opened
  with self.assertRaisesRegex(ValueError,'source_changed'):self.run_recovery()
  self.assertEqual(self.row(),self.original_row)
 def test_new_journal_not_created(self):
  self.journal.unlink()
  with self.assertRaisesRegex(ValueError,'existing_shared_journal'):self.run_recovery()
  self.assertFalse(self.journal.exists());self.assertEqual(self.device.requests,[])
 def test_current_nonzero_output_rejected_without_STOP(self):
  self.device.append(4,30)
  with self.assertRaises(ValueError):self.run_recovery()
  self.assertEqual(self.row(),self.original_row);self.assertTrue(all(v is None for _,v in self.device.requests))
 def test_current_queue_not_empty_rejected_without_clear(self):
  self.device.queue=[{'joint':'shoulder'}]
  with self.assertRaises(ValueError):self.run_recovery()
  self.assertEqual(self.row(),self.original_row);self.assertTrue(all(v is None for _,v in self.device.requests))
 def test_appended_source_artifact_during_GET_rejected(self):
  original=self.device.open
  def opened(*a,**k):
   result=original(*a,**k);(self.root/'new-command.json').write_text('{}');return result
  self.device.open=opened
  with self.assertRaisesRegex(ValueError,'source_changed'):self.run_recovery()
  self.assertEqual(self.row(),self.original_row)
 def test_matching_JSON_with_changed_database_bytes_invalidates_approval(self):
  proposal=self.proposal()
  with sqlite3.connect(self.journal) as db:db.execute('UPDATE supervised_blocks SET result_json=result_json || ?',(' ',))
  with self.assertRaisesRegex(ValueError,'proposal_approval'):self.run_recovery(confirmed_proposal_sha256=plan_digest(proposal))
  self.assertEqual(self.device.requests,[])
 def test_original_unknown_never_becomes_learning_eligible(self):
  self.run_recovery()
  from learning.task_transition_data import audit_manual_m4
  with self.assertRaises(ValueError):audit_manual_m4(self.root)
 def test_readonly_port_forbids_stop_even_on_failure(self):
  self.run_recovery()
  with self.assertRaisesRegex(ValueError,'GET_only'):self.calls[0]._call('/command',dict(cmd='stop'),emergency=True)

if __name__=='__main__':unittest.main()
