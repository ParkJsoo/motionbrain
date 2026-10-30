"""Evidence-preserving recovery and readiness. No real device access."""
import copy
import hashlib
import io
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from learning.action_execution import BoundedActionExecutor
from learning.action_journal import (BoundedActionJournal, digest, observe_resolution, prepare_resolution,
                                     read_journal, resolution_valid, validate_evidence, ResolutionReader)
from tests.test_action_execution import FakeClock, FakePort, limits, request


class Reader:
    def __init__(self, record):
        self.record=copy.deepcopy(record)
        self.calls=[]
        self.status_value=dict(state='IDLE',motors={f'M{i}':dict(enabled=False,speed=0) for i in range(1,6)},
            sensor=dict(connected=True,imuOk=True,simulated=False,faultLatched=False,blocked=False),
            teleop=dict(deadman=False,controlActive=False))
        self.capability_value=dict(bootId=record['bootId'],enabled=True,active=False,recordCount=record['ordinal'])
        entries=[dict(sequence=i,atMs=1,actionOrdinal=0,motor=i,pwm=0,reason='forced_off') for i in range(1,6)]
        entries.append(dict(sequence=6,atMs=record['startedAtMs'],actionOrdinal=record['ordinal'],
                            motor=record['motor'],pwm=record['requestedPwm'],reason='driver_write'))
        entries += [dict(sequence=6+i,atMs=record['endedAtMs'],actionOrdinal=record['ordinal'],motor=i,pwm=0,reason='forced_off') for i in range(1,6)]
        self.outputs_value=FakePort.log(record['bootId'],entries)
    def lookup(self, r):self.calls.append('lookup');return copy.deepcopy(self.record)
    def status(self):self.calls.append('status');return copy.deepcopy(self.status_value)
    def outputs(self):self.calls.append('outputs');return copy.deepcopy(self.outputs_value)
    def capability(self):self.calls.append('capability');return copy.deepcopy(self.capability_value)


class JournalTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.path=self.root/'commands.sqlite'
        self.clock=FakeClock();self.port=FakePort(self.clock)
        self.core=BoundedActionExecutor(self.path,self.port,limits(),clock=self.clock)
        self.addCleanup(self.core.close)
        self.port.before_submit=self.core.request_stop
        self.core.propose(request())
        self.core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)
        self.original=self.core.journal.get('first')
        self.assertEqual(self.original['phase'],'outcome_unknown')
        self.reader=Reader(self.port.last_record)
    def evidence(self):return observe_resolution(self.original,self.reader,clock=self.clock)
    def resolve(self):
        evidence=self.evidence()
        return self.core.journal.resolve(evidence,digest(evidence),self.reader,clock=self.clock)
    def test_resolution_preserves_row_and_id_but_allows_future_id(self):
        resolution=self.resolve()
        self.assertEqual(self.core.journal.get('first'),self.original)
        self.assertTrue(resolution_valid(self.original,resolution))
        self.assertFalse(resolution['trainingEligible'])
        self.assertEqual(self.core.journal.reserve('first')[1],'proposal_already_consumed')
        self.assertFalse(self.core.journal.readiness('first')['ready'])
        self.assertTrue(self.core.journal.readiness('next')['ready'])
        self.core.propose(request('next'))
        self.assertIsNone(self.core.journal.reserve('next')[1])
        self.assertEqual(len(self.port.posts),1)
        self.assertEqual(self.reader.calls,['capability','lookup','status','outputs','capability']*2)
    def test_resolution_survives_restart_and_legacy_policy_stays_blocked(self):
        self.resolve();self.core.close()
        j=BoundedActionJournal(self.path,10000)
        self.assertEqual(j.get('first'),self.original)
        self.assertTrue(j.readiness()['ready']);j.close()
        from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.m4_command_journal import M4CommandJournal
        legacy=M4CommandJournal(self.path,10000)
        try:
            legacy.insert('next','test',{},1,{})
            self.assertEqual(legacy.reserve('next')[1],'unresolved_outcome_unknown')
        finally:legacy.close()
    def test_exact_confirmation_required_before_any_reads(self):
        evidence=self.evidence();self.reader.calls=[]
        with self.assertRaisesRegex(ValueError,'exact_resolution'):
            self.core.journal.resolve(evidence,'0'*64,self.reader)
        self.assertEqual(self.reader.calls,[])
        self.assertFalse(self.core.journal.readiness()['ready'])
    def test_terminal_change_or_new_boot_during_apply_does_not_append(self):
        evidence=self.evidence()
        for mutate in [lambda r:r.record.update(reason='state_changed'),
                       lambda r:r.capability_value.update(bootId='b'*32),
                       lambda r:r.outputs_value.update(bootId='b'*32)]:
            r=Reader(self.reader.record);mutate(r)
            with self.assertRaises(ValueError):self.core.journal.resolve(evidence,digest(evidence),r)
        self.assertEqual(self.core.journal.connection.execute('SELECT COUNT(*) FROM action_resolutions').fetchone()[0],0)
    def test_rejects_unverified_or_inconsistent_evidence(self):
        good=self.evidence()
        changes=[('readDurationSeconds',5),('readDurationSeconds',float('nan')),
            ('originalRowSha256','0'*64),('commandId','other')]
        for key,value in changes:
            bad=copy.deepcopy(good);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):validate_evidence(self.original,bad)
        mutations=[lambda e:e['terminalRecord'].update(state='running'),
            lambda e:e['terminalRecord'].update(outputOff=False),
            lambda e:e['terminalRecord'].update(commandId='other'),
            lambda e:e['terminalRecord'].update(elapsedMs=2),
            lambda e:e['capability'].update(active=True),
            lambda e:e['capability'].update(recordCount=2),
            lambda e:e['initialCapability'].update(recordCount=2),
            lambda e:e['capability'].update(recordCount=True),
            lambda e:e['capability'].update(bootId='b'*32),
            lambda e:e['status'].update(state='ARMED'),
            lambda e:e['status']['sensor'].update(faultLatched=True),
            lambda e:e['status']['teleop'].update(deadman=True),
            lambda e:e['status']['motors']['M2'].update(speed=1),
            lambda e:e['outputs'].update(dropped=1),
            lambda e:e['outputs']['entries'][-1].update(pwm=30),
            lambda e:e['outputs']['entries'][-1].update(sequence=99),
            lambda e:e['outputs']['entries'].clear(),
            lambda e:e['outputs']['entries'][5].update(actionOrdinal=2)]
        for i,mutate in enumerate(mutations):
            bad=copy.deepcopy(good);mutate(bad)
            with self.subTest(case=i),self.assertRaises(ValueError):validate_evidence(self.original,bad)
    def test_new_action_between_output_read_and_final_capability_blocks(self):
        original=self.reader.outputs
        def outputs():
            value=original();self.reader.capability_value['recordCount']+=1;return value
        self.reader.outputs=outputs
        with self.assertRaisesRegex(ValueError,'unchanged_last_action'):self.evidence()
        self.assertFalse(self.core.journal.readiness()['ready'])
    def test_cumulative_read_deadline_and_transport_error_block(self):
        original=self.reader.capability
        def slow():self.clock.now+=4.1;return original()
        self.reader.capability=slow
        with self.assertRaisesRegex(ValueError,'expired'):self.evidence()
        self.reader.lookup=lambda r:(_ for _ in ()).throw(TimeoutError())
        with self.assertRaises(TimeoutError):self.evidence()
    def test_resolution_immutable_and_stale_binding_fail_closed(self):
        resolution=self.resolve();j=self.core.journal
        for query in ['DELETE FROM action_resolutions','UPDATE action_resolutions SET resolution_json=resolution_json',
                      'DELETE FROM action_history','UPDATE action_history SET before_json=before_json']:
            with self.assertRaises(sqlite3.IntegrityError):j.connection.execute(query)
            j.connection.rollback()
        changed=copy.deepcopy(self.original);changed['result']['reason']='different'
        self.assertFalse(resolution_valid(changed,resolution))
        with self.assertRaisesRegex(ValueError,'already_recorded'):self.resolve()
    def test_save_history_and_recovery_are_atomic(self):
        j=self.core.journal
        for cid in ['pending','dispatching']:
            self.core.propose(request(cid))
        j.save('dispatching','dispatching',{})
        before=j.records()
        j.connection.execute("CREATE TRIGGER fail_history BEFORE INSERT ON action_history WHEN NEW.command_id='dispatching' BEGIN SELECT RAISE(ABORT,'disk failure'); END")
        j.connection.commit()
        with self.assertRaises(sqlite3.IntegrityError):j.recover_interrupted(digest(before))
        self.assertEqual(j.records(),before)
        j.connection.execute('DROP TRIGGER fail_history');j.connection.commit()
        ids=j.recover_interrupted(digest(before))
        self.assertEqual(ids,['pending','dispatching'])
        self.assertEqual(j.get('pending')['phase'],'not_sent')
        self.assertEqual(j.get('dispatching')['phase'],'outcome_unknown')
        self.assertEqual(j.get('first'),self.original)
        histories=[json.loads(row[0]) for row in j.connection.execute('SELECT before_json FROM action_history')]
        self.assertIn(next(r for r in before if r['commandId']=='pending'),histories)
        self.assertFalse(j.readiness()['ready'])
        with self.assertRaisesRegex(ValueError,'exact_recovery'):j.recover_interrupted(digest(before))
    def test_blocked_confirmation_is_consumed_and_reason_survives_restart(self):
        self.core.propose(request('blocked'))
        result=self.core.confirm('blocked',confirmed_request=request('blocked'),dispatch_guard=lambda r:None)
        self.assertEqual(result['reason'],'unresolved_outcome_unknown')
        self.assertEqual(self.core.journal.get('blocked')['phase'],'not_sent')
        self.resolve();self.core.close()
        reopened=BoundedActionExecutor(self.path,self.port,limits(),clock=self.clock)
        try:
            replay=reopened.confirm('blocked',confirmed_request=request('blocked'),dispatch_guard=lambda r:None)
            self.assertTrue(replay['replayed']);self.assertEqual(replay['reason'],result['reason'])
            self.assertEqual(len(self.port.posts),1)
        finally:reopened.close()
    def test_resolution_of_one_does_not_clear_other_unknown(self):
        self.core.propose(request('other'));self.core.journal.save('other','outcome_unknown',{})
        self.resolve()
        check=self.core.journal.readiness()
        self.assertEqual(check['blockers'],[dict(commandId='other',reason='unresolved_outcome_unknown')])
    def test_readonly_status_and_prepare_leave_bytes_and_pending_unchanged(self):
        self.core.propose(request('pending'));self.core.close()
        before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.iterdir()}
        with patch.object(BoundedActionJournal,'__init__',side_effect=AssertionError('mutation forbidden')):
            check=read_journal(self.path)
            evidence=prepare_resolution(self.path,'first',self.reader,clock=self.clock)
        self.assertFalse(check['ready']);self.assertEqual(len(check['blockers']),2)
        self.assertEqual(evidence['originalRowSha256'],digest(self.original))
        after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.iterdir()}
        self.assertEqual(before,after)
    def test_readonly_lock_and_missing_or_corrupt_journal(self):
        with self.assertRaises(BlockingIOError):read_journal(self.path)
        missing=self.root/'missing.sqlite'
        self.assertFalse(read_journal(missing)['journalExists']);self.assertFalse(missing.exists())
        self.assertFalse(Path(str(missing)+'.lock').exists())
        self.core.close();self.path.write_bytes(b'bad database')
        with self.assertRaises(sqlite3.DatabaseError):read_journal(self.path)
    def test_readonly_protocol_and_unknown_phase_fail_closed(self):
        j=self.core.journal;j.connection.execute("UPDATE commands SET phase='unexpected'");j.connection.commit();self.core.close()
        with self.assertRaisesRegex(ValueError,'unknown_journal_phase'):read_journal(self.path)
        conn=sqlite3.connect(self.path);conn.execute("UPDATE commands SET request_json='{}'");conn.commit();conn.close()
        with self.assertRaisesRegex(ValueError,'protocol'):read_journal(self.path)
    def test_foreign_protocol_rejected_before_schema_migration(self):
        from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.m4_command_journal import M4CommandJournal
        path=self.root/'legacy.sqlite'
        legacy=M4CommandJournal(path,10);legacy.insert('legacy','hash',{},0,{})
        legacy.close();before=path.read_bytes()
        with self.assertRaisesRegex(ValueError,'protocol'):BoundedActionJournal(path,10)
        self.assertEqual(path.read_bytes(),before)
    def test_readonly_http_adapter_initializes_boot_and_uses_only_five_gets(self):
        reader=ResolutionReader('http://127.0.0.1:12345')
        cap={**self.reader.capability_value,'schemaVersion':'motionbrain.bounded_action.v1',
             'clockDomain':'controller_boot_millis'}
        calls=[]
        def get(service,path,params=None):
            calls.append((service,path,params))
            value=(self.reader.record if params else cap) if path=='/actions' else (
                   self.reader.status_value if path=='/status' else self.reader.outputs_value)
            return dict(status=200,json=copy.deepcopy(value)),b''
        with patch.object(reader._port.transport,'get',side_effect=get),patch.object(reader._port,'_post',side_effect=AssertionError('POST forbidden')):
            evidence=observe_resolution(self.original,reader)
        self.assertEqual(evidence['terminalRecord'],self.reader.record)
        self.assertEqual([c[1] for c in calls],['/actions','/actions','/status','/action_outputs','/actions'])
        self.assertFalse(reader._port.writes_enabled)
        self.assertIsNone(reader._port.token_provider)
    def test_management_cli_prepare_and_resolve_preserve_original(self):
        from tools import manage_action_journal as cli
        self.core.close()
        with patch.object(cli,'ROOT',self.root),patch.object(cli,'ResolutionReader',return_value=self.reader):
            local=self.root/'.local';local.mkdir();path=local/'commands.sqlite'
            self.path.rename(path);Path(str(self.path)+'.lock').rename(Path(str(path)+'.lock'))
            before=path.read_bytes();out=local/'evidence'
            with patch('sys.stdout',new_callable=io.StringIO):
                self.assertEqual(cli.main(['--journal',str(path),'prepare','--command-id','first','--controller-url','unused','--output-dir',str(out)]),0)
            self.assertEqual(path.read_bytes(),before)
            evidence=json.loads((out/'resolution-evidence.json').read_text())
            with patch('sys.stdout',new_callable=io.StringIO):
                self.assertEqual(cli.main(['--journal',str(path),'resolve','--evidence',str(out/'resolution-evidence.json'),'--controller-url','unused','--confirm-sha256',digest(evidence)]),0)
            j=BoundedActionJournal(path,10000)
            try:
                self.assertEqual(j.get('first'),self.original)
                self.assertTrue(j.readiness()['ready'])
            finally:j.close()
    def test_cli_blocked_before_token_camera_or_constructor(self):
        from tools.run_visual_action_pilot import main
        from tests.test_visual_action_live import plan
        self.core.close()
        helper=self.root/'helper';helper.write_bytes(b'helper')
        p=plan();p['helperSha256']=hashlib.sha256(helper.read_bytes()).hexdigest()
        path=self.root/'plan.json';path.write_text(json.dumps(p))
        base=['--plan',str(path),'--helper',str(helper),'--journal',str(self.path)]
        for mode in ['--check-journal','--execute']:
            with patch.dict('os.environ',{},clear=True),patch.object(BoundedActionExecutor,'__init__',side_effect=AssertionError('executor forbidden')),patch('socket.socket',side_effect=AssertionError('network forbidden')),patch('sys.stdout',new_callable=io.StringIO) as out:
                self.assertEqual(main(base+[mode]),2)
                self.assertEqual(json.loads(out.getvalue())['mode'],'journal_readiness')
    def test_management_cli_requires_private_existing_and_exact_confirmation(self):
        from tools import manage_action_journal as cli
        self.core.close()
        with self.assertRaisesRegex(ValueError,'private_journal'):cli.main(['--journal',str(self.path),'status'])
        with patch.object(cli,'ROOT',self.root):
            local=self.root/'.local';local.mkdir()
            path=local/'commands.sqlite';self.path.rename(path);Path(str(self.path)+'.lock').rename(Path(str(path)+'.lock'))
            with patch.object(BoundedActionJournal,'__init__',side_effect=AssertionError('writable forbidden')):
                with self.assertRaisesRegex(ValueError,'exact_recovery'):
                    cli.main(['--journal',str(path),'recover','--confirm-snapshot-sha256','bad'])
            with patch('sys.stdout',new_callable=io.StringIO):
                self.assertEqual(cli.main(['--journal',str(path),'status']),2)


if __name__=='__main__':unittest.main()
