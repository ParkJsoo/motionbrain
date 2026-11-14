"""실패 원본·현재 GET OFF 원본과 결합한 단발 수동 DOWN 복구. 장치 I/O 없음."""
import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from learning import m4_up_commissioning as core
from learning import m4_band_recovery as recovery
from learning.artifacts import ArtifactStore
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort,RECOVERY_DOWN_DEADLINE_POLICY
from tests import test_m4_target_up as target_fixtures
from tests import test_m4_up_commissioning as fixtures


class RecoveryCodeBindingTests(unittest.TestCase):
    def test_changed_recovery_validator_invalidates_live_implementation_binding(self):
        original=core.implementation_digest();read=Path.read_bytes
        def changed(path):
            raw=read(path)
            return raw+b'\n# changed recovery validator\n' if path.name=='m4_band_recovery.py' else raw
        with patch.object(Path,'read_bytes',changed):
            self.assertNotEqual(core.implementation_digest(),original)
        self.assertEqual(core.implementation_digest(),original)


class BandRecoveryTests(unittest.TestCase):
    def setUp(self):
        owner=target_fixtures.TargetUpTests();owner.setUp();self.addCleanup(owner.doCleanups)
        self.owner=owner;self.repo=owner.repo;self.clock=owner.clock;self.device=owner.device
        self.device.after_run=lambda device:setattr(device,'angle',197.55)
        def after(record):
            self.device.angle=198.72;owner.trial._tick(True)
        owner.observer.on_after=after
        failed=owner.run_trial()
        self.assertEqual(failed['commandOutcome'],'receipt_recorded',failed)
        self.assertTrue(failed['finalIdleOffVerified']);self.assertEqual(failed['status'],'failed')
        self.source_bytes={str(p):p.read_bytes() for p in owner.store.root.iterdir() if p.is_file()}
        self.offstore=ArtifactStore(self.repo/'.local/current-off-source')
        port=SequencePort('http://controller.invalid','private-token','a'*32,self.offstore,
                          compact_reads=True,clock=self.clock);port.opener=self.device
        snapshot=port.preflight()
        proofs=[]
        for intent in sorted(self.offstore.root.glob('http-*-intent.json')):
            proofs.append(dict(intent=self.bind(intent),response=self.bind(intent.with_name(intent.name.replace('-intent','-response')))))
        off=dict(schemaVersion=recovery.OFF_SCHEMA,controllerBootId='a'*32,cameraBootId='b'*32,
            observedAt=self.clock.now,outputSequence=port.last_sequence,snapshot=snapshot,sourceProofs=proofs)
        self.offpath=self.repo/'.local/current-off.json';self.offpath.write_text(json.dumps(off))
        context=dict(owner.scope['context'],poseRegime='explicit-band-recovery',supportMode='gripper_supported')
        self.scope=dict(schemaVersion=core.BAND_RETURN_RECOVERY_SCHEMA,trialId='one-band-recovery',hostClockId='one-band-recovery',
            controllerUrl='http://controller.invalid',controllerBootId='a'*32,implementationSha256=core.implementation_digest(),
            context=context,pulse=copy.deepcopy(core.RETURN_PULSE),defaultSpeed=255,maxCommands=1,
            expectedStartAngleDeg=198.72,startAngleToleranceDeg=.10,maxObservedDeltaDeg=1.,maxSeconds=60.,
            maxObservationAgeSeconds=10.,postOffSeconds=3.,goalAngleDeg=198.10,
            originalGlobalReferenceDeg=195.29,originalGlobalUpperDeg=198.29,recoveryEntryUpperDeg=198.82)
        self.authpath=self.repo/'.local/recovery-authorization.json'
        self.auth=dict(schemaVersion=recovery.AUTH_SCHEMA,approved=True,automaticRetry=False,modelSelected=False,
            trainingEligible=False,userInstruction='합의된 단발 DOWN 복구',
            sourceFailureSha256=self.bind(owner.store.root/'result.json')['sha256'],
            currentOffSha256=self.bind(self.offpath)['sha256'],motionContractSha256=recovery.recovery_motion_digest(self.scope))
        self.authpath.write_text(json.dumps(self.auth))
        self.scope['recoveryEvidence']=dict(sourceScope=self.bind(owner.store.root/'scope.json'),
            sourceFailure=self.bind(owner.store.root/'result.json'),currentOff=self.bind(self.offpath),authorization=self.bind(self.authpath))
        self.store=ArtifactStore(self.repo/'.local/recovery-run')
        self.observer=fixtures.Observer(self.store.root,self.clock,self.scope)
        def support(record):
            record['onsite']['tableSupportVerified']=False
            record['evidenceSha256']=self.observer.save(record['evidenceFile'],
                {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
            self.observer.latest_record=copy.deepcopy(record)
        self.observer.on_before=support;self.observer.on_after=support
        self.owner.decode_patch.stop()
        from unittest.mock import patch
        self.decode=patch.object(core,'decode',side_effect=self.observer.decode);self.decode.start();self.addCleanup(self.decode.stop)
    def bind(self,path):
        return dict(path=str(path.relative_to(self.repo/'.local')),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    def actor(self):
        actor=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.store.root,clock=self.clock)
        actor.port.opener=self.device;return actor
    def run_actor(self,actor):return actor.run(confirmed_scope_sha256=plan_digest(self.scope))
    def since(self,base):return [(p,v) for p,v in self.device.requests[base:] if v is not None]
    def unchanged(self):
        self.assertTrue(all(Path(p).read_bytes()==raw for p,raw in self.source_bytes.items()))

    def test_single_DOWN_normal_goal_stop_returns_band_with_dual_view_review(self):
        actor=self.actor();start=len(self.device.requests)
        self.device.after_run=lambda device:setattr(device,'angle',198.05)
        result=self.run_actor(actor)
        self.assertEqual(result['status'],'completed',result);self.assertTrue(result['bandReturned'])
        self.assertTrue(result['recoveryAfterRetentionReviewed']);self.assertFalse(result['tableReturnVerified'])
        self.assertFalse(result['modelSelected']);self.assertFalse(result['trainingEligible'])
        self.assertEqual(result['receipt']['monitorTimingPolicy'],RECOVERY_DOWN_DEADLINE_POLICY)
        self.assertEqual(result['receipt']['stopReason'],'observed_band_recovery_goal')
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.since(start)),1)
        self.assertTrue(result['finalIdleOffVerified']);self.unchanged()

    def test_observed_one_degree_corridor_failure_stops_OFF_without_retry(self):
        actor=self.actor();base=len(self.device.requests)
        self.device.after_run=lambda device:setattr(device,'angle',197.5)
        result=self.run_actor(actor);self.assertEqual(result['status'],'failed')
        self.assertFalse(result['bandReturned']);self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.since(base)),1)
        self.unchanged()

    def test_known_cap_shortfall_above_original_band_is_not_recovery_success(self):
        actor=self.actor();self.device.after_run=lambda device:setattr(device,'angle',198.4)
        result=self.run_actor(actor);self.assertEqual(result['status'],'failed')
        self.assertFalse(result['bandReturned']);self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(result['commandOutcome'],'receipt_recorded')

    def test_stale_current_OFF_and_live_boot_change_rejected(self):
        self.clock.sleep(30.01)
        with self.assertRaisesRegex(ValueError,'fresh_same_boot'):self.actor()

    def test_source_bytes_and_unknown_failed_RUN_are_not_overridden(self):
        path=self.owner.store.root/'result.json';source=json.loads(path.read_text())
        source['commandOutcome']='unknown';path.write_text(json.dumps(source))
        self.scope['recoveryEvidence']['sourceFailure']=self.bind(path)
        with self.assertRaisesRegex(ValueError,'known_failed_UP'):self.actor()

    def test_before_live_boot_drift_no_DOWN_RUN(self):
        actor=self.actor();self.device.boot='c'*32;base=len(self.device.requests)
        result=self.run_actor(actor);self.assertFalse(result['mayHaveRun']);self.assertFalse(result['bandReturned'])
        self.assertFalse(any(v=={'action':'run'} for _,v in self.since(base)))

    def test_exact_recovery_only_scope_and_no_parent_model_hooks(self):
        for change in [dict(pulse=core.LEARNING_PULSE),dict(maxObservedDeltaDeg=3.),dict(goalAngleDeg=196.59),
                       dict(originalGlobalUpperDeg=199.),dict(expectedStartAngleDeg=198.8),dict(startAngleToleranceDeg=.25)]:
            with self.subTest(change=change),self.assertRaises(ValueError):core.validate_scope(dict(self.scope,**change))
        with self.assertRaisesRegex(ValueError,'task_hooks'):
            core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.store.root,clock=self.clock,
                task_hooks=dict(prepare=lambda:None,before_run=lambda:None,status_guard=lambda status:None,decisionSha256='f'*64))

    def test_unknown_recovery_RUN_is_single_and_shared_journal_blocks_replacement(self):
        actor=self.actor();base=len(self.device.requests)
        self.device.after_run=lambda device:(_ for _ in ()).throw(TimeoutError('lost recovery RUN reply'))
        result=self.run_actor(actor)
        self.assertEqual(result['commandOutcome'],'unknown');self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.since(base)),1)
        self.assertFalse(result['bandReturned'])
        count=len(self.device.requests)
        other=core.M4UpCommissioning(self.scope,self.observer,'private-token',ArtifactStore(self.repo/'.local/replacement'),
            evidence_root=self.store.root,clock=self.clock);other.port.opener=self.device
        self.assertEqual(other.run(confirmed_scope_sha256=plan_digest(self.scope))['status'],'failed')
        self.assertEqual(count,len(self.device.requests))

    def test_changed_failure_bytes_or_false_authorization_rejected_before_IO(self):
        self.auth['approved']=False;self.authpath.write_text(json.dumps(self.auth))
        self.scope['recoveryEvidence']['authorization']=self.bind(self.authpath)
        with self.assertRaisesRegex(ValueError,'motion_authorization'):self.actor()
        self.auth['approved']=True;self.authpath.write_text(json.dumps(self.auth))
        self.scope['recoveryEvidence']['authorization']=self.bind(self.authpath)
        path=self.owner.store.root/'result.json';path.write_bytes(path.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'source_changed'):self.actor()

    def test_negative_dual_view_after_review_cannot_mark_band_returned(self):
        actor=self.actor();self.device.after_run=lambda device:setattr(device,'angle',198.05)
        def rejected(record):
            record['onsite']['jawContactVerified']=False
            record['evidenceSha256']=self.observer.save(record['evidenceFile'],
                {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
            self.observer.latest_record=copy.deepcopy(record)
        self.observer.on_after=rejected
        result=self.run_actor(actor)
        self.assertEqual(result['status'],'failed');self.assertFalse(result['bandReturned'])
        self.assertTrue(result['finalIdleOffVerified']);self.assertTrue(result['currentOriginalBandReentered'])
