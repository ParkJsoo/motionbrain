"""사람 판독은 원본 결합과 유지 시간 증거 없이 성공으로 승격되지 않는다."""
import copy
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from learning.artifacts import ArtifactStore
from learning.current_cycle_phase import CurrentCyclePhase,SCHEMA,source_record
from learning.supervised_collection import plan_digest
from tests.test_supervised_transport import Clock


class PhaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)/"reviews";self.store=ArtifactStore(self.root);self.clock=Clock()
        self.audited=[];self.reject=False
        def audit(stage,record):self.audited.append(record);return not self.reject
        self.requests=ArtifactStore(Path(self.tmp.name)/'requests')
        self.phase=CurrentCyclePhase(self.root,audit_observation=audit,clock=self.clock,request_store=self.requests)
        self.source=SimpleNamespace(describe_source=Mock(return_value=dict(original=True)),
            scope=dict(retentionSeconds=1.,stages=[dict(kind=k) for k in ('approach','close','lift','lower','release')]),stationary={k:{} for k in ('approach','close','lift','lower','release')})
        self.phase.bind_source(self.source)
        self.phase.preflight(dict(retentionSeconds=1.,stages=[dict(kind=k) for k in ('approach','close','lift','lower','release')]),dict(phaseMode='source_bound_human',
            phaseProtocol='nonce_seed_anchor_fresh_once_v1'))
        self.stage=dict(kind='close')
        self.record=dict(frames=[dict(camera='external',frameId='e',pixelSha256='a'*64,sourceEarliest=9.,sourceLatest=9.05,persistedAt=9.06),
                                dict(camera='gripper',frameId='g',pixelSha256='b'*64,sourceEarliest=9.1,sourceLatest=9.15,persistedAt=9.16)],
                         context=dict(object='current'),hostClockId='cycle',outputSequence=3,
                         controllerBootId='a'*32,stable=True,sourceRoute='idle')

    def review(self,record=None,stage=None,**changes):
        record=record or self.record;stage=stage or self.stage
        value=dict(schemaVersion=SCHEMA,sourceSha256=plan_digest(source_record(record)),
            stage=stage['kind'],humanAssisted=True,method='explicit_dual_source_visual_review',
            frames=record['frames'],reached=True,stable=True,labels=dict(bilateralContact=True))
        value['labels'].update(operatorPresent=True,clearance=True)
        value.update(changes)
        value['labels']={'operatorPresent':True,'clearance':True,**value['labels']}
        original=self.requests.write_json
        def publish(path,request):
            saved=original(path,request)
            if not path.endswith('-request.json'):return saved
            reply=dict(value,nonce=request['nonce'],reviewedAt=self.clock.monotonic());reply.setdefault('accepted',True)
            self.store.write_json(request['nonce']+'-response.json',reply)
            return saved
        self.phase.anchors.clear()  # independent explicit reviews for unit fixtures
        self.requests.write_json=publish
        try:return self.phase.resolve(stage,record,deadline=20.,tick=lambda:None)
        finally:self.requests.write_json=original

    def test_no_review_or_model_pixel_target_can_invent_contact(self):
        with self.assertRaisesRegex(ValueError,'not_reviewed'):self.phase.goal(self.stage,self.record)

    def test_source_bound_goal_explicit_human_assistance_and_reaudit(self):
        self.review();self.record['entry']=self.phase.entry(self.stage,self.record)
        result=self.phase.goal(self.stage,self.record)
        self.assertTrue(result['humanAssisted']);self.assertTrue(self.phase.verify_goal(self.stage,self.record,result))
        self.reject=True
        with self.assertRaisesRegex(ValueError,'raw_audit'):self.phase.verify_goal(self.stage,self.record,result)

    def test_label_or_source_change_rejected(self):
        self.review(labels=dict(bilateralContact=False))
        with self.assertRaisesRegex(ValueError,'goal_labels'):self.phase.goal(self.stage,self.record)
        changed=copy.deepcopy(self.record);changed['frames'][0]['frameId']='another'
        with self.assertRaisesRegex(ValueError,'not_reviewed'):self.phase.goal(self.stage,changed)

    def test_false_goal_preserves_failure_and_does_not_choose_command(self):
        self.review(reached=False,labels=dict(bilateralContact=False))
        result=self.phase.goal(self.stage,self.record)
        self.assertFalse(result['reached']);self.assertNotIn('command',result)

    def test_positive_lift_requires_fresh_pair_after_both_camera_intervals(self):
        self.stationary_source();self.stage=dict(kind='lift')
        current,_=self.review(labels=dict(airborne=True,retained=True))
        result=self.phase.goal(self.stage,current)
        self.assertEqual(self.refreshes,1)
        self.assertGreaterEqual(result['observationTimes'][1]-result['observationTimes'][0],1.)

    def test_two_distinct_raw_pairs_and_span_required(self):
        stage=dict(kind='lift');first=copy.deepcopy(self.record)
        last=copy.deepcopy(first);last['frames']=[dict(first['frames'][i],frameId=name,sourceEarliest=t,sourceLatest=t+.05,persistedAt=t+.06)
                                               for i,(name,t) in enumerate([('e2',10.2),('g2',10.3)])]
        self.clock.now=10.5
        refs=[self.store.write_json(name,record) for name,record in [('first.json',first),('last.json',last)]]
        refs=[dict(path=r['artifactRef'],sha256=r['sha256']) for r in refs]
        labels=dict(airborne=True,retained=True,bilateralContact=True)
        self.review(first,stage,labels=labels,reached=False)
        self.review(last,stage,labels=labels,retention=refs)
        result=self.phase.goal(stage,last)
        self.assertEqual(result['observationTimes'],[9.15,10.2]);self.assertEqual(len(self.audited),2)

    def test_human_false_not_accepted_as_autonomous_visual_success(self):
        with self.assertRaisesRegex(ValueError,'reply_rejected'):self.review(humanAssisted=False)

    def stationary_source(self):
        self.known={};self.refreshes=0;self.shift=0.;self.proof_bad=False
        def pair(stage,record):
            result=dict(context=record['context'],controllerBootId='a'*32,cameraBootId='b'*32,
                outputSequence=record['outputSequence'],frames=record['frames'],reviewFeatures=dict(
                    externalObjectBox=[0.,0.,10.,10.],externalToolCenter=[5.,5.],
                    gripperObjectBox=[self.shift,0.,10.,10.],gripperReferencePoints=[1.,1.,8.,1.],M4AngleDeg=195.29))
            self.known[plan_digest(result)]=copy.deepcopy(result);return result
        def refresh(stage,record,*,deadline,not_before=None):
            self.refreshes+=1;self.clock.now=max(self.clock.now,not_before or self.clock.now)+.1;fresh=copy.deepcopy(record)
            fresh['frames']=[dict(f,frameId=f['frameId']+'fresh',sourceEarliest=self.clock.now-.05,
                                  sourceLatest=self.clock.now-.02,persistedAt=self.clock.now-.01) for f in record['frames']]
            return fresh
        self.source.stationary_pair=pair
        self.source.audit_stationary_pair=lambda p:self.known.get(plan_digest({k:v for k,v in p.items() if k!='outputProofObservedAt'})) is not None
        self.source.refresh_record=Mock(side_effect=refresh)
        self.source.zero_output_bridge=lambda a,c:dict(controllerBootId='a'*32,beforeSequence=3,afterSequence=3,
            dropped=0,allFiveOff=True,state='IDLE',queueCount=0,entries=[],observedAt=self.clock.monotonic())
        self.source.audit_zero_bridge=lambda proof:not self.proof_bad

    def test_published_nonce_request_and_late_positive_seed_refresh_once(self):
        self.stationary_source();original=self.requests.write_json
        def publish(path,request):
            result=original(path,request);self.clock.now+=4.
            reply=dict(schemaVersion=SCHEMA,nonce=request['nonce'],sourceSha256=request['sourceSha256'],
                stage='close',frames=request['record']['frames'],accepted=True,reviewedAt=self.clock.now,
                humanAssisted=True,method='explicit_dual_source_visual_review',stable=True,reached=True,
                labels=dict(operatorPresent=True,clearance=True,bilateralContact=True))
            self.store.write_json(request['nonce']+'-response.json',reply);return result
        self.requests.write_json=publish
        current,entry=self.phase.resolve(self.stage,self.record,deadline=20.,tick=lambda:None)
        self.assertEqual(self.refreshes,1);self.assertNotEqual(current['frames'],self.record['frames'])
        self.assertEqual(self.record['frames'][0]['sourceEarliest'],9.)
        self.assertTrue(entry['humanAssisted']);self.assertTrue(self.phase.goal(self.stage,current)['reached'])
        self.assertTrue(list(self.requests.root.glob('*-request.json')))

    def test_negative_or_expired_review_never_refreshes(self):
        self.stationary_source()
        with self.assertRaisesRegex(ValueError,'reply_rejected'):self.review(accepted=False)
        self.source.refresh_record.assert_not_called()
        self.phase.anchors.clear();self.requests.write_json=lambda path,value:None
        with self.assertRaisesRegex(ValueError,'deadline_no_retry'):
            self.phase.resolve(self.stage,self.record,deadline=10.2,tick=lambda:None)
        self.source.refresh_record.assert_not_called()

    def test_ARM_without_idle_anchor_does_not_request_or_capture(self):
        self.stationary_source();record=dict(self.record,sourceRoute='M4_armed')
        with self.assertRaisesRegex(ValueError,'ARM_anchor_missing'):
            self.phase.resolve(self.stage,record,deadline=20.,tick=lambda:None)
        self.source.refresh_record.assert_not_called();self.assertFalse(list(self.requests.root.glob('*-request.json')))

    def test_asynchronous_cameras_do_not_overstate_retention(self):
        stage=dict(kind='lift');first=copy.deepcopy(self.record);last=copy.deepcopy(first)
        for frame,t in zip(first['frames'],[9.2,11.]):frame.update(sourceEarliest=t,sourceLatest=t+.05,persistedAt=t+.06)
        for frame,t in zip(last['frames'],[12.2,12.3]):frame.update(frameId=frame['frameId']+'2',sourceEarliest=t,sourceLatest=t+.05,persistedAt=t+.06)
        self.clock.now=12.5;self.phase.scope['retentionSeconds']=3.
        refs=[self.store.write_json(name,r) for name,r in [('early.json',first),('late.json',last)]]
        refs=[dict(path=r['artifactRef'],sha256=r['sha256']) for r in refs]
        self.clock.now=11.5
        self.review(first,stage,labels=dict(airborne=True,retained=True),reached=False)
        self.clock.now=12.5
        self.review(last,stage,labels=dict(airborne=True,retained=True),retention=refs)
        with self.assertRaisesRegex(ValueError,'span_or_current_pair'):self.phase.goal(stage,last)


if __name__=='__main__':unittest.main()
