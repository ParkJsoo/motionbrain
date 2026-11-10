import copy
import json
from pathlib import Path
import tempfile
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning import batch_interaction as interaction
from learning import m4_up_commissioning as m4
from tests.test_m4_up_commissioning import Observer, scope
from tests.test_supervised_transport import Clock


class BatchInteractionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.repository = Path(self.temp.name)
        self.private = self.repository/'.local'; self.private.mkdir()
        self.root = self.private/'evidence'; self.root.mkdir()
        self.clock = Clock()
        self.scope = scope()
        self.observer = Observer(self.root, self.clock, self.scope)
        self.observer.start()
        record = self.observer.observe(after_off=None, deadline=10, tick=lambda:None)
        record['onsite']['jawContactVerified'] = False
        raw = {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
        record['evidenceSha256'] = self.observer.save(record['evidenceFile'],raw)
        self.context = dict(viewId='test-view', poseRegime='contact-local', object='cylinder', load='unknown')
        self.spec = dict(self.context)
        self.spec_raw = json.dumps(self.spec).encode()
        (self.private/'spec.json').write_bytes(self.spec_raw)
        self.plan = dict(controllerBootId='a'*32, viewId='test-view', runtime=dict(
            deviceId='camera-id', poseContext='contact-local', observerSpecPath='.local/spec.json',
            observerSpecSha256=interaction.sha(self.spec_raw), gripper=dict(bootId='b'*32)))
        self.condition = dict(id='close', maxAttempts=1,
                              command=dict(joint='gripper',direction='close',percent=40,durationMs=350))
        self.plan['conditions']=[self.condition]
        self.envelope = dict(schemaVersion=interaction.SCHEMA, conditionId='close',stage='before',
            interaction='contact_probe',outcome='unknown',controllerBootId='a'*32,
            context=copy.deepcopy(self.context), outputSequence=100, scope=self.scope,
            afterOffAt=None, review=dict(observations=[dict(root='.',record=record)]),
            registration=None,retentionContext=None)
        self.addCleanup(patch.stopall)
        patch.object(m4,'decode',side_effect=lambda raw,length,sha:self.observer.payloads[raw]).start()

    def checked(self, *, purpose='dispatch'):
        raw=json.dumps(self.envelope).encode();path=self.root/'batch-review.json';path.write_bytes(raw)
        binding=dict(route='generic',interaction=self.envelope['interaction'],reviewPath='.local/evidence/batch-review.json',reviewSha256=interaction.sha(raw))
        return interaction.validate_binding(self.plan,self.condition,binding,self.repository,purpose=purpose),binding

    def verify(self):
        checked,_=self.checked()
        return interaction.verify_review(checked,now=self.clock.now,output_sequence=100)

    def test_unknown_contact_validates_original_dual_frames_without_success_claim(self):
        proof=self.verify()
        self.assertEqual(proof.report['contactState'],'unknown')
        self.assertFalse(proof.report['tableContactReady'])
        self.assertFalse(proof.report['graspVerified'])
        self.assertFalse(proof.report['liftVerified'])
        self.assertGreaterEqual(len(proof.snapshots),8)
        result=interaction.before_run_cached(proof,self.clock.now,self.clock.now,100)
        self.assertFalse(result['physicalJudgmentGenerated'])

    def test_original_image_tamper_after_review_blocks_run_guard(self):
        proof=self.verify()
        name=next(p for p in proof.snapshots if p.endswith('.jpg'))
        Path(name).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'interaction_original_evidence_changed'):
            interaction.before_run_cached(proof,self.clock.now,self.clock.now,100)

    def test_new_object_view_or_load_cannot_reuse_old_review(self):
        for key,value in [('object','new-object'),('viewId','new-view'),('load','held')]:
            original=copy.deepcopy(self.envelope)
            self.envelope['context'][key]=value
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError,'interaction_scene_context_changed'):
                    self.checked()
            self.envelope=original

    def test_new_gripper_boot_or_controller_boot_rejected(self):
        self.plan['runtime']['gripper']['bootId']='c'*32
        with self.assertRaisesRegex(ValueError,'interaction_gripper_boot_changed'):
            self.checked()
        self.plan['runtime']['gripper']['bootId']='b'*32
        self.envelope['controllerBootId']='d'*32
        with self.assertRaisesRegex(ValueError,'interaction_plan_binding_changed'):
            self.checked()

    def test_output_sequence_change_and_expiration_block_dispatch(self):
        proof=self.verify()
        with self.assertRaisesRegex(ValueError,'interaction_intervening_output'):
            interaction.before_run_cached(proof,self.clock.now,self.clock.now,101)
        with self.assertRaisesRegex(ValueError,'interaction_cached_review_expired'):
            interaction.before_run_cached(proof,self.clock.now+11,self.clock.now,100)

    def test_cached_affirmative_flag_cannot_be_added(self):
        proof=self.verify();proof.report['graspVerified']=True
        with self.assertRaisesRegex(ValueError,'interaction_cached_evidence_changed'):
            interaction.before_run_cached(proof,self.clock.now,self.clock.now,100)

    def test_unknown_probe_cannot_supply_held_or_contact_ready_outcome(self):
        for outcome in ('held','contact_ready'):
            self.envelope['outcome']=outcome
            with self.subTest(outcome=outcome):
                with self.assertRaises(ValueError):self.checked()

    def test_original_table_support_and_operator_checks_are_preserved(self):
        record=self.envelope['review']['observations'][0]['record']
        record['onsite']['tableSupportVerified']=False
        raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
        record['evidenceSha256']=self.observer.save(record['evidenceFile'],raw)
        with self.assertRaisesRegex(ValueError,'m4_probe_requires_table_support_and_unconfirmed_contact'):
            self.verify()

    def test_unknown_after_requires_actual_off_binding(self):
        self.envelope.update(stage='after',afterOffAt=0.)
        checked,_=self.checked(purpose='label')
        with self.assertRaisesRegex(ValueError,'interaction_actual_after_off_required'):
            interaction.verify_review(checked,now=self.clock.now,output_sequence=100)

    def test_before_command_requires_current_off_and_unchanged_sequence(self):
        checked,binding=self.checked()
        manager=SimpleNamespace(port=SimpleNamespace(off_verified=False,last_sequence=100))
        self.clock.time=lambda:self.clock.now
        deadline=self.clock.now+10
        with self.assertRaisesRegex(ValueError,'interaction_current_off_required'):
            interaction.before_command(manager,None,self.condition,binding,deadline,checked=checked,clock=self.clock)
        manager.port.off_verified=True
        token=interaction.before_command(manager,None,self.condition,binding,deadline,checked=checked,clock=self.clock)
        self.assertEqual(token.deadlineWall,deadline)

    def test_contact_probe_original_command_cannot_be_widened(self):
        self.condition['command']['percent']=60
        with self.assertRaisesRegex(ValueError,'interaction_original_contact_probe_command_required'):
            self.checked()

    def test_contact_probe_budget_cannot_be_split_across_condition_ids(self):
        other=copy.deepcopy(self.condition);other.update(id='other',maxAttempts=6)
        self.plan['conditions'].append(other)
        self.plan['runtime']['conditionBindings']={'other':dict(route='generic',interaction='contact_probe')}
        with self.assertRaisesRegex(ValueError,'interaction_contact_probe_six_command_budget'):
            self.checked()

    def test_grasped_state_cannot_request_additional_close(self):
        self.envelope['interaction']='grasped'
        with self.assertRaisesRegex(ValueError,'interaction_no_repeated_close_after_contact'):
            self.checked()

    def test_object_present_noncontact_requires_original_clearance_review(self):
        self.spec['load']='empty';self.envelope['context']['load']='empty'
        self.envelope['interaction']='noncontact'
        raw=json.dumps(self.spec).encode();(self.private/'spec.json').write_bytes(raw)
        self.plan['runtime']['observerSpecSha256']=interaction.sha(raw)
        checked,binding=self.checked()
        token=interaction.verify_review(checked,now=self.clock.now,output_sequence=100)
        self.assertFalse(token.report['graspVerified'])
        self.assertEqual(token.report['interaction'],'noncontact')
        binding.pop('reviewPath');binding.pop('reviewSha256')
        with self.assertRaisesRegex(ValueError,'interaction_private_path_required'):
            interaction.validate_binding(self.plan,self.condition,binding,self.repository)

    def test_held_load_cannot_bypass_review_with_noncontact_name(self):
        self.spec['load']='held'
        raw=json.dumps(self.spec).encode();(self.private/'spec.json').write_bytes(raw)
        self.plan['runtime']['observerSpecSha256']=interaction.sha(raw)
        with self.assertRaisesRegex(ValueError,'interaction_noncontact_empty_load_required'):
            interaction.validate_binding(self.plan,self.condition,dict(interaction='noncontact'),self.repository)

    def after_packet(self):
        off=self.clock.now-4.
        record=self.observer.observe(after_off=off,deadline=self.clock.now+10,tick=lambda:None)
        self.envelope.update(stage='after',afterOffAt=off,
            review=dict(observations=[dict(root='.',record=record)]))
        checked,_=self.checked(purpose='label')
        token=interaction.after_command(checked,now=self.clock.now,output_sequence=100,after_off=off)
        request=dict(schemaVersion='motionbrain.batch_after_review_request.v1',conditionId='close',
            source=dict(sessionId=self.scope['hostClockId'],path='.local/evidence',sha256='a'*64),
            context=self.context,offAt=off,outputSequence=100,frames=copy.deepcopy(token.report['frameBindings']))
        response=dict(requestSha256=interaction.sha(interaction.encoded(request)),review=self.envelope)
        return request,response,token

    def test_dynamic_after_response_binds_request_source_and_original_frames(self):
        request,response,token=self.after_packet()
        review=interaction.validate_after_response(request,response)
        self.assertEqual(review,self.envelope)
        self.assertEqual(interaction.verify_after_frame_membership(token,request),token.report)
        self.assertFalse(token.report['liftVerified'])

    def test_wrong_request_hash_session_off_or_frame_rejected(self):
        request,response,token=self.after_packet()
        broken=copy.deepcopy(response);broken['requestSha256']='0'*64
        with self.assertRaisesRegex(ValueError,'after_review_response_request_changed'):
            interaction.validate_after_response(request,broken)
        for changed in ('hostClockId','offAt','frame'):
            broken=copy.deepcopy(response)
            if changed=='hostClockId':broken['review']['scope']['hostClockId']='other'
            elif changed=='offAt':broken['review']['afterOffAt']+=1
            else:broken['review']['review']['observations'][0]['record']['external']['ingressFile']='host-ingress/9999.json'
            with self.subTest(changed=changed):
                with self.assertRaises(ValueError):interaction.validate_after_response(request,broken)
        changed=copy.deepcopy(request);changed['frames']['gripper'][0]['pixelSha256']='0'*64
        with self.assertRaisesRegex(ValueError,'after_review_verified_frame_not_requested:gripper'):
            interaction.verify_after_frame_membership(token,changed)


class BatchConfirmedContactTests(unittest.TestCase):
    def setUp(self):
        from tests.test_close_lift_trial import ConnectionTests
        self.fixture=ConnectionTests();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        for pair in self.fixture.pairs:
            pair['gripper']['pixelSha256']='c'*64
        self.repository=self.fixture.root
        self.private=self.repository/'.local';self.private.mkdir()
        self.root=self.private/'evidence'
        self.root.mkdir()
        for name in ('0','1','contact.json'):
            source=self.repository/name
            if source.is_dir():shutil.copytree(source,self.root/name)
            else:shutil.copy2(source,self.root/name)
        self.fixture.root=self.root
        self.context=dict(viewId='test-view',poseRegime='contact-local',object='same-object',load='table_supported')
        legacy=dict(physicalView='fixed',poseRegime='contact-local',object='same-object',supportMode='table_supported',
                    externalDeviceId='camera-id',externalViewId='test-view',gripperCameraBootId='b'*32)
        self.scope=dict(controllerBootId='a'*32,hostClockId='contact-host',context=legacy,
                        maxObservationAgeSeconds=10.,postOffSeconds=3.)
        for observation in self.fixture.proof['observations']:
            record=observation['record'];record['context']=copy.deepcopy(legacy)
            location=self.root/observation['root']
            raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            self.fixture.save(location,record['evidenceFile'],raw)
            record['evidenceSha256']=interaction.sha((location/record['evidenceFile']).read_bytes())
        self.fixture.seal_proof()
        self.spec_raw=json.dumps(self.context).encode();(self.private/'spec.json').write_bytes(self.spec_raw)
        self.registration_raw=json.dumps(self.fixture.lift.registration).encode()
        (self.private/'registration.json').write_bytes(self.registration_raw)
        self.plan=dict(controllerBootId='a'*32,viewId='test-view',runtime=dict(deviceId='camera-id',poseContext='contact-local',
            observerSpecPath='.local/spec.json',observerSpecSha256=interaction.sha(self.spec_raw),gripper=dict(bootId='b'*32)))
        self.condition=dict(id='lift')
        self.envelope=dict(schemaVersion=interaction.SCHEMA,conditionId='lift',stage='before',interaction='grasped',
            outcome='contact_ready',controllerBootId='a'*32,context=self.context,outputSequence=100,scope=self.scope,
            afterOffAt=10.,review=self.fixture.proof,retentionContext=None,
            registration=dict(path='.local/registration.json',sha256=interaction.sha(self.registration_raw)))

    def checked(self, *, purpose='dispatch', route='m4_up'):
        raw=json.dumps(self.envelope).encode();(self.root/'batch-review.json').write_bytes(raw)
        binding=dict(route=route,interaction='grasped',reviewPath='.local/evidence/batch-review.json',reviewSha256=interaction.sha(raw))
        return interaction.validate_binding(self.plan,self.condition,binding,self.repository,purpose=purpose)

    def test_legacy_contact_metrology_contract_preserved_without_lift_claim(self):
        token=interaction.verify_review(self.checked(),now=20.,output_sequence=100)
        self.assertTrue(token.report['tableContactReady'])
        self.assertFalse(token.report['graspVerified'])
        self.assertFalse(token.report['retainedLift'])
        self.assertEqual(interaction.before_run_cached(token,20.,20.,100),token.report)

    def test_missing_depth_and_slip_are_rejected_by_original_contact_checker(self):
        self.fixture.proof['depthVerified']=False;self.fixture.seal_proof()
        with self.assertRaisesRegex(ValueError,'connection_contact_not_ready'):
            interaction.verify_review(self.checked(),now=20.,output_sequence=100)
        self.fixture.proof['depthVerified']=True;self.fixture.seal_proof()
        self.fixture.slips[1]=[5.,0.]
        with self.assertRaisesRegex(ValueError,'connection_contact_slip_exceeded'):
            interaction.verify_review(self.checked(),now=20.,output_sequence=100)

    def held_envelope(self):
        observations=copy.deepcopy(self.fixture.proof['observations'])
        context=dict(physicalView='fixed',poseRegime='contact-local',object='same-object',
            landmarks='manual-points',cameraIdentity=dict(camera='external'),imageShape=[1080,1920])
        reviewed=[]
        for index,observation in enumerate(observations):
            self.fixture.pairs[index]['gripper']['pixelSha256']='c'*64
            record=observation['record'];record['gripper']['label']='after'
            record['onsite']['tableSupportVerified']=False
            location=self.root/observation['root']
            for suffix in ('.jpg','.json'):
                shutil.copy2(location/'gripper'/('esp32-before'+suffix),location/'gripper'/('esp32-after'+suffix))
            raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            self.fixture.save(location,record['evidenceFile'],raw)
            record['evidenceSha256']=interaction.sha((location/record['evidenceFile']).read_bytes())
            frame=self.fixture.pairs[index]['external']
            f={k:frame[k] for k in ('frameId','pixelSha256','width','height')}
            annotation=dict(frameId=f['frameId'],pixelSha256=f['pixelSha256'],accepted=True,
                inputFrameOnly=True,method='analyst_points',points=dict(object=[60,80],jawLeft=[30,60],jawRight=[80,60]))
            reviewed.append(dict(frame=f,annotation=annotation,context=context,hostClockId='contact-host',
                sourceEarliest=frame['sourceEarliest'],sourceLatest=frame['sourceLatest'],
                receivedAt=frame['persistedAt']-.01,persistedAt=frame['persistedAt'],stable=True))
        retention=dict(status='held',method='analyst_retention_review',contactVerified=True,
            retentionVerified=True,supportDistinguished=True,observations=reviewed)
        raw=json.dumps(retention).encode();(self.root/'retention.json').write_bytes(raw)
        retention.update(evidenceFile='retention.json',evidenceSha256=interaction.sha(raw))
        self.envelope.update(stage='after',outcome='held',retentionContext=context,
            review=dict(dualObservations=observations,retention=retention))
        self.envelope['scope']['retentionSeconds']=1.
        return retention

    def test_retention_requires_original_held_review_plus_two_camera_span(self):
        self.held_envelope()
        token=interaction.verify_review(self.checked(purpose='label'),now=16.,output_sequence=100,after_off=10.)
        self.assertTrue(token.report['retainedLift'])
        self.assertTrue(token.report['graspVerified'])
        self.assertTrue(token.report['liftVerified'])

    def test_lift_receipt_or_held_flag_without_support_evidence_is_not_success(self):
        retention=self.held_envelope()
        retention['supportDistinguished']=False
        raw={k:v for k,v in retention.items() if k not in ('evidenceFile','evidenceSha256')}
        body=json.dumps(raw).encode();(self.root/'retention.json').write_bytes(body)
        retention['evidenceSha256']=interaction.sha(body)
        with self.assertRaisesRegex(ValueError,'independent_retention_evidence_required'):
            interaction.verify_review(self.checked(purpose='label'),now=16.,output_sequence=100,after_off=10.)

    def test_close_probe_can_acquire_confirmed_contact_after_label_without_authorizing_another_close(self):
        self.condition['command']=dict(joint='gripper',direction='close',percent=40,durationMs=350)
        self.envelope['stage']='after'
        checked=self.checked(purpose='label',route='generic')
        token=interaction.after_command(checked,now=20.,output_sequence=100,after_off=10.)
        self.assertTrue(token.report['tableContactReady'])
        self.assertFalse(token.report['graspVerified'])
        self.assertFalse(token.report['liftVerified'])
        with self.assertRaisesRegex(ValueError,'interaction_no_repeated_close_after_contact'):
            self.checked(route='generic')
        manager=SimpleNamespace(port=SimpleNamespace(off_verified=True,last_sequence=100))
        clock=SimpleNamespace(time=lambda:20.,monotonic=lambda:20.)
        with self.assertRaisesRegex(ValueError,'interaction_dispatch_review_required'):
            interaction.before_command(manager,None,self.condition,dict(interaction='grasped'),30.,checked=checked,clock=clock)

    def test_after_contact_proof_cannot_make_static_dispatch_ready(self):
        self.envelope['stage']='after'
        with self.assertRaisesRegex(ValueError,'interaction_before_review_required'):
            self.checked()


if __name__=='__main__':unittest.main()
