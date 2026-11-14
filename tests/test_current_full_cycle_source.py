"""실제 저장 원본 파서를 검증한다. 장치·네트워크·촬영은 실행하지 않는다.

M4 metrology 값만 주입하며 JPEG/native/clock parser는 기존 구현을 실행한다.
"""
import copy
import json
import threading
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from learning import current_full_cycle_source as source
from learning.artifacts import ArtifactStore
from learning.action_recorder import ActionRecorder
from learning.grasp_observation import GripperObservationSource
from learning.supervised_collection import plan_digest
from learning.gripper_camera_dataset import read_frame
from learning.current_m1_contract import CurrentM1CollectionGuard
from tests.test_m4_up_commissioning import Observer,scope,Device
from tests.test_supervised_transport import Clock


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve();self.clock=Clock();self.scope=scope()
        self.native=Observer(self.root,self.clock,self.scope);self.native.start()
        self.raw=self.native.observe(after_off=None,deadline=20.,tick=lambda:None)
        self.clock.now+=.1
        self.recorder=object.__new__(ActionRecorder);self.recorder.clock=self.clock
        self.recorder.host_clock_id=self.scope['hostClockId'];self.recorder.check=Mock();self.recorder.close=Mock()
        self.gripper=object.__new__(GripperObservationSource);self.gripper.clock=self.clock
        self.gripper.session_id=self.scope['hostClockId'];self.gripper.check=Mock();self.gripper.close=Mock()
        self.phase=SimpleNamespace(bind_source=lambda owner:None,
            resolve=lambda stage,record,**kw:(record,dict(tableSupported=True,clearance=True,bilateralContact=True)))
        self.factory=Mock(side_effect=AssertionError('factory must not run in preflight'))
        self.port=source.FullCycleSource(self.root,bank=None,idle_factory=self.factory,
            general_factory=self.factory,phase=self.phase,clock=self.clock)
        self.stage=dict(kind='lift',context=copy.deepcopy(self.scope['context']))
        self.port.scope=dict(self.scope,hostClockId='global-cycle')
        from hashlib import sha256
        state_registration=dict(cameraBinding=dict(external=dict(configurationSha256=sha256(b'{"format":"fixed"}').hexdigest(),
            width=8,height=8,videoMirrored=False,videoRotationAngle=0),gripper=dict(bootId='b'*32,width=32,height=32)))
        regraw=json.dumps(state_registration).encode();(self.root/'model-state.json').write_bytes(regraw)
        self.port.registrations['lift']=(dict(sourceContext=self.scope['context'],
            modelStateRegistration=dict(path='model-state.json',sha256=sha256(regraw).hexdigest())),{})
        self.decode=patch('learning.m4_up_commissioning.decode',side_effect=self.native.decode)
        self.decode.start();self.addCleanup(self.decode.stop)
        self.decode2=patch('learning.lossless_frame.decode',side_effect=self.native.decode)
        self.decode2.start();self.addCleanup(self.decode2.stop)
        self.measure=patch.object(source,'measure',return_value=dict(state=dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.)))
        self.measure.start();self.addCleanup(self.measure.stop)

    def record(self):
        self.owner=SimpleNamespace(observer=SimpleNamespace(recorder=self.recorder,gripper=self.gripper),
            root=self.root,scope=self.scope,before=self.raw,port=object())
        self.port._status=Mock(return_value=dict(sequence=3,angle=195.29,state='ARMED'))
        return self.port.observe_m4_armed(self.stage,self.owner,after_off=None,deadline=20.)

    def test_preflight_reports_each_missing_stage_without_factories_or_IO(self):
        stages=[dict(kind=k,context={},modelSha256='a'*64) for k in source.STAGES]
        with self.assertRaises(source.SourcePreflightError) as result:
            self.port.preflight(dict(stages=stages),{})
        self.assertEqual(set(result.exception.missing),set(source.STAGES))
        self.assertTrue(all('frozen_model_missing' in errors for errors in result.exception.missing.values()))
        self.factory.assert_not_called()

    def test_actual_actor_raw_route_preserves_producer_clock_and_reaudits(self):
        record=self.record()
        self.assertEqual(record['producerHostClockId'],self.scope['hostClockId'])
        self.assertEqual(record['hostClockId'],'global-cycle')
        self.assertTrue(self.port.verify_clock_bridge(self.stage,record,'M4_armed',self.owner))
        self.assertTrue(self.port.verify_observation(self.stage,record))

    def test_wrong_clock_object_rejected_even_with_same_clock_id(self):
        self.gripper.clock=Clock()
        with self.assertRaisesRegex(ValueError,'same_python_clock'):self.record()

    def test_changed_jpeg_or_source_metadata_rejected(self):
        record=self.record();path=self.root/'gripper/esp32-before.jpg';path.write_bytes(path.read_bytes()+b'x')
        with self.assertRaisesRegex(ValueError,'source_changed'):self.port.verify_observation(self.stage,record)

    def test_relative_escape_and_symlink_rejected_by_original_reader(self):
        original=self.raw['external']['root'];self.raw['external']['root']='../outside'
        with self.assertRaises(RuntimeError):self.record()
        self.raw['external']['root']=original
        self.port.failed=False  # independent new rejection case, not a live retry
        (self.root/'alias').symlink_to(self.root/'external',target_is_directory=True)
        self.raw['external']['root']='alias'
        with self.assertRaises(RuntimeError):self.record()

    def test_stale_before_and_pre_OFF_pair_rejected(self):
        self.clock.now+=4
        with self.assertRaisesRegex(ValueError,'pair_stale'):self.record()
        self.clock.now-=4
        self.port.failed=False  # independent new rejection case
        self.owner=SimpleNamespace(observer=SimpleNamespace(recorder=self.recorder,gripper=self.gripper),
            root=self.root,scope=self.scope,before=self.raw,port=object())
        self.port._status=Mock(return_value=dict(sequence=3,angle=195.29,state='ARMED'))
        with self.assertRaisesRegex(ValueError,'after_OFF'):
            self.port.observe_m4_armed(self.stage,self.owner,after_off=9.,deadline=20.)

    def test_record_owner_and_route_cannot_be_substituted(self):
        record=self.record()
        with self.assertRaisesRegex(ValueError,'clock_owner'):
            self.port.verify_clock_bridge(self.stage,record,'M4_armed',object())
        changed=copy.deepcopy(record);changed['m4AngleDeg']=196.
        with self.assertRaisesRegex(ValueError,'unowned_record'):self.port.verify_observation(self.stage,changed)

    def test_actor_close_is_owned_by_actor_and_idle_close_is_owned_by_source(self):
        self.port.close_m4(object());self.recorder.close.assert_not_called();self.gripper.close.assert_not_called()
        self.port.idle=source.OwnedDualSource(self.root,self.recorder,self.gripper,object())
        self.port.close_idle();self.recorder.close.assert_called_once();self.gripper.close.assert_called_once()
        self.port.close();self.port.close()

    def test_no_memory_gate_without_actual_ARM_observation(self):
        with self.assertRaisesRegex(ValueError,'ARM_gate_missing'):
            self.port.validate_general_memory(self.stage,object(),{})

    def prepare_pair(self):
        """생산자 시작/캡처만 모형이며 actor 테스트가 별도로 raw parser를 실행한다."""
        folder=self.root/'gripper/capture-0001';folder.mkdir()
        for suffix in ('json','jpg'):
            (self.root/f'gripper/esp32-before.{suffix}').rename(folder/f'esp32-frame.{suffix}')
        frame,meta,_=read_frame(folder,'frame',self.scope['hostClockId'])
        raw=dict(frame={k:frame[k] for k in ('frameId','pixelSha256','width','height')},
            context={k:self.scope['context'][k] for k in ('physicalView','object','poseRegime')},
            hostClockId=self.scope['hostClockId'],sourceEarliest=meta['requestStartedAt'],
            sourceLatest=meta['receivedAt'],receivedAt=meta['receivedAt'],persistedAt=meta['persistedAt'])
        self.gripper.store=SimpleNamespace(root=self.root/'gripper')
        self.gripper._bindings={frame['frameId']:dict(folder='capture-0001',record=raw,
            metadataSha256=self.raw['gripper']['metadataSha256'],imageSha256=self.raw['gripper']['imageSha256'])}
        self.gripper.boot='b'*32;self.gripper.start=Mock();self.gripper.capture=Mock(return_value=raw)
        self.recorder.root=self.root/'external';self.recorder.lock=threading.Lock()
        self.recorder.frames=[json.loads((self.recorder.root/'host-ingress/0000.json').read_text())]
        self.recorder.start=Mock();self.recorder.wait_ready=Mock()
        epoch=dict(sequence=3,angle=195.29,state='IDLE',bootId='a'*32,allFiveOff=True)
        self.port._status=Mock(return_value=epoch)
        frames=[dict(camera=c,frameId=c+'1',pixelSha256='a'*64,sourceEarliest=10.1,
                     sourceLatest=10.12,persistedAt=10.14) for c in ('external','gripper')]
        self.audit_patch=patch.object(self.port,'_audit',return_value=(frames,dict(shaftSeparationPx=80.),{}))
        self.audit_patch.start();self.addCleanup(self.audit_patch.stop)
        return raw

    def test_idle_starts_owns_and_releases_actual_pair_producers(self):
        self.prepare_pair()
        owner=source.OwnedDualSource(self.root,self.recorder,self.gripper,object())
        self.port.idle_factory=Mock(return_value=owner)
        record=self.port.observe_idle(self.stage,after_off=None,deadline=20.)
        self.recorder.start.assert_called_once();self.gripper.start.assert_called_once()
        self.assertTrue(self.port.verify_clock_bridge(self.stage,record,'idle',None))
        self.port.close_idle();self.recorder.close.assert_called_once();self.gripper.close.assert_called_once()

    def test_general_ARM_memory_gate_after_disarm_and_close_ownership(self):
        self.prepare_pair();stage=dict(kind='close',context=self.stage['context'])
        config=dict(hostClockId=self.scope['hostClockId'],value='registered-guard')
        raw=json.dumps(config).encode();(self.root/'guard.json').write_bytes(raw)
        from hashlib import sha256
        spec=dict(sourceContext=self.scope['context'],guardConfig=dict(path='guard.json',sha256=sha256(raw).hexdigest()))
        self.port.registrations['close']=(spec,{})
        port=SimpleNamespace(last_sequence=3,last_outputs={i:0 for i in range(1,6)})
        manager=SimpleNamespace(camera=self.recorder,port=port,store=SimpleNamespace(root=self.root),tick=Mock())
        guard=object.__new__(CurrentM1CollectionGuard)
        guard.config=config;guard.recorder=self.recorder;guard.port=port
        for name in ('review_armed','review','check','validate_command','disarm'):setattr(guard,name,Mock())
        self.port.general_factory=Mock(return_value=(self.gripper,guard))
        record=self.port.observe_general_armed(stage,manager,after_off=None,deadline=20.)
        guard.review_armed.assert_called_once()
        self.port.validate_general_memory(stage,manager,dict(direction='close'))
        guard.validate_command.assert_called_once()
        with self.assertRaisesRegex(ValueError,'ARM_gate_missing'):
            self.port.validate_general_memory(stage,manager,{})
        # A new after pair would be required by the runtime frame-ID gate; here
        # isolate source ownership and guard disarm order without a RUN.
        self.port.observe_general_after(stage,manager,dict(offAt=6.),deadline=20.)
        guard.disarm.assert_called_once();guard.review.assert_called_once()
        self.port.close_general(manager);self.gripper.close.assert_called_once()
        self.recorder.close.assert_not_called()

    def test_sensor_jitter_is_separate_from_output_epoch(self):
        before=dict(sequence=3,angle=196.69,state='IDLE',bootId='a'*32,allFiveOff=True)
        source.FullCycleSource._same_epoch(before,dict(before,angle=196.78))
        with self.assertRaisesRegex(ValueError,'stationary_angle'):
            source.FullCycleSource._same_epoch(before,dict(before,angle=197.))
        with self.assertRaisesRegex(ValueError,'epoch_changed'):
            source.FullCycleSource._same_epoch(before,dict(before,sequence=4))

    def test_zero_bridge_uses_real_SequencePort_GET_and_immutable_HTTP_files(self):
        from learning.supervised_transport import SequencePort
        device=Device(self.clock);device.angle=195.29
        http=ArtifactStore(self.root/'controller')
        port=SequencePort('http://controller.invalid','private-token',device.boot,http,clock=self.clock,compact_reads=True)
        port.opener=device;port.preflight()
        owner=source.OwnedDualSource(self.root,self.recorder,self.gripper,port)
        epoch=dict(sequence=port.last_sequence,angle=device.angle,state='IDLE',bootId=device.boot,allFiveOff=True)
        record=self.port._record(self.stage,self.root,self.raw['external'],self.raw['gripper'],
            self.scope['hostClockId'],epoch,'idle',owner,None,20.)
        proof=self.port.zero_output_bridge(record,record)
        self.assertTrue(self.port.audit_zero_bridge(proof));self.assertTrue(proof['httpBindings'])
        self.assertFalse(any(payload is not None for _,payload in device.requests))
        binding=proof['httpBindings'][0];(http.root/binding['path']).write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError,'HTTP_changed'):self.port.audit_zero_bridge(proof)

    def test_M4_state_registration_is_separate_from_pixel_metrology(self):
        from tests.test_task_cycle_workflow import CycleWorkflowTests,m4_rows
        from tests.test_task_visual_measurement import images,encode,parameters
        from learning import task_visual_measurement as visual,task_action_models as actions
        from learning.task_cycle_workflow import FrozenCycleModelBank
        from learning.pipeline_learning import canonical_bytes
        self.measure.stop()
        fixture=CycleWorkflowTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        bank=fixture.freeze();external,gripper=images();e,g=encode(external),encode(gripper)
        registration=visual.register(e,g,parameters())
        from hashlib import sha256
        def bind(name,value):
            raw=value if isinstance(value,bytes) else canonical_bytes(value)
            (self.root/name).write_bytes(raw);return dict(path=name,sha256=sha256(raw).hexdigest())
        ctx=dict(object='block',physicalView='current',poseRegime='loaded')
        actual=dict(ctx,externalDeviceId='camera-id',externalViewId='test-view',gripperCameraBootId='b'*32,supportMode='table_supported')
        state_reg=dict(schemaVersion='motionbrain.task_visual_state_registration.v1',context=ctx,
            cameraBinding=dict(external=dict(configurationSha256='c'*64,width=400,height=300,videoMirrored=False,videoRotationAngle=0),
                               gripper=dict(bootId='b'*32,width=320,height=240)),acceptedSourceContexts=[actual],
            measurementMethod=registration['schemaVersion']+':'+plan_digest(registration))
        model_context=dict(ctx,registrationSha256=plan_digest(state_reg))
        rows=m4_rows()
        for row in rows:row['context']=model_context
        model=actions.fit_m4(rows,model_context)
        model_ref=bind('new-lift-model.json',model)
        bindings=copy.deepcopy(bank.bindings);bindings['lift']=dict(path=str(self.root/model_ref['path']),sha256=model_ref['sha256'])
        self.port.bank=FrozenCycleModelBank(bindings)
        metrology=bind('metrology.json',registration);e_ref=bind('reference.png',e);g_ref=bind('reference.jpg',g)
        spec=dict(context=model_context,sourceContext=actual,registration=metrology,
            modelStateRegistration=bind('state-registration.json',state_reg),externalReference=e_ref,gripperReference=g_ref,
            stationaryRegistration=metrology,stationaryExternalReference=e_ref,stationaryGripperReference=g_ref)
        stages=[dict(kind=k,context=self.port.bank.models[k]['context'],modelSha256=self.port.bank.hashes[k]) for k in source.STAGES]
        planned=dict(stages=stages,controllerBootId='a'*32)
        with self.assertRaises(source.SourcePreflightError) as result:self.port.preflight(planned,dict(sourceStages=dict(lift=spec)))
        self.assertNotIn('lift',result.exception.missing)
        bad=copy.deepcopy(spec);bad['modelStateRegistration']=metrology
        with self.assertRaises(source.SourcePreflightError) as result:self.port.preflight(planned,dict(sourceStages=dict(lift=bad)))
        self.assertIn('visual_model_state_measurement_contract_changed',result.exception.missing['lift'])


if __name__=='__main__':unittest.main()
