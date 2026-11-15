"""Synthetic native angle route, cameras and journal; never accesses hardware."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import urllib.parse

import cv2
import numpy as np

from learning import finite_shoulder_angle_actor as actor
from learning.action_journal import BoundedActionJournal
from learning.artifacts import ArtifactStore
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Clock, Controller, Response


class Device(Controller):
    def __init__(self,clock,mode='normal'):
        super().__init__(clock);self.boot='a'*32;self.angle=193.
        self.native_active=False;self.goal=190.;self.native_reason='NONE';self.polls=0;self.mode=mode
    def status(self):
        result=super().status()
        result['shoulderAngle'].update(active=self.native_active,targetDeg=self.goal,
                                      requestedPercent=75,lastStopReason=self.native_reason,
                                      settledSuccessToleranceDeg=.4)
        return result
    def open(self,request,timeout):
        address=urllib.parse.urlsplit(request.full_url)
        payload=None if request.data is None else {k:v[0] for k,v in urllib.parse.parse_qs(request.data.decode()).items()}
        if address.path=='/shoulder':
            self.clock.sleep(.01);self.requests.append(('/shoulder',payload))
            self.native_active=True;self.goal=float(payload['degrees']);self.append(4,-191)
            if self.mode=='unknown':raise TimeoutError('lost native POST response')
            if self.mode=='wrong_motor':self.append(2,30)
            accepted=self.status()
            if self.mode=='wrong_receipt':accepted['shoulderAngle']['targetDeg']+=1.
            return Response(dict(accepted,success=True))
        if payload is None and address.path in ('/collection_status','/status'):
            self.clock.sleep(.01);self.requests.append((address.path,None))
            if self.native_active:
                self.polls+=1
                if self.mode=='reverse':self.angle=196.1
                elif self.mode=='deadline':self.angle=192.
                elif self.polls==1:self.angle=192.
                elif self.polls==2:self.angle=self.goal-.6;self.append(4,191)
                else:
                    self.angle=self.goal+(-1.2 if self.mode=='missed' else 1.2 if self.mode=='coast' else .2)
                    self.append(4,0);self.native_active=False
                    self.native_reason='TARGET_REACHED' if self.mode!='missed' else 'TARGET_MISSED'
            result=self.status()
            if self.mode=='wrong_goal' and self.native_active:result['shoulderAngle']['targetDeg']+=1.
            if address.path=='/collection_status':
                # Actual collection.v1 has no native target/percent/reason.
                for key in ('targetDeg','requestedPercent','lastStopReason'):
                    result['shoulderAngle'].pop(key)
                result['schemaVersion']='collection.v1'
            return Response(result)
        if payload is None and address.path=='/action_outputs':
            self.clock.sleep(.01);self.requests.append(('/action_outputs'+('?' +address.query if address.query else ''),None))
            entries=copy.deepcopy(self.entries[-128:])
            if address.query:
                query=urllib.parse.parse_qs(address.query);cursor=int(query['after'][0])
                selected=[e for e in entries if e['sequence']>cursor][:int(query['limit'][0])]
                return Response(dict(bootId=self.boot,evidence='driver_write',clockDomain='controller_boot_millis',
                    entries=selected,dropped=entries[0]['sequence']-1,oldestSequence=entries[0]['sequence'],
                    latestSequence=entries[-1]['sequence'],gap=bool(cursor and cursor<entries[0]['sequence']-1),
                    hasMore=bool(selected and selected[-1]['sequence']<entries[-1]['sequence'])))
            return Response(dict(bootId=self.boot,evidence='driver_write',clockDomain='controller_boot_millis',
                                 entries=entries,dropped=entries[0]['sequence']-1))
        result=super().open(request,timeout)
        if payload=={'cmd':'stop'}:self.native_active=False
        return result


class FiniteShoulderAngleActorTests(unittest.TestCase):
    def setup_actor(self,*,mode='normal',observation_mode=None):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        root=Path(directory.name);assets=root/'assets';assets.mkdir();clock=Clock()
        model=assets/'model.json';model.write_text('{"frozen":true}')
        model_ref=dict(path='model.json',sha256=hashlib.sha256(model.read_bytes()).hexdigest())
        initial=dict(targetAngleDeg=190.,modelSha256=model_ref['sha256'])
        scope=dict(schemaVersion=actor.SCHEMA,trialId='native-angle-01',controllerUrl='http://controller.invalid',
            controllerBootId='a'*32,cameraBootId='b'*32,model=model_ref,initialDecision=initial,
            initialDecisionSha256=plan_digest(initial),expectedStartAngleDeg=193.,targetAngleDeg=190.,
            percent=75,maxMotionSeconds=12.,maxObservationAgeSeconds=3.)
        records=[]
        def observe(stage,snapshot):
            clock.sleep(.05);sources={}
            for camera in ('external','gripper'):
                frame_id=stage+'-'+camera;earliest=clock.now-.02;latest=clock.now-.01
                pixels=np.full((8,8,4),120,np.uint8)
                raw=cv2.imencode('.png',pixels)[1].tobytes() if camera=='external' else b'synthetic-jpeg-'+stage.encode()
                image=assets/f'{stage}-{camera}.png';image.write_bytes(raw)
                image_ref=dict(path=image.name,sha256=hashlib.sha256(raw).hexdigest())
                if camera=='external':
                    meta=dict(frameId=frame_id,sourceEarliest=earliest,sourceLatest=latest,
                              width=8,height=8,pixelSha256=hashlib.sha256(pixels.tobytes()).hexdigest())
                else:
                    meta=dict(requestStartedAt=earliest,receivedAt=latest,sha256=image_ref['sha256'],headers={
                        'X-MotionBrain-Frame-Id':frame_id,'X-MotionBrain-Camera-Boot-Id':'b'*32,
                        'X-MotionBrain-Capture-Mode':'after_request'})
                if stage=='armed' and observation_mode=='metadata_forged' and camera=='external':
                    meta['frameId']='different-original-frame'
                if stage=='armed' and observation_mode=='pixels_changed' and camera=='external':
                    image.write_bytes(cv2.imencode('.png',pixels+1)[1].tobytes())
                    image_ref['sha256']=hashlib.sha256(image.read_bytes()).hexdigest()
                if stage=='armed' and observation_mode=='gripper_binding' and camera=='gripper':
                    meta['requestStartedAt']-=1.
                metadata=assets/f'{stage}-{camera}-metadata.json';metadata.write_text(json.dumps(meta))
                sources[camera]=dict(image=image_ref,metadata=dict(path=metadata.name,sha256=hashlib.sha256(metadata.read_bytes()).hexdigest()),
                    frameId=frame_id,sourceEarliest=earliest,sourceLatest=latest)
                if camera=='gripper':sources[camera]['cameraBootId']='b'*32
                if stage=='armed' and observation_mode=='same_frame':sources[camera]['frameId']='before-'+camera
                if stage=='armed' and observation_mode=='stale':sources[camera]['sourceEarliest']=clock.now-4.
            target=189. if stage=='armed' and observation_mode=='goal_changed' else 190.
            decision=dict(targetAngleDeg=target,stage=stage)
            record=dict(schemaVersion=actor.OBSERVATION_SCHEMA,stage=stage,modelSha256=model_ref['sha256'],
                targetAngleDeg=target,decision=decision,decisionSha256=plan_digest(decision),sources=sources)
            if stage=='armed' and observation_mode=='armed_drift':device.angle=194.
            if stage=='armed' and observation_mode=='armed_output':device.append(2,20)
            records.append(record);return record
        path=root/'journal.sqlite'
        journal=BoundedActionJournal(str(path),10000);journal.close()
        store=ArtifactStore(root/'run')
        result=actor.FiniteShoulderAngleActor(scope,store,'secret',journal_path=path,asset_root=assets,
                                             observe_and_select=observe,clock=clock)
        device=Device(clock,mode);result.port.opener=device
        return result,device,records,path

    def run_actor(self,instance):return instance.run(confirmed_scope_sha256=instance.digest)

    def test_model_bound_one_native_POST_allows_corrections_and_final_OFF(self):
        instance,device,records,path=self.setup_actor()
        result=self.run_actor(instance)
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['deviceTargetReached']);self.assertTrue(result['finalIdleOffVerified'])
        self.assertTrue(result['nativeReportedTargetReached']);self.assertTrue(result['actualAngleWithinNativeTolerance'])
        self.assertFalse(result['taskGoalEvaluated'])
        self.assertAlmostEqual(result['actualAngleErrorDeg'],.2)
        self.assertEqual([record['stage'] for record in records],['before','armed','after'])
        self.assertEqual(sum(route=='/shoulder' for route,_ in device.requests),1)
        self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)
        self.assertTrue(any(e['pwm']==191 for e in result['outputTrace']))
        self.assertFalse(any(device.speeds.values()))
        self.assertTrue(instance.port.full_output_snapshot)
        native_index=next(i for i,(route,_) in enumerate(device.requests) if route=='/shoulder')
        stop_index=next(i for i,(_,payload) in enumerate(device.requests) if payload=={'cmd':'stop'})
        active_outputs=[route for route,_ in device.requests[native_index+1:stop_index] if route.startswith('/action_outputs')]
        self.assertTrue(active_outputs)
        self.assertTrue(all('?after=' in route and route.endswith('&limit=8') for route in active_outputs))
        active_statuses=[route for route,_ in device.requests[native_index+1:stop_index]
                         if route in ('/status','/collection_status')]
        self.assertTrue(active_statuses)
        self.assertTrue(all(route=='/status' for route in active_statuses))
        self.assertTrue(any(route=='/action_outputs' for route,_ in device.requests[:native_index]))
        self.assertTrue(any(route=='/action_outputs' for route,_ in device.requests[stop_index+1:]))
        with self.assertRaisesRegex(ValueError,'unconsumed'):self.run_actor(instance)

    def test_native_missed_preserved_separately_from_task_progress(self):
        instance,_,records,_=self.setup_actor(mode='missed')
        result=self.run_actor(instance)
        self.assertEqual(result['status'],'native_failed',result)
        self.assertEqual(result['nativeTerminalReason'],'TARGET_MISSED')
        self.assertFalse(result['deviceTargetReached']);self.assertFalse(result['taskGoalEvaluated'])
        self.assertTrue(result['finalIdleOffVerified']);self.assertEqual(records[-1]['stage'],'after')

    def test_native_report_and_current_angle_mismatch_is_not_a_task_failure(self):
        instance,_,records,_=self.setup_actor(mode='coast')
        result=self.run_actor(instance)
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['nativeReportedTargetReached'])
        self.assertFalse(result['actualAngleWithinNativeTolerance'])
        self.assertFalse(result['deviceTargetReached'])
        self.assertAlmostEqual(result['actualAngleErrorDeg'],1.2)
        self.assertEqual(result['errors'],[])
        self.assertFalse(result['taskGoalEvaluated'])
        self.assertTrue(result['finalIdleOffVerified']);self.assertEqual(records[-1]['stage'],'after')

    def test_changed_target_reused_or_stale_frame_prevents_native_POST(self):
        for mode in ('goal_changed','same_frame','stale','metadata_forged','pixels_changed','gripper_binding',
                     'armed_drift','armed_output'):
            with self.subTest(mode=mode):
                instance,device,_,_=self.setup_actor(observation_mode=mode)
                result=self.run_actor(instance)
                self.assertEqual(result['status'],'failed')
                self.assertFalse(result['mayHaveRun'])
                self.assertFalse(any(route=='/shoulder' for route,_ in device.requests))
                self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)

    def test_unknown_native_POST_stops_once_without_retry_and_keeps_unknown(self):
        instance,device,records,_=self.setup_actor(mode='unknown')
        result=self.run_actor(instance)
        self.assertEqual(result['commandOutcome'],'unknown',result)
        self.assertTrue(result['mayHaveRun']);self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(route=='/shoulder' for route,_ in device.requests),1)
        self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)
        self.assertEqual(records[-1]['stage'],'after')
        self.assertTrue(instance.port.full_output_snapshot)

    def test_wrong_goal_other_axis_reverse_corridor_or_deadline_stops(self):
        for mode in ('wrong_receipt','wrong_goal','wrong_motor','reverse','deadline'):
            with self.subTest(mode=mode):
                instance,device,_,_=self.setup_actor(mode=mode)
                result=self.run_actor(instance)
                self.assertEqual(result['status'],'failed',result)
                self.assertEqual(sum(route=='/shoulder' for route,_ in device.requests),1)
                self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)
                self.assertFalse(any(device.speeds.values()))
                self.assertTrue(instance.port.full_output_snapshot)

    def test_general_port_cannot_dispatch_native_angle_route(self):
        port=SequencePort('http://controller.invalid','secret','boot',None)
        with self.assertRaisesRegex(ValueError,'finite_shoulder_exact_run_required'):
            port._call('/shoulder',dict(action='angle',degrees=190.,percent=75),run=True)

    def test_sparse_collection_status_cannot_prove_native_goal(self):
        instance,device,_,_=self.setup_actor()
        compact=device.status()
        for key in ('targetDeg','requestedPercent','lastStopReason'):
            compact['shoulderAngle'].pop(key)
        with self.assertRaisesRegex(ValueError,'native_goal_fields_missing'):
            instance._native_goal(compact)
        instance._native_goal(device.status())

    def test_invalid_goal_or_selection_binding_rejected_before_hardware(self):
        instance,device,_,_=self.setup_actor()
        for changes in (dict(targetAngleDeg=194.),dict(percent=100),dict(initialDecisionSha256='f'*64)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                actor.validate_scope(dict(instance.scope,**changes))
        self.assertFalse(device.requests)
