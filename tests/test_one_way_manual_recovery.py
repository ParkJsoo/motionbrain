"""실제 장치 없이 별도 수동 복귀 계약·근거·단일 소비를 검사한다."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from learning.approach_external_guard import CURRENT_SCENE_SCHEMA, CURRENT_REFERENCE_MEANING
from learning.artifacts import ArtifactStore, StorageError
from learning.gripper_camera_dataset import POLICY
from learning.one_way_manual_recovery import (OneWayManualRecoveryGuard, SCHEMA, EVIDENCE_SCHEMA,
    MEANING, COMMAND, BEFORE_MINIMUM, BEFORE_MAXIMUM, AFTER_MINIMUM, AFTER_MAXIMUM, approval_scope_sha256)
from learning.relative_action_learning import fit
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from tests import test_approach_external_guard as external_tests


class ManualRecoveryTests(unittest.TestCase):
    write=external_tests.CurrentSceneGuardTests.write
    write_json=external_tests.CurrentSceneGuardTests.write_json

    def put(self,root,name,value):
        raw=json.dumps(value).encode() if isinstance(value,dict) else value
        path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
        return dict(path=str(path.relative_to(self.repo)),sha256=sha256(raw))

    def setUp(self):
        external_tests.CurrentSceneGuardTests.setUp(self)
        self.read_frames={};rows=[];samples=[]
        context=dict(physicalView='synthetic',object='synthetic-pink',poseRegime='synthetic-only',
            landmarks='development-gripper-visible-v1:'+'a'*64,
            cameraIdentity=dict(camera='gripper',cameraBootId='b'*32,timestampSemantics='first_dma_buffer',
                                observationPolicy=copy.deepcopy(POLICY)),imageShape=[240,320],
            stateFields=['objectVisibleMinusCornerMidXpx','objectVisibleMinusCornerMidYpx','visibleCornerHorizontalSpanPx'],
            executionTimingPolicy='elbow-deadline-350-v1')
        for i,delta in enumerate([[-1,-3.5,0],[-2,-3,0],[-2,-2.5,0]]):
            sid=f'synthetic-UP-{i}';source=self.repo/f'.local/sources/{sid}';source.mkdir(parents=True)
            session=dict(sessionId=sid,phase='completed',finalIdleOffVerified=True,events=[dict(kind='command',
                pulse=dict(COMMAND,motor=3,expectedPwm=-153),monitorTimingPolicy='elbow-deadline-350-v1')])
            session_binding=self.put(source,'session.json',session)
            before=[3.+i,19.+i,124.];after=[v+d for v,d in zip(before,delta)]
            row=dict(sessionId=sid,sourceRoot=str(source),sourceHash=session_binding['sha256'],
                context=context,command=COMMAND,before=before,after=after,captureGroup='synthetic',
                episode=['boot',sid],frameIds=[f'frame-{i}-before',f'frame-{i}-after'],
                developmentObservationOnly=True,recoveryDevelopmentOnly=True)
            sample=dict(sessionId=sid,sourceRoot=str(source.relative_to(self.repo)),session=session_binding,
                        gripperDelta=delta,forearmDelta=[-1 if i==0 else -2,-3])
            for j,side in enumerate(['before','after']):
                metadata=dict(status=200,method='GET',path='/capture',query={'fresh':'1'})
                meta_binding=self.put(source/f'gripper-pair',f'esp32-{side}.json',metadata)
                frame=dict(frameId=row['frameIds'][j],imageSha256='c'*64,pixelSha256='d'*64)
                observed=dict(state=row[side],hostClockId=sid,status='complete_development',
                    sourceFreshMetadataVerified=True,referenceImageSelfReplay=False,
                    observationContract=context['landmarks'],cameraBootId='b'*32,sourceFrame=frame,
                    sourceMetadataSha256=meta_binding['sha256'])
                sample['gripperBefore' if j==0 else 'gripperAfter']=self.put(source,f'current-gripper-{side}.json',observed)
                self.read_frames[(sid,side)]=(frame,metadata,'b'*32)
                external_root=source/'native-camera'
                image_binding=self.put(external_root,f'{side}.bin',b'synthetic-native-image')
                ext_meta=self.put(external_root,f'{side}.json',dict(synthetic=True))
                point=[1028.5+i,432.+i]
                if j:point=[v+d for v,d in zip(point,sample['forearmDelta'])]
                reviewed=dict(schemaVersion=CURRENT_SCENE_SCHEMA,accepted=True,referenceMeaning=CURRENT_REFERENCE_MEANING,
                    external=dict(forearmCenter=point),externalSource=dict(root=str(external_root),
                        frame=dict(imageFile=f'{side}.bin',imageSha256=image_binding['sha256'],
                                   metadataFile=f'{side}.json',metadataSha256=ext_meta['sha256'])))
                sample['externalBefore' if j==0 else 'externalAfter']=self.put(source,f'approach-external-{side}.json',reviewed)
            rows.append(row);samples.append(sample)
        model=fit(rows,allow_development_observations=True)
        model_binding=self.write_json('recovery-training-model.json',model)
        manifest=dict(controlEligible=False,independentTest=False,rows=rows,model=model_binding)
        self.manifest=manifest;self.rows=rows
        manifest_binding=self.write_json('recovery-training-manifest.json',manifest)
        self.evidence=dict(schemaVersion=EVIDENCE_SCHEMA,trainingModel=model_binding,trainingManifest=manifest_binding,
                           trainingRowsSha256=plan_digest(rows),samples=samples)
        evidence_binding=self.write_json('recovery-evidence.json',self.evidence)
        self.config.update(schemaVersion=SCHEMA,recovery=dict(meaning=MEANING,command=COMMAND,maxAttempts=1,
            beforeMinimum=BEFORE_MINIMUM,beforeMaximum=BEFORE_MAXIMUM,afterMinimum=AFTER_MINIMUM,
            afterMaximum=AFTER_MAXIMUM,evidence=evidence_binding,approval={}))
        self.approval=dict(approved=True,scopeSha256=approval_scope_sha256(self.config),
                           fullPlanSha256='e'*64,userReply='합성 테스트 전용 승인; 실제 장치 실행 금지')
        self.config['recovery']['approval']=self.write_json('recovery-approval.json',self.approval)
        self.binding=self.write_json('one-way-config.json',self.config)
        self.patch_frame=patch('learning.one_way_manual_recovery.read_frame',side_effect=lambda root,side,sid:
                               copy.deepcopy(self.read_frames[(sid,side)]))
        self.patch_frame.start();self.addCleanup(self.patch_frame.stop)
        self.guard=self.make_guard()
        self.result['state']=[3.,24.5,124.]

    def make_guard(self):
        return OneWayManualRecoveryGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)

    def review(self,*,forearm=None):
        external=dict(forearmCenter=forearm or [1033.5,441.],objectCenter=[575.5,700.5],
                      objectBox=[550,650,601,751],ncc=1.,peakGap=1.)
        # 실제 영상의 template 계산 대신 같은 측정 형식만 합성한다.
        with patch('learning.approach_external_guard.ApproachExternalGuard._measure',return_value=external):
            return self.guard.review(copy.deepcopy(self.record),copy.deepcopy(self.result),deadline=60.,tick=lambda:None)

    def test_narrow_before_and_empirical_endpoints_are_explicit_not_model_success(self):
        outcome=self.review()
        proof=json.loads((self.store.root/outcome['evidence']['path']).read_bytes())
        self.assertEqual(proof['schemaVersion'],SCHEMA)
        self.assertEqual(proof['recoveryPhase'],'before_one_way_input')
        self.assertFalse(proof['modelSelected'])
        self.assertFalse(proof['expectedEndpointsAreGuarantees'])
        self.assertEqual(len(proof['expectedEndpointsFromHistoricalDeltas']),3)
        self.assertEqual(proof['experimentalFence']['forearmMaximumDistancePx'],11)

    def test_exact_one_UP_consumed_durably_and_original_monitor_restored(self):
        self.review();original=self.port._check_status
        self.guard.validate_command(COMMAND)
        self.assertIsNot(self.port._check_status,original)
        self.guard.disarm();self.assertIs(self.port._check_status,original)
        with self.assertRaisesRegex(ValueError,'one_UP_attempt'):self.guard.validate_command(COMMAND)
        self.store=ArtifactStore(self.repo/'.local/observations-next')
        self.guard=self.make_guard()
        with self.assertRaises(StorageError):self.review()

    def test_wrong_direction_and_missing_approval_cannot_execute(self):
        self.review()
        with self.assertRaisesRegex(ValueError,'one_UP_attempt'):
            self.guard.validate_command(dict(COMMAND,direction='down'))
        self.approval['approved']=False
        self.config['recovery']['approval']=self.write_json('rejected-approval.json',self.approval)
        self.binding=self.write_json('rejected-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'explicit_approval'):self.make_guard()

    def test_before_bounds_rejected_and_same_approval_cannot_start_a_new_guard(self):
        self.result['state']=[2.9,24.5,124.]
        with self.assertRaisesRegex(ValueError,'state_fence'):self.review()
        claim=self.guard._claim_store.root
        self.assertTrue((claim/'claim.json').exists())
        self.assertTrue((claim/'failed.json').exists())
        self.assertFalse((claim/'intent.json').exists())
        self.guard=self.make_guard();self.result['state']=[3.,24.5,124.]
        with self.assertRaises(StorageError):self.review()
        with self.assertRaisesRegex(ValueError,'scope_failed'):self.guard.validate_command(COMMAND)

    def test_expected_endpoint_outside_original_fence_rejected(self):
        with self.assertRaisesRegex(ValueError,'expected_endpoint'):
            self.review(forearm=[1039.4,432.1])

    def test_wrong_DOWN_validation_consumes_scope_and_cannot_be_followed_by_UP(self):
        self.review()
        with self.assertRaisesRegex(ValueError,'one_UP_attempt'):
            self.guard.validate_command(dict(COMMAND,direction='down'))
        with self.assertRaisesRegex(ValueError,'scope_failed'):self.guard.validate_command(COMMAND)
        claim=self.guard._claim_store.root
        self.assertTrue((claim/'failed.json').exists())
        self.assertFalse((claim/'intent.json').exists())

    def test_first_validation_without_review_claims_scope_and_refuses_replacement(self):
        with self.assertRaisesRegex(ValueError,'endpoint_checks'):self.guard.validate_command(COMMAND)
        self.assertIsNotNone(self.guard._claim_store)
        with self.assertRaisesRegex(ValueError,'scope_failed'):self.guard.validate_command(COMMAND)
        self.guard=self.make_guard()
        with self.assertRaises(StorageError):self.review()

    def test_constructor_is_readonly_and_unapproved_constructor_cannot_claim_scope(self):
        attempts=self.repo/'.local/one-way-recovery-attempts'
        self.assertFalse(attempts.exists())
        self.approval['approved']=False
        self.config['recovery']['approval']=self.write_json('unapproved.json',self.approval)
        self.binding=self.write_json('unapproved-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'explicit_approval'):self.make_guard()
        self.assertFalse(attempts.exists())

    def test_after_requires_original_fence_and_never_grants_another_UP(self):
        self.review();self.guard.validate_command(COMMAND);self.guard.disarm()
        self.result['state']=[1.,21.,124.]
        proof=self.review(forearm=[1031.5,438.])
        saved=json.loads((self.store.root/proof['evidence']['path']).read_bytes())
        self.assertEqual(saved['recoveryPhase'],'after_original_fence')
        self.assertEqual(saved['experimentalFence']['forearmMaximumDistancePx'],10)
        self.result['state']=[1.,22.1,124.]
        with self.assertRaisesRegex(ValueError,'state_fence'):self.review(forearm=[1031.5,438.])
        with self.assertRaisesRegex(ValueError,'scope_failed'):self.guard.check()

    def test_manifest_row_state_or_deltas_cannot_be_relabelled(self):
        self.evidence['samples'][0]['gripperDelta']=[-1,-2.5,0]
        self.config['recovery']['evidence']=self.write_json('changed-evidence.json',self.evidence)
        # 승인 해시까지 변경해도 원본 행과 다른 delta는 거절한다.
        self.approval['scopeSha256']=approval_scope_sha256(self.config)
        self.config['recovery']['approval']=self.write_json('changed-approval.json',self.approval)
        self.binding=self.write_json('changed-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'observed_delta'):self.make_guard()


if __name__=='__main__':unittest.main()
