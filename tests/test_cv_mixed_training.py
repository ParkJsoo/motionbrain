"""Mixed-session guards and warm start; no detector fitting or downloads."""
import copy
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from learning.cv_dataset import CVDataset, validate_mixed_inventory
from learning.cv_sanity import _strict_success
from learning.cv_training import Plan, MODEL, load_plan, preflight, run_loop
from tests.test_cv_detection_training import EvaluatingCounterBackend


class MixedFixture(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        rows = []
        for i in range(12):
            path = self.root / f'{i}.png'
            path.write_bytes(f'fixture-{i}'.encode())
            rows.append(dict(id=f'f{i}', imageId=i+1, image=path.name,
                sourceSha256=hashlib.sha256(path.read_bytes()).hexdigest(), width=20, height=20,
                coordinates='original_xyxy_no_rotation_no_mirror',
                sessionId=f's{i//6}', leakageGroup=f'g{i//6}', split='train_development_pilot',
                labels=[dict(**{'class':cls}, presence='present' if box else 'absent',
                    bbox_xyxy=box, bbox_valid=bool(box), occlusion='none', truncated=False,
                    humanApproved=True, reviewStatus='human_approved')
                    for cls,box in [('lip_balm',[1,1,4,6] if i%6<4 else None),
                                    ('gripper',[8,8,15,15])]]))
        self.rows = rows
        self.manifest = self.write_manifest('mixed.json', rows)
        self.parent_manifest = self.write_manifest('parent.json', rows[:6])
        self.weights = self.root/'weights.pt'; self.weights.write_bytes(b'fixture checkpoint')
        self.plan = Plan(str(self.manifest), self.sha(self.manifest), str(self.weights),
                         self.sha(self.weights), str(self.root/'output'), mode='mixed_session_heads',
                         roi_sampling='negative_cap_3x', epochs=50, steps=300)

    def sha(self, path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write_manifest(self, name, rows):
        p=self.root/name
        p.write_text(json.dumps(dict(schema='motionbrain.cv_dataset.v1',
            classIds={'background_reserved_not_annotation':0,'lip_balm':1,'gripper':2},
            splits={'train_development_pilot':[r['id'] for r in rows]}, records=rows)))
        return p


class MixedContractTest(MixedFixture):
    def test_mixed_preflight_retains_both_sessions_and_legacy_rejects_twelve(self):
        result=preflight(self.plan)
        self.assertEqual(result['sessions'], ['s0','s1'])
        self.assertIsNone(result['session'])
        with self.assertRaisesRegex(ValueError,'six'):
            preflight(replace(self.plan,mode='detection_heads'))

    def test_rejects_imbalanced_sessions_shared_groups_and_changed_presence(self):
        frames=CVDataset(self.manifest).frames
        variants=[frames[:6], tuple(replace(f,session='s0') for f in frames),
                  tuple(replace(f,group='g0') for f in frames),
                  (replace(frames[0],boxes=(None,frames[0].boxes[1])),)+frames[1:],
                  (replace(frames[0],sha256=frames[1].sha256),)+frames[1:],
                  (replace(frames[0],approved=False),)+frames[1:]]
        for changed in variants:
            with self.subTest(changed=changed[0].id), self.assertRaises(ValueError):
                validate_mixed_inventory(changed)

    def test_mixed_schedule_stops_only_on_success_and_honors_step_cap(self):
        backend=EvaluatingCounterBackend(success_at=100)
        state=run_loop(backend,self.plan,lambda event:None,900,clock=lambda:0)
        self.assertEqual((state['stepsCompleted'],state['status']),(100,'fit_observed'))
        self.assertEqual(backend.evaluations,[0,30,100])
        backend=EvaluatingCounterBackend()
        state=run_loop(backend,self.plan,lambda event:None,900,clock=lambda:0)
        self.assertEqual(state['stepsCompleted'],150)  # 50 epochs x 3 mock batches
        self.assertFalse(state['observedSuccess'])

    def test_success_cannot_use_old_six_denominators_or_hide_new_session_failure(self):
        m={'frames':12,'processingFailures':0,'classes':{
            'lip_balm':{'tp':8,'fp':0,'fn':0},'gripper':{'tp':12,'fp':0,'fn':0}}}
        for k in ('pairCoverage','positionSuccess','directionSuccess'): m[k]={'count':8,'denominator':8}
        self.assertTrue(_strict_success({'metrics':m},mixed_session=True))
        self.assertFalse(_strict_success({'metrics':m}))
        for changes in ('gripper','pair','failure'):
            bad=copy.deepcopy(m)
            if changes=='gripper': bad['classes']['gripper'].update(tp=6,fn=6)
            elif changes=='pair': bad['pairCoverage']={'count':4,'denominator':4}
            else: bad['processingFailures']=1
            self.assertFalse(_strict_success({'metrics':bad},mixed_session=True))

    def test_plan_is_explicit_and_bounded(self):
        from dataclasses import asdict
        p=self.root/'plan.json'; d=dict(schema='motionbrain.cv_training_plan.v1',**asdict(self.plan))
        p.write_text(json.dumps(d)); self.assertEqual(load_plan(p)[0].mode,'mixed_session_heads')
        for bad in ({'steps':301},{'seconds':901},{'roi_sampling':'default'}):
            p.write_text(json.dumps(dict(d,**bad)))
            with self.assertRaises(ValueError): load_plan(p)


@unittest.skipUnless(importlib.util.find_spec('torch') is not None,'optional torch runtime')
class MixedWarmStartTest(MixedFixture):
    def setUp(self):
        super().setUp()
        import torch
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        from torchvision.models.detection._utils import BalancedPositiveNegativeSampler
        from learning.cv_sanity import state_digest
        self.torch=torch
        class TinyModel(torch.nn.Module):
            def __init__(self,num_classes=3,**kwargs):
                super().__init__()
                self.backbone=torch.nn.Linear(2,2)
                self.rpn=torch.nn.Module(); self.rpn.head=torch.nn.Linear(2,2)
                self.rpn.fg_bg_sampler=BalancedPositiveNegativeSampler(256,.5)
                self.roi_heads=torch.nn.Module(); self.roi_heads.box_head=torch.nn.Linear(2,2)
                self.roi_heads.box_predictor=FastRCNNPredictor(2,num_classes)
                self.roi_heads.fg_bg_sampler=BalancedPositiveNegativeSampler(512,.25)
        self.factory=TinyModel; self.model=TinyModel()
        self.saved={'state_dict':copy.deepcopy(self.model.state_dict()),'provenance':{
            'model':MODEL,'frames':[r['id'] for r in self.rows[:6]],
            'plan':{'manifest':str(self.parent_manifest)},'manifestSha256':self.sha(self.parent_manifest),
            'modelStateSha256':state_digest(self.model.state_dict())}}

    def test_backend_restores_predictor_exactly_and_never_updates_or_downloads(self):
        from learning.cv_training import TorchBackend
        from learning.cv_sanity import state_digest
        self.torch.save(self.saved,self.weights)
        plan=replace(self.plan,weights_sha256=self.sha(self.weights))
        optimizer=MagicMock()
        def sgd(parameters,**kwargs):
            optimizer.param_groups=[{'params':list(parameters)}]; return optimizer
        with patch('torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn',side_effect=self.factory) as build, \
             patch('torch.optim.SGD',side_effect=sgd), \
             patch('torchvision.models.detection.faster_rcnn.FastRCNNPredictor',side_effect=AssertionError('must not reset trained predictor')), \
             patch('torch.hub.download_url_to_file',side_effect=AssertionError('no download')):
            backend=TorchBackend(plan)
        self.assertEqual(build.call_args.kwargs['num_classes'],3)
        self.assertEqual(state_digest(backend.model.state_dict()),self.saved['provenance']['modelStateSha256'])
        self.assertTrue(backend.mixed_session); self.assertTrue(backend.sanity)
        backend.check_scope(); optimizer.step.assert_not_called()

    def test_rejects_changed_parent_labels_or_hash_or_nonfinite_checkpoint(self):
        from learning.cv_training import restore_training_checkpoint
        from learning.cv_sanity import state_digest
        frames=CVDataset(self.manifest).frames
        with self.assertRaisesRegex(ValueError,'retained unchanged'):
            restore_training_checkpoint(self.model,self.saved,
                (replace(frames[0],boxes=((2,2,4,6),frames[0].boxes[1])),)+frames[1:],self.torch)
        bad=copy.deepcopy(self.saved); bad['provenance']['manifestSha256']='0'*64
        with self.assertRaisesRegex(ValueError,'sha256 mismatch'):
            restore_training_checkpoint(self.model,bad,frames,self.torch)
        bad=copy.deepcopy(self.saved); next(iter(bad['state_dict'].values())).fill_(float('nan'))
        bad['provenance']['modelStateSha256']=state_digest(bad['state_dict'])
        with self.assertRaisesRegex(ValueError,'nonfinite'):
            restore_training_checkpoint(self.model,bad,frames,self.torch)

# Exercise the same failure, timeout, checkpoint publication and one-use paths
# under the newly introduced mode, using only counters and byte fixtures.
from tests.test_cv_detection_training import DetectionWorkerArtifactTest, ScheduledEvaluationTest


class MixedWorkerArtifactTest(DetectionWorkerArtifactTest):
    def setUp(self):
        super().setUp()
        self.plan=replace(self.plan,mode='mixed_session_heads')
        self.backend.initial_state_sha='c'*64


class MixedScheduledEvaluationTest(ScheduledEvaluationTest):
    def setUp(self):
        super().setUp()
        self.plan=replace(self.plan,mode='mixed_session_heads')
