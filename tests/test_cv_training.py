"""Runner tests use fake updates only: no model fitting or weight downloads."""
import contextlib
import copy
from dataclasses import replace
import hashlib
import io
import json
import multiprocessing
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from learning.cv_training import Plan, atomic_json, execute, load_plan, preflight, run_loop, supervise
from tools.train_cv import main


class Clock:
    def __init__(self): self.now = 0
    def __call__(self): return self.now


class FakeBackend:
    def __init__(self, clock=None):
        self.calls = 0; self.grad_calls = 0; self.clock = clock
        self.losses = {'loss':1.0}; self.fail_step = False; self.empty = False
    def batches(self, epoch): return [] if self.empty else range(3)
    def loss_and_grad(self, batch):
        self.grad_calls += 1
        if self.clock: self.clock.now += 1
        return self.losses
    def step(self):
        self.calls += 1  # Counter only, not an optimizer or tensor update.
        if self.fail_step: raise RuntimeError('mock update outcome unknown')


class BoundedLoopTest(unittest.TestCase):
    def setUp(self):
        self.plan = Plan('manifest', 'a'*64, 'weights', 'b'*64, 'output')
        self.events = []; self.clock = Clock(); self.backend = FakeBackend()
    def run_fake(self, **kwargs):
        return run_loop(self.backend, kwargs.pop('plan',self.plan), self.events.append,
                        kwargs.pop('deadline',900), clock=self.clock, **kwargs)
    def test_step_cap_never_starts_update_31(self):
        r = self.run_fake()
        self.assertEqual((r['status'],r['stepIntents'],r['stepsCompleted']),('step_limit',30,30))
        self.assertEqual(self.backend.calls,30)
        self.assertEqual(len([e for e in self.events if e['event']=='step_intent']),30)
    def test_epoch_and_smaller_step_caps(self):
        r = self.run_fake(plan=replace(self.plan,epochs=2))
        self.assertEqual((r['status'],r['epochsCompleted'],r['stepsCompleted']),('epoch_limit',2,6))
        self.backend = FakeBackend()
        r = self.run_fake(plan=replace(self.plan,steps=1))
        self.assertEqual(self.backend.calls,1); self.assertEqual(r['stepsCompleted'],1)
    def test_deadline_before_start_and_after_gradient_prevents_update(self):
        r = self.run_fake(deadline=0)
        self.assertEqual(r['status'],'time_limit'); self.assertEqual(self.backend.grad_calls,0)
        self.backend = FakeBackend(self.clock)
        r = self.run_fake(deadline=1)
        self.assertEqual(r['status'],'time_limit'); self.assertEqual(self.backend.calls,0)
    def test_deadline_after_intent_does_not_claim_completion(self):
        def emit(event):
            self.events.append(event)
            if event['event']=='step_intent': self.clock.now=900
        r=run_loop(self.backend,self.plan,emit,900,clock=self.clock)
        self.assertEqual((r['stepIntents'],r['stepsCompleted']),(1,0))
        self.assertEqual(self.backend.calls,0)
    def test_failure_during_step_records_uncertain_attempt_and_no_retry(self):
        self.backend.fail_step=True
        r=self.run_fake()
        self.assertEqual((r['status'],r['stepIntents'],r['stepsCompleted']),('failed',1,0))
        self.assertEqual(self.backend.calls,1)
    def test_nonfinite_empty_loss_and_empty_loader_fail_closed(self):
        for losses in ({},{'loss':float('nan')},{'loss':float('inf')}):
            self.backend=FakeBackend(); self.backend.losses=losses
            r=self.run_fake();self.assertEqual(r['status'],'failed');self.assertEqual(self.backend.calls,0)
        self.backend=FakeBackend();self.backend.empty=True
        self.assertEqual(self.run_fake()['status'],'failed')
    def test_write_failure_before_step_does_not_mutate(self):
        def emit(event):
            if event['event']=='step_intent':raise OSError('disk full')
        r=run_loop(self.backend,self.plan,emit,900,clock=self.clock)
        self.assertEqual(r['status'],'failed');self.assertEqual(self.backend.calls,0)


class FakeProcess:
    def __init__(self, target=None, args=None, mode='success'):
        self.target=target;self.args=args;self.mode=mode;self.pid=None;self.exitcode=None
        self.alive=False;self.terminated=False;self.killed=False
    def start(self):
        self.pid=123;self.alive=True
        if self.mode=='start_error':raise OSError('spawn failed after start')
        if self.mode=='success' and self.args:
            _,output,_=self.args
            Path(output,'checkpoint.pt').write_bytes(b'fake checkpoint, no model')
            atomic_json(Path(output)/'progress.json',{'status':'step_limit','stepIntents':1,
                        'stepsCompleted':1,'checkpoint':'checkpoint.pt',
                        'checkpointSha256':hashlib.sha256(b'fake checkpoint, no model').hexdigest()})
    def join(self, timeout=None):
        if self.mode=='interrupt' and not self.terminated:raise KeyboardInterrupt()
        if self.mode in ('success','crash','missing') or self.terminated and self.mode!='stubborn':
            self.alive=False;self.exitcode=0 if self.mode!='crash' else 9
        if self.killed:self.alive=False;self.exitcode=-9
    def is_alive(self):return self.alive
    def terminate(self):self.terminated=True
    def kill(self):self.killed=True


class SupervisorTest(unittest.TestCase):
    def test_timeout_terminates_and_kills_stubborn_worker(self):
        p=FakeProcess(mode='stubborn')
        self.assertEqual(supervise(p,0,clock=lambda:0),'time_limit')
        self.assertTrue(p.terminated and p.killed);self.assertFalse(p.alive)
    def test_interrupt_cleans_up(self):
        p=FakeProcess(mode='interrupt')
        self.assertEqual(supervise(p,900,clock=lambda:0),'interrupted')
        self.assertTrue(p.terminated);self.assertFalse(p.alive)
    def test_spawn_failure_cleans_up_a_partly_started_worker(self):
        p=FakeProcess(mode='start_error')
        with self.assertRaises(OSError):supervise(p,900,clock=lambda:0)
        self.assertTrue(p.terminated);self.assertFalse(p.alive)
    def test_real_sleeping_child_is_stopped_without_ml_runtime(self):
        p=multiprocessing.get_context('spawn').Process(target=time.sleep,args=(10,))
        start=time.monotonic()
        self.assertEqual(supervise(p,start+0.15),'time_limit')
        self.assertFalse(p.is_alive());self.assertLess(time.monotonic()-start,4)


class PlanAndExecutionTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve();self.path=self.root/'plan.json'
        self.weights=self.root/'weights.pth';self.weights.write_bytes(b'fake bytes; never deserialized')
        records=[]
        for i in range(6):
            raw=f'fake-image-{i}'.encode();p=self.root/f'{i}.png';p.write_bytes(raw)
            labels=[]
            for cls in ['lip_balm','gripper']:
                present=cls=='gripper' or i<4
                labels.append({'class':cls,'presence':'present' if present else 'absent',
                    'bbox_xyxy':[1,1,3,3] if present else None,'bbox_valid':present,
                    'occlusion':'none' if present else 'unknown','truncated':False,
                    'humanApproved':True,'reviewStatus':'human_approved'})
            records.append({'id':str(i),'imageId':i+1,'image':p.name,
                'sourceSha256':hashlib.sha256(raw).hexdigest(),'width':4,'height':4,
                'coordinates':'original_xyxy_no_rotation_no_mirror','sessionId':'s','leakageGroup':'g',
                'split':'train_development_pilot','labels':labels})
        self.manifest=self.root/'manifest.json'
        self.data={'schema':'motionbrain.cv_dataset.v1',
            'classIds':{'background_reserved_not_annotation':0,'lip_balm':1,'gripper':2},
            'records':records,'splits':{'train_development_pilot':[str(i) for i in range(6)]}}
        self.manifest.write_text(json.dumps(self.data))
        self.doc={'schema':'motionbrain.cv_training_plan.v1','manifest':'manifest.json',
            'manifest_sha256':hashlib.sha256(self.manifest.read_bytes()).hexdigest(),
            'weights':'weights.pth','weights_sha256':hashlib.sha256(self.weights.read_bytes()).hexdigest(),
            'output_dir':'run'}
        self.write_plan()
    def write_plan(self):self.path.write_text(json.dumps(self.doc))
    def test_plan_caps_types_schema_and_paths(self):
        p,_=load_plan(self.path);self.assertEqual(p.manifest,str(self.manifest));self.assertEqual(p.seconds,900)
        for key,value in [('epochs',11),('steps',31),('seconds',901),('steps',True),('seconds',0),
                          ('seed',-1),('schema','other'),('weights_sha256','bad'),('unknown',1)]:
            with self.subTest(key=key,value=value):
                original=copy.deepcopy(self.doc);self.doc[key]=value;self.write_plan()
                with self.assertRaises((ValueError,TypeError)):load_plan(self.path)
                self.doc=original
    def test_preflight_checks_hashes_and_pilot_counts_without_decoding(self):
        plan,_=load_plan(self.path);self.assertEqual(len(preflight(plan)['frames']),6)
        for p in [self.weights,self.manifest,self.root/'0.png']:
            raw=p.read_bytes();p.write_bytes(raw+b'changed')
            with self.subTest(p=p),self.assertRaises((ValueError,json.JSONDecodeError)):preflight(plan)
            p.write_bytes(raw)
        self.data['records'].pop();self.data['splits']['train_development_pilot'].pop()
        self.manifest.write_text(json.dumps(self.data))
        with self.assertRaisesRegex(ValueError,'six'):preflight(replace(plan,manifest_sha256=hashlib.sha256(self.manifest.read_bytes()).hexdigest()))
    def test_execution_requires_explicit_flag_and_check_has_no_writes(self):
        with self.assertRaisesRegex(ValueError,'--execute'):execute(self.path)
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(main(['check','--plan',str(self.path)]),0)
        self.assertFalse((self.root/'run').exists());self.assertFalse(Path(str(self.path)+'.consumed').exists())
    def test_one_use_and_existing_output_preservation(self):
        context=type('Context',(),{'Process':FakeProcess})()
        with patch('learning.cv_training.multiprocessing.get_context',return_value=context):
            r=execute(self.path,confirmed=True)
        self.assertTrue(r['runConsumed']);self.assertEqual(r['stepsCompleted'],1)
        original=(self.root/'run'/'run.json').read_bytes()
        with self.assertRaises(ValueError):execute(self.path,confirmed=True)
        self.doc['output_dir']='new-run';self.write_plan()
        with self.assertRaises(FileExistsError):execute(self.path,confirmed=True)
        self.assertEqual((self.root/'run'/'run.json').read_bytes(),original)
        self.assertFalse((self.root/'new-run').exists())
    def test_crashed_or_incomplete_child_never_reports_success(self):
        for mode in ('crash','missing'):
            context=type('Context',(),{'Process':lambda _,**kw:FakeProcess(**kw,mode=mode)})()
            with patch('learning.cv_training.multiprocessing.get_context',return_value=context):
                r=execute(self.path,confirmed=True)
            self.assertIn(r['status'],('worker_exit','worker_incomplete'))
            # Fresh independent fixture to exercise a different failure, not a retry of a run.
            self.path=self.root/f'{mode}-next-plan.json';self.doc['output_dir']=f'{mode}-next-output';self.write_plan()
    def test_supervisor_error_retains_durable_step_counts(self):
        class JoinError(FakeProcess):
            def join(self, timeout=None):
                if not self.terminated:
                    raise OSError('mock supervisor join failure after worker wrote counts')
                super().join(timeout)
        context=type('Context',(),{'Process':JoinError})()
        with patch('learning.cv_training.multiprocessing.get_context',return_value=context):
            r=execute(self.path,confirmed=True)
        self.assertEqual(r['status'],'failed')
        self.assertEqual((r['stepIntents'],r['stepsCompleted']),(1,1))
        self.assertTrue(r['trainingMayHaveStarted'])
        self.assertIn('join failure',r['error'])
    def test_plan_duplicate_keys_are_rejected(self):
        raw=self.path.read_text()
        self.path.write_text(raw[:-1]+',"steps":1,"steps":30}')
        with self.assertRaisesRegex(ValueError,'duplicate'):load_plan(self.path)
    def test_wrong_checkpoint_hash_never_reports_success(self):
        class BadCheckpoint(FakeProcess):
            def start(self):
                super().start()
                Path(self.args[1],'checkpoint.pt').write_bytes(b'tampered after reporting')
        context=type('Context',(),{'Process':BadCheckpoint})()
        with patch('learning.cv_training.multiprocessing.get_context',return_value=context):
            r=execute(self.path,confirmed=True)
        self.assertEqual(r['status'],'failed');self.assertEqual(r['stepsCompleted'],1)
        self.assertIn('checkpoint verification failed',r['error'])
    def test_import_and_help_do_not_load_torch(self):
        r=subprocess.run([sys.executable,'-c',"import sys; sys.modules['torch']=None; from learning.cv_training import load_plan; from tools.train_cv import main; main(['--help'])"],capture_output=True,text=True)
        self.assertEqual(r.returncode,0,r.stderr)


class WorkerArtifactTest(unittest.TestCase):
    def run_worker(self, fail_save=False):
        from learning.cv_training import _worker
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        root=Path(tmp.name)
        backend=FakeBackend()
        backend.torch=type('Version',(),{'__version__':'fake-test-no-ML'})()
        backend.frozen_before='fake digest'
        def save(path, provenance):
            self.assertEqual(provenance['result']['stepsCompleted'],2)
            Path(path).write_bytes(b'synthetic checkpoint test')
            if fail_save:raise OSError('mock partial write failure')
        backend.save=save
        plan=Plan('m','a'*64,'w','b'*64,str(root),steps=2)
        with patch('learning.cv_training.preflight',return_value={}), \
             patch('learning.cv_training.TorchBackend',return_value=backend), \
             patch('socket.create_connection'), patch('socket.socket.connect'), patch('urllib.request.urlopen'):
            _worker(plan,str(root),time.monotonic()+30)
        return root,json.loads((root/'progress.json').read_text())
    def test_success_publishes_only_one_final_checkpoint_and_provenance(self):
        root,r=self.run_worker()
        self.assertEqual(r['status'],'step_limit');self.assertEqual(r['stepsCompleted'],2)
        self.assertEqual(r['checkpointSha256'],hashlib.sha256((root/'checkpoint.pt').read_bytes()).hexdigest())
        self.assertFalse((root/'checkpoint.pt.partial').exists())
        events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
        self.assertEqual([e['event'] for e in events if e['event'].startswith('step_')],
                         ['step_intent','step_complete','step_intent','step_complete'])
    def test_checkpoint_write_failure_preserves_partial_and_is_not_success(self):
        root,r=self.run_worker(fail_save=True)
        self.assertEqual(r['status'],'failed');self.assertEqual(r['stepsCompleted'],2)
        self.assertFalse((root/'checkpoint.pt').exists());self.assertTrue((root/'checkpoint.pt.partial').exists())


import importlib.util
@unittest.skipUnless(importlib.util.find_spec('torch') is not None,'optional torch runtime not installed')
class TorchBackendContractTest(unittest.TestCase):
    """Tiny synthetic modules and mocked optimizer; no optimizer step is executed."""
    def setUp(self):
        import torch
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        from learning.cv_training import TorchBackend
        self.torch=torch
        class TinyModel(torch.nn.Module):
            def __init__(self):
                super().__init__();self.backbone=torch.nn.Sequential(torch.nn.Linear(2,2),torch.nn.BatchNorm2d(2))
                self.roi_heads=torch.nn.Module();self.roi_heads.box_predictor=FastRCNNPredictor(2,91)
            def forward(self, images, targets):
                return {'loss':sum(p.square().sum() for p in self.roi_heads.box_predictor.parameters())}
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        weights=self.root/'weights.pth';weights.write_bytes(b'mocked local state; no download')
        self.plan=Plan('mock-manifest','a'*64,str(weights),hashlib.sha256(weights.read_bytes()).hexdigest(),str(self.root))
        state=TinyModel().state_dict()
        dataset=type('Dataset',(),{'manifest_sha256':'a'*64,'__len__':lambda s:6})()
        from unittest.mock import MagicMock
        self.optimizer=MagicMock()
        with patch('torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn',side_effect=lambda **kw:TinyModel()) as factory, \
             patch('torch.load',return_value=state) as load, \
             patch('learning.cv_tensors.TorchCVDataset',return_value=dataset), \
             patch('torch.optim.SGD',return_value=self.optimizer), \
             patch('torch.hub.download_url_to_file',side_effect=AssertionError('No downloads allowed')):
            self.backend=TorchBackend(self.plan)
        self.assertEqual(factory.call_args.kwargs,{'weights':None,'weights_backbone':None,'num_classes':91})
        self.assertTrue(load.call_args.kwargs['weights_only'])
        self.assertEqual(load.call_args.kwargs['map_location'],'cpu')
    def test_only_predictor_trainable_and_backbone_bn_frozen(self):
        self.assertTrue(self.backend.model.training)
        self.assertFalse(self.backend.model.backbone.training)
        from torchvision.ops.misc import FrozenBatchNorm2d
        self.assertIsInstance(self.backend.model.backbone[1], FrozenBatchNorm2d)
        self.assertEqual(self.backend.model.backbone[1].eps, 1e-5)
        for name,p in self.backend.model.named_parameters():
            self.assertEqual(p.requires_grad,name.startswith('roi_heads.box_predictor.'))
        self.assertEqual(self.backend.model.roi_heads.box_predictor.cls_score.out_features,3)
        self.optimizer.step.assert_not_called()
    def test_loss_backward_produces_head_grads_without_optimizer_update(self):
        before={n:p.detach().clone() for n,p in self.backend.model.named_parameters()}
        losses=self.backend.loss_and_grad(([],[]));self.assertGreater(losses['loss'],0)
        self.optimizer.step.assert_not_called()
        for n,p in self.backend.model.named_parameters():self.torch.testing.assert_close(p,before[n])
        self.assertEqual(self.backend.frozen_digest(),self.backend.frozen_before)
    def test_nonfinite_loss_is_rejected_before_backward(self):
        with patch.object(self.backend.model,'forward',return_value={'loss':self.torch.tensor(float('nan'),requires_grad=True)}):
            with self.assertRaisesRegex(ValueError,'nonfinite'):self.backend.loss_and_grad(([],[]))
        self.assertTrue(all(p.grad is None for p in self.backend.parameters));self.optimizer.step.assert_not_called()
    def test_frozen_buffer_mutation_blocks_checkpoint(self):
        self.backend.model.backbone[1].running_mean.add_(1)
        with self.assertRaisesRegex(ValueError,'frozen'):self.backend.save(self.root/'final.pt',{})
        self.assertFalse((self.root/'final.pt').exists());self.optimizer.step.assert_not_called()


if __name__ == '__main__':
    unittest.main()
