"""Detection-head runner contracts using counters and tiny mocked modules only."""
import copy
from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from learning.cv_training import Plan, load_plan, run_loop


class FakeClock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now


class EvaluatingCounterBackend:
    """No tensors, optimizer, or detector; a step increments a Python counter."""
    def __init__(self, success_at=None):
        self.calls = 0
        self.grad_calls = 0
        self.evaluations = []
        self.success_at = success_at

    def batches(self, epoch):
        return range(3)

    def loss_and_grad(self, batch):
        self.grad_calls += 1
        return {'loss': 1.0}

    def step(self):
        self.calls += 1

    def evaluate(self, step, check_deadline):
        check_deadline()
        assert step == self.calls
        self.evaluations.append(step)
        return {'observedSuccess': step == self.success_at,
                'modelStateSha256': 'a' * 64,
                'metrics': {'syntheticCounter': self.calls}}


class DetectionPlanTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / 'plan.json'
        self.doc = dict(schema='motionbrain.cv_training_plan.v1', manifest='manifest.json',
                        manifest_sha256='a' * 64, weights='weights.pt', weights_sha256='b' * 64,
                        output_dir='output')

    def load(self, **changes):
        self.path.write_text(json.dumps(dict(self.doc, **changes)))
        return load_plan(self.path)[0]

    def test_legacy_mode_and_caps_remain_default(self):
        self.assertEqual(self.load().mode, 'box_predictor_only')
        for changes in ({'steps': 31}, {'epochs': 11}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.load(**changes)

    def test_explicit_detection_mode_accepts_bounded_larger_budget(self):
        plan = self.load(mode='detection_heads', roi_sampling='negative_cap_3x',
                         epochs=100, steps=300, seconds=900)
        self.assertEqual((plan.epochs, plan.steps, plan.seconds), (100, 300, 900))

    def test_detection_mode_rejects_invalid_or_uncontrolled_settings(self):
        base = dict(mode='detection_heads', roi_sampling='negative_cap_3x')
        for changes in ({'roi_sampling': 'default'}, {'epochs': 101}, {'steps': 301},
                        {'seconds': 901}, {'steps': True}, {'mode': 'all'}, {'mode': None}):
            with self.subTest(changes=changes), self.assertRaises((ValueError, TypeError)):
                self.load(**dict(base, **changes))
        with self.assertRaises(ValueError):
            self.load(mode='detection_heads')


class ScheduledEvaluationTest(unittest.TestCase):
    def setUp(self):
        self.plan = Plan('manifest', 'a' * 64, 'weights', 'b' * 64, 'output',
                         mode='detection_heads', roi_sampling='negative_cap_3x',
                         epochs=100, steps=300)
        self.backend = EvaluatingCounterBackend()
        self.clock = FakeClock()
        self.events = []

    def emit(self, event):
        # Durable JSON events must be snapshots, not references to mutable lists.
        self.events.append(copy.deepcopy(event))

    def run_fake(self, plan=None, emit=None):
        return run_loop(self.backend, plan or self.plan, emit or self.emit,
                        900, clock=self.clock)

    def test_schedule_includes_initial_and_completed_update_300(self):
        state = self.run_fake()
        self.assertEqual(self.backend.evaluations, [0, 30, 100, 200, 300])
        self.assertEqual((state['status'], state['stepsCompleted']), ('step_limit', 300))
        self.assertEqual(self.backend.calls, 300)
        self.assertEqual([r['step'] for r in state['evaluations']], [0, 30, 100, 200, 300])

    def test_observed_success_stops_before_another_update_but_is_not_verified(self):
        self.backend.success_at = 30
        state = self.run_fake()
        self.assertEqual(state['status'], 'fit_observed')
        self.assertTrue(state['observedSuccess'])
        self.assertFalse(state['verifiedArtifact'])
        self.assertEqual((self.backend.calls, self.backend.grad_calls), (30, 30))
        self.assertEqual(self.backend.evaluations, [0, 30])

    def test_initial_observed_success_runs_no_update_or_gradient(self):
        self.backend.success_at = 0
        state = self.run_fake()
        self.assertEqual(state['status'], 'fit_observed')
        self.assertEqual((self.backend.calls, self.backend.grad_calls), (0, 0))
        self.assertFalse(state['verifiedArtifact'])

    def test_short_budget_marks_unreached_evaluations(self):
        for plan in (replace(self.plan, steps=31), replace(self.plan, epochs=1)):
            with self.subTest(steps=plan.steps, epochs=plan.epochs):
                self.backend = EvaluatingCounterBackend()
                state = self.run_fake(plan=plan)
                rows = {r['step']: r for r in state['evaluations']}
                for step in (100, 200, 300):
                    self.assertEqual(rows[step]['status'], 'not_reached')
                self.assertNotIn(100, self.backend.evaluations)

    def test_evaluation_error_is_failure_and_does_not_resume_updates(self):
        original = self.backend.evaluate
        def evaluate(step, check):
            if step == 30:
                raise RuntimeError('synthetic evaluation failure')
            return original(step, check)
        self.backend.evaluate = evaluate
        state = self.run_fake()
        self.assertEqual(state['status'], 'failed')
        self.assertEqual(self.backend.calls, 30)
        self.assertIn('synthetic evaluation failure', state['error'])

    def test_evaluation_deadline_is_checked_before_updates_resume(self):
        original = self.backend.evaluate
        def evaluate(step, check):
            result = original(step, check)
            if step == 30:
                self.clock.now = 900
                check()
            return result
        self.backend.evaluate = evaluate
        state = self.run_fake()
        self.assertEqual(state['status'], 'time_limit')
        self.assertEqual(self.backend.calls, 30)

    def test_success_returned_after_deadline_is_not_accepted(self):
        def evaluate(step, check):
            self.clock.now = 900
            return {'observedSuccess': True, 'modelStateSha256': 'a' * 64, 'metrics': {}}
        self.backend.evaluate = evaluate
        state = self.run_fake()
        self.assertEqual(state['status'], 'time_limit')
        self.assertEqual(self.backend.calls, 0)

    def test_malformed_evaluation_cannot_become_observed_success(self):
        valid = {'observedSuccess': True, 'modelStateSha256': 'a' * 64, 'metrics': {}}
        for changed in ({'observedSuccess': 'false'}, {'modelStateSha256': 'bad'},
                        {'metrics': None}):
            with self.subTest(changed=changed):
                self.backend = EvaluatingCounterBackend()
                self.backend.evaluate = lambda step, check: dict(valid, **changed)
                state = self.run_fake()
                self.assertEqual(state['status'], 'failed')
                self.assertEqual(self.backend.calls, 0)

    def test_eval_start_is_persisted_before_call_and_write_failure_blocks_it(self):
        original = self.backend.evaluate
        def evaluate(step, check):
            self.assertEqual(self.events[-1]['event'], 'state_eval_start')
            return original(step, check)
        self.backend.evaluate = evaluate
        self.run_fake(plan=replace(self.plan, steps=1))
        self.backend = EvaluatingCounterBackend()
        def emit(event):
            if event['event'] == 'state_eval_start':
                raise OSError('synthetic full disk')
            self.emit(event)
        state = self.run_fake(emit=emit)
        self.assertEqual(state['status'], 'failed')
        self.assertEqual(self.backend.evaluations, [])
        self.assertEqual(self.backend.calls, 0)

    def test_legacy_mode_does_not_add_scheduled_evaluations(self):
        plan = Plan('manifest', 'a' * 64, 'weights', 'b' * 64, 'output')
        state = self.run_fake(plan=plan)
        self.assertEqual(state['status'], 'step_limit')
        self.assertEqual(self.backend.evaluations, [])
        self.assertEqual(self.backend.calls, 30)


class DetectionWorkerArtifactTest(unittest.TestCase):
    """Worker publication tests with byte fixtures, never real model weights."""
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.plan = Plan('manifest', 'a' * 64, 'weights', 'b' * 64, str(self.root),
                         mode='detection_heads', roi_sampling='negative_cap_3x',
                         epochs=100, steps=300)
        self.backend = EvaluatingCounterBackend(success_at=0)
        self.backend.torch = type('Runtime', (), {'__version__': '2.14.1'})()
        self.backend.model = MagicMock()
        self.backend.model.state_dict.return_value = {'synthetic': 'no tensors required'}
        self.backend.frozen_before = 'f' * 64
        self.backend.save = MagicMock(side_effect=lambda path, provenance: Path(path).write_bytes(b'mock checkpoint'))
        self.backend.verify_checkpoint = MagicMock(return_value='a' * 64)
        self.clock = FakeClock()
        self.preflight = dict(manifestSha256='a' * 64, frames=list(range(6)), session='mock',
                              weightsSha256='b' * 64, model='mock', purpose='train_development_only',
                              heldOutEvaluation=False)

    def run_worker(self, digest='a' * 64):
        from learning.cv_training import _worker
        with patch('learning.cv_training.preflight', side_effect=lambda plan: copy.deepcopy(self.preflight)), \
             patch('learning.cv_training.TorchBackend', return_value=self.backend), \
             patch('learning.cv_training.time.monotonic', side_effect=self.clock), \
             patch('learning.cv_sanity.state_digest', return_value=digest), \
             patch('socket.create_connection'), patch('socket.socket.connect'), \
             patch('urllib.request.urlopen'):
            # run_loop's default clock was bound at definition; use its explicit
            # injection so this worker exercises the same controllable deadline.
            with patch('learning.cv_training.run_loop', side_effect=lambda b, p, emit, deadline:
                       run_loop(b, p, emit, deadline, clock=self.clock)):
                _worker(self.plan, str(self.root), 900)
        return json.loads((self.root / 'progress.json').read_text())

    def test_observation_becomes_verified_only_after_checkpoint_validation(self):
        state = self.run_worker()
        self.assertEqual(state['status'], 'fit_observed')
        self.assertEqual(state['outcome'], 'fit_verified')
        self.assertTrue(state['verifiedArtifact'])
        self.assertEqual(state['checkpointEvaluationStep'], 0)
        self.assertTrue((self.root / 'checkpoint.pt').exists())
        self.backend.verify_checkpoint.assert_called_once()
        events = [json.loads(line) for line in (self.root / 'events.jsonl').read_text().splitlines()]
        observed = next(row for row in events if row['event'] == 'fit_observed')
        self.assertFalse(observed['verifiedArtifact'])
        self.assertTrue(events[-1]['verifiedArtifact'])

    def test_save_failure_retains_observation_without_claiming_artifact(self):
        def fail_save(path, provenance):
            Path(path).write_bytes(b'partial')
            raise OSError('synthetic save failure')
        self.backend.save.side_effect = fail_save
        state = self.run_worker()
        self.assertEqual(state['status'], 'failed')
        self.assertTrue(state['observedSuccess'])
        self.assertFalse(state['verifiedArtifact'])
        self.assertEqual(state['outcome'], 'incomplete')
        self.assertFalse((self.root / 'checkpoint.pt').exists())
        self.assertTrue((self.root / 'checkpoint.pt.partial').exists())
        self.backend.verify_checkpoint.assert_not_called()

    def test_failed_safe_reload_does_not_publish_final_checkpoint(self):
        self.backend.verify_checkpoint.side_effect = ValueError('synthetic reload failure')
        state = self.run_worker()
        self.assertEqual(state['status'], 'failed')
        self.assertFalse(state['verifiedArtifact'])
        self.assertFalse((self.root / 'checkpoint.pt').exists())
        self.assertTrue((self.root / 'checkpoint.pt.partial').exists())

    def test_checkpoint_state_must_match_successful_evaluation(self):
        state = self.run_worker(digest='c' * 64)
        self.assertEqual(state['status'], 'failed')
        self.assertFalse(state['verifiedArtifact'])
        self.backend.save.assert_not_called()

    def test_deadline_during_save_preserves_partial_and_never_verifies(self):
        def slow_save(path, provenance):
            Path(path).write_bytes(b'partial')
            self.clock.now = 900
        self.backend.save.side_effect = slow_save
        state = self.run_worker()
        self.assertEqual(state['status'], 'time_limit')
        self.assertFalse(state['verifiedArtifact'])
        self.assertFalse((self.root / 'checkpoint.pt').exists())
        self.backend.verify_checkpoint.assert_not_called()

    def test_epoch_cap_between_evaluations_is_verified_but_inconclusive(self):
        self.backend.success_at = None
        self.plan = replace(self.plan, epochs=1)
        state = self.run_worker()
        self.assertEqual((state['status'], state['stepsCompleted']), ('epoch_limit', 3))
        self.assertTrue(state['verifiedArtifact'])
        self.assertFalse(state['observedSuccess'])
        self.assertIsNone(state['checkpointEvaluationStep'])
        self.assertEqual(state['outcome'], 'inconclusive_budget')

    def test_parent_timeout_clears_even_stale_verified_progress(self):
        from learning.cv_training import execute
        output = self.root / 'new-output'
        plan = replace(self.plan, output_dir=str(output))
        plan_path = self.root / 'plan.json'
        plan_path.write_text(json.dumps(dict(schema='motionbrain.cv_training_plan.v1', **asdict(plan))))
        process = MagicMock(exitcode=0)
        def timeout(proc, deadline):
            (output / 'progress.json').write_text(json.dumps(dict(status='fit_observed',
                observedSuccess=True, verifiedArtifact=True, outcome='fit_verified',
                stepIntents=30, stepsCompleted=30)))
            return 'time_limit'
        with patch('learning.cv_training.multiprocessing.get_context') as context, \
             patch('learning.cv_training.supervise', side_effect=timeout):
            context.return_value.Process.return_value = process
            state = execute(plan_path, confirmed=True)
        self.assertEqual(state['status'], 'time_limit')
        self.assertEqual(state['stepsCompleted'], 30)
        self.assertTrue(state['observedSuccess'])
        self.assertFalse(state['verifiedArtifact'])
        self.assertEqual(state['outcome'], 'incomplete')
        process.start.assert_not_called()

    def test_parent_termination_finalizes_only_running_evaluation_rows(self):
        from learning.cv_training import execute
        for reason, exitcode, expected_status, evaluation_status in (
                ('time_limit', -15, 'time_limit', 'interrupted'),
                ('interrupted', -15, 'interrupted', 'interrupted'),
                (None, 1, 'worker_exit', 'failed')):
            with self.subTest(reason=reason, exitcode=exitcode):
                name = reason or 'worker-crash'
                output = self.root / name
                plan_path = self.root / f'{name}.json'
                plan = replace(self.plan, output_dir=str(output))
                plan_path.write_text(json.dumps(dict(
                    schema='motionbrain.cv_training_plan.v1', **asdict(plan))))
                completed = {'step': 0, 'status': 'completed', 'observedSuccess': False,
                             'modelStateSha256': 'a' * 64, 'metrics': {'preserve': True}}
                pending = {'step': 100, 'status': 'not_reached'}
                process = MagicMock(exitcode=exitcode)
                def terminate(proc, deadline):
                    (output / 'progress.json').write_text(json.dumps(dict(
                        status='running', stepIntents=30, stepsCompleted=30,
                        observedSuccess=False, verifiedArtifact=False,
                        evaluations=[completed, {'step': 30, 'status': 'running'}, pending])))
                    return reason
                with patch('learning.cv_training.multiprocessing.get_context') as context, \
                     patch('learning.cv_training.supervise', side_effect=terminate):
                    context.return_value.Process.return_value = process
                    state = execute(plan_path, confirmed=True)
                self.assertEqual(state['status'], expected_status)
                self.assertEqual(state['evaluations'][0], completed)
                self.assertEqual(state['evaluations'][1]['status'], evaluation_status)
                self.assertEqual(state['evaluations'][2], pending)
                self.assertEqual(state['stepsCompleted'], 30)
                self.assertFalse(state['verifiedArtifact'])
                self.assertEqual(json.loads((output / 'run.json').read_text())['evaluations'],
                                 state['evaluations'])
                process.start.assert_not_called()


@unittest.skipUnless(importlib.util.find_spec('torch') is not None,
                     'optional torch runtime not installed')
class DetectionScopeTest(unittest.TestCase):
    """Tiny factory replacement; all optimizer methods are MagicMock objects."""
    def setUp(self):
        import torch
        from torchvision.models.detection._utils import BalancedPositiveNegativeSampler
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        from learning.cv_training import TorchBackend
        self.torch = torch
        class TinyModel(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.backbone = torch.nn.Sequential(torch.nn.Linear(2, 2), torch.nn.BatchNorm2d(2))
                self.rpn = torch.nn.Module()
                self.rpn.head = torch.nn.Linear(2, 2)
                self.rpn.head.register_buffer('frozen_stat', torch.zeros(1))
                self.rpn.fg_bg_sampler = BalancedPositiveNegativeSampler(256, .5)
                self.roi_heads = torch.nn.Module()
                self.roi_heads.box_head = torch.nn.Linear(2, 2)
                self.roi_heads.box_predictor = FastRCNNPredictor(2, 91)
                self.roi_heads.fg_bg_sampler = BalancedPositiveNegativeSampler(512, .25)

            def forward(self, images, targets):
                self.rpn.fg_bg_sampler([t['rpn_labels'] for t in targets])
                self.roi_heads.fg_bg_sampler([t['roi_labels'] for t in targets])
                rpn = sum(p.square().sum() for p in self.rpn.head.parameters())
                box = sum(p.square().sum() for p in self.roi_heads.box_head.parameters())
                predictor = sum(p.square().sum() for p in self.roi_heads.box_predictor.parameters())
                return {'loss_objectness': rpn, 'loss_rpn_box_reg': rpn * .5,
                        'loss_classifier': box + predictor, 'loss_box_reg': predictor * .5}
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        weights = self.root / 'mock-weights.pt'
        weights.write_bytes(b'not actual model weights')
        plan = Plan('manifest', 'a' * 64, str(weights), hashlib.sha256(weights.read_bytes()).hexdigest(),
                    str(self.root / 'output'), mode='detection_heads', roi_sampling='negative_cap_3x')
        frames = [type('Frame', (), {'image_id': i, 'id': f'image-{i}'})() for i in range(6)]
        dataset = type('Dataset', (), {'manifest_sha256': 'a' * 64,
                                      '__len__': lambda self: 6, 'frames': frames})()
        initial = TinyModel().state_dict()
        self.optimizer = MagicMock()
        def mock_sgd(parameters, **kwargs):
            self.optimizer.param_groups = [{'params': list(parameters)}]
            return self.optimizer
        with patch('torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn',
                   side_effect=lambda **kwargs: TinyModel()), \
             patch('torch.load', return_value=initial), \
             patch('learning.cv_tensors.TorchCVDataset', return_value=dataset), \
             patch('torch.optim.SGD', side_effect=mock_sgd), \
             patch('torch.hub.download_url_to_file', side_effect=AssertionError('download forbidden')):
            self.backend = TorchBackend(plan)

    def test_only_detection_heads_are_trainable_and_backbone_is_eval(self):
        prefixes = ('rpn.head.', 'roi_heads.box_head.', 'roi_heads.box_predictor.')
        for name, parameter in self.backend.model.named_parameters():
            self.assertEqual(parameter.requires_grad, name.startswith(prefixes), name)
        self.assertFalse(self.backend.model.backbone.training)
        self.assertTrue(self.backend.model.rpn.training)
        self.assertTrue(self.backend.model.roi_heads.training)
        self.optimizer.step.assert_not_called()

    def test_scope_guard_rejects_optimizer_membership_changes(self):
        self.optimizer.param_groups[0]['params'].append(next(self.backend.model.backbone.parameters()))
        with self.assertRaisesRegex(ValueError, 'optimizer parameter membership'):
            self.backend.check_scope()
        self.optimizer.step.assert_not_called()

    def test_scope_guard_rejects_backbone_training_mode(self):
        self.backend.model.backbone.train()
        with self.assertRaisesRegex(ValueError, 'evaluation mode'):
            self.backend.check_scope()
        self.optimizer.step.assert_not_called()

    def test_scope_guard_rejects_new_trainable_parameter(self):
        self.backend.model.backbone[0].weight.requires_grad_(True)
        with self.assertRaisesRegex(ValueError, 'trainable scope'):
            self.backend.check_scope()
        self.optimizer.step.assert_not_called()

    def test_native_rpn_masks_and_rng_are_preserved(self):
        from torchvision.models.detection._utils import BalancedPositiveNegativeSampler
        t = self.torch
        labels = [t.tensor([1.] * 140 + [0.] * 300 + [-1.] * 8),
                  t.tensor([1.] * 3 + [0.] * 300)]
        t.manual_seed(234)
        expected = BalancedPositiveNegativeSampler(256, .5)(labels)
        rng = t.get_rng_state().clone()
        t.manual_seed(234)
        actual = self.backend.rpn_sampler(labels)
        for group_a, group_b in zip(expected, actual):
            for a, b in zip(group_a, group_b):
                self.assertTrue(t.equal(a, b))
        self.assertTrue(t.equal(rng, t.get_rng_state()))
        self.assertEqual([r['selectedPositive'] for r in self.backend.rpn_sampler.last_stats], [128, 3])
        self.assertEqual([r['selectedNegative'] for r in self.backend.rpn_sampler.last_stats], [128, 253])

    def synthetic_batch(self):
        t = self.torch
        return [None, None], [
            {'image_id': t.tensor([2]), 'rpn_labels': t.tensor([1.] * 3 + [0.] * 300),
             'roi_labels': t.tensor([1, 2] + [0] * 100)},
            {'image_id': t.tensor([0]), 'rpn_labels': t.tensor([1.] + [0.] * 10),
             'roi_labels': t.tensor([2] + [0] * 40)}]

    def test_tiny_backward_records_actual_sampling_and_all_group_gradients(self):
        before = {n: v.clone() for n, v in self.backend.model.state_dict().items()}
        losses = self.backend.loss_and_grad(self.synthetic_batch())
        self.assertEqual(set(losses), {'loss_objectness', 'loss_rpn_box_reg',
                                      'loss_classifier', 'loss_box_reg'})
        diag = self.backend.batch_diagnostics()
        self.assertEqual(diag['imageIds'], ['image-2', 'image-0'])
        self.assertEqual(diag['classificationDenominator'], 12)
        self.assertEqual(diag['rpn']['objectnessDenominator'], 267)
        self.assertEqual(diag['rpn']['boxRegressionDenominator'], 267)
        self.assertEqual([r['id'] for r in diag['rpn']['images']], ['image-2', 'image-0'])
        self.assertEqual(set(diag['gradientNorms']),
                         {'rpn.head.', 'roi_heads.box_head.', 'roi_heads.box_predictor.'})
        self.assertTrue(all(n > 0 for n in diag['gradientNorms'].values()))
        for name, tensor in self.backend.model.state_dict().items():
            self.assertTrue(self.torch.equal(tensor, before[name]))
        self.optimizer.step.assert_not_called()

    def test_missing_rpn_losses_are_rejected_before_backward(self):
        t = self.torch
        with patch.object(self.backend.model, 'forward', return_value={
                'loss_classifier': t.tensor(1., requires_grad=True),
                'loss_box_reg': t.tensor(1., requires_grad=True)}):
            with self.assertRaisesRegex(ValueError, 'four losses'):
                self.backend.loss_and_grad(self.synthetic_batch())
        self.assertTrue(all(p.grad is None for p in self.backend.parameters))
        self.optimizer.step.assert_not_called()

    def test_even_a_buffer_inside_trainable_head_is_frozen(self):
        self.backend.model.rpn.head.frozen_stat.add_(1)
        output = self.root / 'invalid.pt'
        with self.assertRaisesRegex(ValueError, 'frozen'):
            self.backend.save(output, {})
        self.assertFalse(output.exists())
        self.optimizer.step.assert_not_called()

    def test_tiny_checkpoint_safe_reload_detects_changed_tensor(self):
        from learning.cv_sanity import state_digest
        digest = state_digest(self.backend.model.state_dict())
        provenance = {'modelStateSha256': digest, 'purpose': 'synthetic-only'}
        output = self.root / 'tiny.pt'
        self.backend.save(output, provenance)
        self.assertEqual(self.backend.verify_checkpoint(output, provenance), digest)
        saved = self.torch.load(output, weights_only=True, map_location='cpu')
        saved['state_dict']['rpn.head.weight'].add_(1)
        self.torch.save(saved, output)
        with self.assertRaisesRegex(ValueError, 'tensor state mismatch'):
            self.backend.verify_checkpoint(output, provenance)
        self.optimizer.step.assert_not_called()


if __name__ == '__main__':
    unittest.main()
