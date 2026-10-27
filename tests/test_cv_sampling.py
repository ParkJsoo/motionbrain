"""Sampling-only tests: synthetic labels, no detector fitting or optimizer."""
import importlib.util
import unittest

from learning.cv_sampling import RecordedROISampler, sampling_settings, selected_counts


class SamplingCountsTest(unittest.TestCase):
    def test_default_and_cap_for_sparse_positive_counts(self):
        for pos,neg in [(7,337),(2,303),(4,325),(2,332),(1,335),(1,327)]:
            self.assertEqual(selected_counts(pos,neg,'default'),(pos,neg))
            self.assertEqual(selected_counts(pos,neg,'negative_cap_3x'),(pos,3*pos))
    def test_caps_shortages_and_negative_only_fallback(self):
        for values,expected in [((200,900),(128,384)),((3,2),(3,2)),((0,90),(0,32)),
                                ((0,3),(0,3)),((200,0),(128,0)),((0,0),(0,0))]:
            self.assertEqual(selected_counts(*values,'negative_cap_3x'),expected)
    def test_invalid_counts_and_policies(self):
        for policy in ('other',None,True,[],{}):
            with self.assertRaises(ValueError):sampling_settings(policy)
        for count in (-1,True,1.2):
            with self.assertRaises(ValueError):selected_counts(count,4,'negative_cap_3x')


@unittest.skipUnless(importlib.util.find_spec('torch') is not None,'optional torch runtime not installed')
class ROISamplerTest(unittest.TestCase):
    def setUp(self):
        import torch
        from torchvision.models.detection._utils import BalancedPositiveNegativeSampler
        self.torch=torch
        self.delegate_type=BalancedPositiveNegativeSampler
    def sampler(self,policy):return RecordedROISampler(self.delegate_type(512,0.25),policy)
    def test_default_preserves_exact_masks_and_rng_state(self):
        t=self.torch
        labels=[t.tensor([1]*200+[2]*200+[0]*800+[-1]*7),t.tensor([2]+[0]*327),
                t.tensor([-1,-1]),t.empty(0,dtype=t.int64)]
        t.manual_seed(87);expected=self.delegate_type(512,0.25)(labels);rng=t.get_rng_state().clone()
        t.manual_seed(87);actual=self.sampler('default')(labels)
        for group1,group2 in zip(expected,actual):
            for a,b in zip(group1,group2):self.assertTrue(t.equal(a,b))
        self.assertTrue(t.equal(t.get_rng_state(),rng))
    def test_capped_masks_keep_all_sparse_positives_and_exclude_ignored(self):
        t=self.torch;s=self.sampler('negative_cap_3x')
        labels=[t.tensor([1]*6+[2]+[0]*337+[-1]*5),t.tensor([2]+[0]*327)]
        pos,neg=s(labels)
        for i,(p,n) in enumerate(zip(pos,neg)):
            self.assertEqual(p.dtype,t.uint8);self.assertEqual(p.shape,labels[i].shape)
            self.assertFalse(bool((p.bool()&n.bool()).any()))
            self.assertTrue(bool((labels[i][p.bool()]>0).all()))
            self.assertTrue(bool((labels[i][n.bool()]==0).all()))
        self.assertEqual([int(x.sum()) for x in pos],[7,1])
        self.assertEqual([int(x.sum()) for x in neg],[21,3])
        self.assertEqual(s.last_stats[0]['selectedPositiveByClass'],{'lip_balm':6,'gripper':1})
        self.assertEqual(s.last_stats[0]['ignored'],5)
    def test_capped_seed_reproducibility_and_positive_limit(self):
        t=self.torch;labels=[t.tensor([1]*200+[2]*200+[0]*800)]
        t.manual_seed(19);a=self.sampler('negative_cap_3x')(labels)
        t.manual_seed(19);b=self.sampler('negative_cap_3x')(labels)
        self.assertTrue(all(t.equal(x[0],y[0]) for x,y in zip(a,b)))
        self.assertEqual([int(g[0].sum()) for g in a],[128,384])
    def test_negative_only_is_bounded_and_empty_ignored_fail_closed(self):
        t=self.torch;s=self.sampler('negative_cap_3x')
        p,n=s([t.zeros(200,dtype=t.int64)])
        self.assertEqual((int(p[0].sum()),int(n[0].sum())),(0,32))
        self.assertTrue(s.last_stats[0]['negativeOnlyFallback'])
        for labels in (t.tensor([-1,-1]),t.empty(0,dtype=t.int64)):
            with self.assertRaisesRegex(ValueError,'no selectable'):s([labels])
            self.assertEqual(s.last_stats,[])
    def test_invalid_labels_and_delegate_configuration(self):
        t=self.torch;s=self.sampler('negative_cap_3x')
        for labels in (t.tensor([1.0]),t.tensor([[1]]),t.tensor([3]),t.tensor([-2])):
            with self.assertRaises(ValueError):s([labels])
        with self.assertRaises(ValueError):RecordedROISampler(self.delegate_type(256,0.25),'default')
    def test_real_roi_assignment_sampler_integration_without_model_forward(self):
        # Exercise torchvision's real GT append/matching/sampling path on boxes only.
        import torch
        from torchvision.models.detection.roi_heads import RoIHeads
        from torchvision.ops import MultiScaleRoIAlign
        roi=RoIHeads(MultiScaleRoIAlign(['0'],7,2),torch.nn.Identity(),torch.nn.Identity(),
                     0.5,0.5,512,0.25,None,0.05,0.5,100)
        roi.fg_bg_sampler=self.sampler('negative_cap_3x')
        proposal=torch.tensor([[30.,30.,40.,40.]]*100)
        gt={'boxes':torch.tensor([[1.,1.,10.,10.],[15.,15.,25.,25.]]),'labels':torch.tensor([1,2])}
        selected,_,labels,regression=roi.select_training_samples([proposal],[gt])
        self.assertEqual(len(selected[0]),8)
        self.assertEqual(int((labels[0]>0).sum()),2)
        self.assertEqual(int((labels[0]==0).sum()),6)
        self.assertTrue(bool(torch.isfinite(regression[0]).all()))


if __name__=='__main__':unittest.main()
