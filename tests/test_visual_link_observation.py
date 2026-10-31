"""Synthetic causal geometry/quality tests; no private images or hardware."""
import copy
import json
import math
import unittest
from unittest.mock import patch

from learning.visual_link_observation import CausalLinkObserver, GROUPS, projected_relative_angle


def fixture():
    centres = [(10,10),(150,10),(30,100),(130,100),(40,200),(140,200)]
    groups, features = {}, []
    for group, (x,y) in zip(GROUPS, centres):
        groups[group] = []
        for i,(dx,dy) in enumerate([(-2,-1),(2,-1),(0,2)]):
            name=f'{group}-{i}';groups[group].append(name)
            features.append(dict(id=name,x=x+dx,y=y+dy,visible=True,confidence=1.))
    return groups,features


def rotate(features, prefix, degrees, origin=(40,200)):
    result=copy.deepcopy(features);angle=math.radians(degrees)
    c,s=math.cos(angle),math.sin(angle)
    for f in result:
        if f['id'].startswith(prefix):
            x,y=f['x']-origin[0],f['y']-origin[1]
            f['x'],f['y']=origin[0]+c*x-s*y,origin[1]+s*x+c*y
    return result


class LinkObservationTest(unittest.TestCase):
    def setUp(self):
        self.groups,self.features=fixture()

    def observer(self, **kwargs):
        return CausalLinkObserver(self.groups,**kwargs)

    def test_relative_geometry_translation_rotation_and_wrap(self):
        self.assertEqual(projected_relative_angle((0,0),(1,0),(0,0),(0,1)),90)
        def vector(degrees):
            return math.cos(math.radians(degrees)),math.sin(math.radians(degrees))
        self.assertAlmostEqual(projected_relative_angle((0,0),vector(179),(0,0),vector(-179)),2)
        self.assertEqual(projected_relative_angle((10,10),(10,11),(20,20),(19,20)),90)
        with self.assertRaises(ValueError):projected_relative_angle((0,0),(0,0),(0,0),(1,0))

    def test_child_rotation_is_measured_without_parent_fixed_assumption(self):
        o=self.observer();o.update(0,self.features)
        frame=rotate(self.features,'child_',12)
        for f in frame:
            if not f['id'].startswith('background_'):
                f['x']+=30;f['y']+=15
        r=o.update(.1,frame)
        self.assertTrue(r['valid']);self.assertAlmostEqual(r['relativeDeltaDeg'],12)

    def test_common_link_rotation_cancels(self):
        o=self.observer();o.update(0,self.features)
        frame=rotate(rotate(self.features,'child_',20,(30,100)),'parent_',20,(30,100))
        r=o.update(.1,frame)
        self.assertTrue(r['valid']);self.assertAlmostEqual(r['relativeDeltaDeg'],0)

    def test_future_loss_never_changes_prefix_and_does_not_reacquire(self):
        left,right=self.observer(),self.observer()
        prefix_a=[];prefix_b=[]
        for i in range(7):
            prefix_a.append(left.update(i*.125,self.features))
            prefix_b.append(right.update(i*.125,self.features))
        saved=json.dumps(prefix_a,sort_keys=True)
        missing=copy.deepcopy(self.features);missing[-1].update(visible=False,x=None,y=None,confidence=0)
        r=left.update(.875,missing)
        self.assertEqual(r['reason'],'fixed_point_missing_or_unreliable')
        self.assertIsNone(r['relativeAngleDeg'])
        self.assertFalse(left.update(1.,self.features)['valid'])
        self.assertTrue(right.update(.875,self.features)['valid'])
        self.assertEqual(prefix_a,prefix_b);self.assertEqual(saved,json.dumps(prefix_a,sort_keys=True))

    def test_initial_missing_point_cannot_start_new_cohort_implicitly(self):
        o=self.observer();self.assertFalse(o.update(0,self.features[:-1])['valid'])
        self.assertFalse(o.update(.1,self.features)['valid'])

    def test_low_confidence_and_missing_id_are_invalid(self):
        for mode in ['confidence','missing']:
            frame=copy.deepcopy(self.features)
            if mode=='confidence':frame[-1]['confidence']=.1
            else:frame.pop()
            self.assertEqual(self.observer().update(0,frame)['reason'],'fixed_point_missing_or_unreliable')

    def test_background_is_checked_per_point_not_cancelling_centres(self):
        o=self.observer();o.update(0,self.features)
        frame=copy.deepcopy(self.features);frame[0]['x']+=3;frame[1]['x']-=3
        self.assertEqual(o.update(.1,frame)['reason'],'external_background_motion')

    def test_second_background_patch_is_not_ignored(self):
        o=self.observer();o.update(0,self.features)
        frame=copy.deepcopy(self.features);frame[3]['x']+=3
        self.assertEqual(o.update(.1,frame)['reason'],'external_background_motion')

    def test_distorted_patch_does_not_report_rigid_link_angle(self):
        o=self.observer();o.update(0,self.features)
        frame=copy.deepcopy(self.features);frame[-1]['x']+=10
        r=o.update(.1,frame)
        self.assertEqual(r['reason'],'patch_not_rigid_or_tracking_drift')
        self.assertIsNone(r['relativeAngleDeg'])

    def test_separated_rigid_patches_still_check_link_length(self):
        o=self.observer();o.update(0,self.features)
        frame=copy.deepcopy(self.features)
        for f in frame:
            if f['id'].startswith('child_distal'):f['x']+=20
        self.assertEqual(o.update(.1,frame)['reason'],'projected_link_length_changed')

    def test_stationary_dwell_needs_elapsed_time_and_six_samples(self):
        o=self.observer()
        for i in range(6):self.assertFalse(o.update(i*.125,self.features)['stableWindow'])
        r=o.update(.75,self.features)
        self.assertTrue(r['stableWindow']);self.assertEqual(r['windowStart'],0);self.assertEqual(r['windowEnd'],.75)
        self.assertFalse(r['trainingEligible']);self.assertFalse(r['controlEligible'])
        short=self.observer()
        for i in range(7):self.assertFalse(short.update(i*.01,self.features)['stableWindow'])
        sparse=self.observer(max_frame_gap_seconds=.4)
        for t in (0,.25,.5,.75):self.assertFalse(sparse.update(t,self.features)['stableWindow'])

    def test_rotation_does_not_cancel_window_stability(self):
        o=self.observer()
        for i in range(9):
            r=o.update(i*.125,rotate(self.features,'child_',i))
        self.assertTrue(r['valid']);self.assertFalse(r['stableWindow'])
        self.assertGreater(r['patchTemporalDiameterP90Px']['child_distal'],5)

    def test_motion_of_only_one_patch_is_not_diluted_by_background(self):
        o=self.observer()
        for i in range(9):
            frame=copy.deepcopy(self.features)
            for f in frame:
                if f['id'].startswith('child_distal'):f['y']+=i*.2
            r=o.update(i*.125,frame)
        self.assertTrue(r['valid']);self.assertFalse(r['stableWindow'])

    def test_slow_cumulative_drift_is_not_stationary(self):
        o=self.observer()
        for i in range(9):
            frame=copy.deepcopy(self.features)
            for f in frame:
                if f['id'].startswith('child_'):f['x']+=i*.2
            r=o.update(i*.125,frame)
        self.assertFalse(r['stableWindow'])

    def test_later_motion_revokes_current_stability_not_previous_record(self):
        o=self.observer()
        for i in range(7):r=o.update(i*.125,self.features)
        before=copy.deepcopy(r);self.assertTrue(r['stableWindow'])
        current=o.update(.875,rotate(self.features,'child_',2))
        self.assertFalse(current['stableWindow']);self.assertEqual(r,before)

    def test_time_gap_duplicate_regression_and_nonfinite_latch(self):
        for time,reason in [(0,'frame_time_repeated_or_regressed'),(-1,'frame_time_repeated_or_regressed'),(.251,'frame_gap_exceeded'),(float('nan'),'finite_time_required'),(True,'finite_time_required')]:
            with self.subTest(time=time):
                o=self.observer();o.update(0,self.features)
                r=o.update(time,self.features);self.assertEqual(r['reason'],reason)
                self.assertFalse(o.update(.1,self.features)['valid'])
                json.dumps(r,allow_nan=False)

    def test_high_frame_rate_has_explicit_window_capacity_failure(self):
        o=self.observer(max_window_frames=6)
        for i in range(6):self.assertTrue(o.update(i*.01,self.features)['valid'])
        r=o.update(.06,self.features)
        self.assertEqual(r['reason'],'observation_window_capacity_exceeded')
        self.assertIsNone(r['relativeAngleDeg'])

    def test_duplicate_unknown_and_nonfinite_feature_rules(self):
        for mode in ['duplicate','nan','boolean','invisible_coordinate']:
            frame=copy.deepcopy(self.features)
            if mode=='duplicate':frame.append(copy.deepcopy(frame[0]))
            if mode=='nan':frame[0]['x']=float('nan')
            if mode=='boolean':frame[0]['x']=True
            if mode=='invisible_coordinate':frame[0]['visible']=False
            self.assertFalse(self.observer().update(0,frame)['valid'])
        extra=copy.deepcopy(self.features);extra.append(dict(id='unused',x=0,y=0,visible=True,confidence=1))
        self.assertTrue(self.observer().update(0,extra)['valid'])

    def test_configuration_is_copied_and_groups_must_be_disjoint(self):
        o=self.observer();self.groups['background_a'].clear()
        self.assertTrue(o.update(0,self.features)['valid'])
        groups,_=fixture();groups['background_a'][0]=groups['background_b'][0]
        with self.assertRaises(ValueError):CausalLinkObserver(groups)
        for bad in [None,{},dict(background_a=['a'])]:
            with self.assertRaises(ValueError):CausalLinkObserver(bad)

    def test_duplicate_physical_points_and_overflowing_coordinates_fail(self):
        frame=copy.deepcopy(self.features);frame[1]['x']=frame[0]['x'];frame[1]['y']=frame[0]['y']
        self.assertEqual(self.observer().update(0,frame)['reason'],'distinct_patch_points_required')
        frame=copy.deepcopy(self.features);frame[0]['x']=1e308
        self.assertEqual(self.observer().update(0,frame)['reason'],'feature_coordinates_out_of_range')

    def test_huge_json_integers_latch_instead_of_overflowing(self):
        for field in ('at','x','confidence'):
            with self.subTest(field=field):
                o=self.observer();o.update(0,self.features)
                frame=copy.deepcopy(self.features);at=.1
                if field=='at':at=10**400
                else:frame[0][field]=10**400
                result=o.update(at,frame)
                self.assertFalse(result['valid']);self.assertIsNotNone(result['reason'])
                self.assertFalse(o.update(.2,self.features)['valid'])
        with self.assertRaises(ValueError):self.observer(max_background_motion_px=10**400)

    def test_invalid_settings_and_short_links_are_rejected(self):
        for kwargs in [dict(max_background_motion_px=0),dict(max_patch_deformation_px=float('nan')),dict(max_link_length_change=1),dict(min_confidence=2),dict(max_window_frames=True),dict(max_frame_gap_seconds=1.)]:
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):self.observer(**kwargs)
        self.assertEqual(self.observer(min_link_length_px=101).update(0,self.features)['reason'],'projected_link_too_short')

    def test_standalone_computation_does_not_open_network(self):
        with patch('socket.socket',side_effect=AssertionError('network forbidden')):
            self.assertTrue(self.observer().update(0,self.features)['valid'])


if __name__=='__main__':unittest.main()
