import copy
import unittest
from learning.grasp_geometry import evaluate_geometry,METHOD


class GraspGeometryTests(unittest.TestCase):
    def setUp(self):
        frame=dict(frameId='frame-1',pixelSha256='a'*64,width=320,height=240)
        self.review=dict(frame=frame,annotation=dict(method=METHOD,frameId='frame-1',pixelSha256='a'*64,
            object=dict(bodyBbox=[130,60,200,170],visible=True,uncertaintyPx=2),
            tips={name:dict(point=point,kind='actual_tip',visible=True,uncertaintyPx=3)
                  for name,point in [('jawLeft',[100,100]),('jawRight',[225,105])]}))
    def onsite(self):
        return dict(frameId='frame-1',pixelSha256='a'*64,objectBetweenJaws=True,depth=True,clearance=True)
    def test_2d_alignment_never_confirms_depth_or_contact(self):
        result=evaluate_geometry(self.review)
        self.assertEqual(result['closeEntry'],'needsreview')
        self.assertTrue(result['diagnostic']['bodyCenterXWithinTipXProjection'])
        self.assertFalse(result['depthInferred']);self.assertFalse(result['contactInferred'])
        self.assertFalse(result['graspConfirmed']);self.assertFalse(result['controlEligible'])
        self.assertEqual(result,evaluate_geometry(copy.deepcopy(self.review)))
    def test_only_all_onsite_conditions_bound_to_same_frame_allow_entry(self):
        self.review['onsite']=self.onsite()
        self.assertEqual(evaluate_geometry(self.review)['closeEntry'],'onsite_confirmed')
        for key,value in [('pixelSha256','b'*64),('frameId','other'),('depth',False),('clearance',1),('objectBetweenJaws',None)]:
            changed=copy.deepcopy(self.review);changed['onsite'][key]=value
            self.assertEqual(evaluate_geometry(changed)['closeEntry'],'needsreview')
    def test_hidden_tip_stays_partial_even_with_onsite_confirmation(self):
        self.review['onsite']=self.onsite();item=self.review['annotation']['tips']['jawRight']
        item.update(visible=False,point=None)
        result=evaluate_geometry(self.review)
        self.assertEqual(result['status'],'partial');self.assertEqual(result['closeEntry'],'needsreview')
        self.assertNotIn('tipDistancePx',result['diagnostic'])
    def test_shaft_inferred_points_and_nonmanual_annotations_rejected(self):
        for kind in ['shaft','shaft_crosssection','inferred_tip']:
            changed=copy.deepcopy(self.review);changed['annotation']['tips']['jawLeft']['kind']=kind
            with self.assertRaisesRegex(ValueError,'cannot_replace_actual_tip'):evaluate_geometry(changed)
        self.review['annotation']['method']='registered_gripper_crosssection_points'
        with self.assertRaisesRegex(ValueError,'manual_actual_tip'):evaluate_geometry(self.review)
    def test_frame_bbox_coordinates_and_uncertainty_are_validated(self):
        changed=copy.deepcopy(self.review);changed['annotation']['pixelSha256']='b'*64
        with self.assertRaisesRegex(ValueError,'frame_mismatch'):evaluate_geometry(changed)
        changed=copy.deepcopy(self.review);changed['annotation']['object']['bodyBbox']=[0,0,321,100]
        with self.assertRaisesRegex(ValueError,'bbox_invalid'):evaluate_geometry(changed)
        changed=copy.deepcopy(self.review);changed['annotation']['tips']['jawLeft']['point']=[float('nan'),20]
        with self.assertRaisesRegex(ValueError,'point_invalid'):evaluate_geometry(changed)
        self.review['annotation']['object']['uncertaintyPx']=-1
        with self.assertRaisesRegex(ValueError,'uncertainty'):evaluate_geometry(self.review)

if __name__=='__main__':unittest.main()
