"""합성 상태/명령의 거절·선택·고정 검사. 장치·실제 모델 파일 접근 없음."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from learning import task_action_models as m
from learning import m1_shaft_learning as shaft
from tests.test_m1_shaft_learning import scalar_sample

CONTEXT=dict(object='pink-prism',physicalView='current-fixed',poseRegime='loaded',registrationSha256='a'*64)
UP=dict(joint='shoulder',direction='up',percent=50,durationMs=650)
DOWN=dict(joint='shoulder',direction='down',percent=35,durationMs=400)
OPEN=dict(joint='gripper',direction='open',percent=60,durationMs=650)


def state(rise=0.,angle=196.):
    return dict(objectRisePx=rise,toolRisePx=rise,slipDyPx=0.,startAngleDeg=angle)


def sample(i,command=UP,support='table_supported'):
    down=command['direction']=='down'
    before=state(10.+i if down else float(i),198.+i*.1 if down else 196.+i*.1)
    after={k:v for k,v in before.items()}
    for k in ('objectRisePx','toolRisePx'): after[k]+= -5. if down else 5.
    after['startAngleDeg']+= -1. if down else 1.
    return dict(id=str(i),context=copy.deepcopy(CONTEXT),supportMode=support,command=copy.deepcopy(command),
                before=before,after=after,sourceHash=hashlib.sha256(str(i).encode()).hexdigest(),
                episode=['boot',str(i)],frameIds=[f'{i}-a',f'{i}-b'])


class M4Tests(unittest.TestCase):
    def setUp(self):
        self.rows=[sample(0),sample(1)]
        self.model=m.fit_m4(self.rows,CONTEXT)

    def test_up_action_and_stop_compete(self):
        x=state(.5,196.05); target=state(5.5,197.05)
        result=m.select_m4(self.model,x,target,[UP],CONTEXT,'table_supported')
        self.assertEqual(result['selected']['command'],UP)
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['executionAuthorized'])
        result=m.select_m4(self.model,x,x,[UP],CONTEXT,'table_supported')
        self.assertEqual(result['selected']['kind'],'stop')

    def test_down_is_a_distinct_supported_candidate(self):
        rows=[sample(0,DOWN,'gripper_supported'),sample(1,DOWN,'gripper_supported')]
        model=m.fit_m4(rows,CONTEXT); x=state(10.5,198.05);target=state(5.5,197.05)
        result=m.select_m4(model,x,target,[DOWN],CONTEXT,'gripper_supported')
        self.assertEqual(result['selected']['command'],DOWN)
        rejected=m.select_m4(model,x,target,[DOWN],CONTEXT,'table_supported')
        self.assertEqual(rejected['selected']['kind'],'stop')

    def test_unseen_before_and_command_abstain(self):
        for x,commands in [(state(2.,196.05),[UP]),(state(.5,196.05),[DOWN])]:
            result=m.select_m4(self.model,x,state(10.,197.),commands,CONTEXT,'table_supported')
            self.assertEqual(result['selected']['kind'],'stop');self.assertTrue(result['rejected'])

    def test_context_and_duplicate_candidates_reject(self):
        changed=dict(CONTEXT,registrationSha256='b'*64)
        with self.assertRaises(ValueError):m.select_m4(self.model,state(),state(),[UP],changed,'table_supported')
        with self.assertRaises(ValueError):m.select_m4(self.model,state(),state(),[UP,UP],CONTEXT,'table_supported')

    def test_provenance_reuse_rejects_and_does_not_mutate(self):
        original=copy.deepcopy(self.rows)
        for key in ('id','sourceHash','episode','frameIds'):
            rows=copy.deepcopy(self.rows);rows[1][key]=copy.deepcopy(rows[0][key])
            with self.subTest(key=key),self.assertRaises(ValueError):m.fit_m4(rows,CONTEXT)
        self.assertEqual(original,self.rows)

    def test_two_camera_frame_quartets_are_preserved_and_unique(self):
        rows=copy.deepcopy(self.rows)
        for row in rows:
            row['frameIds']=[f'{row["id"]}-{camera}-{side}' for camera in ('external','gripper') for side in ('before','after')]
        model=m.fit_m4(rows,CONTEXT)
        self.assertEqual(len(model['trainingRows'][0]['frameIds']),4)
        rows[0]['frameIds'][-1]=rows[0]['frameIds'][0]
        with self.assertRaisesRegex(ValueError,'frame_pair'):m.fit_m4(rows,CONTEXT)

    def test_single_row_and_actual_output_inputs_reject(self):
        with self.assertRaises(ValueError):m.fit_m4(self.rows[:1],CONTEXT)
        rows=copy.deepcopy(self.rows);rows[0]['before']['actualPwm']=127
        with self.assertRaises(ValueError):m.fit_m4(rows,CONTEXT)
        rows=copy.deepcopy(self.rows);rows[0]['outputTrace']=[]
        with self.assertRaises(ValueError):m.fit_m4(rows,CONTEXT)

    def test_command_caps_are_not_relaxed(self):
        for command in [dict(UP,percent=51),dict(UP,durationMs=651),dict(DOWN,percent=36),dict(DOWN,durationMs=401)]:
            rows=[sample(0,command),sample(1,command)]
            with self.subTest(command=command),self.assertRaises(ValueError):m.fit_m4(rows,CONTEXT)

    def test_frozen_support_must_be_derived_from_training_rows(self):
        for change in ('range','count','coefficient','missing_rows','source_changed'):
            bad=copy.deepcopy(self.model)
            if change=='range':bad['models'][0]['maximum'][0]+=20
            elif change=='count':bad['models'][0]['count']+=1
            elif change=='coefficient':bad['models'][0]['coefficient'][0][0]+=1
            elif change=='missing_rows':bad.pop('trainingRows')
            else:bad['trainingRows'][0]['before']['objectRisePx']+=.2
            with self.subTest(change=change),self.assertRaises(ValueError):m.validate_m4(bad)

    def test_partial_coverage_does_not_claim_model_or_expand_bounds(self):
        probe=dict(stage='lift',command=UP,supportMode='table_supported',state=state(.5,196.05))
        for count in (0,1,2):
            report=m.coverage_m4(self.rows[:count],CONTEXT,[probe]);p=report['probes'][0]
            self.assertEqual(p['missingRows'],max(0,2-count))
            self.assertEqual(p['modelSupportCovered'],count==2)
            self.assertFalse(report['controlEligible'])
        outside=copy.deepcopy(probe);outside['state']=state(4.,196.05)
        p=m.coverage_m4(self.rows,CONTEXT,[outside])['probes'][0]
        self.assertEqual(p['missingRows'],0);self.assertFalse(p['modelSupportCovered']);self.assertTrue(p['needsStateCoverage'])
        bad=copy.deepcopy(probe);bad['stage']='lower'
        with self.assertRaises(ValueError):m.coverage_m4(self.rows,CONTEXT,[bad])

    def test_frozen_bytes_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'model.json';sha=m.freeze_model(path,self.model)
            self.assertEqual(m.load_frozen(path,sha),self.model)
            with self.assertRaises(FileExistsError):m.freeze_model(path,self.model)
            path.write_bytes(path.read_bytes()+b' ')
            with self.assertRaisesRegex(ValueError,'changed'):m.load_frozen(path,sha)


class OpenTests(unittest.TestCase):
    def setUp(self):
        rows=[scalar_sample('o1','train',68.),scalar_sample('o2','train',70.)]
        for row in rows:
            row['command']=copy.deepcopy(OPEN);row['afterShaftPx']=row['beforeShaftPx']+30.
        self.model=shaft.fit(rows);self.context=self.model['context']

    def choose(self,**kwargs):
        args=dict(table_supported=True,m4_angle_deg=195.55,expected_m4_angle_deg=195.55)
        args.update(kwargs)
        return m.select_open(self.model,69.,100.,[OPEN],self.context,**args)

    def test_open_prediction_and_stop(self):
        self.assertEqual(self.choose()['selected']['command'],OPEN)
        result=m.select_open(self.model,69.,69.,[OPEN],self.context,table_supported=True,m4_angle_deg=195.55,expected_m4_angle_deg=195.55)
        self.assertEqual(result['selected']['kind'],'stop')
        with self.assertRaisesRegex(ValueError,'close_only'):shaft.select(self.model,69.,100.,[OPEN],self.context)

    def test_table_and_pose_guards_remain_separate(self):
        for args in [dict(table_supported=False),dict(m4_angle_deg=195.29),dict(tolerance_deg=1.)]:
            with self.subTest(args=args),self.assertRaises(ValueError):self.choose(**args)

    def test_open_ood_and_bounded_direction(self):
        result=m.select_open(self.model,67.,100.,[OPEN],self.context,table_supported=True,m4_angle_deg=195.55,expected_m4_angle_deg=195.55)
        self.assertEqual(result['selected']['kind'],'stop');self.assertTrue(result['rejected'])
        for command in [dict(OPEN,direction='close'),dict(OPEN,percent=61),dict(OPEN,durationMs=651)]:
            with self.subTest(command=command),self.assertRaises(ValueError):m.select_open(self.model,69.,100.,[command],self.context,table_supported=True,m4_angle_deg=195.55,expected_m4_angle_deg=195.55)


if __name__=='__main__':unittest.main()
