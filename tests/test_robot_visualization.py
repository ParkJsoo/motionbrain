"""Viewer exports preserve evidence, evaluation denominators, and model geometry."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import cv2

from learning.robot_visualization import generate_models, load_lift_evaluation, public_metrics, portal_html, publication_payload, publication_digest, publication_matches

ROOT=Path(__file__).resolve().parents[1]


def save(path,value):
    path.write_text(json.dumps(value,allow_nan=False))
    return hashlib.sha256(path.read_bytes()).hexdigest()


class EvaluationVisualizationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        context={'view':'test-only'}
        model={'schema':'motionbrain.fixed_scene_lift.short500.v1','contractSha256':'a'*64,
               'context':context,'trainingIds':['train0','train1','train2','train3'],
               'candidates':{'0':{'delta':[0,.1,0],'absoluteAfterMean':[1,2.1,0]},
                             '1':{'delta':[2,1,0],'absoluteAfterMean':[3,3,0]}}}
        model_hash=save(self.root/'frozen-short500-model.json',model)
        for i,(percent,duration,after) in enumerate([(16,200,[1,2.3,0]),(50,500,[4,3.2,0])]):
            case=self.root/f'm4-evaluation-{i}';case.mkdir()
            features={}
            for phase,state in [('before',[1,2,0]),('after',after)]:
                refs={}
                for camera in ['external','gripper']:
                    path=case/f'{phase}-{camera}.png';cv2.imwrite(str(path),np.full((4,4,3),(i*20,30,50),dtype=np.uint8))
                    refs[camera]={'path':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
                features[phase]={'context':context,'contractSha256':'a'*64,'state':state,
                                 'sourceSha256':{k:v['sha256'] for k,v in refs.items()}}
                save(case/f'{phase}-measurement-source.json',refs)
            pulses=[{'joint':'shoulder','direction':'up','percent':p,'durationMs':d} for p,d in [(16,200),(50,500)]]
            decision={'features':features['before'],'modelSha256':model_hash,
                      'predictions':[{'pulse':pulses[0],'state':[1,2.1,0]},{'pulse':pulses[1],'state':[3,3,0]}]}
            digest=save(case/'decision.json',decision)
            result={'status':'completed','beforeFeatures':features['before'],'afterFeatures':features['after'],
                    'selectedPulse':pulses[i],'modelSelected':False,
                    'modelDecision':dict(decision,decisionPath=str(case/'decision.json'),decisionSha256=digest),
                    'receipt':{'outputTrace':[]}}
            result_hash=save(case/'result.json',result)
            save(case/'audited-row.json',{'resultSha256':result_hash,'phase':'evaluation',
                 'before':features['before'],'after':features['after'],'contractSha256':'a'*64})
        (self.root/'evaluation').mkdir()
        self.evaluation={'modelSha256':model_hash,'metrics':{'evaluated':2,'meanAbsoluteErrorPx':{
            'local_visual_translation':[.5,.2,0],'commandOnly':[.5,.2,0],'noChange':[1.5,.75,0]}}}
        save(self.root/'evaluation/result.json',self.evaluation)

    def test_only_verified_scalars_reach_publishable_preview(self):
        data=load_lift_evaluation(self.root)
        self.assertEqual(2,data['evaluated'])
        self.assertAlmostEqual(.5,data['metrics']['local_visual_translation'][0])
        metrics=public_metrics(data)
        self.assertTrue(all(type(v) in (int,float) for v in metrics.values()))
        self.assertNotIn(str(self.root),json.dumps(metrics))
        self.assertEqual(2,metrics['evaluation/rows'])

    def test_publication_receipt_is_bound_to_model_metrics_and_cases(self):
        import copy
        data=load_lift_evaluation(self.root)
        payload=publication_payload(data)
        receipt=dict(payload,payloadSha256=publication_digest(payload))
        self.assertTrue(publication_matches(data,receipt))
        for field in ['modelSha256','metrics','evaluationRows']:
            stale=copy.deepcopy(receipt)
            if field=='modelSha256':stale[field]='b'*64
            elif field=='metrics':stale[field]['evaluation/rows']=3
            else:stale[field][0][1]=99
            self.assertFalse(publication_matches(data,stale),field)

    def test_changed_source_image_is_refused(self):
        (self.root/'m4-evaluation-0/before-external.png').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'source_path_or_hash_mismatch'):
            load_lift_evaluation(self.root)

    def test_changed_prospective_prediction_is_refused(self):
        save(self.root/'m4-evaluation-0/decision.json',{})
        with self.assertRaisesRegex(ValueError,'prospective_decision_hash_mismatch'):
            load_lift_evaluation(self.root)

    def test_omitted_case_cannot_silently_improve_denominator(self):
        self.evaluation['metrics']['evaluated']=3
        save(self.root/'evaluation/result.json',self.evaluation)
        with self.assertRaisesRegex(ValueError,'evaluation_denominator_mismatch'):
            load_lift_evaluation(self.root)

    def test_stale_summary_metrics_are_refused(self):
        self.evaluation['metrics']['meanAbsoluteErrorPx']['noChange'][0]=.01
        save(self.root/'evaluation/result.json',self.evaluation)
        with self.assertRaisesRegex(ValueError,'recomputed_metric_mismatch'):
            load_lift_evaluation(self.root)


class PortalLauncherTest(unittest.TestCase):
    def test_launcher_and_model_follow_selected_environment_and_output(self):
        import html
        import re
        import shlex
        template=(ROOT/'tools/visualization/index.html').read_text()
        for env in ['.local/venvs/visual-tools/bin/python','.local/venvs/dated environment/bin/python']:
            generated=portal_html(template,env,'.local/evaluation folder/model.xml',system='darwin')
            command=html.unescape(re.search(r'<code>(.*?)</code>',generated).group(1))
            args=shlex.split(command)
            self.assertEqual(str(Path(env).with_name('mjpython')),args[0])
            self.assertEqual('.local/evaluation folder/model.xml',args[-1])
            self.assertNotIn('{{',generated)
        linux=portal_html(template,'/venv/bin/python','/data/model.xml',system='linux')
        self.assertIn('/venv/bin/python tools/visualization/mujoco_viewer.py /data/model.xml',linux)


class SimulationModelTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        generate_models(ROOT/'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf',self.root)

    def test_sdf_names_and_elevation_axes(self):
        sdf=ET.parse(self.root/'world.sdf').getroot()
        robot=sdf.find("world/model[@name='MotionBrain_uncalibrated']")
        for name in ['shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint']:
            self.assertEqual('0 -1 0',robot.find(f"joint[@name='{name}']/axis/xyz").text)
        self.assertIsNotNone(robot.find("link[@name='tool0']"))
        ground=sdf.find("world/model[@name='ground']/link")
        self.assertNotEqual(ground.find('visual').get('name'),ground.find('collision').get('name'))

    def test_robot_scene_has_no_unrequested_test_cube_but_diagnostic_can_opt_in(self):
        self.assertIsNone(ET.parse(self.root/'model.xml').find("worldbody/body[@name='test_object']"))
        self.assertIsNone(ET.parse(self.root/'world.sdf').find("world/model[@name='test_object']"))
        (self.root/'diagnostic').mkdir()
        generate_models(ROOT/'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf',
                        self.root/'diagnostic',include_test_object=True)
        self.assertIsNotNone(ET.parse(self.root/'diagnostic/model.xml').find("worldbody/body[@name='test_object']"))
        self.assertIsNotNone(ET.parse(self.root/'diagnostic/world.sdf').find("world/model[@name='test_object']"))

    def test_shared_colors_and_mirrored_jaw_contract_survive_both_exports(self):
        mj=ET.parse(self.root/'model.xml').getroot()
        sdf=ET.parse(self.root/'world.sdf').getroot()
        robot=sdf.find("world/model[@name='MotionBrain_uncalibrated']")
        ring=robot.find("link[@name='base_link']/visual[5]/material/diffuse")
        self.assertEqual(ring.text,'1 0.72 0.015 1')
        self.assertEqual(mj.find(".//geom[@name='base_link_visual_4']").get('rgba'),ring.text)
        self.assertEqual(len(mj.findall('actuator/position')),5)
        self.assertEqual(mj.find('equality/joint').get('polycoef'),'0 -1 0 0 0')
        mimic=robot.find("joint[@name='gripper_opposing_joint']/axis/mimic")
        self.assertEqual(mimic.get('joint'),'gripper_joint')
        self.assertEqual(mimic.find('multiplier').text,'-1')
        self.assertTrue(sdf.find('world/plugin/engine').get('filename').endswith('bullet-featherstone-plugin'))
        self.assertEqual(robot.find("joint[@name='shoulder_pitch_joint']/axis/limit/effort").text,'10')
        self.assertEqual(robot.find("link[@name='upper_arm_link']/gravity").text,'false')
        self.assertEqual(mj.find(".//body[@name='upper_arm_link']").get('gravcomp'),'1')

    @unittest.skipUnless(importlib.util.find_spec('mujoco'),'MuJoCo optional runtime not installed')
    def test_extra_visual_does_not_change_placeholder_mass_or_inertia(self):
        import copy,mujoco
        source=ET.parse(ROOT/'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf')
        link=source.find("link[@name='upper_arm_link']")
        visual=copy.deepcopy(link.find('visual'));visual.set('name','extra_appearance_only');link.append(visual)
        source.write(self.root/'extra.urdf')
        (self.root/'extra').mkdir();generate_models(self.root/'extra.urdf',self.root/'extra')
        a=mujoco.MjModel.from_xml_path(str(self.root/'model.xml'))
        b=mujoco.MjModel.from_xml_path(str(self.root/'extra/model.xml'))
        np.testing.assert_allclose(a.body_mass,b.body_mass,atol=0)
        np.testing.assert_allclose(a.body_inertia,b.body_inertia,atol=0)
        self.assertEqual(b.ngeom,a.ngeom+1)

    @unittest.skipUnless(importlib.util.find_spec('mujoco'),'MuJoCo optional runtime not installed')
    def test_jaw_mimic_coupling_is_enforced_during_simulation(self):
        import mujoco
        m=mujoco.MjModel.from_xml_path(str(self.root/'model.xml'));d=mujoco.MjData(m)
        d.ctrl[m.actuator('gripper_joint').id]=.2
        for _ in range(500):mujoco.mj_step(m,d)
        self.assertAlmostEqual(float(d.joint('gripper_joint').qpos[0]),.2,places=3)
        self.assertAlmostEqual(float(d.joint('gripper_opposing_joint').qpos[0]),-.2,places=3)

    @unittest.skipUnless(importlib.util.find_spec('mujoco'),'MuJoCo optional runtime not installed')
    def test_mujoco_tool_frame_matches_fk_for_mixed_poses_and_opening(self):
        import mujoco
        import sys
        sys.path.insert(0,str(ROOT/'ros2_ws/src/motionbrain_ros_bridge'))
        from motionbrain_ros_bridge.motionbrain_kinematics import JointAngles,forward_kinematics,tool_orientation_quaternion
        m=mujoco.MjModel.from_xml_path(str(self.root/'model.xml'));d=mujoco.MjData(m)
        tool=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'tool0')
        names=['base_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','gripper_joint']
        for values in [[.4,.5,-.3,.2,.6],[-.2,-.4,.6,.1,-.3]]:
            for name,value in zip(names,values):
                joint=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
                d.qpos[m.jnt_qposadr[joint]]=value
            mujoco.mj_forward(m,d)
            pose=forward_kinematics(JointAngles(*values))
            np.testing.assert_allclose(d.xpos[tool],[pose.x_m,pose.y_m,pose.z_m],atol=1e-10)
            q=tool_orientation_quaternion(pose.yaw_rad,pose.pitch_rad)
            np.testing.assert_allclose(d.xquat[tool],[q[3],*q[:3]],atol=1e-10)
        for _ in range(50):mujoco.mj_step(m,d)
        self.assertTrue(np.isfinite(d.qpos).all())


class ViewerServerPrivacyTest(unittest.TestCase):
    def test_get_and_head_cannot_escape_curated_files(self):
        import http.client
        import importlib.util
        import threading
        from http.server import ThreadingHTTPServer
        spec=importlib.util.spec_from_file_location('visual_server',ROOT/'tools/visualization/serve.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'media').mkdir();(root/'index.html').write_text('viewer')
            (root/'wandb-run.json').write_text('private');(root/'media/image.png').write_bytes(b'image')
            (root/'media/escape.png').symlink_to(root/'wandb-run.json')
            server=ThreadingHTTPServer(('127.0.0.1',0),module.make_handler(root))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                for method in ['GET','HEAD']:
                    for path in ['/media/../wandb-run.json','/media/%2e%2e/wandb-run.json',
                                 '/media/escape.png','/media/','/wandb-run.json']:
                        conn=http.client.HTTPConnection('127.0.0.1',server.server_port)
                        conn.request(method,path);response=conn.getresponse()
                        self.assertEqual(404,response.status,(method,path));response.read();conn.close()
                    conn=http.client.HTTPConnection('127.0.0.1',server.server_port)
                    conn.request(method,'/media/image.png');response=conn.getresponse()
                    self.assertEqual(200,response.status);response.read();conn.close()
            finally:server.shutdown();server.server_close();thread.join(timeout=2)


@unittest.skipUnless(importlib.util.find_spec('wandb'),'W&B optional runtime not installed')
class WandbDisclosureTest(unittest.TestCase):
    def test_offline_sdk_does_not_collect_environment_or_dependencies(self):
        import wandb
        from learning.robot_visualization import restricted_wandb_settings
        with tempfile.TemporaryDirectory() as tmp:
            run=wandb.init(project='motionbrain-visualization-test',mode='offline',dir=tmp,
                           settings=restricted_wandb_settings(wandb))
            try:run.log({'synthetic_test_metric':1})
            finally:run.finish()
            for name in ['requirements.txt','wandb-metadata.json','output.log']:
                self.assertFalse(list(Path(tmp).rglob(name)),name)
