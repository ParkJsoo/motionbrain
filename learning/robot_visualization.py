"""Offline model conversion and hash-checked fixed-scene evaluation views.

No device transport, training, command selection or cloud upload is provided.
"""
from __future__ import annotations

import hashlib
import html
import shlex
import sys
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def read_json(path):
    return json.loads(Path(path).read_text())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_path(root, relative, digest):
    path = (Path(root) / relative).resolve()
    if not path.is_relative_to(Path(root).resolve()) or sha256(path) != digest:
        raise ValueError('source_path_or_hash_mismatch')
    return path


def load_lift_evaluation(root):
    """Read the fixed_scene_lift.short500 contract only; refuse schema mixing."""
    root = Path(root).resolve()
    model_path = root / 'frozen-short500-model.json'
    model = read_json(model_path)
    evaluation = read_json(root / 'evaluation/result.json')
    if model.get('schema') != 'motionbrain.fixed_scene_lift.short500.v1':
        raise ValueError('unsupported_observation_contract')
    if sha256(model_path) != evaluation['modelSha256']:
        raise ValueError('frozen_model_hash_mismatch')
    cases = []
    for path in sorted(root.glob('*evaluation-*/result.json')):
        result = read_json(path)
        case_root = path.parent
        audited = read_json(case_root / 'audited-row.json')
        if sha256(path) != audited['resultSha256'] or audited['phase'] != 'evaluation':
            raise ValueError('evaluation_receipt_hash_or_phase_mismatch')
        if (audited['before'] != result['beforeFeatures'] or
                audited['after'] != result['afterFeatures'] or
                audited['contractSha256'] != model['contractSha256']):
            raise ValueError('audited_observation_mismatch')
        receipt = result['modelDecision']
        # Decision paths are private and may be absolute, but must stay in this case.
        decision_path = Path(receipt['decisionPath'])
        if decision_path.parent.resolve() != case_root or sha256(decision_path) != receipt['decisionSha256']:
            raise ValueError('prospective_decision_hash_mismatch')
        decision = read_json(decision_path)
        if decision != {k: v for k, v in receipt.items() if k not in ('decisionPath', 'decisionSha256')}:
            raise ValueError('prospective_decision_content_mismatch')
        if decision['modelSha256'] != evaluation['modelSha256'] or decision['features'] != result['beforeFeatures']:
            raise ValueError('decision_model_or_input_mismatch')
        images = {}
        for phase in ('before', 'after'):
            features = result[phase+'Features']
            if features['context'] != model['context'] or features['contractSha256'] != model['contractSha256']:
                raise ValueError('camera_or_observation_contract_changed')
            source = read_json(case_root / (phase+'-measurement-source.json'))
            images[phase] = {}
            for camera in ('external', 'gripper'):
                ref = source[camera]
                if ref['sha256'] != features['sourceSha256'][camera]:
                    raise ValueError('image_feature_hash_mismatch')
                images[phase][camera] = checked_path(case_root, ref['path'], ref['sha256'])
        pulse = result['selectedPulse']
        key = lambda p: tuple(p[k] for k in ('joint', 'direction', 'percent', 'durationMs'))
        predictions = decision['predictions']
        selected = [p for p in predictions if key(p['pulse']) == key(pulse)]
        candidate = [v for v in model['candidates'].values()
                     if np.allclose(v['delta'] + np.array(result['beforeFeatures']['state']), selected[0]['state'])] if len(selected) == 1 else []
        if len(candidate) != 1:
            raise ValueError('unique_saved_candidate_required')
        cases.append(dict(label=f'평가 {len(cases)+1}', status=result['status'],
                          before=result['beforeFeatures']['state'], after=result['afterFeatures']['state'],
                          predicted=selected[0]['state'], commandOnly=candidate[0]['absoluteAfterMean'],
                          candidates=predictions, pulse={k: pulse[k] for k in ('joint','direction','percent','durationMs')},
                          modelSelected=result['modelSelected'], images=images,
                          outputTrace=result['receipt']['outputTrace']))
    if not cases or len(cases) != evaluation['metrics']['evaluated']:
        raise ValueError('evaluation_denominator_mismatch')
    metrics = {name: np.mean([np.abs(np.array(c[field])-c['after']) for c in cases], axis=0).tolist()
               for name, field in [('local_visual_translation','predicted'), ('commandOnly','commandOnly'), ('noChange','before')]}
    for name, value in metrics.items():
        if not np.allclose(value, evaluation['metrics']['meanAbsoluteErrorPx'][name], atol=1e-9, rtol=0):
            raise ValueError('recomputed_metric_mismatch')
    return dict(modelSha256=evaluation['modelSha256'], trainingRows=len(model['trainingIds']),
                evaluated=len(cases), metrics=metrics, cases=cases,
                limitations='별도 조건 일반화·영상의 성능 우위·연속 집기 성공은 미검증')


def public_metrics(data):
    """Allowlisted scalars only: no paths, camera IDs, raw media or credentials."""
    metrics = {'training/rows': data['trainingRows'], 'evaluation/rows': data['evaluated']}
    for name, vector in data['metrics'].items():
        for field, value in zip(('object_rise_px','tool_rise_px','slip_dy_px'), vector):
            metrics[f'mae/{name}/{field}'] = value
    return metrics





def publication_payload(data):
    rows=[[i+1,c['after'][0],c['predicted'][0],abs(c['after'][0]-c['predicted'][0]),c['modelSelected']]
          for i,c in enumerate(data['cases'])]
    return dict(modelSha256=data['modelSha256'],metrics=public_metrics(data),evaluationRows=rows)


def publication_digest(payload):
    raw=json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def publication_matches(data, receipt):
    expected=publication_payload(data)
    return all(receipt.get(k)==v for k,v in expected.items()) and receipt.get('payloadSha256')==publication_digest(expected)


def portal_html(template, python_executable, model_path, *, system=sys.platform):
    launcher=Path(python_executable).with_name('mjpython') if system=='darwin' else Path(python_executable)
    return template.replace('{{MJ_LAUNCHER}}',html.escape(shlex.quote(str(launcher)))).replace(
        '{{MODEL_PATH}}',html.escape(shlex.quote(str(model_path))))


def restricted_wandb_settings(wandb):
    """Disable automatic code, host metadata, console and system-metric uploads."""
    return wandb.Settings(disable_git=True, disable_code=True, disable_job_creation=True,
                          x_disable_meta=True, x_disable_stats=True,
                          x_disable_machine_info=True, x_save_requirements=False, console='off')


def export_evaluation_rerun(data, output, *, sdk=None):
    if sdk is None:
        import rerun as sdk
    from PIL import Image
    import rerun.blueprint as bp
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    rec = sdk.RecordingStream('motionbrain_fixed_evaluation', recording_id=data['modelSha256'])
    rec.save(str(output))
    try:
        rec.send_blueprint(bp.Blueprint(bp.Vertical(
            bp.Horizontal(bp.Spatial2DView(origin='/cameras/external'), bp.Spatial2DView(origin='/cameras/gripper')),
            bp.TimeSeriesView(origin='/evaluation'), bp.TextLogView(origin='/events'))))
        for i, case in enumerate(data['cases']):
            for j, phase in enumerate(('before', 'after')):
                rec.set_time('replay_step', sequence=i*2+j)
                for camera, path in case['images'][phase].items():
                    rec.log(f'cameras/{camera}', sdk.Image(np.asarray(Image.open(path).convert('RGB'))))
                rec.log('events', sdk.TextLog(json.dumps(dict(case=case['label'],phase=phase,
                    pulse=case['pulse'],status=case['status'],modelSelected=case['modelSelected'],
                    note='기록 재생 순번; 실제 시간 간격·두 카메라 동시 촬영을 뜻하지 않음'), ensure_ascii=False)))
                rec.log(f'evaluation/case_{i+1}/predicted_object_rise_px', sdk.Scalars(case['predicted'][0]))
                for n, candidate in enumerate(case['candidates']):
                    rec.log(f'evaluation/case_{i+1}/candidate_{n}_object_rise_px', sdk.Scalars(candidate['state'][0]))
                rec.log(f'evaluation/case_{i+1}/observed_object_rise_px', sdk.Scalars(case[phase][0]))
                if phase == 'after':
                    rec.log(f'evaluation/case_{i+1}/absolute_error_px', sdk.Scalars(abs(case['predicted'][0]-case['after'][0])))
        rec.flush()
    finally:
        rec.disconnect()


def _numbers(text, default='0 0 0'):
    return list(map(float, (text or default).split()))


def _quat(rpy):
    r,p,y = (a/2 for a in rpy)
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def generate_models(urdf, output, *, include_test_object=False):
    """Same URDF geometry in MuJoCo and Gazebo; all physics values are placeholders."""
    output = Path(output)
    root = ET.parse(urdf).getroot()
    links = {e.get('name'): e for e in root.findall('link')}
    joints = root.findall('joint')
    has_mimic = any(j.find('mimic') is not None for j in joints)
    materials = {e.get('name'): e.find('color').get('rgba') for e in root.iter('material') if e.find('color') is not None}
    display_pose = read_json(Path(__file__).resolve().parents[1]/'tools/visualization/display-pose.json')['joints']
    mj = ET.Element('mujoco', model='MotionBrain_uncalibrated')
    ET.SubElement(mj,'compiler',angle='radian')
    ET.SubElement(mj,'option',timestep='0.002',gravity='0 0 -9.81')
    visual = ET.SubElement(mj,'visual'); ET.SubElement(visual,'global',offwidth='1200',offheight='800')
    ET.SubElement(visual,'headlight',ambient='.5 .5 .5',diffuse='.8 .8 .8',specular='.2 .2 .2')
    world = ET.SubElement(mj,'worldbody')
    ET.SubElement(world,'light',pos='0.3 -1 2',dir='0 0 -1',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',name='floor',type='plane',size='1 1 .1',rgba='.72 .75 .78 1')
    ET.SubElement(world,'camera',name='overview',pos='1.1 -1.1 .8',xyaxes='.72 .69 0 -.29 .30 .91')
    actuators=ET.SubElement(mj,'actuator')
    equality=ET.SubElement(mj,'equality')
    def body(parent, link_name, joint=None):
        attrs={'name':link_name,'gravcomp':'1'}
        if joint is not None:
            origin=joint.find('origin')
            if origin is not None:
                attrs['pos']=origin.get('xyz','0 0 0');attrs['quat']=' '.join(map(str,_quat(_numbers(origin.get('rpy')))))
        node=ET.SubElement(parent,'body',attrs)
        if links[link_name].find('visual') is not None or (joint is not None and joint.get('type')=='revolute'):
            # Appearance complexity must not multiply the placeholder mass.
            ET.SubElement(node,'inertial',pos='0 0 0',mass='.1',diaginertia='.001 .001 .001')
        if joint is not None and joint.get('type')=='revolute':
            limit=joint.find('limit')
            ET.SubElement(node,'joint',name=joint.get('name'),type='hinge',axis=joint.find('axis').get('xyz'),
                          range=limit.get('lower')+' '+limit.get('upper'),damping='1',armature='.01')
            mimic=joint.find('mimic')
            if mimic is None:
                ET.SubElement(actuators,'position',name=joint.get('name'),joint=joint.get('name'),kp='30',kv='3')
            else:
                ET.SubElement(equality,'joint',name=joint.get('name')+'_coupling',
                              joint1=joint.get('name'),joint2=mimic.get('joint'),
                              polycoef=f"{mimic.get('offset','0')} {mimic.get('multiplier','1')} 0 0 0")
        for n,v in enumerate(links[link_name].findall('visual')):
            geom=v.find('geometry'); shape=list(geom)[0];origin=v.find('origin')
            a=dict(name=link_name+f'_visual_{n}',type=shape.tag,mass='0',contype='0',conaffinity='0')
            if origin is not None:
                a.update(pos=origin.get('xyz','0 0 0'),quat=' '.join(map(str,_quat(_numbers(origin.get('rpy'))))))
            if shape.tag=='box':a['size']=' '.join(str(x/2) for x in _numbers(shape.get('size')))
            elif shape.tag=='cylinder':a['size']=shape.get('radius')+' '+str(float(shape.get('length'))/2)
            elif shape.tag=='sphere':a['size']=shape.get('radius')
            material=v.find('material');a['rgba']=materials.get(material.get('name'),'.5 .6 .7 1') if material is not None else '.5 .6 .7 1'
            ET.SubElement(node,'geom',a)
        for child in joints:
            if child.find('parent').get('link')==link_name:body(node,child.find('child').get('link'),child)
    body(world,'base_link')
    # Optional engine diagnostic, absent from the robot's normal display scene.
    if include_test_object:
        obj=ET.SubElement(world,'body',name='test_object',pos='.30 .15 .15')
        ET.SubElement(obj,'freejoint');ET.SubElement(obj,'geom',type='box',size='.025 .025 .025',mass='.05',rgba='.2 .8 .5 1')
    ET.indent(mj);ET.ElementTree(mj).write(output/'model.xml',encoding='unicode')
    sdf=ET.Element('sdf',version='1.11');w=ET.SubElement(sdf,'world',name='motionbrain_offline')
    for file,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        plugin=ET.SubElement(w,'plugin',filename='gz-sim-'+file+'-system',name='gz::sim::systems::'+name)
        if file=='physics' and has_mimic:
            ET.SubElement(plugin,'engine',filename='gz-physics-bullet-featherstone-plugin',name='gz::physics::bullet_featherstone::Plugin')
    scene=ET.SubElement(w,'scene');ET.SubElement(scene,'ambient').text='.6 .6 .6 1';ET.SubElement(scene,'background').text='.85 .88 .91 1'
    ET.SubElement(w,'gravity').text='0 0 -9.81'
    light=ET.SubElement(w,'light',name='sun',type='directional');ET.SubElement(light,'pose').text='0 0 3 0 0 0';ET.SubElement(light,'direction').text='-.3 .2 -1'
    ground=ET.SubElement(w,'model',name='ground');ET.SubElement(ground,'static').text='true';g=ET.SubElement(ground,'link',name='ground')
    for kind in ['visual','collision']:
        e=ET.SubElement(g,kind,name=kind+'_ground');geom=ET.SubElement(e,'geometry');plane=ET.SubElement(geom,'plane');ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='4 4'
        if kind=='visual':
            mat=ET.SubElement(e,'material');ET.SubElement(mat,'diffuse').text='.72 .75 .78 1'
    if include_test_object:
        obj=ET.SubElement(w,'model',name='test_object')
        ET.SubElement(obj,'pose').text='.30 .15 .15 0 0 0'
        link=ET.SubElement(obj,'link',name='object')
        inertia=ET.SubElement(link,'inertial');ET.SubElement(inertia,'mass').text='.05'
        tensor=ET.SubElement(inertia,'inertia')
        for key in ['ixx','iyy','izz']:ET.SubElement(tensor,key).text='.000021'
        for kind in ['visual','collision']:
            item=ET.SubElement(link,kind,name=kind+'_object')
            geometry=ET.SubElement(item,'geometry');box=ET.SubElement(geometry,'box');ET.SubElement(box,'size').text='.05 .05 .05'
            if kind=='visual':
                mat=ET.SubElement(item,'material');ET.SubElement(mat,'diffuse').text='.2 .8 .5 1'
    robot=ET.SubElement(w,'model',name='MotionBrain_uncalibrated')
    state_plugin=ET.SubElement(robot,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(state_plugin,'topic').text='/motionbrain/visual_joint_state'
    for name,link in links.items():
        if name=='world':continue
        l=ET.SubElement(robot,'link',name=name)
        ET.SubElement(l,'gravity').text='false'  # Gravity-compensated display, not identified dynamics.
        parent=next((j for j in joints if j.find('child').get('link')==name),None)
        if parent is not None:
            ref=parent.find('parent').get('link');o=parent.find('origin')
            ET.SubElement(l,'pose',relative_to='__model__' if ref=='world' else ref).text=(o.get('xyz','0 0 0')+' '+o.get('rpy','0 0 0')) if o is not None else '0 0 0 0 0 0'
        inertia=ET.SubElement(l,'inertial');ET.SubElement(inertia,'mass').text='.1';tensor=ET.SubElement(inertia,'inertia')
        for key in ['ixx','iyy','izz','ixy','ixz','iyz']:ET.SubElement(tensor,key).text='.001' if key in ['ixx','iyy','izz'] else '0'
        for n,v in enumerate(link.findall('visual')):
            vis=ET.SubElement(l,'visual',name=f'visual{n}');o=v.find('origin')
            if o is not None:ET.SubElement(vis,'pose').text=o.get('xyz','0 0 0')+' '+o.get('rpy','0 0 0')
            geometry=ET.SubElement(vis,'geometry');shape=list(v.find('geometry'))[0];elem=ET.SubElement(geometry,shape.tag)
            for k,value in shape.attrib.items():ET.SubElement(elem,k).text=value
            material=ET.SubElement(vis,'material');mat=v.find('material')
            ET.SubElement(material,'diffuse').text=materials.get(mat.get('name'),'.3 .5 .7 1') if mat is not None else '.3 .5 .7 1'
    for j in joints:
        joint=ET.SubElement(robot,'joint',name=j.get('name'),type=j.get('type'))
        for part in ['parent','child']:ET.SubElement(joint,part).text=j.find(part).get('link')
        ET.SubElement(joint,'pose',relative_to=j.find('child').get('link')).text='0 0 0 0 0 0'
        if j.get('type')=='revolute':
            axis=ET.SubElement(joint,'axis');ET.SubElement(axis,'xyz').text=j.find('axis').get('xyz')
            dynamics=ET.SubElement(axis,'dynamics');ET.SubElement(dynamics,'damping').text='.05'
            lim=ET.SubElement(axis,'limit')
            for k,value in j.find('limit').attrib.items():ET.SubElement(lim,k).text=value
            lim.find('effort').text='10'  # Display servo only; not the hardware/URDF effort contract.
            mimic=j.find('mimic')
            if mimic is not None:
                coupling=ET.SubElement(axis,'mimic',joint=mimic.get('joint'),axis='axis')
                ET.SubElement(coupling,'multiplier').text=mimic.get('multiplier','1')
                ET.SubElement(coupling,'offset').text=mimic.get('offset','0')
                ET.SubElement(coupling,'reference').text='0'
            else:
                plugin=ET.SubElement(robot,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
                ET.SubElement(plugin,'joint_name').text=j.get('name');ET.SubElement(plugin,'p_gain').text='30'
                ET.SubElement(plugin,'d_gain').text='3'
                # This is an uncalibrated display servo, not a PWM motor model.
                ET.SubElement(plugin,'use_velocity_commands').text='true'
                ET.SubElement(plugin,'initial_position').text=str(display_pose.get(j.get('name'),0))
    ET.indent(sdf);ET.ElementTree(sdf).write(output/'world.sdf',encoding='unicode')
