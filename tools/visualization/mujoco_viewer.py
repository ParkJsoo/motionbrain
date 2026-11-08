#!/usr/bin/env python3
"""MuJoCo 네이티브 Viewer: 개략 모델·가상 위치 구동기만 사용한다."""
import argparse
import json
from pathlib import Path
import time
import mujoco
import mujoco.viewer

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('model')
a = p.parse_args()
m = mujoco.MjModel.from_xml_path(a.model)
d = mujoco.MjData(m)
# Same illustrative pose as the other viewers; never a hardware estimate.
pose=json.loads(Path(__file__).with_name('display-pose.json').read_text())['joints']
for name, value in pose.items():
    joint=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
    d.qpos[m.jnt_qposadr[joint]]=value
    actuator=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,name)
    d.ctrl[actuator]=value
follower=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'gripper_opposing_joint')
if follower>=0:
    d.qpos[m.jnt_qposadr[follower]]=-pose['gripper_joint']
mujoco.mj_forward(m,d)
with mujoco.viewer.launch_passive(m,d) as viewer:
    viewer.cam.lookat[:]=[.08,0,.12]
    viewer.cam.distance=.6
    viewer.cam.azimuth=135
    viewer.cam.elevation=-25
    while viewer.is_running():
        started=time.monotonic()
        mujoco.mj_step(m,d)
        viewer.sync()
        time.sleep(max(0,m.opt.timestep-(time.monotonic()-started)))
