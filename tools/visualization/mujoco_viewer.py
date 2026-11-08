#!/usr/bin/env python3
"""MuJoCo 네이티브 Viewer: 개략 모델·가상 위치 구동기만 사용한다."""
import argparse
import time
import mujoco
import mujoco.viewer

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('model')
a = p.parse_args()
m = mujoco.MjModel.from_xml_path(a.model)
d = mujoco.MjData(m)
# A raised display pose, not an estimate of the connected robot.
for name, value in [('shoulder_pitch_joint',.5),('elbow_pitch_joint',-.3)]:
    joint=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
    d.qpos[m.jnt_qposadr[joint]]=value
    actuator=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,name)
    d.ctrl[actuator]=value
mujoco.mj_forward(m,d)
with mujoco.viewer.launch_passive(m,d) as viewer:
    viewer.cam.lookat[:]=[.35,0,.15]
    viewer.cam.distance=1.3
    viewer.cam.azimuth=135
    viewer.cam.elevation=-25
    while viewer.is_running():
        started=time.monotonic()
        mujoco.mj_step(m,d)
        viewer.sync()
        time.sleep(max(0,m.opt.timestep-(time.monotonic()-started)))
