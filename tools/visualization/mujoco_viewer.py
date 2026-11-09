#!/usr/bin/env python3
"""MuJoCo 네이티브 Viewer: 개략 모델·가상 위치 구동기만 사용한다."""
import argparse
import json
from pathlib import Path
import time
import math
import sys
import mujoco
import mujoco.viewer

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('model')
p.add_argument('--pose-file', type=Path, help='마지막 실측 M4를 포함한 표시 전용 JSON; 변경 시 다시 읽음')
a = p.parse_args()
m = mujoco.MjModel.from_xml_path(a.model)
d = mujoco.MjData(m)
pose_file=a.pose_file or Path(__file__).with_name('display-pose.json')

def read_pose():
    record=json.loads(pose_file.read_text())
    expected={'base_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','gripper_joint'}
    if record.get('controlEligible') is not False or set(record['joints'])!=expected or not all(
        type(v) in (int,float) and math.isfinite(v) for v in record['joints'].values()):
        raise ValueError('invalid_display_pose')
    return record

def apply_pose(record):
    pose=record['joints']
    for name,value in pose.items():
        joint=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
        actuator=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,name)
        if joint<0 or actuator<0:raise ValueError('display_joint_missing:'+name)
        d.qpos[m.jnt_qposadr[joint]]=value
        d.qvel[m.jnt_dofadr[joint]]=0
        d.ctrl[actuator]=value
    follower=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'gripper_opposing_joint')
    if follower>=0:
        d.qpos[m.jnt_qposadr[follower]]=-pose['gripper_joint']
        d.qvel[m.jnt_dofadr[follower]]=0
    mujoco.mj_forward(m,d)

record=read_pose();apply_pose(record)
version=pose_file.stat().st_mtime_ns
with mujoco.viewer.launch_passive(m,d) as viewer:
    viewer.cam.lookat[:]=[.055,0,.10]
    viewer.cam.distance=.5
    viewer.cam.azimuth=-90
    viewer.cam.elevation=-5
    checked=0.
    while viewer.is_running():
        started=time.monotonic()
        if started-checked>=.25:
            checked=started
            try:
                current=pose_file.stat().st_mtime_ns
                if current!=version:
                    record=read_pose()
                    with viewer.lock():apply_pose(record)
                    version=current
                measured=record.get('lastMeasuredM4')
                text=(f'Last measured M4 {measured["sensorDeg"]:.2f} deg -> approx {measured["displayJointDeg"]:.2f} deg'
                      if measured else 'Static photo approximation; no measured current joint pose')
                viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    text,'Other joints assumed; no live feedback; physics uncalibrated'))
            except (OSError,ValueError,KeyError,TypeError) as exc:
                print('Display pose update rejected: '+str(exc),file=sys.stderr)
        mujoco.mj_step(m,d)
        viewer.sync()
        time.sleep(max(0,m.opt.timestep-(time.monotonic()-started)))
