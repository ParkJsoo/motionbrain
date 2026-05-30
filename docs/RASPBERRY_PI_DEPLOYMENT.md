# Raspberry Pi 배포 운영

[English](RASPBERRY_PI_DEPLOYMENT.en.md)

이 문서는 Raspberry Pi에서 MotionBrain ROS2 bridge를 systemd 서비스로 운영하는
절차다. 실제 Wi-Fi 비밀번호와 `MOTIONBRAIN_HTTP_TOKEN` 값은 repo에 저장하지
않는다.

## 목표

수동 터미널 launch 대신 아래 운영 경계를 만든다.

```text
systemd
  -> /etc/motionbrain/ros-bridge.env
  -> tools/raspi/start_ros_bridge.sh
  -> ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
  -> JSON/typed topics + /joint_states + TF + kinematics + C++ guard + mission state
```

## 사전 조건

Pi에서 ROS2 workspace가 build되어 있어야 한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

## 환경 파일 설치

```bash
sudo mkdir -p /etc/motionbrain
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.env.example \
  /etc/motionbrain/ros-bridge.env
sudo chmod 600 /etc/motionbrain/ros-bridge.env
sudo nano /etc/motionbrain/ros-bridge.env
```

설정해야 하는 값:

- `MOTIONBRAIN_HOST`
- `MOTIONBRAIN_CAMERA_URL`
- `MOTIONBRAIN_HTTP_TOKEN`

DHCP IP가 바뀌면 `.local`이 되는 환경에서는 hostname을 쓰고, Pi에서 `.local`이
불안정하면 router DHCP reservation을 잡은 IP를 쓴다.

## 서비스 설치

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.service \
  /etc/systemd/system/motionbrain-ros-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable motionbrain-ros-bridge.service
sudo systemctl start motionbrain-ros-bridge.service
```

## 상태 확인

```bash
systemctl status motionbrain-ros-bridge.service --no-pager
journalctl -u motionbrain-ros-bridge.service -n 80 --no-pager
```

Health check:

```bash
~/develop/arduino/motionbrain/tools/raspi/check_ros_bridge_health.sh
```

공개용 terminal evidence를 한 번에 남기려면:

```bash
cd ~/develop/arduino/motionbrain
tools/raspi/capture_ros2_evidence.sh
```

기본 모드는 service, health, package/interface/topic inventory, typed topic
sample, JSON compatibility sample만 기록하고 actuator command는 publish하지
않는다. Mission command boundary가 필요할 때만
`CAPTURE_MISSION_BOUNDARY=1`을 사용한다.

기대 결과:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics_typed
OK topic: /motionbrain/control_guard_typed
OK topic: /motionbrain/mission_state_typed
OK status typed sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics typed sample
OK control guard typed sample
OK mission state typed sample
```

## 2026-05-27 Pi 검증 결과

Raspberry Pi 4에서 systemd 배포 경로를 실제로 설치하고 검증했다.

- `/etc/motionbrain/ros-bridge.env` 설치 및 권한 `600` 적용
- `/etc/systemd/system/motionbrain-ros-bridge.service` 설치
- `systemctl enable motionbrain-ros-bridge.service` 성공
- `systemctl restart motionbrain-ros-bridge.service` 후 서비스 상태:
  `active (running)`
- `motionbrain_status_node`와 `motionbrain_joint_state_node`가 systemd
  service cgroup 아래에서 실행됨
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh` 결과:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics
OK status typed sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics sample
```

검증 중 `set -u`가 ROS2 setup 파일의 선택적 환경 변수 참조와 충돌하는 문제가
발견되어 `start_ros_bridge.sh`와 `check_ros_bridge_health.sh`는 ROS setup을
source한 뒤에 `set -u`를 적용하도록 수정했다.

## 2026-05-28 C++ Control Guard 검증 결과

Raspberry Pi 4에서 C++ ROS2 control guard까지 systemd 경로로 검증했다.

- `colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_ros_bridge motionbrain_description`
  성공
- `systemctl daemon-reload` 후 `motionbrain-ros-bridge.service` 재시작
- 서비스 상태: `active (running)`
- service cgroup에 아래 프로세스가 함께 실행됨:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
- `/motionbrain/control_guard` 샘플:
  - `ready=true`
  - `reason=ready`
  - `statusFresh=true`
  - `detectionFresh=true`
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh`가
  `/motionbrain/control_guard` topic과 sample까지 통과

## 2026-05-28 Mission Supervisor 검증 결과

Raspberry Pi 4에서 lightweight mission supervisor까지 systemd 경로로 검증했다.

- `colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description`
  성공
- `motionbrain-ros-bridge.service` 재시작 후 서비스 상태:
  `active (running)`
- service cgroup에 아래 프로세스가 함께 실행됨:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
  - `motionbrain_mission_supervisor`
- `/motionbrain/mission_state` 샘플:
  - `state=IDLE`
  - `reason=idle`
  - guard/status/detection freshness 정보 포함
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh`가
  `/motionbrain/mission_state` topic과 sample까지 통과
- 안전한 command boundary만 확인:
  - `/motionbrain/mission_cmd`에 `start` publish 후 `WAIT_DETECTION`
  - `/motionbrain/mission_cmd`에 `reset` publish 후 `IDLE`

`confirm`은 실제 `/motionbrain/light_cmd_typed`를 publish할 수 있어서 이번 검증에서는
실행하지 않았다. 현재 mission supervisor는 자동 주행이 아니라, 감지/정렬 판단과
작업자 확인 단계를 ROS2 topic으로 구조화하는 포트폴리오용 mission layer다.

## 2026-05-30 Typed Interface Cleanup 검증 결과

Raspberry Pi 4에서 typed guard/mission/kinematics topic 전환을 systemd
경로로 검증했다.

- 커밋: `2874df7 Use typed ROS2 guard and mission topics`
- `/etc/motionbrain/ros-bridge.env`를 현재 Home Wi-Fi IP로 갱신:
  - `MOTIONBRAIN_HOST=192.168.219.110`
  - `MOTIONBRAIN_CAMERA_URL=http://192.168.219.113`
- `motionbrain-ros-bridge.service` 재시작 후 서비스 상태:
  `active (running)`
- service cgroup에 아래 프로세스가 함께 실행됨:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
  - `motionbrain_mission_supervisor`
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh`가 typed topic과
  sample까지 통과:
  - `/motionbrain/status_typed`
  - `/camera/detection_typed`
  - `/joint_states`
  - `/motionbrain/end_effector_pose`
  - `/motionbrain/kinematics_typed`
  - `/motionbrain/control_guard_typed`
  - `/motionbrain/mission_state_typed`
- Public-safe text evidence:
  [docs/evidence/2026-05-30-ros2-typed-systemd.md](evidence/2026-05-30-ros2-typed-systemd.md)

## 2026-05-30 Evidence Helper 검증 결과

Raspberry Pi 4에서 public-safe ROS2 evidence helper를 systemd 서비스가 실행
중인 상태로 검증했다.

- 커밋: `99154d2 Fix ROS2 evidence interface listing`
- 스크립트: `tools/raspi/capture_ros2_evidence.sh`
- 기본 모드에서 actuator command는 publish하지 않음
- `ros2 interface list` 출력의 leading space 때문에 첫 검증에서 interface
  grep이 실패했고, `99154d2`에서 `motionbrain_msgs/msg` 포함 검색으로 수정
- 수정 후 출력 파일:
  `/tmp/motionbrain_ros2_evidence_helper_99154d2.txt`
- 최종 결과: `Result: OK`
- Pi repo 상태: `99154d2`, clean
- `motionbrain-ros-bridge.service`: `active`

## 운영 명령

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl stop motionbrain-ros-bridge.service
sudo systemctl disable motionbrain-ros-bridge.service
```

## 문제 해결

환경 파일 수정 후에는 서비스를 재시작한다.

```bash
sudo systemctl restart motionbrain-ros-bridge.service
```

서비스가 시작되지 않으면:

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
```

토큰 오류는 `/motionbrain/light_result`에서 `HTTP Error 403: Forbidden`으로
보인다. 이 경우 `/etc/motionbrain/ros-bridge.env`의
`MOTIONBRAIN_HTTP_TOKEN`이 ESP32에 provision된 token과 같은지 확인한다.
