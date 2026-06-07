# Raspberry Pi 배포 운영

[English](RASPBERRY_PI_DEPLOYMENT.en.md)

이 문서는 Raspberry Pi에서 MotionBrain ROS2 bridge, perception service,
dashboard를 systemd 서비스로 운영하는 절차다. 실제 Wi-Fi 비밀번호와
`MOTIONBRAIN_HTTP_TOKEN` 값은 repo에 저장하지 않는다.

## 목표

수동 터미널 launch 대신 아래 운영 경계를 만든다.

```text
systemd
  -> motionbrain-ros-bridge.service
     -> /etc/motionbrain/ros-bridge.env
     -> tools/raspi/start_ros_bridge.sh
     -> ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
     -> JSON/typed topics + /joint_states + TF + kinematics + C++ guard + mission state
  -> motionbrain-perception.service
     -> /etc/motionbrain/perception.env
     -> tools/raspi/start_perception_service.sh
     -> ESP32-CAM capture + target detection API
  -> motionbrain-dashboard.service
     -> /etc/motionbrain/dashboard.env
     -> tools/raspi/start_dashboard_service.sh
     -> LAN dashboard at http://<pi-ip>:8765
```

## 사전 조건

Pi에서 ROS2 workspace가 build되어 있어야 한다.
이 문서의 systemd unit은 기본적으로 `motionbrain` 사용자와
`/home/motionbrain/develop/arduino/motionbrain` checkout 경로를 사용한다. 다른
사용자나 경로를 쓰면 unit과 env 파일의 경로를 같이 수정한다.

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
- `MOTIONBRAIN_PERCEPTION_URL` if the same Pi perception service should provide
  `/camera/detection` instead of direct ESP32-CAM polling

DHCP IP가 바뀌면 `.local`이 되는 환경에서는 hostname을 쓰고, Pi에서 `.local`이
불안정하면 router DHCP reservation을 잡은 IP를 쓴다.

객체 인식 또는 tracked camera overlay를 Pi perception service로 운영할 때는
같은 Pi 내부 endpoint인 `MOTIONBRAIN_PERCEPTION_URL=http://127.0.0.1:8766`을
설정한다.
이 값을 비워두면 ROS2 bridge가 기존처럼 `MOTIONBRAIN_CAMERA_URL/capture`를 직접
폴링해서 색상 감지를 수행한다.
다른 LAN host에서 perception API를 직접 호출해야 할 때만
`MOTIONBRAIN_PERCEPTION_HOST=0.0.0.0`으로 공개하고 Pi LAN IP를 사용한다.

## Dashboard / Perception 환경 파일 설치

```bash
sudo mkdir -p /etc/motionbrain
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-perception.env.example \
  /etc/motionbrain/perception.env
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard.env.example \
  /etc/motionbrain/dashboard.env
sudo chmod 600 /etc/motionbrain/perception.env /etc/motionbrain/dashboard.env
sudo nano /etc/motionbrain/perception.env
sudo nano /etc/motionbrain/dashboard.env
```

설정해야 하는 값:

- `MOTIONBRAIN_CAMERA_URL`
- `MOTIONBRAIN_MOTION_HOST`
- `MOTIONBRAIN_HTTP_TOKEN`
- `MOTIONBRAIN_OBJECT_MODEL`
- `MOTIONBRAIN_OBJECT_LABELS`
- `MOTIONBRAIN_OBJECT_TARGET`

기본 구성은 `motionbrain.local`, `motionbrain-cam.local`,
`motionbrain-pi.local`을 우선 사용한다. mDNS가 흔들리면 ROS2 bridge,
dashboard, perception service wrapper가 같은 LAN의 `/status` endpoint를 스캔해서
controller와 ESP32-CAM의 현재 IP를 자동으로 찾는다. 또한 reconcile timer가 1분마다
dashboard/perception/ROS2 bridge가 현재 발견된 장치 IP와 맞는지 확인하고, Pi가
켜진 상태에서 ESP32를 껐다 켜 IP가 바뀐 경우 필요한 서비스를 재시작해 다시
붙인다. 따라서 ESP32와 ESP32-CAM을 매일 껐다 켜도 env 파일이나 RViz bridge
입력값을 매번 수정하지 않는다.

perception API는 Pi 내부 `127.0.0.1:8766`에만 bind하고, dashboard만 LAN에
`0.0.0.0:8765`로 공개한다. 브라우저에서는 `http://motionbrain-pi.local:8765`를
열고, mDNS가 안 잡히는 환경에서만 router DNS나 `http://<pi-ip>:8765`를 쓴다.
Mac/browser에서 `.local`이 공인 IP로 잘못 해석되면 dashboard 자체는 Pi IP로 열고,
ESP32 Control의 `API` 필드도 Pi IP 또는 router DNS로 맞춘다. Control `STREAM`은
Pi dashboard `/api/config`의 현재 camera URL을 읽어 기본 `motionbrain-cam.local`
값을 자동 보정한다.

discovery fallback을 끄려면 `/etc/motionbrain/ros-bridge.env`,
`/etc/motionbrain/perception.env`, `/etc/motionbrain/dashboard.env`에
`MOTIONBRAIN_DISCOVERY=0`을 설정한다. 특정 subnet만 스캔하려면
`MOTIONBRAIN_DISCOVERY_CIDR=192.168.219.0/24`처럼 지정한다.

현재 cup known-object 데모에서는 `MOTIONBRAIN_OBJECT_TARGET=cup`,
`MOTIONBRAIN_OBJECT_MIN_CONFIDENCE=0.25`, `MOTIONBRAIN_DISPLAY_HOLD_SECONDS=1.5`
를 기준으로 한다. 또한 ESP32-CAM은 `MOTIONBRAIN_CAMERA_FRAMESIZE=qvga`,
`MOTIONBRAIN_CAMERA_QUALITY=4`로 맞춘다. service wrapper는 시작 시 이 profile을
적용하고, reconcile timer는 ESP32-CAM 재부팅으로 profile이 초기화되면 다시 적용한
뒤 dashboard/perception을 재시작한다. 다른 카메라를 쓸 때는
`MOTIONBRAIN_CAMERA_PROFILE=0`으로 끈다. 현재 구도에서 흰 컵이 인접 COCO label로
흔들릴 때만 known-mislabel alias를 추가한다. 기본 env example처럼 alias는 비워두는
것이 기준이다.

## ROS2 Bridge 서비스 설치

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.service \
  /etc/systemd/system/motionbrain-ros-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable motionbrain-ros-bridge.service
sudo systemctl start motionbrain-ros-bridge.service
```

## Dashboard / Perception 서비스 설치

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-perception.service \
  /etc/systemd/system/motionbrain-perception.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard.service \
  /etc/systemd/system/motionbrain-dashboard.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard-reconcile.service \
  /etc/systemd/system/motionbrain-dashboard-reconcile.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard-reconcile.timer \
  /etc/systemd/system/motionbrain-dashboard-reconcile.timer
sudo systemctl daemon-reload
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-dashboard-reconcile.timer
```

`motionbrain-dashboard.service`는 `motionbrain-perception.service` 뒤에 시작된다.
perception이 일시 실패해도 dashboard는 재시작 정책으로 복구를 기다린다.

## 상태 확인

```bash
systemctl status motionbrain-ros-bridge.service --no-pager
systemctl status motionbrain-perception.service --no-pager
systemctl status motionbrain-dashboard.service --no-pager
systemctl status motionbrain-dashboard-reconcile.timer --no-pager
journalctl -u motionbrain-ros-bridge.service -n 80 --no-pager
journalctl -u motionbrain-perception.service -n 80 --no-pager
journalctl -u motionbrain-dashboard.service -n 80 --no-pager
journalctl -u motionbrain-dashboard-reconcile.service -n 80 --no-pager
```

ROS2 health check:

```bash
~/develop/arduino/motionbrain/tools/raspi/check_ros_bridge_health.sh
```

Dashboard/perception health check:

```bash
CHECK_SERVICE=1 ~/develop/arduino/motionbrain/tools/raspi/check_dashboard_health.sh
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

- `colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description`
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
  `/motionbrain/control_guard` topic과 sample까지 통과. 카메라가 실제로
  사용 가능한 상태인지까지 실패 조건으로 보려면 `STRICT_CAMERA_AVAILABLE=1`을
  추가한다.

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

## 2026-06-04 Pi Dashboard / Perception 검증 결과

Raspberry Pi에서 dashboard와 perception service를 ROS2 bridge와 별도
운영 프로세스로 실행해 현재 camera-mode split을 확인했다.

- Controller: `192.168.219.111`
- ESP32-CAM: `192.168.219.113`
- Raspberry Pi: `192.168.219.114`
- ESP32-CAM profile: `qvga`, JPEG quality `4`
- Perception service: Pi port `8766`, object mode, OpenCV DNN YOLOv5s,
  target `cup`, 당시 confidence gate `0.5`, display hold `1.5s`
- Dashboard: Pi port `8765`, `--perception-url http://127.0.0.1:8766`
- Result: dashboard `/api/detection`이 그 구도에서 당시 설정된 threshold 이상으로
  `label=cup`을 반환했다.
- Browser check: `motionbrain.local`, controller IP page, and
  `http://192.168.219.114:8765` were opened and visible to the operator.

This validates the current operating split: `STREAM` for responsive manual
camera feedback, `TRACKED` for slower fixed/slow-target recognition checks.

## 운영 명령

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl restart motionbrain-perception.service
sudo systemctl restart motionbrain-dashboard.service
sudo systemctl restart motionbrain-dashboard-reconcile.timer
sudo systemctl stop motionbrain-ros-bridge.service
sudo systemctl stop motionbrain-perception.service
sudo systemctl stop motionbrain-dashboard.service
sudo systemctl stop motionbrain-dashboard-reconcile.timer
sudo systemctl disable motionbrain-ros-bridge.service
sudo systemctl disable motionbrain-perception.service
sudo systemctl disable motionbrain-dashboard.service
sudo systemctl disable motionbrain-dashboard-reconcile.timer
```

## 문제 해결

환경 파일 수정 후에는 서비스를 재시작한다.

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl restart motionbrain-perception.service
sudo systemctl restart motionbrain-dashboard.service
sudo systemctl restart motionbrain-dashboard-reconcile.timer
```

서비스가 시작되지 않으면:

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
journalctl -u motionbrain-perception.service -n 120 --no-pager
journalctl -u motionbrain-dashboard.service -n 120 --no-pager
journalctl -u motionbrain-dashboard-reconcile.service -n 120 --no-pager
```

토큰 오류는 `/motionbrain/light_result`에서 `HTTP Error 403: Forbidden`으로
보인다. 이 경우 `/etc/motionbrain/ros-bridge.env`의
`MOTIONBRAIN_HTTP_TOKEN`과 `/etc/motionbrain/dashboard.env`의
`MOTIONBRAIN_HTTP_TOKEN`이 ESP32에 provision된 token과 같은지 확인한다.
ESP32 쪽 token만 바꿔 Pi env와 맞추려면 controller serial monitor에서
`wifi token <new-command-token>`을 실행한다. 이 명령은 Wi-Fi SSID/password를
지우지 않고 NVS token만 갱신한 뒤 controller를 재부팅한다.
