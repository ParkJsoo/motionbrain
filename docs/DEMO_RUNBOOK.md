# MotionBrain 데모 런북

[English](DEMO_RUNBOOK.en.md)

이 문서는 MotionBrain 포트폴리오 증거를 공개용으로 캡처할 때 따르는
실행 절차다. 실제 Wi-Fi 비밀번호, 실제 command token, 라우터 관리자 화면은
문서와 캡처에 남기지 않는다.

## 목표

짧은 데모에서 아래 체인이 실제 하드웨어로 동작함을 보여준다.

```text
STM32 handheld safety/teleop
  -> ESP32 motion controller
  -> ESP32-CAM vision input
  -> Raspberry Pi ROS2 bridge
  -> ROS2 command
  -> 실제 SearchLight 출력
```

성공 경로뿐 아니라 token gate, deadman release 같은 안전/권한 경계도 같이
보여준다.

## 안전 / 공개 원칙

- 실제 Wi-Fi 비밀번호를 보여주지 않는다.
- 실제 `MOTIONBRAIN_HTTP_TOKEN` 값을 보여주지 않는다.
- 라우터 관리자 화면을 녹화하지 않는다.
- live motion command는 보수적으로, 명시적으로 opt-in 된 경우에만 실행한다.
- ROS2 command demo는 `/light?action=toggle`을 사용한다. 모터가 아닌 안전한
  actuator 경로다.
- vision alignment demo는 timed base nudge만 사용한다.
- base-mounted IMU 또는 encoder가 붙기 전까지 `/base?action=angle` 데모는
  하지 않는다.

## 준비물

- ESP32 MotionBrain controller
- STM32 handheld teleop/safety board
- ESP32-CAM
- Ubuntu Server 24.04 + ROS2 Jazzy가 설치된 Raspberry Pi 4
- 로봇팔 하드웨어와 SearchLight
- SSH, 브라우저, 화면 녹화를 할 Mac 또는 operator machine

네트워크 조건:

- ESP32 controller, ESP32-CAM, Raspberry Pi, Mac은 같은 trusted Home Wi-Fi에
  있어야 한다.
- 가능하면 `.local` hostname을 사용한다.
- `.local`이 안 되면 router DHCP lease나 serial log에서 IP를 확인한다.

2026-05-26 실기 검증 때 관측한 DHCP IP:

```text
Raspberry Pi: 192.168.219.105
ESP32 controller: 192.168.219.113
ESP32-CAM: 192.168.219.114
```

2026-05-27 재부팅 후 typed ROS2 message 검증 때 관측한 DHCP IP:

```text
Raspberry Pi: 192.168.219.111
ESP32 controller: 192.168.219.109
ESP32-CAM: 192.168.219.110
```

위 IP는 예시 관측값이며 네트워크 상황에 따라 바뀔 수 있다.

## 사전 점검

Mac에서 controller와 camera 연결을 확인한다.

```bash
ping -c 1 motionbrain.local
ping -c 1 motionbrain-cam.local
```

hostname이 실패하면 IP로 확인한다.

```bash
curl -sS http://<controller-ip>/status
curl -I http://<camera-ip>/capture
```

Pi에서 ROS2와 workspace를 확인한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
printenv ROS_DISTRO
ros2 pkg list | grep motionbrain
```

기대값:

```text
jazzy
motionbrain_msgs
motionbrain_control
motionbrain_mission
motionbrain_ros_bridge
```

패키지가 없으면 빌드한다.

```bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

systemd bridge가 이미 실행 중이면 공개용 텍스트 증거를 한 번에 캡처할 수 있다.
이 스크립트는 기본값으로 health check, topic list, typed topic sample, JSON
compatibility sample만 기록하고 실제 actuator command는 publish하지 않는다.

```bash
cd ~/develop/arduino/motionbrain
tools/raspi/capture_ros2_evidence.sh
```

mission command boundary까지 남기려면 `confirm` 없이 `start`와 `reset`만
publish하는 opt-in 모드를 사용한다.

```bash
cd ~/develop/arduino/motionbrain
CAPTURE_MISSION_BOUNDARY=1 tools/raspi/capture_ros2_evidence.sh
```

출력 파일 기본 위치:

```text
/tmp/motionbrain_ros2_evidence_<timestamp>.txt
```

## 캡처 목록

주말 데모에서는 아래 증거를 짧게 캡처한다.

1. 하드웨어 overview
   - Raspberry Pi
   - ESP32 controller
   - ESP32-CAM
   - STM32 handheld controller
   - 로봇팔과 SearchLight

2. ROS2 bridge 증거
   - `printenv ROS_DISTRO`
   - `ros2 topic list`
   - `/motionbrain/status`
   - `/motionbrain/status_typed`
   - `/camera/detection`
   - `/camera/detection_typed`
   - `/motionbrain/light_cmd`
   - `/motionbrain/light_cmd_typed`
   - `/motionbrain/light_result`
   - `/motionbrain/light_result_typed`
   - `/joint_states`
   - `/motionbrain/end_effector_pose`
   - `/motionbrain/kinematics_typed`
   - `/motionbrain/control_guard_typed`
   - `/motionbrain/mission_state_typed`
   - 선택: RViz RobotModel/TF 화면
   - 실제 SearchLight 점등

3. 안전/권한 증거
   - token이 없거나 틀리면 `HTTP Error 403: Forbidden`
   - deadman release 후 모터 정지
   - 선택: `/status`의 stale/safety block 상태

4. Vision 증거
   - ESP32-CAM capture 또는 dashboard camera view
   - red target detection
   - `LEFT`, `CENTER`, `RIGHT`, `LOST` alignment state
   - 선택: timed nudge와 즉시 stop

## Segment 1: ROS2 Bridge + Light Command

Raspberry Pi SSH terminal 3개를 연다.

### Terminal 1: Bridge 실행

hostname이 동작하면 기본 launch를 사용한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Pi에서 `.local` 해석이 안 되면 IP fallback을 사용한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

기대 로그:

```text
MotionBrain ROS2 bridge polling http://<controller-ip>:80
```

### Terminal 2: Topic 확인

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
```

기대 topic:

```text
/camera/detection
/camera/detection_typed
/joint_states
/motionbrain/control_guard
/motionbrain/end_effector_pose
/motionbrain/events
/motionbrain/events_typed
/motionbrain/kinematics
/motionbrain/kinematics_typed
/motionbrain/mission_cmd
/motionbrain/mission_cmd_typed
/motionbrain/mission_state
/motionbrain/mission_state_typed
/motionbrain/control_guard
/motionbrain/control_guard_typed
/motionbrain/light_cmd
/motionbrain/light_cmd_typed
/motionbrain/light_result
/motionbrain/light_result_typed
/motionbrain/status
/motionbrain/status_typed
```

status 캡처:

```bash
ros2 topic echo /motionbrain/status --once
ros2 topic echo /motionbrain/status_typed --once
ros2 topic echo /motionbrain/kinematics_typed --once
ros2 topic echo /motionbrain/control_guard_typed --once
ros2 topic echo /motionbrain/mission_state_typed --once
```

camera detection 캡처:

```bash
ros2 topic echo /camera/detection --once
ros2 topic echo /camera/detection_typed --once
```

`light_result`는 latched topic이 아니다. command를 보내기 전에 먼저 echo를
켜둔다.

```bash
ros2 topic echo /motionbrain/light_result --once
```

### Terminal 3: Light command publish

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd std_msgs/msg/String "{data: toggle}"
```

typed command 캡처가 필요하면 아래 명령을 사용한다.

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd_typed motionbrain_msgs/msg/LightCommand "{action: toggle}"
```

기대 결과:

- Terminal 2에 `/motionbrain/light_result` JSON payload가 출력된다.
- 실제 SearchLight가 toggle 된다.

## Segment 2: Token Gate 확인

이 segment는 선택 사항이지만 포트폴리오 가치가 높다. ROS2 command가 바로
하드웨어를 움직이는 것이 아니라 ESP32 token gate를 통과해야 한다는 점을
보여준다.

bridge를 멈춘 뒤 `MOTIONBRAIN_HTTP_TOKEN` 없이 또는 잘못된 token으로 다시
launch한다.

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

이후 `/motionbrain/light_result` echo와 `/motionbrain/light_cmd` publish를
동일하게 실행한다.

기대 결과:

```text
HTTP Error 403: Forbidden
```

이 결과는 ROS2 graph와 bridge는 동작하지만, ESP32의 state-changing command
경계가 token으로 보호되고 있음을 의미한다.

## Segment 2-B: URDF / TF / Joint State 확인

ROS2 bridge가 실행 중인 상태에서 다른 terminal에서 robot description launch를
실행한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch motionbrain_description display.launch.py
```

확인:

```bash
ros2 topic echo /joint_states --once
```

desktop 환경에서 RViz2가 있으면:

```bash
ros2 launch motionbrain_description display.launch.py use_rviz:=true
```

캡처 포인트:

- `base_yaw_joint`, `shoulder_pitch_joint`, `elbow_pitch_joint`,
  `wrist_pitch_joint`, `gripper_joint`가 `/joint_states`에 보인다.
- RViz RobotModel이 표시된다.
- TF tree가 `world -> base_link -> ... -> gripper_link` 구조로 보인다.

## Segment 3: ESP32-CAM Vision 증거

Mac에서 실행한다.

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --once
```

기대 log field:

```text
detected=Y
red_ratio=<non-zero>
align=LEFT|CENTER|RIGHT
suggest=base_left|hold|base_right
```

dashboard view가 필요하면:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

브라우저에서 연다.

```text
http://127.0.0.1:8765
```

## Segment 4: Timed Vision Nudge

로봇 주변이 비어 있고 시스템 상태가 안전할 때만 실행한다.

시작 조건:

- 먼저 stop을 보낸다.
- test 직전에 arm 한다.
- red target을 camera view 안에 둔다.
- 낮은 percent와 짧은 nudge duration을 사용한다.

명령:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --align-mode nudge \
  --align-nudge-ms 250 \
  --align-percent 25 \
  --enable-align-action \
  --once
```

기대 결과:

```text
ACTION base.left nudge=250ms success=True stopped=True
```

또는:

```text
ACTION base.right nudge=250ms success=True stopped=True
```

끝난 뒤 controller 상태를 확인한다.

```bash
curl -sS http://<controller-ip>/status
```

데모 종료 상태는 조용해야 한다.

```text
state=IDLE
motors off
fault=false
```

## Segment 5: Teleop Safety 증거

녹화할 장면:

- controller arm
- deadman hold
- 작은 handheld tilt로 보수적인 motion 발생
- deadman release
- robot stop

확인할 status field:

```bash
curl -sS http://<controller-ip>/status
```

관찰 항목:

- `teleop`
- `deadman`
- `sensor`
- `state`
- `faultLatched`

## Troubleshooting

### Mac에서는 `.local`이 되는데 Pi에서는 안 될 때

launch command에 IP fallback을 사용한다.

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

### `light_result`가 출력되지 않을 때

`/motionbrain/light_result`는 latched topic이 아니다. 먼저 echo를 실행하고,
그 다음 `/motionbrain/light_cmd`를 publish한다.

### `403 Forbidden`

token gate가 active인 상태다. bridge launch 전에 아래 값을 설정한다.

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
```

이후 command를 다시 publish한다.

### Camera detection이 느리거나 누락될 때

hardened default를 사용한다.

```bash
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6 \
  --capture-retries 2 \
  --interval 3 \
  --once
```

### 데모 종료 전

robot을 조용한 상태로 돌린다.

```bash
curl -X POST -H "X-MotionBrain: 1" -H "X-MotionBrain-Token: <local-controller-token>" \
  "http://<controller-ip>/command?cmd=stop"
```

상태를 확인한다.

```bash
curl -sS http://<controller-ip>/status
```

## 포트폴리오 캡션 예시

- Raspberry Pi 4가 ROS2 Jazzy host bridge 역할을 수행한다.
- ESP32 motion controller는 real-time motor와 safety boundary를 유지한다.
- ESP32-CAM detection 결과가 JSON/typed ROS2 topic으로 publish된다.
- ROS2 `/motionbrain/light_cmd_typed`가 token-gated ESP32 command path를
  통과한다.
- 실제 SearchLight 점등으로 end-to-end command execution을 증명한다.
- Deadman release와 token rejection으로 safety/authorization boundary를
  보여준다.
