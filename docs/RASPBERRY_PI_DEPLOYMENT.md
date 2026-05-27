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
  -> JSON/typed topics + /joint_states + TF + kinematics + C++ control guard
```

## 사전 조건

Pi에서 ROS2 workspace가 build되어 있어야 한다.

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_ros_bridge motionbrain_description
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

기대 결과:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics
OK topic: /motionbrain/control_guard
OK status typed sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics sample
OK control guard sample
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
