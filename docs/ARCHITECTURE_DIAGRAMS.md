# MotionBrain 아키텍처 다이어그램

[English](ARCHITECTURE_DIAGRAMS.en.md)

이 문서는 포트폴리오 설명과 데모 캡처에 사용할 구조 다이어그램이다.
GitHub Markdown에서 Mermaid로 바로 렌더링된다.

## 전체 시스템

```mermaid
flowchart LR
  operator[Operator / Mac / Phone]
  stm32[STM32 handheld\nMPU-6050 + HC-SR04\ndeadman + teleop frame]
  esp32[ESP32 Motion Controller\nSafetyGate + Dispatcher\nRobotArm + EventLog]
  cam[ESP32-CAM\ncapture + stream + camera profile]
  pi[Raspberry Pi 4\nUbuntu 24.04 + ROS2 Jazzy\nperception + dashboard + bridge]
  motors[TB6612FNG x3\n5-axis DC motors]
  light[SearchLight]

  operator -->|dashboard / SSH| pi
  operator -->|HTTP control page\nSTREAM manual camera| esp32
  operator -->|browser loads /stream| cam
  stm32 -->|UART teleop + safety telemetry| esp32
  cam -->|HTTP capture| pi
  pi -->|token-gated HTTP command| esp32
  esp32 -->|PWM / direction| motors
  esp32 -->|/light?action=toggle| light
  esp32 -->|/status /events| pi
```

## ROS2 데모 경로

```mermaid
sequenceDiagram
  participant User as Operator terminal
  participant ROS as Raspberry Pi ROS2
  participant Bridge as motionbrain_ros_bridge
  participant ESP as ESP32 controller
  participant CAM as ESP32-CAM
  participant Light as SearchLight

  User->>ROS: ros2 launch motionbrain_home_wifi.launch.py
  ROS->>Bridge: start polling node
  Bridge->>ESP: GET /status
  ESP-->>Bridge: status JSON
  Bridge-->>ROS: publish /motionbrain/status
  Bridge->>CAM: GET /capture
  CAM-->>Bridge: frame
  Bridge-->>ROS: publish /camera/detection
  Note over Bridge,CAM: If perception_url is set, Bridge consumes Pi perception /api/detection instead.
  User->>ROS: publish /motionbrain/light_cmd "toggle"
  ROS->>Bridge: std_msgs/String
  Bridge->>ESP: POST /light?action=toggle + token
  ESP->>Light: toggle output
  ESP-->>Bridge: command result
  Bridge-->>ROS: publish /motionbrain/light_result
```

## 안전 / 권한 경계

```mermaid
flowchart TD
  cmd[Incoming command\nserial / HTTP / ROS2 bridge]
  token{State-changing HTTP?\nvalid token?}
  state{Controller state\nARMED and safe?}
  fresh{Teleop frame fresh?\ndeadman held?}
  execute[Execute conservative action]
  reject[Reject command\n403 / safety block / stop]
  stop[Motor stop]

  cmd --> token
  token -- no --> reject
  token -- yes or serial bench path --> state
  state -- unsafe --> reject
  state -- safe --> fresh
  fresh -- no --> stop
  fresh -- yes --> execute
```

## 포트폴리오에서 강조할 점

- Raspberry Pi/ROS2는 상위 orchestration 계층이고, ESP32가 motor/safety
  boundary를 계속 소유한다.
- ROS2 command도 ESP32 token gate와 SafetyGate를 우회하지 않는다.
- ESP32-CAM raw stream은 수동 조작용 `STREAM` 경로로 쓰고, Pi perception
  결과는 `/camera/detection`과 embedded `TRACKED` 확인 경로로 관측한다.
- Motion action은 detection과 분리된 별도 opt-in command다.
- STM32 handheld teleop는 deadman과 frame freshness를 통해 live motion을
  제한한다.
