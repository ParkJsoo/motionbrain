# MotionBrain Architecture Diagrams

[한국어](ARCHITECTURE_DIAGRAMS.md)

This document provides Mermaid diagrams for portfolio explanation and demo
capture. GitHub renders these diagrams directly in Markdown.

## Full System

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

## ROS2 Demo Path

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

## Safety And Authorization Boundary

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

## Portfolio Talking Points

- Raspberry Pi/ROS2 provides host-side orchestration while ESP32 keeps the
  motor and safety boundary.
- ROS2 commands do not bypass the ESP32 token gate or SafetyGate.
- ESP32-CAM raw stream is used for manual `STREAM` operation, while Pi
  perception is observed through `/camera/detection` and the embedded
  `TRACKED` confirmation view.
- Motion action remains a separate opt-in command from detection.
- STM32 handheld teleop constrains live motion through deadman and frame
  freshness checks.
