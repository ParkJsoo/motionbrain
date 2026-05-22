# Wired Handheld Teleop Bring-Up

이 문서는 유선 handheld teleop v1을 실기 확인할 때 필요한 배선, 버튼 매핑, 점검 절차를 정리한다. 공개 README에는 전체 구조와 상태만 남기고, bench bring-up 세부사항은 이 문서를 기준으로 관리한다.

## 구조

유선 handheld remote v1은 STM32에서 `teleop` JSON frame을 만들고, ESP32가 `teleop_adapter`로 수신하는 구조다. 현재 teleop frame에는 safety telemetry도 같이 포함되어, 단일 STM32 bench 구성에서도 ESP32 safety monitor가 실제 센서 freshness를 확인할 수 있다.

```text
STM32 handheld remote
  MPU-6050 + HC-SR04 + buttons
  -> UART teleop JSON frame with embedded safety telemetry
  -> ESP32 Serial1 RX
  -> TeleopAdapter
  -> SafetyMonitor + RobotArm / MotorControl
```

## 현재 배선

- `STM32 USART2 TX = PD5 = D1` -> `ESP32 GPIO34 = Serial1 RX`
- `STM32 GND` -> `ESP32 GND`
- `HC-SR04 TRIG = PD4 = D2`
- `HC-SR04 ECHO = PC8 = D3`

ESP32 teleop 수신 기준:

- `Serial1`
- `RX only`
- `GPIO34`
- frame timeout: 약 `200ms`
- 권장 frame rate: 약 `25Hz`
- embedded safety stale timeout: 약 `1000ms`

Teleop JSON frame에는 기존 조작 필드와 함께 아래 safety 필드가 포함된다.
STM32는 MPU가 감지되지 않아도 UART heartbeat를 계속 보내며, 이때 `imu_ok=false`로 ESP32 safety가 `IMU_FAULT`를 유지한다. 즉 `teleop.connected=YES`와 `sensor.source=teleop_embedded`는 UART 경로 확인, `imu_ok=YES`는 IMU 경로 확인으로 나누어 본다.

`teleop_embedded` source는 handheld remote 자체에서 오는 값이므로 freshness, `imu_ok`, `range_ok`는 safety gate에 사용하지만, distance threshold 기반 `OBSTACLE`과 vibration latch 기반 `VIBRATION_FAULT`는 적용하지 않는다. 손으로 조작하는 리모컨의 초음파/IMU 값이 로봇 본체의 충돌/진동으로 오인되기 때문이다. 로봇 본체에 별도 safety sensor channel을 붙이면 그 채널에서는 obstacle/vibration safety를 다시 적용한다.

```json
{
  "type": "teleop",
  "deadman": true,
  "reach": 0.0,
  "lift": 0.0,
  "twist": 0.0,
  "imu_ok": true,
  "range_ok": true,
  "dist_cm": 50.0,
  "vibe": 0.0,
  "imu_status": 1,
  "imu_addr": 104,
  "imu_error": 0
}
```

`status`의 `IMU diag` 값:

- `status=1`: MPU ready
- `status=2`: I2C probe fail
- `status=3`: WHO_AM_I read/value fail
- `status=4`: init register write fail
- `status=5`: calibration fail
- `status=6`: runtime read fail
- `addr=0x68` 또는 `0x69`: 마지막으로 확인한 MPU address 후보
- `err`: STM32 HAL I2C error bitmask

## STM32 버튼 매핑

- `PE4 = D10`: `deadman`
- `PB4 = D9`: `LED toggle`
- `PE2 = D13`: `grip open`
- `PE6 = D11`: `grip close`

버튼 배선 기준:

- 각 버튼은 한쪽만 해당 STM32 GPIO로 연결
- 버튼 다른 쪽은 `STM32 GND` 공통 rail로 연결
- 내부 pull-up + active-low 기준
- 미입력은 HIGH, 눌리면 LOW

## 핀 선택 메모

- 위 버튼 핀은 2026-04-27 실기 기준으로 확인한 기준이다.
- 이 보드의 Arduino `A0~A5` 헤더는 `PA0/PA1/PA4/PB0`가 아니다.
- `D2/D3`는 현재 `HC-SR04`가 쓰므로 버튼에서 제외한다.
- `D1`은 teleop UART TX, `D14/D15`는 I2C2라서 버튼에서 제외한다.
- `PE3(D8)`는 deadman 후보에서 제외했다.
- `PE5(D12)`는 grip open 후보에서 제외했다.
- 현재는 이미 쓰는 핀을 피하기 위해 digital header `D9/D10/D11/D13`를 버튼 입력으로 사용한다.
- 실제 handheld 배선이 정해지면 STM32 `main.c` 상단 매크로만 바꾸면 된다.

## Teleop 동작 기준

- Teleop v1은 `ARM/DISARM`을 직접 처리하지 않는다.
- 사용 전 ESP32를 `ARMED` 상태로 올려야 한다.
- `deadman=false`이면 ESP32는 teleop가 제어하던 관절을 즉시 정지한다.
- frame freshness timeout이 지나면 deadman release와 동일하게 정지한다.
- 새 deadman session이 시작되면 STM32 remote의 현재 자세를 중립으로 다시 잡는다.

## 빠른 실기 체크 순서

1. ESP32 펌웨어 업로드 후 `status` 또는 웹 `/status` 확인
2. STM32 teleop remote 펌웨어 업로드
3. `STM32 PD5 -> ESP32 GPIO34`, `GND common` 연결
4. `status`에서 `sensor.source=teleop_embedded`, `sensor.blocked=false` 확인
   - `teleop.connected=YES`인데 `blockReason=IMU_FAULT`이면 UART와 HC-SR04는 살아 있고 MPU-6050 I2C를 확인해야 한다.
   - 현재 STM32 I2C2 기준은 `PB10/D15=SCL`, `PC12/D14=SDA`, 공통 `GND`, MPU 전원 연결이다.
5. ESP32를 `arm`
6. Deadman을 누른 채 STM32를 중립 자세로 잡기
7. Deadman을 떼고 다시 누르며 새 중립이 잡히는지 확인
8. Deadman을 누른 채 앞/뒤/좌/우/비틀기 입력으로 teleop 반응 확인
9. `/status.teleop` 또는 시리얼 `status`에서 `connected`, `deadman`, `reach`, `lift`, `twist`, `gripOpen`, `gripClose`, `lastStopReason` 확인
10. `/status.sensor`에서 `source`, `connected`, `imuOk`, `rangeOk`, `distCm`, `vibe`, `blocked` 확인
11. Deadman release 또는 선 분리 시 `FRAME_TIMEOUT` / `DEADMAN_RELEASE` 정지 확인
12. STM32 UART 선 분리 시 sensor가 `SENSOR_STALE`로 막히는지 확인

## 현재 제약

- 현재 STM32 remote는 teleop와 safety telemetry를 같은 UART frame에 실어 보낸다.
- Bench MVP에서는 단일 STM32로 teleop + safety freshness를 같이 검증한다.
- 최종 제품형 구조에서는 safety sensor channel을 별도 MCU, 별도 UART, 또는 독립 safety node로 분리할 수 있도록 source 표시와 simulation 경로를 유지한다.
- 2026-05-22 실기에서 deadman + IMU tilt가 실제 모터 출력으로 이어지고, deadman release에서 정지하는 것을 확인했다.
- 같은 실기에서 작은 tilt에도 `M4`가 최대 `100%`, `M3`가 약 `85%`까지 올라갔기 때문에 이후 튜닝에서 angle full-scale을 키우고 ESP32 continuous teleop output을 `35%`로 제한했다.
- Teleop mixer 부호와 비중은 로봇팔 초기 자세, 링크 배치, 리모컨을 잡는 방향이 고정된 뒤 추가 튜닝한다.

## Safety Simulation

하드웨어 없이 safety 경로를 빠르게 점검하거나 fault case를 강제로 재현할 때는 아래 simulation 명령을 사용할 수 있다. 실제 teleop+safety frame이 들어오는 상태에서는 simulation 없이도 safety가 clear되어야 한다.

```text
sensor sim healthy
sensor sim obstacle 10
sensor sim vibration 9
sensor sim rotate left 15
sensor sim stale
sensor sim off
```

기본 점검 순서:

1. `sensor sim off`
2. `sensor sim healthy`
3. `status`
4. `sensor sim obstacle 10`
5. `arm`
6. `sensor sim healthy`
7. `arm`
8. `sensor sim vibration 9`
9. `stop`
10. `sensor sim stale`
11. 잠시 대기 후 `status`
12. `sensor sim off`

상태를 더 보기 쉽게 보려면 다른 터미널에서 host watcher를 같이 실행한다.

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```
