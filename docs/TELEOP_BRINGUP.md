# Wired Handheld Teleop Bring-Up

이 문서는 유선 handheld teleop v1을 실기 확인할 때 필요한 배선, 버튼 매핑, 점검 절차를 정리한다. 공개 README에는 전체 구조와 상태만 남기고, bench bring-up 세부사항은 이 문서를 기준으로 관리한다.

## 구조

유선 handheld remote v1은 STM32에서 `teleop` JSON frame을 만들고, ESP32가 `teleop_adapter`로 수신하는 구조다.

```text
STM32 handheld remote
  -> UART teleop JSON frame
  -> ESP32 Serial1 RX
  -> TeleopAdapter
  -> RobotArm / MotorControl
```

## 현재 배선

- `STM32 USART2 TX = PD5 = D1` -> `ESP32 GPIO34 = Serial1 RX`
- `STM32 GND` -> `ESP32 GND`

ESP32 teleop 수신 기준:

- `Serial1`
- `RX only`
- `GPIO34`
- frame timeout: 약 `200ms`
- 권장 frame rate: 약 `25Hz`

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
4. 센서 허브 없이 단일 STM32 remote만 bench 테스트한다면 ESP32 시리얼에서 `sensor sim healthy` 실행
5. ESP32를 `arm`
6. Deadman을 누른 채 STM32를 중립 자세로 잡기
7. Deadman을 떼고 다시 누르며 새 중립이 잡히는지 확인
8. Deadman을 누른 채 앞/뒤/좌/우/비틀기 입력으로 teleop 반응 확인
9. `/status.teleop` 또는 시리얼 `status`에서 `connected`, `deadman`, `reach`, `lift`, `twist`, `gripOpen`, `gripClose`, `lastStopReason` 확인
10. Deadman release 또는 선 분리 시 `FRAME_TIMEOUT` / `DEADMAN_RELEASE` 정지 확인

## 현재 제약

- 현재 STM32 펌웨어는 `APP_MODE_TELEOP_REMOTE`와 `APP_MODE_SENSOR_BRIDGE` 중 하나로 동작한다.
- 한 개 STM32를 remote 모드로 쓰는 bench에서는 ESP32 sensor bridge가 실제 센서 패킷을 받지 못하므로 `sensor sim healthy`가 필요하다.
- Single-STM32 remote bench에서는 `GY-521`이 active handheld 입력이고, `HC-SR04`는 연결돼 있어도 active safety stream에 올라오지 않는다.
- 최종 실장에서는 `HC-SR04` safety stream을 별도 sensor bridge로 유지하거나, 동등한 본체 safety 입력 채널을 따로 확보해야 한다.
- Teleop mixer 부호와 비중은 로봇팔 초기 자세, 링크 배치, 리모컨을 잡는 방향이 고정된 뒤 튜닝한다.

## Safety Simulation

하드웨어 없이 safety 경로를 빠르게 점검할 때는 아래 simulation 명령을 사용할 수 있다.

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
