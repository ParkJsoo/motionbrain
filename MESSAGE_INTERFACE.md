# MotionBrain Message Interface

Phase 3-C 기준으로 ESP32 내부 모션 제어와 상위 호스트 사이의 메시지 경계를 정리한 문서다.

목적은 두 가지다.

- 시리얼과 HTTP에서 같은 의미를 가진 명령을 일관된 이름으로 유지한다.
- Phase 4에서 ROS2 브리지로 옮길 때 필드 재설계 비용을 줄인다.

## 1. 입력 명령 경계

### Serial

직접 모터 구동용 명령과 폐루프 base 명령을 분리한다.

```text
arm
disarm
stop
joint base left 40
joint base right 40
joint base stop
base angle left 45 40
base angle right 30
base stop
sequence add base left 40 angle=45
```

규칙:

- `joint base ...` 는 개방루프 수동 구동이다.
- `base angle ...` 는 센서 기반 상대각 폐루프 구동이다.
- `base stop` 은 현재 base 상대각 제어를 취소하고 base 모터를 정지한다.
- `sequence add base ... angle=...` 는 시퀀스 안에 base 상대각 폐루프 step을 추가한다.

### HTTP

현재 웹 경계는 다음 라우트를 사용한다.

- `POST /command`
- `POST /motor`
- `POST /joint`
- `POST /base`
- `POST /sequence`
- `POST /light`
- `GET /status`
- `GET /events`

base 상대각 제어는 전용 `/base` 경로로 분리한다.

상대각 회전:

```http
POST /base?action=angle&direction=left&degrees=45&percent=40
X-MotionBrain: 1
```

정지:

```http
POST /base?action=stop
X-MotionBrain: 1
```

시퀀스용 base angle step:

```http
POST /sequence?action=add&joint=base&direction=left&speed=40&degrees=45
X-MotionBrain: 1
```

규칙:

- `direction` 은 `left|right`
- `degrees` 는 `3.0 .. 180.0`
- `percent` 는 `1 .. 100`, 생략 시 기본값 `40`
- `/sequence?action=add` 는 `duration` 또는 `degrees` 중 하나를 사용한다.
- `degrees` 는 `joint=base` 일 때만 허용한다.

### Wired Handheld Teleop

handheld remote v1은 discrete command 나열이 아니라 line-delimited teleop frame을 유선 serial/UART로 보낸다.

이 채널은 현재 serial/web command를 대체하지 않고, "이미 `ARMED`인 시스템에 연속 조작 입력을 공급하는 보조 입력 채널"로 취급한다.

권장 구조:

```text
remote
  -> teleop frame (wired UART)
  -> ESP32 teleop adapter
  -> RobotArm / MotorControl
```

v1 frame은 flat JSON 한 줄을 사용한다.

```json
{
  "type": "teleop",
  "ts_ms": 12345,
  "seq": 18,
  "session": 3,
  "deadman": true,
  "reach": 0.42,
  "lift": -0.18,
  "twist": 0.31,
  "grip_open": false,
  "grip_close": true,
  "led_toggle_seq": 2
}
```

필드 의미:

- `type`: 항상 `teleop`
- `ts_ms`: remote 기준 timestamp
- `seq`: frame sequence
- `session`: deadman을 새로 누를 때마다 증가하는 조작 세션 번호
- `deadman`: motion enable hold 상태
- `reach`: `-1.0 .. 1.0`, 앞/뒤 기울이기에서 나온 normalized 축
- `lift`: `-1.0 .. 1.0`, 좌/우 기울이기에서 나온 normalized 축
- `twist`: `-1.0 .. 1.0`, 손잡이 축 비틀기에서 나온 normalized 축
- `grip_open`: 그리퍼 열기 버튼 상태
- `grip_close`: 그리퍼 닫기 버튼 상태
- `led_toggle_seq`: LED toggle rising edge 누적 카운터

규칙:

- `reach/lift/twist`는 remote에서 중립 재설정과 deadzone을 반영한 뒤 normalized 값으로 보낸다.
- `deadman=false`이면 ESP32는 teleop가 제어하던 관절을 즉시 정지한다.
- frame freshness가 timeout을 넘기면 deadman release와 동일하게 즉시 정지한다.
- v1 freshness timeout 시작값은 약 `200ms`다.
- v1 frame rate 시작값은 `20~50Hz` 범위를 쓰고, 초기 구현은 `25Hz` 전후를 권장한다.
- `grip_open`과 `grip_close`가 동시에 true이거나 동시에 false이면 그리퍼는 정지한다.
- `led_toggle_seq`는 level이 아니라 edge counter다. ESP32는 값이 증가했을 때만 `LIGHT_TOGGLE`을 한 번 수행한다.
- `twist`는 v1에서 base open-loop manual control로 해석한다. 즉, `BASE_ANGLE_RUN`이 아니라 base manual jog 성격이다.

## 2. 상태 메시지 경계

`GET /status` 응답은 아래 상위 필드를 유지한다.

```json
{
  "schemaVersion": "phase3.v1",
  "messageType": "status",
  "uptimeMs": 18234,
  "state": "ARMED",
  "motorEnabled": true,
  "motors": {},
  "light": false,
  "sensor": {},
  "baseAngle": {},
  "teleop": {}
}
```

### `sensor`

```json
{
  "connected": true,
  "simulated": false,
  "simulationMode": "OFF",
  "lastUpdateMs": 83,
  "packetsReceived": 120,
  "parseErrors": 0,
  "imuOk": true,
  "rangeOk": true,
  "sourceTimestampMs": 123456,
  "gyroX": 0.23,
  "gyroY": -0.11,
  "gyroZ": 14.62,
  "roll": -2.10,
  "pitch": 1.40,
  "distCm": 42.7,
  "vibe": 1.34,
  "blocked": false,
  "blockReason": "NONE",
  "faultLatched": false,
  "faultReason": "NONE"
}
```

의미:

- `simulated`: 현재 STM32 UART 대신 내부 simulation snapshot을 쓰는지 여부
- `simulationMode`: `OFF|AUTO|FROZEN`
- `lastUpdateMs`: 마지막 센서 패킷 이후 경과 시간
- `blocked`: 현재 모션 차단 여부
- `blockReason`: `NONE|SENSOR_STALE|IMU_FAULT|RANGE_FAULT|OBSTACLE|VIBRATION`

### `baseAngle`

```json
{
  "active": true,
  "direction": "left",
  "targetDeg": 45.0,
  "currentDeg": 18.6,
  "remainingDeg": 26.4,
  "percent": 40,
  "elapsedMs": 1120,
  "timeoutMs": 7500,
  "processedSamples": 18,
  "lastRateDps": 14.62,
  "lastStopReason": "NONE",
  "lastTransitionMs": 8421
}
```

의미:

- `active`: 현재 base 상대각 제어 활성 여부
- `direction`: 현재 목표 회전 방향
- `targetDeg`: 목표 상대각
- `currentDeg`: 시작 시점부터 적분된 현재 상대각 추정치
- `remainingDeg`: 남은 상대각 추정치
- `processedSamples`: 현재 명령 동안 적분에 사용한 샘플 수
- `lastRateDps`: 마지막에 사용한 회전 속도 추정값
- `lastStopReason`: 마지막 종료 이유

### `teleop`

```json
{
  "connected": true,
  "deadman": true,
  "controlActive": true,
  "lastFrameAgeMs": 18,
  "packetsReceived": 231,
  "parseErrors": 0,
  "session": 4,
  "seq": 91,
  "reach": 0.42,
  "lift": -0.18,
  "twist": 0.31,
  "gripOpen": false,
  "gripClose": true,
  "ledToggleSeq": 2,
  "lastStopReason": "NONE"
}
```

의미:

- `connected`: teleop frame freshness timeout 안에 최근 프레임이 있는지 여부
- `deadman`: 최근 frame의 `deadman` 상태
- `controlActive`: 현재 teleop가 실제 모터 출력을 점유 중인지 여부
- `lastFrameAgeMs`: 마지막 teleop frame 이후 경과 시간
- `packetsReceived`: 누적 teleop frame 수
- `parseErrors`: teleop frame 파싱 실패 수
- `session`: 최근 teleop session 번호
- `seq`: 최근 teleop sequence 번호
- `reach`, `lift`, `twist`: 최근 normalized primitive 값
- `gripOpen`, `gripClose`: 최근 teleop 그리퍼 버튼 상태
- `ledToggleSeq`: 최근 LED toggle edge counter
- `lastStopReason`: 마지막 teleop 정지 이유

`lastStopReason` 값:

- `NONE`
- `TARGET_REACHED`
- `TIMEOUT`
- `NO_ROTATION_FEEDBACK`
- `SENSOR_BLOCK`
- `STATE_CHANGED`
- `MANUAL_STOP`
- `OVERRIDDEN`
- `START_FAILED`

## 3. 종료 이유 의미

base 상대각 제어는 다음 종료 이유를 가진다.

- `TARGET_REACHED`: 목표각 허용 오차 내 도달
- `TIMEOUT`: 최대 회전 시간 초과
- `NO_ROTATION_FEEDBACK`: IMU가 base 회전에 함께 움직이지 않거나 잘못된 축을 보고 있어 의미 있는 각속도 피드백이 없음
- `SENSOR_BLOCK`: safety block 발생
- `STATE_CHANGED`: `ARMED` 이탈
- `MANUAL_STOP`: 사용자가 `base stop` 또는 base 정지 명령 수행
- `OVERRIDDEN`: 다른 base 수동 명령이나 시퀀스가 제어를 덮어씀
- `START_FAILED`: base 모터 시작 실패 또는 의존성 부족

## 4. 명령 응답 경계

모든 `POST` 명령 응답은 같은 envelope를 사용한다.

```json
{
  "schemaVersion": "phase3.v1",
  "messageType": "command_result",
  "success": true,
  "commandId": 42,
  "message": "System armed successfully",
  "state": "ARMED",
  "sensorBlocked": false,
  "blockReason": "NONE",
  "faultLatched": false,
  "faultReason": "NONE",
  "baseAngleActive": false,
  "baseAngleReason": "NO_ROTATION_FEEDBACK"
}
```

규칙:

- `commandId` 는 ESP32 내부 실행 단위 식별자다.
- `message` 는 사용자 표시용 짧은 설명이다.
- `state`, `sensorBlocked`, `blockReason`, `faultLatched`, `baseAngleActive` 는 상위 호스트가 후속 행동을 결정하는 데 쓸 수 있는 최소 상태 요약이다.
- 각 라우트는 여기에 추가 필드를 덧붙일 수 있다.
  - 예: `/light` 는 `light`
  - 예: `/sequence` 는 `count`
  - 예: `/base` 는 `baseAngleActive`, `baseAngleReason`

## 5. 에러 응답 경계

유효성 검사 실패나 초기화 실패는 아래 구조를 사용한다.

```json
{
  "schemaVersion": "phase3.v1",
  "messageType": "error",
  "success": false,
  "error": "Missing 'action' parameter",
  "details": "angle",
  "state": "IDLE",
  "sensorBlocked": false,
  "blockReason": "NONE",
  "faultLatched": false,
  "faultReason": "NONE",
  "baseAngleActive": false,
  "baseAngleReason": "NONE"
}
```

규칙:

- `error` 는 짧은 실패 이유다.
- `details` 는 선택적 보조 정보다.
- 에러 응답도 상태 요약을 포함하므로, 상위 호스트는 실패 직후 추가 `/status` 호출 없이도 기본 판단이 가능하다.

## 6. 시퀀스 상태 응답

`GET /sequence` 는 별도 메시지 타입을 사용한다.

```json
{
  "schemaVersion": "phase3.v1",
  "messageType": "sequence_status",
  "state": "ARMED",
  "sensorBlocked": false,
  "blockReason": "NONE",
  "faultLatched": false,
  "faultReason": "NONE",
  "baseAngleActive": false,
  "baseAngleReason": "NONE",
  "sequence": {
    "state": "IDLE",
    "currentStep": 1,
    "totalCount": 0,
    "remainingMs": 0,
    "full": false
  }
}
```

## 7. 이벤트 응답 경계

`GET /events` 는 최근 시스템 이벤트를 oldest-first 배열로 반환한다.

```json
{
  "schemaVersion": "phase3.v1",
  "messageType": "event_list",
  "state": "IDLE",
  "sensorBlocked": false,
  "blockReason": "NONE",
  "faultLatched": false,
  "faultReason": "NONE",
  "baseAngleActive": false,
  "baseAngleReason": "NO_ROTATION_FEEDBACK",
  "count": 3,
  "events": [
    {
      "id": 1,
      "tsMs": 1500,
      "severity": "INFO",
      "category": "system",
      "code": "BOOT_COMPLETE",
      "detail": "IDLE"
    },
    {
      "id": 2,
      "tsMs": 8420,
      "severity": "INFO",
      "category": "base_angle",
      "code": "BASE_ANGLE_START",
      "detail": "dir=left target_deg=20.0 speed_pct=35"
    },
    {
      "id": 3,
      "tsMs": 9950,
      "severity": "WARN",
      "category": "base_angle",
      "code": "BASE_ANGLE_STOP",
      "detail": "reason=NO_ROTATION_FEEDBACK current_deg=0.9 target_deg=20.0 imu not moving with base?"
    }
  ]
}
```

규칙:

- 기본은 최근 전체 이벤트를 반환하고, `limit` 쿼리로 마지막 N개만 잘라 받을 수 있다.
- `severity` 는 `INFO|WARN|ERROR`
- `category` 는 현재 `system|safety|base_angle` 를 사용한다.
- `code` 는 상위 호스트에서 문자열 비교가 가능하도록 안정적인 식별자 이름을 유지한다.
- `detail` 은 짧은 설명 문자열이며, 상위 호스트 UI 표시나 디버그 로그 연결 용도다.

현재 발생 가능한 대표 이벤트:

- `BOOT_COMPLETE`
- `FAULT_CLEARED`
- `BLOCK_CLEARED`
- `BLOCK_CHANGED`
- `EMERGENCY_STOP`
- `EMERGENCY_FAULT`
- `BASE_ANGLE_START`
- `BASE_ANGLE_TARGET_REACHED`
- `BASE_ANGLE_STOP`

## 8. Phase 4로 넘길 때 유지할 약속

- 센서 상태는 `sensor` 객체 안에 계속 둔다.
- 폐루프 base 상태는 `baseAngle` 객체로 분리 유지한다.
- 상위 호스트는 문자열 로그 파싱 대신 `GET /status` 필드를 사용한다.
- 이벤트 스트림은 `GET /events` 구조를 기반으로 확장하고, 종료 이유 문자열은 위 enum 이름을 유지한다.
- `schemaVersion` 과 `messageType` 는 Phase 4에서도 유지한다.
