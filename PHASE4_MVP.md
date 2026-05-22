# MotionBrain Phase 4 MVP

Phase 4의 첫 목표는 Raspberry Pi를 바로 사기 전에 Mac을 상위 제어 노드처럼 써서 `camera -> host decision -> MotionBrain command` 경로를 검증하는 것이다.

## MVP 구조

```text
ESP32-CAM
  /capture, /stream
      ->
Mac host script
  camera frame fetch
  MotionBrain /status, /events polling
  safe demo decision
      ->
ESP32 Motion Controller
  Dispatcher + SafetyGate
  /light, /joint, /sequence
```

## 네트워크 기준

- MotionBrain motion controller는 `MotionBrain-AP`를 띄운다.
- ESP32-CAM은 `MotionBrain-AP`에 station으로 접속한다.
- Mac도 `MotionBrain-AP`에 접속한다.
- Phase 4 MVP에서는 Mac host, ESP32-CAM, phone viewer 동시 접속을 위해 MotionBrain AP 최대 client 수를 3으로 둔다.

집/벤치 테스트에서는 Mac Wi-Fi 전환을 피하기 위해 controller와 ESP32-CAM을 기존 집 Wi-Fi에 station으로 붙일 수 있다. 이 모드는 serial monitor에서 입력한 값을 ESP32 NVS flash에 저장해서 사용하며, 실제 SSID/password와 command token은 프로젝트 파일이나 git에 남기지 않는다. 절차는 [docs/HOME_WIFI_MODE.md](docs/HOME_WIFI_MODE.md)를 따른다.

기본값:

| 항목 | 값 |
| ---- | -- |
| MotionBrain SSID | `MotionBrain-AP` |
| MotionBrain password | `motionbrain` |
| MotionBrain IP | `192.168.4.1` |
| ESP32-CAM URL | serial log의 `ESP32-CAM IP` 확인 |

`motionbrain`은 로컬 데모 AP 기본값이며 개인 계정 비밀번호가 아니다. 집 Wi-Fi station 모드에서는 부팅 직후 serial monitor 프롬프트에 SSID/password를 입력한다. 저장된 값을 지우려면 boot prompt에서 `CLEAR`를 입력한다.

## ESP32-CAM 펌웨어

경로:

```bash
firmware/esp32cam
```

빌드:

```bash
pio run -d firmware/esp32cam
```

업로드:

```bash
pio run -d firmware/esp32cam -t upload
```

업로드 보드를 처음 연결한 날에는 포트를 먼저 확인한다.

```bash
ls /dev/cu.usb*
pio device list
```

포트가 자동 선택되지 않거나 다른 ESP32/STM32가 같이 연결돼 있으면 명시한다.

```bash
pio run -d firmware/esp32cam -t upload --upload-port /dev/cu.usbserial-xxxx
```

업로드 후 serial log에서 IP를 확인한다.

```bash
pio device monitor -d firmware/esp32cam -b 115200 -p /dev/cu.usbserial-xxxx
```

확인:

```text
http://<esp32-cam-ip>/status
http://<esp32-cam-ip>/capture
http://<esp32-cam-ip>/stream
```

주의:

- ESP32-CAM은 전원이 약하면 brownout/reboot가 잦다.
- AI Thinker ESP32-CAM pin map 기준이다.
- 업로드 후 serial log에서 IP를 확인한다.

## 실기 체크리스트

### 1. MotionBrain controller 준비

메인 ESP32에는 Phase 4 펌웨어가 올라가 있어야 한다.

```bash
pio run -t upload
```

업로드 후 monitor에서 아래 기준을 확인한다.

```bash
pio device monitor -b 115200 -p /dev/cu.usbserial-1110
```

확인할 로그:

```text
MotionBrain-AP
Max clients: 3
192.168.4.1
BOOT_COMPLETE
```

업로드를 다시 할 때 serial monitor가 포트를 잡고 있으면 `Ctrl+C`로 먼저 종료한다.

### 2. ESP32-CAM 업로드

ESP32-CAM을 `CH340 micro B upload board`에 꽂고 Mac에 연결한다. 새 포트를 확인한 뒤 업로드한다.

```bash
ls /dev/cu.usb*
pio run -d firmware/esp32cam -t upload --upload-port /dev/cu.usbserial-xxxx
```

업로드 성공 후 serial monitor를 열고 IP를 적어둔다.

```bash
pio device monitor -d firmware/esp32cam -b 115200 -p /dev/cu.usbserial-xxxx
```

정상 로그 예:

```text
MotionBrain ESP32-CAM boot
Connecting to MotionBrain-AP...
ESP32-CAM IP: 192.168.4.x
Camera HTTP server ready
```

### 3. Mac Wi-Fi 전환

Mac Wi-Fi를 `MotionBrain-AP`로 바꾼다.

```text
SSID: MotionBrain-AP
password: motionbrain
```

이 상태에서는 인터넷이 끊길 수 있다. 검증이 끝나면 원래 Wi-Fi로 되돌린다.

### 4. HTTP 수동 확인

브라우저에서 ESP32-CAM 응답을 확인한다.

```text
http://<esp32-cam-ip>/status
http://<esp32-cam-ip>/capture
http://<esp32-cam-ip>/stream
```

MotionBrain controller 상태도 확인한다.

```text
http://192.168.4.1/status
http://192.168.4.1/events?limit=5
```

### 5. Host MVP 검증

OpenCV 없이 먼저 카메라 fetch와 MotionBrain status fetch를 같은 루프에서 확인한다.

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --assume-detected --once
```

성공하면 안전한 light command path를 확인한다.

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --assume-detected --enable-action --once
```

마지막으로 색상 감지 경로를 확인한다.

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --detect-color red --once
```

### 5-1. 운영 대시보드 확인

CLI loop 확인 후에는 브라우저 대시보드로 상태, 이벤트, 카메라 capture, 색상 감지, light command 결과를 한 화면에서 확인한다.

```bash
python3 tools/motionbrain_dashboard.py --camera-url http://<esp32-cam-ip>
```

브라우저:

```text
http://127.0.0.1:8765
```

대시보드 확인 항목:

- `Status`: state, safety block/fault, sensor, light 상태
- `Base Angle`: 폐루프 base command 진행 상태와 마지막 stop reason
- `Teleop`: handheld connection, deadman, axes, grip 입력
- `Events`: `GET /events` 기반 최신 시스템 이벤트
- `Camera Detection`: ESP32-CAM capture와 red target detection ratio
- `Action Log`: 대시보드에서 보낸 light / align nudge command 결과
- `Nudge Once`: token이 있고, controller가 `ARMED`, safety clear, target이 `LEFT` 또는 `RIGHT`, base가 idle일 때만 활성화된다. 클릭 시 서버가 최신 camera frame을 다시 검증한 뒤 timed base nudge를 보내고 즉시 stop한다.

### 6. 실패 시 우선 확인

- `ESP32-CAM IP`가 serial log에 찍혔는지 확인한다.
- ESP32-CAM과 Mac이 모두 `MotionBrain-AP`에 붙어 있는지 확인한다.
- `http://192.168.4.1/status`가 Mac에서 열리는지 확인한다.
- `/stream`을 장시간 열어둔 뒤 `/status`나 `/capture`가 timeout되면 ESP32-CAM을 reset하거나 최신 펌웨어를 다시 올린다. 최신 펌웨어는 stream을 시간 제한으로 끊어 HTTP 서버가 회복되게 한다.
- `/capture`가 느리거나 실패하면 `/status`의 `captures`, `captureFailures`, `clientWriteFailures`, `lastCaptureMs`, `maxCaptureMs`, `lastFrameBytes`를 먼저 확인한다.
- 반복 timeout이면 최신 ESP32-CAM 펌웨어를 다시 올린다. vision용 기본 카메라 프로파일은 QVGA/JPEG quality 15이며 host loop 기본값은 timeout 6초, interval 3초, capture retry 2회다.
- 최신 펌웨어에서도 `/capture`가 회복되지 않으면 ESP32-CAM 전원을 다시 연결한다.
- `brownout` 또는 반복 reboot가 보이면 ESP32-CAM 전원 부족으로 본다.
- 업로드가 실패하면 다른 serial monitor가 포트를 잡고 있는지 확인한다.
- `--enable-action`은 `/status.sensor.blocked=false`이고 controller state가 `IDLE` 또는 `ARMED`일 때만 동작한다.

## Host MVP

dry-run:

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --once
```

반복 테스트는 ESP32-CAM HTTP 서버 부담을 줄이기 위해 기본 3초 간격으로 돈다. 필요하면 `--timeout`, `--capture-retries`, `--capture-retry-delay`, `--interval`로 조정한다.

OpenCV 없이 카메라/명령 경로만 검증:

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --assume-detected --once
```

red target 감지 시 서치라이트 toggle:

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --detect-color red --enable-action
```

색상 감지를 쓰려면 host에 OpenCV가 필요하다.

```bash
python3 -m pip install opencv-python numpy
```

기본 action은 안전한 비모션 명령인 `/light?action=toggle`이다. 모터 명령은 카메라/상태/이벤트 경계가 안정된 뒤에 붙인다.

## Vision-Based Alignment MVP

light action 경로가 검증된 뒤의 다음 단계는 `detect -> align`이다. Host는 색상 mask에서 target centroid를 계산하고, frame 중심 대비 normalized horizontal offset을 만든다.

- `alignment=LEFT`: target이 화면 왼쪽에 있어 `commandSuggestion=base_left`
- `alignment=RIGHT`: target이 화면 오른쪽에 있어 `commandSuggestion=base_right`
- `alignment=CENTER`: offset이 deadband 안에 있어 `commandSuggestion=hold`
- `alignment=LOST`: target 없음 또는 frame decode 실패, `commandSuggestion=none`

Detection payload는 기존 호환 필드인 `centroidX`, `centroidY`, `ratio`를 유지하면서, dashboard/ROS2/포트폴리오 설명용 필드인 `centerX`, `centerY`, `areaRatio`, `offsetX`, `offsetY`, `alignment`, `commandSuggestion`을 함께 제공한다.

기본 실행은 dry-run이라 base motor를 움직이지 않는다.

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --detect-color red --once
```

로그 예:

```text
detected=Y red_ratio=0.254 offset_x=+0.32 align=RIGHT suggest=base_right align_allowed=Y
```

실제 base 보정은 명시적으로 켠다. 현재 handheld STM32 telemetry 구성에서는 base-mounted gyro feedback이 없으므로 기본값은 timed nudge다. controller가 `ARMED`, sensor clear, 기존 base angle inactive일 때만 통과하고, host가 `/joint?joint=base&action=<left|right>`를 짧게 보낸 뒤 즉시 stop한다.

```bash
python3 tools/vision_host_mvp.py \
  --camera-url http://<esp32-cam-ip> \
  --detect-color red \
  --enable-align-action \
  --align-mode nudge \
  --align-nudge-ms 250 \
  --align-percent 35
```

로봇 base에 별도 gyro feedback이 있을 때만 `--align-mode angle --align-degrees 5`로 `/base?action=angle` 폐루프를 검증한다.

대시보드의 `Camera Detection`도 target center, x offset, alignment 상태, command suggestion을 표시한다. Dashboard action은 token-gated `Nudge Once`만 제공하며, 서버가 클릭 시점에 camera frame과 controller status를 다시 확인한 뒤 실행한다. ROS2 bridge의 `/camera/detection` JSON에도 같은 필드가 포함된다.

## 완료 기준

- ESP32-CAM `/capture`가 Mac에서 읽힌다.
- Mac script가 MotionBrain `/status`와 ESP32-CAM frame을 같은 루프에서 읽는다.
- target 감지 결과가 로그에 남는다.
- `--enable-action`에서 MotionBrain `/light` 명령이 성공한다.
- target center, horizontal offset, alignment 결정, command suggestion이 dashboard와 `/camera/detection`에 노출된다.
- `--enable-align-action`에서 안전 조건을 만족할 때만 MotionBrain base nudge 또는 base angle 명령이 전송되고, nudge mode는 즉시 stop까지 확인된다.
- 이 흐름이 나중에 ROS2 node로 옮길 host-side bridge 경계가 된다.

## 2026-05-20 실기 검증 결과

- ESP32-CAM CH340 upload board 기준 firmware upload 성공
- ESP32-CAM은 `MotionBrain-AP`에 접속했고 `192.168.4.2`로 확인
- iPhone Safari에서 `/status`, `/capture`, `/stream` 확인 완료
- Mac host가 `MotionBrain-AP`에 연결된 상태에서 controller `/status`와 camera `/capture`를 같은 loop에서 읽음
- `sensor sim healthy` 후 safe host action 확인:
  - `state=ARMED detected=Y assume_detected frame=14743B allowed=Y`
  - `ACTION light.toggle success=True`
- OpenCV red target detection 확인:
  - dry-run: `detected=Y red_ratio=0.254 frame=13089B allowed=Y`
  - action run: red target 감지 시 `ACTION light.toggle success=True`
  - target 제거 시 `detected=N red_ratio=0.000`
- 사용자 실기 확인 기준 search light 실제 점등 완료

## 2026-05-22 실기 검증 결과

- Home Wi-Fi mode에서 controller `192.168.219.105`, ESP32-CAM `192.168.219.106` 조합으로 Mac Wi-Fi 전환 없이 테스트했다.
- ESP32-CAM capture 안정화를 위해 QVGA/JPEG quality 15 프로파일과 `/status` capture diagnostics를 적용했다.
- Host vision 기본값은 ESP32-CAM 부담을 줄이도록 timeout 6초, interval 3초, capture retry 2회로 조정했다.
- Dry-run 기준 red target 위치 판정:
  - left: `align=LEFT`, `suggest=base_left`
  - center: `align=CENTER`, `suggest=hold`
  - right: `align=RIGHT`, `suggest=base_right`
- `ARM` 직후 안전 timeout이 즉시 만료되는 문제를 수정했다. `ARM` 성공도 timeout 기준 시간을 갱신한다.
- `--enable-align-action --align-mode nudge --align-nudge-ms 250 --align-percent 25` 실기 확인:
  - right target: `align=RIGHT`, `ACTION base.right nudge=250ms success=True stopped=True`
  - left target: `align=LEFT`, `ACTION base.left nudge=250ms success=True stopped=True`
- 테스트 종료 후 controller는 `IDLE`, motor off, fault 없음 상태로 확인했다.
- 현재 STM32 IMU는 handheld teleop 장치에 있으므로 `/base?action=angle` 폐루프 검증에는 부적합하다. base-mounted IMU/encoder가 추가되기 전까지 실제 vision alignment action은 timed nudge mode를 기준으로 한다.
