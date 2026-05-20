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

기본값:

| 항목 | 값 |
| ---- | -- |
| MotionBrain SSID | `MotionBrain-AP` |
| MotionBrain password | `motionbrain` |
| MotionBrain IP | `192.168.4.1` |
| ESP32-CAM URL | serial log의 `ESP32-CAM IP` 확인 |

`motionbrain`은 로컬 데모 AP 기본값이며 개인 계정 비밀번호가 아니다. 배포/시연 환경에서 바꾸려면 `firmware/esp32cam/platformio.ini`에 빌드 플래그를 추가한다.

```ini
build_flags =
  -DMOTIONBRAIN_WIFI_SSID=\"MotionBrain-AP\"
  -DMOTIONBRAIN_WIFI_PASSWORD=\"changed-password\"
```

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

### 6. 실패 시 우선 확인

- `ESP32-CAM IP`가 serial log에 찍혔는지 확인한다.
- ESP32-CAM과 Mac이 모두 `MotionBrain-AP`에 붙어 있는지 확인한다.
- `http://192.168.4.1/status`가 Mac에서 열리는지 확인한다.
- `/stream`을 장시간 열어둔 뒤 `/status`나 `/capture`가 timeout되면 ESP32-CAM을 reset하거나 최신 펌웨어를 다시 올린다. 최신 펌웨어는 stream을 시간 제한으로 끊어 HTTP 서버가 회복되게 한다.
- `/capture`가 느리거나 실패하면 ESP32-CAM 전원을 다시 연결한다.
- `brownout` 또는 반복 reboot가 보이면 ESP32-CAM 전원 부족으로 본다.
- 업로드가 실패하면 다른 serial monitor가 포트를 잡고 있는지 확인한다.
- `--enable-action`은 `/status.sensor.blocked=false`이고 controller state가 `IDLE` 또는 `ARMED`일 때만 동작한다.

## Host MVP

dry-run:

```bash
python3 tools/vision_host_mvp.py --camera-url http://<esp32-cam-ip> --once
```

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

## 완료 기준

- ESP32-CAM `/capture`가 Mac에서 읽힌다.
- Mac script가 MotionBrain `/status`와 ESP32-CAM frame을 같은 루프에서 읽는다.
- target 감지 결과가 로그에 남는다.
- `--enable-action`에서 MotionBrain `/light` 명령이 성공한다.
- 이 흐름이 나중에 ROS2 node로 옮길 host-side bridge 경계가 된다.
