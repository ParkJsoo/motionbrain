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
- Phase 4 MVP 브랜치에서는 Mac host와 ESP32-CAM 동시 접속을 위해 MotionBrain AP 최대 client 수를 2로 둔다.

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
