# ESP32-CAM 변경 전후 장시간 비교 — 2026-10-15

최신 후속: [부팅 watchdog 조사와 별도 네트워크 오류 수정](2026-10-15-camera-boot-watchdog.md)을 완료했다. watchdog은 반복 부팅에서 재현되지 않아 원인 미확정이며, 아래 기록은 그 수정 이전의 시험 결과다.

**카메라 위치 고정 이후 동일한 요청 부하에서 변경 전·후 각각 10분 동안 장애가 재현되지 않았다. 최근 원본 프레임 식별 변경이 원인이라는 증거는 확보하지 못했다. 기존 간헐 장애의 근본 원인 확정이나 수정 완료를 뜻하지 않는다.**

[앞선 충돌·지속 무응답 진단](2026-10-15-camera-connectivity.md)의 후속 실물 시험이다. 모터 명령, ARM, 메인 ESP32·STM32 업로드는 하지 않았다. 카메라 펌웨어만 교체하고 RTS로 리셋했다. 특정 가능한 코드 결함이 발견되지 않아 제품 펌웨어 소스는 수정하지 않았다.

## 비교 조건

- 사용자가 카메라 위치를 고정한 뒤 시작했다. 세 보드는 같은 Mac USB 허브/케이블을 유지했다. 배터리함→XL4015는 사용자의 마지막 보고상 분리 상태이며 전압을 실측하지는 않았다.
- CAM MAC `30:76:f5:e5:ea:60`, 당시 포트 `/dev/cu.usbserial-1120`, 주소 `192.168.219.116:80`. 카메라 프로필 QVGA, JPEG quality 15.
- 현재 소스는 `559db54`의 CAM, 변경 전 소스는 `ca48f04^`의 CAM이다. 같은 설치 SDK로 빌드했다: PlatformIO espressif32 6.12.0, Arduino 2.0.17, ESP-IDF 4.4.7. 과거 SDK/flash 전체를 복원한 비교가 아니다.
- 현재→변경 전 순서로 각각 600초. 서버 준비 후 10초 대기하고 측정했다. 두 Mac 클라이언트가 각각 3초마다 `/capture`, 별도 클라이언트가 5초마다 `/status`를 GET했다. 요청 timeout 4초, 일정이 밀리면 몰아서 재시도하지 않고 누락 슬롯을 기록했다. 실제 누락 슬롯은 양쪽 모두 0이다.
- Pi perception 프로세스만 동일 사용자 권한의 SIGSTOP으로 일시 정지했다. 비교 완료 후 SIGCONT로 재개했다. 제어 서비스에는 정지/재시작/구동 요청을 보내지 않았다. 원격 보조 프로세스에 1,800초 자동 재개와 연결 종료 시 재개 경로를 두었다.
- 각 시험 내내 UART를 열어 두고 로그를 저장했다. 이미지 원본은 보관하지 않고 수신 길이·SHA-256·프레임 ID만 기록했다. 이미지 성공 판정은 HTTP 200, 본문 전체 수신 및 JPEG 시작/끝 표식 검사이며 모든 이미지를 별도 디코더로 검증한 것은 아니다.
- 브라우저 탭을 임의로 닫지는 않았다. 각 비교에서 카메라의 `captureRequests` 증가량은 시험 요청 400건과 정확히 같았다. 측정 중 추가 캡처 요청은 관찰되지 않았다.
- 순차 시험이라 RF 환경 자체를 완전히 고정한 것은 아니다. RSSI 범위를 아래에 함께 보존했다.

## 실물 비교 결과

| 항목 | 현재 버전 | 변경 전 버전 |
|---|---:|---:|
| 측정 시간 | 600초 | 600초 |
| 이미지 응답 | 400/400 성공 | 400/400 성공 |
| 상태 응답 | 120/120 성공 | 120/120 성공 |
| 측정 중 UART panic / 재부팅 | 0 / 0 | 0 / 0 |
| 캡처 / 클라이언트 송신 실패 | 0 / 0 | 0 / 0 |
| 이미지 응답 중앙값 / 최대 | 67.5 / 160ms | 70 / 159ms |
| RSSI | -55~-35dBm | -46~-35dBm |
| heapFree 범위 | 190,852~193,288 bytes | 191,048~193,524 bytes |
| 원본 프레임 ID 누락 | 0/400 | 기능 추가 전이므로 400/400 미제공 |
| 동일 프레임 ID의 hash 충돌 | 0 | ID 미제공으로 평가 제외 |

현재 버전은 측정 동안 boot ID 1개를 유지했다. 변경 전 버전은 boot ID 필드가 없으므로 UART와 uptime 감소 여부로 확인했다. 양쪽 모두 uptime 감소가 없었다. 시험 시작 시각은 현재 버전 12:27:18 KST, 변경 전 버전 12:38:41 KST이며, 준비·대기 시간은 600초 측정에서 제외된다.

### 측정 전 부팅 오류를 별도로 보존

현재 버전 첫 리셋 후 약 5.64초에 `TG0WDT_SYS_RESET`이 1회 발생했다. 재부팅 후 `resetReason=task_watchdog`, 약 10.70초에 HTTP 서버 준비가 확인됐다. 이후 10분 측정은 정상이다. 이 부팅 오류를 520/520 성공률로 덮지 않는다.

UART에서 `Loaded stored ESP32-CAM Wi-Fi config` 다음, `Connecting to configured Wi-Fi` 이전에 발생했다. 소스상 두 버전 공통의 Wi-Fi 초기화 구간이며, 새 boot ID 생성 및 이미지 헤더 전송보다 앞선다. 따라서 **해당 오류를 새 HTTP 헤더 처리 때문이라고 볼 직접 증거는 없다.** 메모리 배치·SDK·하드웨어 영향까지 배제하거나 watchdog의 근본 원인을 확정한 것은 아니다. 변경 전 이번 부팅에서는 같은 오류가 관찰되지 않았다.

## 소프트웨어 검사

- 기존 원본 프레임 native 회귀 2개 통과: 식별·나이·재부팅·잘못된 시각·버퍼 부족 처리, 실제 Python 파서와 wire roundtrip.
- 추가 일회성 검사: 임의 시각/출력 버퍼 경계 100,000건, 짧은 boot ID, 최대 int64 시각을 AddressSanitizer와 UndefinedBehaviorSanitizer로 실행해 통과했다. 첫 컴파일의 시스템 SDK 링크 오류는 `xcrun --sdk macosx --show-sdk-path`를 명시해 해결했다.
- 이 검사는 호스트에서 실행한 메타데이터 helper 검사다. ESP32 SDK 내부나 실제 보드의 heap/전원 이상까지 검증하지 않는다. 372개 전체 회귀의 재실행 결과로 표기하지 않는다.

## 복원 후 통합 확인

- 현재 펌웨어 업로드 성공(62.08초), flash hash 검증 성공. 복원 ELF SHA-256은 최초 현재 버전과 동일하다. Pi perception PID 772는 SIGCONT로 재개했고 세 서비스의 `NRestarts=0`을 확인했다.
- Pi 자동 영상 요청을 켠 채 같은 Mac 요청 주기로 300초 추가 검사: 이미지 **200/200**, 상태 **60/60** 성공, UART panic·재부팅·누락 슬롯 0. 이번 리셋의 시작 시 watchdog도 없었다. 측정 중 boot ID 1개 유지, 캡처·송신 실패 0.
- 카메라 캡처 카운터는 이 구간에 263 증가했다. Mac 시험 요청 200건 외 63건은 Pi 요청 등 추가 트래픽이며, 별도 패킷 추적으로 요청자를 확정한 것은 아니다. 이 통합 구간을 앞의 부하 통제 비교와 섞지 않는다.
- Pi에서 동일 원본 ID의 JPEG 재조회·SHA-256 일치, 없는 ID의 JPEG/관측 조회 각각 404, 미교정 시계가 미검증으로 유지됨, 약 19.47초 지난 관측의 `sourceFresh=false`·`controlEligible=false`를 확인했다.
- 최종 읽기 전용 점검: CAM uptime 477,908ms, 같은 boot ID, captureFailures=0. Pi `ok=true`, `fresh=true`, `consecutiveErrors=0`, 서비스 3개 active, `throttled=0x0`. 메인 ESP32 `IDLE`, `motorEnabled=false`, M1~M5 출력 모두 0, STM32 IMU·M4 AS5600 상태 정상.
- 총 부하 측정은 25분, 시험 클라이언트 이미지 1,000건·상태 조회 300건 모두 성공했다. 준비 중 timeout과 최초 watchdog 1회는 이 숫자에서 제외되며 UART/원본 로그에 별도로 보존했다.

## 판단과 남은 확인

1. 이번 조건에서는 현재 코드를 이전 버전으로 되돌릴 근거를 얻지 못했다. 새 frame ID 기능을 유지한다.
2. 위치 고정 후 RSSI가 앞선 장애 관찰(-78~-48dBm)보다 좋아졌다. 위치/무선 환경 변화가 유력한 단서지만 위치만 바꿔 되돌리는 교차 시험을 하지 않아 원인으로 확정하지 않는다.
3. 허브·케이블·보드 전원, 공통 Wi-Fi SDK/초기화 문제는 제외하지 못했다. 앞선 Wi-Fi TX panic·지속 무응답과 이번 시작 시 watchdog이 같은 원인인지도 미확정이다.
4. 재발 시 UART 부팅 구간/연결 이벤트와 네트워크 상태를 먼저 보존하고, CAM 단독 USB 전원과 현재 허브 조건을 비교한다. 가능하면 부팅·송신 순간의 5V/3.3V를 계측한다. 그 결과에 맞춰 Wi-Fi 단계 계측 또는 CAM 전용 SDK 비교를 좁혀 수행한다.
5. HTTP stall 자동 재시작은 기존 기본 OFF를 유지했다. 재부팅으로 증상을 감추는 변경은 하지 않았다. 장시간 학습 데이터 수집 전에는 재발 여부를 추가 관찰해야 한다.

## 증거와 재현 자료

[실험 디렉터리](camera-long-comparison-2026-10-15/)에 요청별 JSONL, UART, 집계, 소스/ELF/bin SHA-256, 시험 스크립트와 sanitizer 입력을 보존한다. 전체 업로드 로그는 `/tmp/motionbrain-cam-long/`에 있으며 칩 식별·flash hash 검증·완료 결과를 `old-upload-summary.txt`, `restore-current-summary.txt`에 보존했다. 펌웨어 바이너리·Wi-Fi 자격 증명·이미지 원본은 저장소에 추가하지 않는다.

실험용 `soak-harness.py`는 당시 CAM IP/포트를 고정하고 RTS 리셋을 수행한다. 재실행 전 장치 MAC과 포트를 다시 확인해야 하며 일반 자동 테스트로 실행하지 않는다. 재현 명령은 다음과 같다.

```sh
~/.platformio/penv/bin/python docs/evidence/camera-long-comparison-2026-10-15/soak-harness.py LABEL --duration 600
python3 -m unittest discover -s tests -p test_camera_source_native.py -v
clang++ -std=c++11 -isysroot "$(xcrun --sdk macosx --show-sdk-path)" \
  -fsanitize=address,undefined -fno-omit-frame-pointer -g \
  -I firmware/esp32cam/src \
  docs/evidence/camera-long-comparison-2026-10-15/metadata-boundary-sanitizer.cpp \
  -o /tmp/motionbrain-camera-metadata-sanitize
/tmp/motionbrain-camera-metadata-sanitize
```

실물 결과는 카메라 통신/관측 검증이다. 실제 모터 동작, 파지, 학습 성능 개선 결과는 아니다.
