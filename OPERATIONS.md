# 빌드와 운영

[English](OPERATIONS.en.md) · [README](README.md) · [핀맵](PIN_MAP.md)

이 문서는 저장소의 일반 재현 절차다. 장치별 주소·자격 증명·관절 보정값은 각 설치 환경에서 설정한다. 빌드·테스트 통과는 펌웨어 업로드나 모터 구동을 뜻하지 않는다.

현재 연구의 목표·과정은 [시각 행동 학습 설계](docs/visual-action-learning.md)에 둔다. 기존 서비스 설정·M4 실행·진단 CLI는 아래 구현 안내를 따른다. 새 [시각 행동 수집기](docs/visual-action-collection.md)는 합성·저장 입력 감사와 별도 유한 실물 연결 CLI를 제공한다. 별도 M2 반복 수집 경로에서 [첫 행동 예측 학습·평가](docs/evidence/2026-10-31-first-action-prediction.md)를 완료했다. 공개 단발 CLI와 그 반복 실행기는 구분하며, 예측 기반 단일 목표 시험 연결은 모의 검증 단계이며 실물 제어 성능은 미검증이다. 파일럿의 무출력 관측, 별도 펌웨어 빌드와 명시적 실행은 해당 문서를 따른다. ARM 전 저널 준비 검사와 원본 결과를 보존하는 명시적 해소 절차도 그 문서에 둔다. 아래 일반 운영 명령이 새 실험 전체를 실행하는 절차는 아니다. 저장 자료의 후보 비교는 장치·학습 접근이 없는 [오프라인 선택기 CLI](docs/offline-action-selection.md)를 따른다. 같은 문서의 단일 목표 시험 코어는 별도 범위 확인과 현장 어댑터가 필요하며 아직 실물 미검증이다.

## 빌드

ESP32는 PlatformIO를 사용한다. 호스트 테스트에는 Python, NumPy, OpenCV와 Node.js(CI: Python 3.12·Node.js 22)가 필요하다. 네이티브 C++ 검사에는 C++ 컴파일러가 필요하며, 없으면 해당 검사는 건너뛴다. 아래 명령은 저장소 루트에서 실행한다. `<...>` 자리표시자는 각 환경의 실제 값으로 바꾼다.

```sh
python3 -m venv .local/venv
.local/venv/bin/python -m pip install numpy opencv-python-headless
.local/venv/bin/python -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

STM32 프로젝트는 `firmware/stm32/MotionBrainSensor/`에 있다. 로컬 STM32CubeIDE 설정 후 `tools/stm32_build.sh`로 빌드한다. 업로드 스크립트는 빌드와 별개다.

ROS2 Jazzy 환경의 빌드·테스트:

```sh
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
source install/setup.bash
colcon test --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test-result --verbose
```

설치 의존성은 [ROS2 CI](.github/workflows/ros2.yml), 펌웨어·Python 검증은 [PlatformIO CI](.github/workflows/platformio.yml)와 대응한다.

## Pi 서비스 설정

Pi 접속은 고정 DHCP IP 대신 현재 설치 환경의 호스트명을 사용한다. `tools/raspi/check_pi_ssh_target.py`는 SSH 대상과 연결을 확인한다. `tools/raspi/discover_device_url.py`는 ESP32 HTTP 탐색 도구이며 Pi SSH 탐색용이 아니다.

```sh
python3 tools/raspi/check_pi_ssh_target.py
```

[systemd 예시](deploy/systemd/)를 검토한 뒤 Pi에서 설치한다. 설치기는 유닛·환경 예시를 복사하고 `daemon-reload`를 수행하지만 서비스를 자동 활성화하지 않는다.

```sh
tools/raspi/install_systemd_units.sh
```

설치기는 아래 표의 환경 파일명을 사용한다. 기존 파일은 덮어쓰지 않으며, 이전 `motionbrain-*.env` 파일만 있으면 내용을 새 이름으로 복사하고 이전 파일도 보존한다. `.env.example`은 최신 템플릿으로 갱신한다. 환경 파일의 장치 URL, 모델 경로, 토큰을 설정한 후 필요한 서비스만 활성화한다. 다른 체크아웃·계정에는 설치기의 `MOTIONBRAIN_REPO`, `MOTIONBRAIN_SERVICE_USER`를 지정한다.

| 서비스 | 환경 파일 | 역할 |
| --- | --- | --- |
| `motionbrain-perception.service` | `/etc/motionbrain/perception.env` | 카메라 획득·인식 API |
| `motionbrain-dashboard.service` | `/etc/motionbrain/dashboard.env` | UI·제어기 프록시 |
| `motionbrain-ros-bridge.service` | `/etc/motionbrain/ros-bridge.env` | ROS2 상태·메시지 브리지 |

```sh
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-ros-bridge.service
```

환경 디렉터리 권한은 `750`, 파일은 `640`이다. 실제 토큰은 저장소·로그·화면에 노출하지 않는다. 기본 대시보드 바인딩은 `127.0.0.1`이다. LAN 노출 시 대시보드 인증과 제어기 인증을 각각 설정한다. ESP32 `/stream`은 비활성화돼 있으며 `/capture` 또는 Pi 추적 영상을 사용한다. 예시 CAM 프로필은 QVGA / JPEG quality 15이며 실제 설정은 해당 환경에서 확인한다.

## 상태 점검과 진단

```sh
CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
journalctl -u motionbrain-ros-bridge.service -n 100 --no-pager
```

장치 전원을 꺼둔 상태에서 대시보드 서비스만 확인할 때는 다음을 사용한다. 구조화된 `degraded` 응답을 허용하므로 구동 준비 판정은 아니다.

```sh
ALLOW_DASHBOARD_DEGRADED=1 CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

ROS2 점검의 기본 진단 한도는 일부 `WARN`을 허용한다. 구동 준비 판단에는 센서·피드백 진단을 별도로 확인한다. 브리지 점검은 기본적으로 실물 루틴 요청의 거부도 검사하며, 상태 조회 성공과 실물 실행 가능 상태를 구분한다.

제어기 HTTP 실패 기록:

```sh
python3 tools/diagnose_controller_http.py --target 'http://<controller-host>' --output '.local/http-diagnostic/<new-session>'
```

이 도구는 최대 네 번의 순차 GET(`/status`, `/sequence`, `/actions`, `/action_outputs`)만 보내고 재시도하지 않는다. 요청 합격 한도는 0.8s, 진단 수집 타임아웃은 2s, 세션 예산은 12s다. 모터 OFF·빈 큐·일치하는 부팅 ID를 확인하지만 전기적 정지나 장기 신뢰성을 증명하지 않는다. [수정 사례와 남은 타임아웃](docs/evidence/2026-10-17-idle-http-client.md)

ESP32의 STA 연결·재연결 시 모뎀 절전을 해제해 센서 조회가 절전 대기를 피하도록 한다. `/status`의 `network.stationPowerSave`는 드라이버 조회 결과로 `none`, `min_modem`, `max_modem`, `not_station`, `unknown`을 반환한다. 설정 실패는 경고하며 연결 자체를 차단하지 않는다. `none`도 20ms 수집 지연을 보장하지 않으므로 실제 요청별 나이를 계속 검사한다. AP 경로와 센서 폴링 주기는 바꾸지 않는다. [유한 지연 계측](docs/mac-direct-camera.md)

## 명령 상태와 복구

M4 실행기는 `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`에 영속 저널을 둔다. 기본값은 `~/.local/state/motionbrain/m4_commands.sqlite3`다. 제어기당 실행기·저널 하나를 사용한다. 같은 파일의 중복 소유는 차단하지만 다른 호스트·경로까지 배타성을 보장하지 않는다.

`accepted`는 접수이며 도달이 아니다. `forwarded`는 전송 시도 또는 중단 전 시도 가능성을 뜻한다. `outcome_unknown`은 새 확인을 차단한다. 결과 불명을 저널 삭제·새 경로·자동 재부팅으로 우회하지 않는다. 중복 명령을 다시 보내지 않으며, `/motionbrain/m4_write_result`는 제어기를 호출하지 않고 저장 결과만 읽는다.

실험용 `/actions`와 기존 M4 `/shoulder` 실행기는 별도 경로다. `/actions`는 기본 비활성화이며 실물 활성화·수집 연결은 미완료다. [행동·데이터 계약](docs/data-pipeline.md)을 참고한다.

## 원본 보관과 학습 자료

```sh
python3 tools/archive_hardware_demo.py '.local/<source-phase>' 'datasets/<new-archive>' --observation placed
```

보관 도구는 장치에 접속하지 않는다. 새 목적지에 계획·결과·사후 이미지와 선택적 카메라 응답을 복사하고 SHA-256·영속 쓰기를 확인한다. 원본은 변경하지 않는다. 파일 확장자는 실제 PNG/JPEG 형식과 일치해야 한다.

`archiveComplete`는 복사 완료이며 작업 성공이 아니다. 관찰 스냅샷은 동기화된 행동 프레임이 아니고, 영상·앵커·원본 저장 시간 검증은 `unknown`이다. `trainingEligible`·`controlEligible`은 false이며 오프라인 복사 속도로 촬영 시점 기준을 충족할 수 없다. 원본 이미지·응답·실행 기록은 `.local/` 또는 `datasets/`에 보관하며 Git에 올리지 않는다.

별도 `esp32cam_register_diag` 빌드는 OV2640 원시 레지스터 34개를 최대 3회 관찰한다. 내부 센서/DMA 초기화가 있지만 이미지 요청·반출은 없으며, 오류·기한 종료 뒤 자동 재시도하지 않는다. 이미 설치된 진단 앱은 USB 전원 연결을 관찰하는 호스트 실행기로 확인할 수 있다. 이 실행기는 UART 연결을 준비 응답부터 HTTP 수집 종료까지 유지하며 펌웨어 업로드나 명시적 리셋 펄스를 보내지 않는다. 2026-10-18 실물 두 시도는 모두 진단 완료에 실패했다(0/2). 첫 시도는 UART 전환 경계에서 중단했고, 둘째는 드라이버 PID `0x3660`(OV3660)을 OV2640 전용 가드가 추가 진단 SCCB 접근 전에 거부했다. 실제 노출 상한은 미검증이며 해당 진단에서 학습 적격 행동 전이는 0건이다. [유한 레지스터 진단](docs/ov2640-register-diagnostic.md)

[학습·평가 재현](docs/data-pipeline.md) · [Mac 카메라 계측](docs/mac-direct-camera.md) · [ROS2 하드웨어 인터페이스](ros2_ws/src/motionbrain_hardware_interface/README.md)

## IP 변경 뒤 화면 자동 복구

컨트롤러·Pi가 함께 이 기능을 포함한 버전으로 배포되어야 한다. 소스 병합이나 브랜치 푸시만으로 실행 중인 장치가 갱신되지는 않는다.

새 설치에서는 서비스 세 개만 시작해도 아래 자동 재탐색이 활성화되지는 않는다. `/etc/motionbrain/dashboard.env`와 `perception.env`의 주소·토큰·카메라 설정을 검토한 뒤 Pi에서 타이머를 별도로 활성화한다. 타이머 실행은 장치 탐색을 수행하고 기본 설정에서는 카메라 프로필을 적용하며, 필요시 관련 서비스를 재시작한다.

```sh
sudo systemctl enable --now motionbrain-dashboard-reconcile.timer
systemctl status motionbrain-dashboard-reconcile.timer --no-pager
systemctl list-timers motionbrain-dashboard-reconcile.timer --all
```

- Pi의 기존 `motionbrain-dashboard-reconcile.timer`는 약 60초마다 장치를 재탐색한다. 주소가 달라지면 관련 서비스를 재시작하고, 정상 대시보드의 주소를 컨트롤러에 등록한다. 일치하는 상태에서도 등록을 갱신한다.
- 등록은 기존 `MOTIONBRAIN_HTTP_TOKEN`으로 인증하는 `POST /vision_endpoint?port=8765`다. 호스트는 요청 본문의 임의 주소가 아니라 인증된 TCP 연결의 사설 IPv4 주소를 사용한다. 주소는 RAM에만 180초간 유효하며, 제어 상태·모터 출력에는 영향을 주지 않는다. 토큰은 URL·등록 응답에 포함하지 않는다.
- 컨트롤 UI는 `GET /vision_endpoint`와 대시보드 `/api/discovery`로 새 Pi·카메라 주소를 확인한다. 기존 저장 IP는 자동 모드의 후보로 취급한다. `CAM`·`API`로 새로 입력한 주소는 수동 설정으로 유지하고, `AUTO`로 자동 탐색에 돌아간다. 자동 주소를 조회해도 단일 사진 모드를 반복 촬영 모드로 바꾸지 않는다.
- 대시보드는 기존 연결이 연속 실패하면 기억한 컨트롤러 주소와 `motionbrain.local`에서 Pi의 새 주소를 찾는다. 새 대시보드의 식별 응답을 확인한 뒤 이동하며 인증 토큰·URL fragment·명령을 전달하거나 재실행하지 않는다. 새 origin에서는 제어용 대시보드 토큰을 다시 입력해야 한다.
- 브라우저 탐색 요청은 각각 2초, 영상 응답 대기는 6초로 제한하며 복구 요청을 겹쳐 보내지 않는다. 숨겨진 탭에서는 새 영상·탐색 요청을 보내지 않는다. 영상 연결 상태와 `NO TARGET`/검출 오류를 별도로 표시한다.
- perception의 실시간 영상은 최초 수신 후 단조 시계 기준 `stale_seconds`를 넘으면 HTTP 503(`no_fresh_frame`)을 반환한다. 대시보드는 이를 HTTP 502로 전달하고 기존 이미지를 지워 재연결 상태로 표시한다. 새 프레임 수신 후에는 자동 복구된다. 명시적인 `frameId`로 요청한 과거 근거 이미지는 캐시에서 제거되기 전까지 조회할 수 있으며, 이 응답은 실시간 연결이나 제어 적격성을 뜻하지 않는다. 메타데이터가 없는 영상의 표시 최신성은 호스트 수신 시각 기준이다.

복구에는 **접근 가능한 컨트롤러 주소 또는 정상적인 Pi 호스트명 조회 중 하나**가 필요하다. 모든 장치 IP가 동시에 바뀌고 호스트명 조회도 전부 실패하면 브라우저가 새 네트워크를 알아낼 수 없다. 이 경우 네트워크/mDNS를 복구하거나 현재 주소를 입력해야 한다. 사설 IPv4 LAN과 단일 MotionBrain 장치 구성이 대상이며, IPv6 전용망·공용 프록시·여러 동명 장치 자동 선택은 지원 범위가 아니다. 서비스 기동 시간·탐색 주기 때문에 즉시 복구를 보장하지 않는다.

소프트웨어 검증은 `python3 -m unittest discover -s tests`로 실행한다. 브라우저 복구 테스트에는 Node.js가 필요하며 모의 HTTP·가상 타이머를 사용한다. 실제 DHCP 변경·장치 재부팅·영상 수집·모터 구동 검증과 구분한다. 설치 시 기존 systemd 설치 스크립트가 `publish_vision_endpoint.py`도 설치하며, 기존 인증·타이머 설정을 사용한다.
