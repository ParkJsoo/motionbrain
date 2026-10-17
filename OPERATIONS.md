# 운영 안내

[English](OPERATIONS.en.md) | [README](README.md) | [임베디드 초기 구동](EMBEDDED_BRINGUP.md)

Raspberry Pi에서 MotionBrain을 운영하고 장애를 점검하는 방법을 정리한다. 실물 시스템의 품질 관리, Linux 운영, 문제 해결 과정에 해당하는 문서다.

## 소프트웨어 구현과 실물 검증의 경계

실물 학습 데이터 수집 어댑터는 아직 연결되지 않았다. 새 MCU 제한 행동 API는 기본 비활성화 상태이며, 호스트 명령 저널의 접수 기록은 동작 완료 증거가 아니다. Pi ROS2 빌드·테스트와 비구동 검사는 [배포 근거](docs/evidence/2026-10-15-deployment-no-drive.md)에 기록했다.

고정 예산으로 수행한 [관찰 시험](docs/evidence/2026-10-15-bounded-observation.md)은 사용성 기준을 통과하지 못했다. [NumPy 학습·평가 파이프라인](docs/evidence/2026-10-15-offline-visual-dynamics.md)은 구현했지만 저장한 합성 시도 11개는 모두 제외되며, 학습 적격 실물 샘플은 없다. 소프트웨어 테스트와 합성 데이터 점검만으로 실물 정지·파지·학습 조작 성능을 입증할 수 없다.

## M4 명령 저널

작업자 확인을 거치는 M4 실행기는 `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`에 명령을 기록한다. 기본 경로는 `~/.local/state/motionbrain/m4_commands.sqlite3`다. 제어기 하나에는 영속 저널 하나와 실행기 하나를 사용한다. `XDG_STATE_HOME`이 비어 있거나 상대 경로이면 홈 디렉터리를 기준으로 삼는다. 명시적으로 지정하는 저널 경로는 절대 경로여야 하며 홈 경로 확장은 허용한다. `command_journal_path`와 `command_journal_max_records`는 ROS 파라미터다. 같은 파일을 두 번째 실행기가 소유하려 하면 차단하지만, 다른 경로나 다른 호스트까지 보호하지는 않는다.

`/motionbrain/m4_write_result`는 요청에 `command_id`만 포함하는 `motionbrain_msgs/srv/M4WriteConfirm`을 사용한다. 읽기 전용이며 제어기를 호출하지 않는다. `raw_json.executionState`를 확인한다. `accepted`는 제어기의 접수 응답이며 목표 도달이 아니다. `forwarded`는 전송을 시도했거나, 충돌 전에 시도했을 가능성을 뜻한다. 실제 구동을 증명하지 않는다. 중복 확인에는 추가 POST 없이 저장된 결과와 `proposal_already_consumed`를 반환한다.

`outcome_unknown` 기록이 있으면 새 확인을 차단한다. 아직 결과 대조·확정 API가 없으므로 데이터베이스 삭제나 경로 변경으로 이 상태를 해제하지 않는다. 기본 10,000개 한도에는 대기·소비된 기록이 모두 포함된다. 한도에 도달하면 옛 ID를 지우지 않고 새 제안을 거부한다. 재시작 시 대기 요청은 취소하며, 전송이 예약된 요청은 결과 불명으로 처리한다. 결과가 없다고 미실행을 뜻하지 않는다. 기존 MCU 엔드포인트에는 명령 ID 중복 제거와 요청별 기한 집행이 없으므로, 이 호스트 변경만으로 학습 기반 실물 시험을 시작할 수 없다. [제한 행동 계약](docs/evidence/2026-10-14-bounded-actions-software.md)과 [소프트웨어 검증](docs/evidence/2026-10-14-command-journal-software.md)을 참고한다.

## 실험용 MCU 제한 행동

새 `/actions` API는 제어기 부팅 ID와 명령 ID, 동시 활성 행동 한 개, 항목을 퇴출하지 않는 고정 32개 레지스트리, MCU 기한을 사용한다. 기본값은 비활성화(`MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=0`)다. 기존 호스트 실행기의 `/shoulder` 전송 방식은 바꾸지 않는다. `/action_outputs`는 소프트웨어 PWM 쓰기 기록이며 전기 측정이나 관절 이동량이 아니다. 결과 불명은 새 구동을 막지만 기존 비상 정지·비활성화 경로를 막지 않는다. 결과 불명이나 용량 초과를 우회하려고 자동 재부팅하거나 기록을 지우지 않는다.

[API·검증·실물 점검표](docs/evidence/2026-10-14-bounded-actions-software.md)를 참고한다. 이 API의 실물 활성화와 검증은 아직 남아 있다.

`learning.action_execution.BoundedActionExecutor`는 이 API 전용 영속 호스트 저널을 제공한다. 제안마다 확인한 요청과 정확히 일치하는 내용, 명시적 축·출력 한도, 사전 점검과 영속 예약 후 호출자가 제공하는 최종 신선도·저장 가드가 필요하다. 제어기당 실행기 하나를 두며 기존 M4 저널과 분리한다. 중복 ID는 재전송하지 않고, 전송 중단이나 모호한 결과는 이후 명령을 차단한다. 대상을 지정한 취소를 최선 노력으로 한 번 시도해도 결과 불명이 해제되지는 않는다. `request_stop()`은 취소 신호를 보내지만 블로킹 네트워크 I/O를 중단하거나 제어기 비상 정지를 대신할 수 없다.

`learning.action_port.BoundedActionHTTP`는 기본 비활성화 상태이며 실물 실행 CLI가 없다. 인증정보는 최종 가드 전에 메모리에 준비하고 응답 기록에는 넣지 않는다. 최종 응답 기록에는 일치하는 제어기 기록과 연속된 출력 OFF 추적이 필요하지만, 실물 도달이나 학습 적격성을 증명하지는 않는다. `JournaledSyntheticCaptureAdapter`는 합성 포트로만 이 경로를 시험한다.

`learning.device_capture.DeviceCaptureSession`은 공통 로컬 전송 경로에서 실행기, 정확한 프레임 GET, 원본 JPEG·메타데이터 저장, 독립 M4 상태 샘플을 연결한다. 새 세션마다 정확한 확인 한 번만 받으며 기존 명령 ID를 재개하거나 재사용하지 않는다. 전송 전 검증된 생성자 시계 근거, 영상 나이 최대 100ms, M4 나이 최대 20ms, 영속 저장 최대 250ms를 요구한다. 앵커 기반 shadow 관측은 거부하며 그 별도 15s 한도는 유지한다.

소프트웨어 출력 OFF 뒤에는 500ms 동안 최대 여섯 샘플로 명시된 허용오차 안의 M4 정지 구간 50ms를 확인해야 한다. 마지막 영상 뒤 출력 이력과 M4를 다시 검사한다. 이는 M4 안정성 근거일 뿐이며 `physicalSettledVerified`와 `trainingEligible`은 false로 유지한다. 별도 초기 검증 스키마는 원격 시계를 보존하며 행동 데이터셋에 들어갈 수 없다. `audit_device_capture`는 명령 실행이나 결과 대조 없이 저장 근거를 검사한다. 현재 보정되지 않은 인식 관측은 전송 가드를 통과하지 못한다. 실물 CLI나 보정 우회 옵션은 없다.

## 원본 관측과 전이 기록

ESP32-CAM `/capture`는 부팅 ID와 첫 DMA 타임스탬프로 원본 카메라 버퍼를 식별한다. 인식 서비스는 `frameId`로 정확한 메타데이터와 JPEG를 제공한다. 만료된 ID는 최신 영상으로 대체하지 않고 404를 반환한다. 이전 카메라 펌웨어는 원본 나이를 입증하지 못하므로 표시 용도로만 사용한다.

`CameraDetection.msg`에 원본 관련 필드가 추가됐으므로 배포 전에 ROS 워크스페이스를 함께 다시 빌드한다. 가드는 브리지·구독자의 ROS 시계가 공유된다고 가정하고 이후에는 단조 시계로 관측 나이를 계산한다. `require_detection=false`이면 상태만으로 준비 판정은 가능하지만, 오래된 관측으로 동작을 제안하지 않는다.

`learning.schema.validate_transition`은 스키마 유효성과 학습 적격성을 분리한다. 결과 불명, 유지되거나 오래된 프레임, 누락된 출력 이력, 검증되지 않은 시계 오차 한도가 있는 원본은 학습에서 제외한다. [계약과 검증](docs/evidence/2026-10-14-source-observations-software.md)을 참고한다.

[dry-run 수집기](docs/evidence/2026-10-14-dry-run-collector-software.md)는 `python3 tools/capture_action_transitions.py --dry-run`으로 실행한다. 합성 원본 JPEG, 해시, 원본 관측, 실행 의도, 부분·오류 기록을 Git에서 제외한 `datasets/action_transitions/dry_run/`에 저장한다. `--audit-run PATH`는 읽기 전용 무결성 검사이며 행동을 재개하거나 재시도하지 않는다. 합성 자료는 실물 학습 부적격 상태를 유지하고, 이 수집기에는 실물 실행 스위치가 없다.

실행기는 영속 의도 기록 전후 모두에서 원본 나이에 보정된 시계 불확실성과 경과 단조 시간을 더해 검사한다. 한도 초과 시 실행 호출은 0회다. 이미 의도를 저장했다면 감사는 보수적으로 `outcome_unknown`을 유지하며 행동을 재개하거나 재시도하지 않는다.

## 오프라인 데이터셋 매니페스트

저장된 실행 디렉터리와 새 출력 디렉터리를 명시한다.

```sh
python3 tools/build_action_dataset.py --runs datasets/action_transitions/dry_run/RUN_ID --output-dir datasets/action_transitions/manifests/MANIFEST_ID
python3 tools/build_action_dataset.py --validate datasets/action_transitions/manifests/MANIFEST_ID/manifest.json
python3 tools/build_action_dataset.py --validate datasets/action_transitions/manifests/MANIFEST_ID/manifest.json --require-ready
```

`manifest.json`은 모든 시도와 제외 사유를 남긴다. `validation.json`은 불변 검증 스냅샷이다. `--validate`는 원본 파일을 다시 읽고 적격성·무결성·분할 배정을 재계산한다. 원본 디렉터리를 매니페스트와 함께 보존한다. 이미지를 다시 쓰거나 기존 출력 디렉터리를 덮어쓰지 않는다.

기본 종료 코드 0은 목록의 정합성을 뜻하며 적격 샘플 0개도 포함한다. 학습 호출자는 `--require-ready`를 사용해야 한다. 종료 코드 2는 잘못된 목록 또는 비어 있는 실물 train/validation/test 분할을 거부한다. 현재 합성 실행은 이 준비 기준에서 올바르게 탈락한다. 실물 출처 선언은 필요하지만 하드웨어 인증은 아니며 실물 학습 수집기는 아직 없다. [데이터셋 정책·테스트·한계](docs/evidence/2026-10-14-dataset-manifest-software.md)를 참고한다.

## Pi 접속과 SSH

Pi에는 호스트명 기반 SSH 별칭을 사용한다. DHCP IP를 별칭에 고정하지 않는다. 과거 Pi가 `192.168.219.114`로 바뀌었는데 Mac은 이전 `192.168.219.110`에 접속해 반복 실패한 사례가 있었다. 이 주소들은 과거 사례이며 현재 주소가 아니다.

Mac의 `~/.ssh/config` 권장 설정:

```sshconfig
Host motionbrain-pi motionbrain-pi.local motionbrain-pi.davolink
    HostName motionbrain-pi.local
    User motionbrain
    HostKeyAlias motionbrain-pi.local
    AddressFamily inet
```

Mac에서 먼저 확인한다.

```bash
python3 tools/raspi/check_pi_ssh_target.py
ssh -o ConnectTimeout=5 motionbrain-pi 'hostname; hostname -I; systemctl is-active ssh'
nc -vz motionbrain-pi.local 22
nc -vz motionbrain-pi.davolink 22
```

결과 해석:

- `motionbrain-pi.local`이 기본 Pi SSH 대상이다.
- `motionbrain-pi.davolink`는 라우터 DNS 대체 경로다. `.local`과 SSH가 정상이어도 사라질 수 있다.
- `motionbrain.local`은 Pi가 아니라 ESP32 모션 제어기다.
- `motionbrain-cam.local`은 Pi가 아니라 ESP32-CAM이다.
- `tools/raspi/discover_device_url.py`는 ESP32 제어기·카메라 HTTP `/status` 탐색 도구이며 Pi SSH 탐색 도구가 아니다.

Pi의 Ubuntu 24.04는 `ssh.socket`으로 SSH를 제공할 수 있다. `ssh.service`가 `disabled`여도 `ssh.socket`이 `enabled`·`active`일 수 있으므로 systemd 상태를 바꾸기 전에 둘 다 확인한다.

```bash
systemctl is-active ssh
systemctl is-enabled ssh.socket
systemctl is-active ssh.socket
ss -ltnp | grep ':22 '
```

새 OS 설치 후 DNS는 정상인데 SSH 호스트 키 변경 경고가 나오면, 이전 Pi 키만 제거하고 다시 접속한다.

```bash
ssh-keygen -R motionbrain-pi.local
ssh-keygen -R motionbrain-pi.davolink
```

## 실행 서비스

systemd 유닛은 `deploy/systemd/`에 있다.

| 서비스 | 역할 | 환경 파일 |
| --- | --- | --- |
| `motionbrain-perception.service` | 카메라 검출을 위한 Pi 인식 API | `/etc/motionbrain/perception.env` |
| `motionbrain-dashboard.service` | Pi 대시보드와 제어기 프록시 | `/etc/motionbrain/dashboard.env` |
| `motionbrain-ros-bridge.service` | ROS2 Jazzy 브리지와 타입이 정의된 토픽 | `/etc/motionbrain/ros-bridge.env` |
| `motionbrain-dashboard-reconcile.timer` | 대시보드·인식 서비스 설정의 주기적 조정 | 서비스별 환경 파일 |

Pi 설치 절차 예시:

```bash
tools/raspi/install_systemd_units.sh
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-ros-bridge.service
```

설치기는 서비스·타이머 유닛을 복사하고 `/etc/motionbrain/` 아래에 환경 파일 예시를 쓴다. 다른 저장소 경로나 Linux 계정을 사용하면 `MOTIONBRAIN_REPO` 또는 `MOTIONBRAIN_SERVICE_USER`를 지정한다. 실물 장비에서 서비스를 켜기 전에 복사한 환경 파일을 수정한다. `MOTIONBRAIN_HTTP_TOKEN`은 Git·로그·화면 캡처·공개 시연에 노출하지 않는다. 대시보드 제어와 ESP32 명령 토큰은 별도의 경계다. 대시보드는 제어기 토큰을 노출하지 않고 읽기 전용 상태를 관찰할 수 있으며, 상태 변경 요청은 제어기 펌웨어 게이트를 통과해야 한다.

기본 대시보드 환경은 `127.0.0.1`에 바인딩한다. 다른 장치에서 Pi 대시보드에 접근해야 하는 감독하의 LAN 시연에만 `MOTIONBRAIN_DASHBOARD_HOST=0.0.0.0`을 사용한다.

설치기는 실행 환경 파일을 비공개 권한으로 유지한다.

```bash
sudo stat -c '%U:%G %a %n' /etc/motionbrain /etc/motionbrain/*.env
```

예상 소유자는 `root:<service-group>`이며 `/etc/motionbrain` 권한은 `750`, 환경 파일은 `640`이다. 기존 환경 파일의 내용은 덮어쓰지 않지만 소유자와 권한은 갱신한다. 조정 타이머는 변경 가능한 저장소 스크립트를 직접 실행하지 않는다. `/usr/local/libexec/motionbrain/` 아래의 root 소유 사본 `reconcile_dashboard_services.sh`, `discover_device_url.py`, `apply_camera_profile.py`를 실행한다.

Pi 물체 인식 모드에서는 `/etc/motionbrain/perception.env`의 `MOTIONBRAIN_OPENCV_THREADS`, `MOTIONBRAIN_PERCEPTION_INTERVAL`, `MOTIONBRAIN_PERCEPTION_STALE_SECONDS`로 CPU 사용량을 제한한다. 기본 systemd 래퍼는 YOLOv5s/OpenCV DNN에 OpenCV 작업 스레드 하나와 보수적인 신선도 구간을 사용한다.

ESP32 제어기·카메라의 `.local` 또는 IP가 바뀌면 오래된 주소를 고정하지 말고 Pi의 탐색·조정 도구를 사용한다.

```bash
python3 tools/raspi/discover_device_url.py --help
tools/raspi/reconcile_dashboard_services.sh
```

systemd 유닛이나 환경 파일을 바꾼 뒤에는 시연 전에 Pi에 설치된 유닛을 검증한다.

```bash
for script in tools/raspi/install_systemd_units.sh tools/raspi/reconcile_dashboard_services.sh; do
  bash -n "${script}"
done
tools/raspi/install_systemd_units.sh
sudo systemd-analyze verify /etc/systemd/system/motionbrain-*.service /etc/systemd/system/motionbrain-*.timer
sudo systemctl daemon-reload
systemctl cat motionbrain-dashboard.service motionbrain-perception.service motionbrain-ros-bridge.service motionbrain-dashboard-reconcile.service
sudo systemctl start motionbrain-dashboard-reconcile.service
systemctl status motionbrain-dashboard-reconcile.service --no-pager
systemd-analyze security motionbrain-dashboard.service motionbrain-perception.service motionbrain-ros-bridge.service
```

## 상태 점검

대시보드·인식 서비스:

```bash
CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

ESP32 제어기나 ESP32-CAM의 전원을 의도적으로 껐다면 대시보드 서비스 점검과 구동 준비 판정을 분리한다.

```bash
ALLOW_DASHBOARD_DEGRADED=1 CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

이 설정은 읽기 전용 관찰을 위해 구조화된 대시보드 `degraded` 응답을 허용한다. 구동 준비 사전 점검은 아니다.

ROS2 브리지:

```bash
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
```

브리지 점검은 먼저 탐색을 검증하고 여러 토픽, 진단, 관리형 수명주기 상태, 서비스 하나, 액션 하나를 순차 샘플링한다. `CHECK_ROUTINE_RUN_REJECTION=0`을 지정하지 않으면 ROS2 브리지 정책이 실물 `run` 요청을 거부하는지도 확인한다. Pi에서는 몇 분 걸릴 수 있다. 토픽 탐색은 통과했지만 설정 기한 안에 샘플이 오지 않으면 최근 브리지 로그를 확인하고 서비스를 재시작한다.

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
sudo systemctl restart motionbrain-ros-bridge.service
CHECK_SERVICE=1 SAMPLE_TIMEOUT_SECONDS=25 tools/raspi/check_ros_bridge_health.sh
```

기본 진단 한도는 현재의 읽기 전용 초기 구동 상태를 허용한다. 측정 각도가 보정 한도 밖이면 M4 어깨 피드백이 `WARN`일 수 있고, `base_yaw_reference` 미설치 중에는 루틴 피드백도 `WARN`일 수 있다. 구동 준비 점검에서는 정상 진단을 명시적으로 요구한다.

```bash
EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL=0 \
EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL=0 \
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
```

ROS2 근거 수집:

```bash
tools/raspi/capture_ros2_evidence.sh
```

ros2_control 모의 장치 근거 수집:

```bash
tools/raspi/capture_ros2_control_mock_evidence.sh
```

ros2_control 개루프 하드웨어 인터페이스 근거 수집:

```bash
tools/raspi/capture_ros2_control_hardware_evidence.sh
```

## 로그

```bash
systemctl status motionbrain-perception.service --no-pager
systemctl status motionbrain-dashboard.service --no-pager
systemctl status motionbrain-ros-bridge.service --no-pager

journalctl -u motionbrain-perception.service -n 100 --no-pager
journalctl -u motionbrain-dashboard.service -n 100 --no-pager
journalctl -u motionbrain-ros-bridge.service -n 100 --no-pager
```

## 확인할 ROS2 토픽

브리지가 제공하는 읽기 전용 토픽:

- `/motionbrain/status_typed`
- `/motionbrain/routine_typed`
- `/motionbrain/diagnostics`
- `/motionbrain/events_typed`
- `/camera/detection_typed`
- `/joint_states`
- `/motionbrain/end_effector_pose`
- `/motionbrain/kinematics_typed`
- `/motionbrain/control_guard_typed`
- `/motionbrain/mission_state_typed`

조회 예시:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /motionbrain/status_typed --once
ros2 topic echo /motionbrain/control_guard_typed --once
```

## HTTP 인터페이스

ESP32 제어기:

- `GET /status`
- `GET /events`
- `GET /routine`
- `POST /command`
- `POST /routine`

ESP32-CAM:

- `GET /status`
- `GET /capture`
- `GET /stream`은 설계상 HTTP 410을 반환한다. `/capture` 또는 Pi 추적 프레임을 사용한다.
- `POST /camera`

Pi 서비스:

- 대시보드 `GET /api/config`
- 대시보드 `GET /api/status`
- 인식 `GET /health`
- 인식 `GET /api/detection`

상태 변경 HTTP 호출은 토큰 검사가 구현된 경로에서 로컬 명령 토큰을 요구한다. 실물 작업자가 없는 경우 근거 수집에는 읽기 전용 엔드포인트를 사용한다.

## 장애 복구 절차

| 증상 | 먼저 확인할 항목 | 복구 방법 |
| --- | --- | --- |
| 대시보드 접근 불가 | `check_dashboard_health.sh`, 서비스 상태 | 대시보드·인식 서비스 재시작 |
| Pi 호스트명 확인 실패 | `python3 tools/raspi/check_pi_ssh_target.py --skip-remote`, 라우터 DHCP 임대, mDNS | `motionbrain-pi.local` 우선, `.davolink`는 대체 경로 |
| Pi 포트 22 닫힘 | `nc -vz motionbrain-pi.davolink 22`, 콘솔의 `ssh.socket` 상태 | `ssh.socket` 또는 `ssh.service` 활성화·시작 |
| SSH 별칭이 이전 IP로 연결됨 | `ssh -G motionbrain-pi`, 라우터 및 `hostname -I`와 비교 | `~/.ssh/config`의 고정 `HostName` IP 제거 |
| SSH 인증 거부 | `ssh -vv motionbrain-pi`, 키 경로, 사용자 | `User motionbrain`과 올바른 키 사용 |
| SSH 호스트 키 변경 | 경고 대상 호스트, 최근 OS 재설치 여부 | 오래된 Pi 호스트명·IP만 `ssh-keygen -R`로 제거 |
| SSH 정상, 서비스 실패 | `systemctl status ...`, 저장소 경로, 환경 파일 | `/home/motionbrain/...` 경로나 서비스 환경 수정 |
| ROS2 토픽 없음 | `check_ros_bridge_health.sh`, `ros2 topic list` | 브리지 재시작, ROS 워크스페이스 오버레이 확인 |
| 토픽은 있지만 샘플 대기 멈춤 | 브리지 로그, 제어기 `/routine`, 샘플 타임아웃 | 브리지 재시작 후 `SAMPLE_TIMEOUT_SECONDS=25`로 재검사 |
| 카메라 영상이 오래됨 | ESP32-CAM `/status`, `/capture`, Wi-Fi, 인식 로그 | `/stream` 비활성 유지. 전원·네트워크를 먼저 확인하고 촬영 재시도 간격 조절로 회복하는지 본 뒤 재시작 |
| Pi 부하 과다 | `uptime`, `vcgencmd get_throttled`, 인식 `/health` 지연 | OpenCV 스레드를 줄이거나 인식 간격을 늘린 뒤 인식 서비스 재시작 |
| 제어기 명령 거부 | `/status`, 고장 래치, 토큰, ARMED 상태 | 실물 점검 후에만 고장 해제 |
| 루틴 차단 | `/routine`, 피드백 준비 상태, 활성 시퀀스 | dry-run 유지, 피드백 차단 사유 확인 |
| 토큰 누락 | 서비스 환경 파일, 대시보드 상태, ESP32 명령 거부 | `/etc/motionbrain/*.env` 복구 후 서비스 재시작 |
| 인식 결과가 오래됨 | 인식 `/health`, `/api/detection`, 카메라 URL, `nextCaptureDelayMs` | `/capture`와 카메라 URL 확인. 재시도 간격 조절로 회복 중이 아닐 때만 재시작 |
| 피드백 미설치 | `/motionbrain/routine_typed`, 진단의 `base_yaw_fault` | 실물 루틴 실행 비활성 유지 |

## 품질 점검표

| 계층 | 검증 방법 |
| --- | --- |
| ESP32 펌웨어 | `pio run` |
| ESP32-CAM 펌웨어 | `pio run -d firmware/esp32cam` |
| 호스트 계약 | `python3 -m unittest discover -s tests` |
| ROS2 워크스페이스 | GitHub Actions Jazzy 컨테이너의 `colcon build/test` |
| Pi 서비스 | `check_dashboard_health.sh`, `check_ros_bridge_health.sh` |
| 실물 시연 | 작업자 원격조작, 데드맨 해제, 제한된 구동 근거 |

이 프로젝트는 개인 로봇 시스템이며 상용 로봇군 운영 환경은 아니다. 설명할 수 있는 운영 경험은 실물 시제품의 CI·systemd·상태 점검이며, 대규모 기업 SRE나 상용 서비스 운영으로 확대하지 않는다.

대시보드의 정렬 보정은 토큰·안전 게이트를 통과하는 제한 명령이며 자율 시각 서보 제어가 아니다. 현재 컵 경로는 작업자 확인을 받는 dry-run 계획이며 실물 컵 파지 시연이 아니다. 별도의 립밤 수동 관찰 시연은 [반복 기록](docs/evidence/2026-10-17-lipbalm-repetitions.md)을 참고한다.
