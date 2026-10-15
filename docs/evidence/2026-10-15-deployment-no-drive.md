# 2026-10-15 배포 및 비구동 검증

기준 코드: `559db54a8db5a9696e454e9e0a6068118a9657ca` (`main`).
**Pi 배포·두 ESP32 펌웨어 업로드 및 기본 비활성 상태의 비구동 통합 검증을 완료했다.** 처음에는 메인 ESP32의 저전압 재부팅으로 중단했으나, 케이블 재연결 후 정상 부팅·STM32·AS5600 복귀를 확인했다. 활성 동작의 중복 억제·deadline·물리 정지는 여전히 미검증이다.

**같은 날 후속 진단에서 CAM의 Wi-Fi 송신 경로 panic과 지속 무응답을 재현했다.** 아래 결과는 해당 시점의 단기 통과 기록이며 카메라의 장시간 안정성이 확보됐다는 뜻이 아니다. 최신 상태와 이전 코드 A/B 결과는 [카메라 연결 진단](2026-10-15-camera-connectivity.md)을 참고한다.

## 실험 조건

- 배터리함→XL4015 전원 분리는 작업자 보고로 확인했다. ARM·모터 구동은 수행하지 않았다.
- Pi 전원·네트워크 연결은 유지했다. 다른 보드들의 케이블은 재연결했다.
- ESP32-CAM은 로봇팔에 고정하지 않고 바닥에 놓인 상태다. 이번 결과는 조작용 시야·좌표 교정 증거가 아니다.
- 새 제한 동작 API는 기본 비활성 설정을 유지했다. STM32 펌웨어는 변경하지 않았다.

## 완료한 소프트웨어 검증

| 검사 | 결과 | 범위 |
|---|---|---|
| Mac 전체 unittest | 372개 통과, 11.129초 | 가상 시간·출력 및 파일/API 회귀 포함 |
| Pi staging 전체 unittest | 같은 372개 통과, 47.743초 | Pi 실행 환경에서의 소프트웨어 시험 |
| Pi staging ROS Jazzy build | 7개 패키지 성공, 5분 21초 | 변경된 메시지·노드·C++ 포함 |
| Pi staging colcon test | 72개, 오류·실패·skip 0 | `ROS_DOMAIN_ID=115`, localhost 한정으로 실제 서비스와 분리 |
| Pi 실제 운영 경로 clean build | 7개 패키지 성공, 5분 1초 | staging install을 복사하지 않고 운영 경로에서 재빌드 |
| ESP32 / ESP32-CAM build | 둘 다 성공 | 제한 실행 기본 비활성 |

위 테스트 수는 실제 모터 실험 횟수가 아니다. 가상 출력 시험 통과는 물리 PWM 파형·정지 시간·파지 성공을 입증하지 않는다.

## 배포 결과

- Pi SSH alias의 오래된 고정 주소를 `motionbrain-pi.local`로 수정했다.
- Pi 운영 저장소를 `99ea0d6`에서 `559db54`로 fast-forward하고 perception/dashboard/ROS bridge 서비스를 재시작했다.
- 세 서비스는 active, 관찰 시 자동 재시작 횟수 0, Pi `get_throttled=0x0`이었다.
- 배포 백업 생성 과정에서 상태 폴더가 root 소유로 생겨 M4 executor가 journal을 열지 못했다. `/home/motionbrain/.local/state`와 그 아래 `motionbrain` 폴더 두 곳의 소유권을 서비스 사용자로 수정하고 ROS 서비스를 재시작했다. journal·lock 파일 생성과 executor 노드 복귀를 확인했다. 기존 journal 삭제나 초기화는 하지 않았다.
- ESP32-CAM은 기존 4MB flash 백업 후 최신 펌웨어 업로드 및 esptool hash 검증에 성공했다.
- 메인 ESP32는 초기 USB 읽기에서 패킷 손상이 반복됐으나 재연결 후 4MB 전체 읽기(379초)가 성공했다. 이후 최신 펌웨어 업로드(64.98초)와 esptool hash 검증까지 성공했다.
- 업로드 성공과 정상 런타임 복귀는 구분한다. 초기 저전압 재부팅 이후, 추가 재연결과 아래 검사에서 정상 복귀를 확인했다. 이 재확인 과정에서는 펌웨어를 다시 업로드하지 않았다.

## 실제 장치로 통과한 비구동 검사

- 실제 CAM → Pi 임시 격리 perception 경로: 서로 다른 원본 frame ID 3개, exact JPEG hash 조회 6회 일치, 누락·퇴거 ID 404 검사 4회, stale 관측 제어 차단.
- CAM 재시작 전후 boot ID 변경 및 `first_dma_buffer` timestamp 의미 확인.
- 실제 운영 perception 경로: 같은 frame ID의 JPEG 2회 조회 bytes 일치, SHA-256이 detection/observation metadata와 일치. 미등록 frame ID는 이미지·관측 모두 404.
- 저장된 관측을 13초 뒤 재조회했을 때 `sourceAgeMs≈17099`, `sourceFresh=false`, `controlEligible=false` 확인.
- 카메라↔호스트 시계 교정은 수행하지 않았으며 `captureHostBoundsCalibrated=false` 유지 확인.
- 실제 ROS `CameraDetection`에서 source frame/boot ID, capture timestamp, clock domain, detection source ID가 전달됨을 확인.
- `/motionbrain/m4_write_result`에 존재하지 않는 같은 ID를 두 번 조회: 두 번 모두 `result_not_found`, `executionState=outcome_unknown`, `forwarded=false`. confirm 서비스는 호출하지 않았다.
- ESP32가 offline인 동안 ROS guard는 `ready=false`, `status_stale`로 차단됐다. 정상 제어 가능 상태로 해석하지 않는다.

### 추가 재연결 후 최종 검사 (같은 날 약 11:38 KST)

- USB 장치 정보와 실제 로그로 포트를 재식별했다. 메인 ESP32는 CP2102N(`/dev/cu.SLAB_USBtoUART`, `/dev/cu.usbserial-1130`은 같은 장치), CAM은 CH340(`/dev/cu.usbserial-1120`), STM32는 STLink(`/dev/cu.usbmodem11103`)였다. 이 번호는 다음 연결 때 다시 확인해야 한다.
- 메인 ESP32 UART 20초 관찰: 정상 Wi-Fi 연결·IDLE 부팅, brownout 0회. 부팅 초기 `i2c=NO_DEVICE` 이후 AS5600 연결 로그 및 HTTP의 connected/fresh/ready를 확인했다. 따라서 초기 한 줄만으로 지속적인 센서 미연결을 판정하지 않는다.
- CAM은 재연결 직후 네트워크 무응답이었다. 칩 MAC 식별(`30:76:f5:e5:ea:60`)과 esptool의 정상 부팅 리셋 후 기존 주소에서 복귀했다. 최종 `captureFailures=0`, Pi perception `ok=true`, `fresh=true`, `consecutiveErrors=0`.
- `/actions`: `enabled=false`, `active=false`, `recordCount=0`, 32자리 boot ID 확인. 다른 boot 조회는 409/`boot_mismatch`, 현재 boot의 없는 ID는 404/`not_found`; 둘 다 `outcome_unknown`.
- 미인증 POST 거부와, 인증된 동일 ID 요청 2회의 `403 bounded_actions_disabled` 거부를 확인했다. 이후에도 recordCount=0, 출력 로그 동일, drop=0. **비활성 경계 거부 시험이며 활성 동작의 dedup 시험이 아니다.**
- `/action_outputs`: 부팅 시 드라이버 write 5개 모두 PWM=0. 이는 드라이버에 전달한 값이며 물리 파형 측정이 아니다.
- 메인 ESP32 63.5초/13회 읽기 전용 관찰: 같은 boot ID 유지, 모든 모터 비활성·출력 0, STM32 실제 IMU 연결 및 AS5600 fresh/ready 유지(약 250.48°). 부팅 직후 UART line overflow에 따른 teleop/sensor parseErrors=1이 있었고 관찰 시작·끝 모두 1로 증가하지 않았다.
- CAM 복귀 후 exact JPEG/hash·없는 ID 404·stale 차단 및 ROS 원본 필드·읽기 전용 결과 조회를 다시 통과했다. 보관 관측은 재조회 시 약 19606ms age, sourceFresh/controlEligible=false였다.
- 최종 ROS guard는 현재의 비물리 운영 설정에서 `ready=true`, `armed=false`, `moving=false`, `target_detected=false`였다. 이 ready는 조작 허용이나 학습용 관측 적격 판정이 아니다. 관측 `controlEligible=false`, 시간 교정 미완료이며, 모터를 구동하려면 별도의 physical-mode guard 검증이 필요하다.
- Pi 서비스 3개 active·자동 재시작 0·throttled=0x0 유지. ARM과 실제 구동은 수행하지 않았다.

## 초기 실패 기록과 남은 실물 검증

추가 재연결 전 메인 ESP32의 20초 UART 수집에서는 Wi-Fi 초기화 시 `Brownout detector was triggered` 4회, 부팅 5회가 관찰됐다. 각 부팅에서 드라이버 초기화 로그는 PWM=0·방향 핀 LOW였지만 HTTP 서버까지 도달하지 못했다. 최초 통합 스크립트도 첫 `/status`에서 실패해 action POST까지 도달하지 않았다. 케이블 재연결 후 위 검증을 재개했다. Pi 연결은 계속 유지했다.

전압 저하 감지는 확인했지만 케이블·허브·전원 분배 중 어느 부분이 원인이었는지는 분리하지 못했다. 추가 재연결 후 단기 정상 동작이 장시간 전원 안정성을 입증하지는 않는다. brownout 검출은 변경하지 않았다.

다음 순서:

1. 배터리함→XL4015 분리를 유지한다. 이후 장시간 수집 전 전원 안정성을 관찰하고, 저전압이 재발하면 케이블·허브·전원 경로를 분리해 원인을 확인한다.
2. 카메라를 고정한 뒤 시야·시간 교정과 원본 획득 지연을 확인한다.
3. 별도 모터 구동 허용, 안전한 실험 자세 및 physical-mode guard 확인 후 한 축 최소 동작·deadline·취소·중복·응답 유실·잔여 이동을 측정한다. 이번 비구동 결과로 대체하지 않는다.
4. 실제 수집 adapter와 행동 전후 시각 경계를 연결한 뒤 학습 적격 실물 데이터를 수집한다. 이번 검사에서 학습 데이터나 조작 성능 개선 결과는 만들지 않았다.

## 복구 자료와 원본 로그

- Pi 이전 build/install: `/home/motionbrain/.local/state/motionbrain/deploy-20261015-559db54/`
- Pi 설정 백업: `/var/backups/motionbrain/20261015-559db54/`
- Mac 두 보드 flash 백업: `/tmp/motionbrain-firmware-backup-20261015/` (디렉터리 700, 파일 600). 무선 인증 정보가 포함될 수 있어 저장소에 추가하지 않는다.
- CAM 백업 SHA-256: `25a27ca1aca4483be996e10bb13b4bf2d803d7c3d6387ca3a7d7df699a84bf01`
- 메인 ESP32 백업 SHA-256: `f65de9f559d7390008459fb4c923322d7bc1e753b1c52f15beb6ab1ee91a2947`
- Mac `/tmp/motionbrain-*-20261015.log`: `main-backup-reconnect`, `main-upload`, `main-serial`, `cam-upload`, `camera-contract`, `live-camera`, `live-ros`, `live-api`, `final-status`, `mac-tests`, `pi-stage`, `pi-ros-tests` 등. `/tmp` 자료는 영구 보관이 보장되지 않는다.
- 재연결 후 로그: `main-reconnected-serial`, `main-alias-serial`(메인 ESP32의 다른 포트 이름), `cam-reidentify`, `live-controller`, `controller-soak`, `reconnected-integration`. 최신 완료 판정은 이 기록을 기준으로 한다.

이번 기록은 소스 변경이나 새 커밋을 의미하지 않는다. 배포 기준은 위 `559db54`이며, 문서 변경은 별도로 검토·커밋할 수 있다.
