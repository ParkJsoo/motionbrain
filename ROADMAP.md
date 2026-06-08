# MotionBrain 로드맵

[English roadmap](ROADMAP.en.md) | [README](README.md) | [포트폴리오](PORTFOLIO.md)

Last updated: 2026-06-08 KST

이 문서는 기존 phase 문서와 physical-AI MVP 계획을 대체하는 단일 공개
로드맵이다. 현재 기준선은 `demo-ready-20260608` 포트폴리오 스냅샷과 최신
`main` 문서다.

## 현재 기준선

MotionBrain은 이제 데모 가능한 임베디드 로보틱스 포트폴리오 프로젝트다.

- ESP32 5축 DC 모터 제어기와 안전 상태 머신.
- STM32 유선 텔레오퍼레이션/센서 계층, deadman, freshness check.
- ESP32-CAM 영상 입력과 `STREAM`, `SNAPSHOT`, `TRACKED` 모드.
- Raspberry Pi 대시보드/인식 서비스와 제한된 `cup` known-object detection.
- ROS2 Jazzy bridge, typed status/detection/joint/kinematics/guard/mission
  topic, RViz 증거.
- 최종 물리 텔레오퍼레이션 데모 README GIF/MP4 공개.
- 안정 스냅샷 태그: `demo-ready-20260608`.

## 로드맵 원칙

- 새 물리 증거 없이 claim을 넓히지 않는다.
- ESP32는 계속 actuator와 safety boundary를 담당한다.
- Pi/ROS2/perception은 action을 제안하거나 시각화할 수 있지만, embedded
  command boundary를 우회하지 않는다.
- sensing과 feedback이 개선되기 전까지 autonomous grasping은 범위 밖이다.
- 넓은 객체 인식 claim보다 하나의 제한되고 반복 가능한 workcell scenario를
  우선한다.

## Track 1: 포트폴리오와 지원

상태: active, 낮은 engineering risk.

목표:

- 완성된 데모를 명확한 지원용 산출물로 사용한다.

다음 작업:

1. 지원 시 `demo-ready-20260608`과 tag-pinned README media link를 사용한다.
2. 직무별 요약을 준비한다.
   - embedded firmware / motor control
   - robot system software
   - ROS2 integration
   - dashboard/perception tooling
3. 공개 변경 후 README, 포트폴리오 문서, demo media를 항상 같은 상태로 맞춘다.
4. 추가 검증 데모 없이 broad AI/autonomy claim을 넣지 않는다.

완료 기준:

- resume bullet, 면접 설명, README, portfolio 문서가 같은 이야기를 한다.

## Track 2: 하드웨어 피드백 업그레이드

상태: future hardware work.

목표:

- 더 자율적인 물리 동작을 시도하기 전에 필요한 feedback을 추가한다.

후보 업그레이드:

- gripper-mounted range 또는 contact sensing.
- 더 나은 camera 위치 또는 camera module.
- 반복 가능한 pose/trajectory 확인을 위한 base 또는 joint feedback.
- fixed-object 실험을 위한 더 안정적인 fixture/workcell.

다음 작업:

1. 먼저 닫을 feedback gap 하나를 고른다.
2. hardware 수정 전 `PIN_MAP.md`와 wiring docs를 갱신한다.
3. 새 signal을 motion에 연결하기 전 read-only로 검증한다.
4. 물리 sequence가 signal을 사용하기 전에 safety behavior와 test를 추가한다.

완료 기준:

- 새 feedback signal이 status/telemetry에 보이고, 실패 상태가 반복 가능하게
  설명된다.

## Track 3: 제한된 Perception To Action

상태: design next; physical execution deferred.

목표:

- `detect -> align -> dry-run plan`에서 operator-confirmed, low-speed,
  fixed-workcell sequence 하나로 확장한다.

허용 범위:

- known target 하나.
- fixed workcell 위치 하나 또는 marker-assisted setup.
- motion 전 operator confirmation.
- low-speed, short-duration command sequence.
- 각 action 후 즉시 stop/status verification.

아직 허용하지 않는 범위:

- arbitrary object recognition.
- text-prompt object search.
- continuous visual servoing.
- improved feedback 없는 autonomous grasping.

다음 작업:

1. marker-assisted 방식 또는 fixed-known-object 방식을 선택한다.
2. 실행 precondition을 정확히 정의한다.
3. 첫 구현은 dry-run/log-only로 유지한다.
4. dry-run state가 안정된 뒤에만 guarded physical sequence 하나를 추가한다.

완료 기준:

- 시스템이 autonomy를 과장하지 않고 guarded plan을 만들고 설명할 수 있으며,
  물리 실행은 operator-confirmed 상태로 유지된다.

## Track 4: ROS2와 Ops Hardening

상태: robotics-system 직무용 optional polish.

목표:

- host-side system을 더 쉽게 시연, 디버그, 설명할 수 있게 만든다.

다음 작업:

1. systemd service docs와 health check를 최신 상태로 유지한다.
2. 새 milestone이 behavior를 바꿀 때만 간결한 ROS2 evidence를 추가 캡처한다.
3. JSON payload 증가가 유지보수 문제가 될 때만 richer typed message를 고려한다.
4. RViz/TF/RobotModel evidence를 physical demo story와 맞춘다.

완료 기준:

- reviewer가 짧은 runbook과 screenshot/log excerpt만으로 controller health,
  perception state, ROS2 bridge behavior를 이해할 수 있다.

## 삭제한 기존 로드맵

기존 phase roadmap, 한국어 장기 roadmap, physical-AI MVP plan은 완료된 내용이
현재 기준선에 흡수됐고, 남은 항목은 위 track으로 재구성됐기 때문에 제거했다.

삭제한 문서:

- `PHASE3_PLAN.md`
- `로드맵.md`
- `docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md`

앞으로 공개 로드맵은 이 파일 하나를 기준으로 관리한다.
