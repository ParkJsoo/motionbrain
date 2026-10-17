# 통합 현황과 다음 단계 — 2026-10-17

## 실제로 확인한 것

| 범위 | 확인한 결과 | 남은 한계 |
| --- | --- | --- |
| 카메라 계측 | Mac/iPhone 유한 수집, 원본 해시, callback/PTS 계측, 고정 프레임 detector replay | 두 세션의 PTS 진단은 각각 12/20으로 기준 미달. 광학 촬영 시각·USB 전송 경로 미검증 |
| 통신 | 빈 accepted HTTP 연결의 5초 대기를 재현하고 첫 byte 대기를 100ms로 제한, 승인된 펌웨어 1회 적용 | partial request/응답 쓰기 포함 전체 지연은 보장하지 않음. 이후 간헐 timeout이 남아 있음 |
| 실물 조작 | 외부 iPhone 화면을 관찰해 선택한 수동 명령으로 립밤 파지·들기, 이후 추가 잡기·들기·놓기 2회 | 첫 추가 반복은 닫기 보정 1회. 학습 정책의 제어나 무보정 반복 성공률 검증은 아님 |
| 시연 원본 | 최초 놓기와 두 반복의 21개 구동 기록, iPhone 21장, CAM 16장·receipt, 로컬 archive 3개 | 비동기 관찰 스냅샷. 데이터셋 builder는 세 archive 모두 학습 입력으로 거부 |
| 학습 기반 | NumPy ridge/무변화 기준선, 검증·분할·학습·평가 CLI | **실물 학습 적격 데이터 0개**, 실물 학습·조작 개선 미검증 |

로봇이 실제로 물체를 잡았다는 증거와, 학습된 모델이 로봇을 제어했다는 주장은
다르다. 현재 확인한 것은 전자이며, 외부 화면 관찰자가 동작을 선택했다.
소프트웨어 통합·Git main 반영은 펌웨어 재업로드나 서비스 재시작을 의미하지 않는다.

## 검증과 기록

- main 기준 전체 변경을 카메라, 통신/제어, 데이터 기록의 세 독립 리뷰로 검토했다.
  검토 당시 남은 P0/P1/P2 결함은 없었다. 실물 영상 판정도 별도 리뷰로 확인했다.
- 격리된 Python 3.13 환경에서 **전체 740개 테스트 통과**. 이전 시스템 Python의
  `cv2` 누락으로 실행하지 못했던 검사도 이번 전체 실행에 포함했다.
- macOS native 카메라 계측 helper 빌드 통과. 실제 카메라 실행은 하지 않았다.
- ESP32 제어기와 ESP32-CAM 빌드 통과. 펌웨어 업로드·새 모터 구동은 이 통합 작업에 포함하지 않았다.
- 최종 소스의 Linux PlatformIO/Python 및 ROS2 통합 결과는
  [PlatformIO CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml)와
  [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml)에서 커밋별로 확인한다.
  로컬 성공을 CI 성공이나 실물 학습 성능으로 바꾸어 표현하지 않는다.
- 과거 증거 문서의 `main unchanged`, `no CI run`은 각 기록 시점의 상태다.
  현재 통합 상태는 Git main ref와 해당 커밋의 CI 결과를 따른다.
- 내부 인계·원본·장치 주소·인증정보는 Git에 포함하지 않는다. 원본 archive에는
  임시 제어 토큰을 넣지 않으며 종료 후 임시 토큰 복사본을 제거했다.

## 다음 작업 순서

1. **학습용 행동·영상 연결**: 기존 bounded-action 계약에 실제 행동 전후 프레임,
   센서, 실제 출력, OFF·정착 상태를 연결한다. 수동 lease 로그를 같은 계약으로
   단순 재표기하지 않는다.
2. **시간·의미 기준 검증**: 검증된 광학 촬영 시각과 립밤 semantic anchor를
   확보한다. 영상 **100ms**, anchor **15초**, 저장 **250ms** 기준은 유지한다.
   PTS·수신 시각·오프라인 복사 시간을 해당 증거로 대신하지 않는다.
3. **유한 반복 수집**: 위 경로가 검증되면 새 view/session으로 성공·실패·보정
   사례를 수집한다. 구도나 관절 상태가 바뀐 과거 anchor/좌표는 재사용하지 않는다.
4. **학습과 별도 평가**: 세션 단위 분할, train 전용 정규화, 무변화 기준선 비교로
   실물 성능을 평가한다. 그 결과 이후에 학습 모델의 제한된 제어 연결을 검토한다.

간헐 HTTP 지연은 별도 미해결 항목이다. 불확실한 명령 응답을 자동 재시도하지
않는 기존 조건을 유지한다. 종료한 시험을 자동 재개하거나 통과할 때까지 기준을
완화하지 않는다. 장치 변경이 필요한 업로드·서비스 시작/재시작은 별도 승인 범위다.

## 상세 근거

- [Mac 카메라 계측과 detector replay](../mac-direct-camera.md)
- [HTTP 진단 도구](../controller-http-diagnostics.md)
- [빈 연결 수정·배포·잔여 timeout·최초 파지](2026-10-17-idle-http-client.md)
- [추가 잡기·놓기 반복](2026-10-17-lipbalm-repetitions.md)
- [원본 기록기와 데이터 경계](../hardware-demonstrations.md)
- [기존 NumPy 학습 계약](2026-10-15-offline-visual-dynamics.md)
