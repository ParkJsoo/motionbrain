# 문서·실험 증거 안내

[프로젝트](../README.md) · [포트폴리오](../PORTFOLIO.md) · [운영](../OPERATIONS.md) · [핀맵](../PIN_MAP.md)

현재 연구 계획, 구현된 도구의 사용법, 날짜별 실측을 구분해 읽을 수 있는 색인이다. 날짜별 기록은 당시 구성의 결과이며 현재 장치 상태나 새 모델의 성능을 뜻하지 않는다.

## 연구와 구현 안내

| 문서 | 역할 |
| --- | --- |
| [시각 행동 학습](visual-action-learning.md) | 현재 목표, 단계, 입력·정답·평가 설계 |
| [연구 근거](research/visual-action-learning.md) | 논문 6편의 실제 실험 조건과 적용 아이디어 |
| [기존 행동 데이터 계약](data-pipeline.md) | 구현된 M4·물체 박스 데이터 검증과 학습 CLI |
| [Mac 카메라 계측](mac-direct-camera.md) | 영상·USB 시각 연결 도구와 계측 결과 |
| [립밤·집게 오프라인 CV](offline-cv.md) | 저장 라벨·예측의 검출 및 상대 위치 채점 |
| [제한된 CV 개발 학습](bounded-cv-training.md) | 단일·혼합 세션의 유한 학습 실행기 |
| [저장 체크포인트 평가](checkpoint-evaluation.md) | 확정 이미지 집합의 모델 평가 실행기 |
| [OV2640 진단](ov2640-register-diagnostic.md) | 보류된 센서 전용 진단. 아이폰 행동 학습의 선행 단계가 아님 |
| [ROS2 하드웨어 인터페이스](../ros2_ws/src/motionbrain_hardware_interface/README.md) | 추정·실측·확인형 M4 명령의 구현 범위 |

## 날짜별 실험 증거

| 날짜·기록 | 확인할 내용 |
| --- | --- |
| [2026-06-28 M4 위치 제어](evidence/2026-06-28-m4-shoulder-closed-loop.md) · [English](evidence/2026-06-28-m4-shoulder-closed-loop.en.md) | 하중별 최종 회귀와 그 이전 실패, 제한된 각도 범위 |
| [2026-07-13 ROS2 M4 실물 경로](evidence/2026-07-13-m4-physical-ros2-control.md) · [English](evidence/2026-07-13-m4-physical-ros2-control.en.md) | 확인형 단일 목표 실행과 중복·비활성 요청 거부 |
| [2026-10-15 유한 관측](evidence/2026-10-15-bounded-observation.md) | 실행 7요청·미실행 13요청을 구분한 신선도·저장 실패 |
| [2026-10-15 카메라 부팅 진단](evidence/2026-10-15-camera-boot-watchdog.md) | 호스트명·타임아웃 단위 수정과 미확정 장애 원인 |
| [2026-10-17 빈 HTTP 연결](evidence/2026-10-17-idle-http-client.md) | 초기 연결 대기 수정과 남은 통신 실패 |
| [2026-10-17 관찰 기반 파지](evidence/2026-10-17-lipbalm-repetitions.md) | 수동 명령으로 수행한 파지와 반복·보정 |
| [2026-10-30 인식 개발](evidence/2026-10-30-cv-development.md) | 학습 12장 결과, 남은 중복 오탐, 일반화 검증과의 구분 |

각 기록에는 조건·실패·분모·재현 한계를 남긴다. 비공개 원본 이미지·실행 응답은 이 공개 저장소에 포함하지 않는다.

## 공개 미디어

- [원격조작 GIF](assets/demo/motionbrain-demo.gif) · [MP4](assets/demo/motionbrain-demo.mp4): 2026-06-08 고정 태그의 약 54초 편집 시연. 손동작·버튼 원격조작과 조명 동작을 보여주며, 학습 정책의 자율 조작 성과가 아니다. README의 고정 태그 링크와 같은 파일이다.
- [RViz 모델 이미지](assets/motionbrain-rviz-robotmodel.png): URDF 모델·좌표축 표시 예시. [ROS2 안내](../ros2_ws/src/motionbrain_hardware_interface/README.md)에서 사용하며 실물 자세 추종이나 기구학 보정 정확도의 증거가 아니다.

영상·그림은 위 범위의 설명 자료이며 현재 장치 연결 상태를 표시하지 않는다.
