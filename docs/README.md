# 문서·실험 증거 안내

[프로젝트](../README.md) · [포트폴리오](../PORTFOLIO.md) · [운영](../OPERATIONS.md) · [핀맵](../PIN_MAP.md)

현재 연구 계획, 구현된 도구의 사용법, 날짜별 실측을 구분해 읽을 수 있는 색인이다. 날짜별 기록은 당시 구성의 결과이며 현재 장치 상태나 새 모델의 성능을 뜻하지 않는다.

## 연구와 구현 안내

| 문서 | 역할 |
| --- | --- |
| [시각 행동 학습](visual-action-learning.md) | 현재 단계, 관측·예측·기구학의 역할과 전체 팔의 집기·들기 계획 |
| [관절별 수동 수집](manual-data-collection.md) | 수집 화면·재생, M4 데이터셋 변환·학습·평가 사용법 |
| [물체·집게 상대 상태](relative-action-selection.md) | 수동 점·등록 특징의 별도 계약, 관절/자세별 모델과 고정 평가·질의 |
| [외부 영상의 팔 표식 상태](projected-arm-learning.md) | 물체 없는 팔의 네 픽셀 상태 학습·고정 평가와 별도 관측 계약 |
| [순차 접근·닫기 코어](model-grasp-trial.md) | 고정 모델 선택·접촉 후 들기 연결, 등록 특징·원호 관측과 실물 미검증 범위 |
| [현장 감독 수집](supervised-action-collection.md) | 기존 M2 정착 전후 17개 자료와 개발 평가 계약 |
| [유한 출력 프로파일](finite-output-profile.md) | M2 고정 출력 구현·실물 시운전과 계측 거부·불확실성 검토 |
| [무명령 관측 반복성](stationary-repeatability.md) | 관측창·조명 조건 비교와 측정 변동 해석의 한계 |
| [오프라인 행동 선택](offline-action-selection.md) | 저장 모델의 후보 순위·hold·abstain, 저장 상태 재생과 정책 평가 한계 |
| [시각 행동 수집기](visual-action-collection.md) | 행동 ID·영상 구간·실패 기록·감사와 공개 단발 CLI의 계약. 별도 반복 실측 경로와 구분 |
| [연구 근거](research/visual-action-learning.md) | 관련 연구의 실험 조건·대안·적용 아이디어와 현재 장치의 차이 |
| [기존 행동 데이터 계약](data-pipeline.md) | 구현된 M4·물체 박스 데이터 검증과 학습 CLI |
| [Mac 카메라 계측](mac-direct-camera.md) | 영상·USB 시각 연결 도구와 계측 결과 |
| [화면 표식의 오프라인 시간 검증](optical-challenge-offline.md) | 기존 계약을 위한 선택적 합성 실험. 물리 노출 인증이나 새 행동 학습의 선행 조건이 아님 |
| [대상 물체·집게 오프라인 CV](offline-cv.md) | 저장 라벨·예측의 검출 및 상대 위치 채점 |
| [제한된 CV 개발 학습](bounded-cv-training.md) | 단일·혼합 세션의 유한 학습 실행기 |
| [저장 체크포인트 평가](checkpoint-evaluation.md) | 확정 이미지 집합의 모델 평가 실행기 |
| [OV2640 진단](ov2640-register-diagnostic.md) | 보류된 센서 전용 진단. 외부 카메라 행동 학습의 선행 단계가 아님 |
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
| [2026-10-31 첫 행동 예측](evidence/2026-10-31-first-action-prediction.md) | M2 32개 전이, 논리 그룹 분할, 기준선 비교와 한계 |
| [2026-10-31 첫 모델 목표 접근](evidence/2026-10-31-first-model-goal-approach.md) | 모델 선택 M2 명령 단발의 목표 오차 변화와 반복 검증 한계 |
| [2026-11-04 HTTP 응답 지연](evidence/2026-11-04-http-response-gap.md) | 연결 종료·본문 재조립 지연의 구분과 수정 범위 |
| [2026-11-04 M4 행동 예측 평가](evidence/2026-11-04-m4-action-prediction.md) | 학습 18건·후속 18건, 명령별 평균과 영상 모델 비교, 중단 분모·평가 한계 |
| [2026-11-05 관절별 상대 상태 학습](evidence/2026-11-05-multi-joint-relative-learning.md) | 33건 확보·24건 학습·9건 후속 평가, 범위 거절·미구동 실패·미시작과 한계 |
| [2026-11-06 M3·M4 표식 학습](evidence/2026-11-06-projected-arm-m3-m4-learning.md) | 학습 9건·4건, 고정 후 관측 품질·상태 범위 거절과 미평가 분모 |
| [2026-11-07 M1 구동 전 예측 확인](relative-action-selection.md#2026-11-07-자세를-분리한-학습과-구동-전-예측-확인) | 자세별 분리 학습, 고정 후 새 1건 비교와 무변화 기준·지원 범위의 한계 |
| [2026-11-08 닫기→들기 연결의 관측·입력 범위](evidence/2026-11-08-close-lift-integration.md) | M1 수집 4·준비 3회와 새 계약 재감사·고정 모델, 평가·선택·연결 0회, 관측 모호함과 범위 거절 |

각 기록에는 조건·실패·분모·재현 한계를 남긴다. 비공개 원본 이미지·실행 응답은 이 공개 저장소에 포함하지 않는다.

## 공개 미디어

- [원격조작 GIF](assets/demo/motionbrain-demo.gif) · [MP4](assets/demo/motionbrain-demo.mp4): 2026-06-08 고정 태그의 약 54초 편집 시연. 손동작·버튼 원격조작과 조명 동작을 보여주며, 학습 정책의 자율 조작 성과가 아니다. README의 고정 태그 링크와 같은 파일이다.
- [RViz 모델 이미지](assets/motionbrain-rviz-robotmodel.png): URDF 모델·좌표축 표시 예시. [ROS2 안내](../ros2_ws/src/motionbrain_hardware_interface/README.md)에서 사용하며 실물 자세 추종이나 기구학 보정 정확도의 증거가 아니다.

영상·그림은 위 범위의 설명 자료이며 현재 장치 연결 상태를 표시하지 않는다.

- [로봇·학습 시각화 도구](robot-visualization.md): RViz·Gazebo·MuJoCo 모델 표시, Rerun 평가 재생, W&B 지표 기록·게시 경로와 한계.
