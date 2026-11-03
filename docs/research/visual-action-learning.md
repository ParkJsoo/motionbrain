# 시각 행동 학습의 연구 근거

[동작 학습 설계](../visual-action-learning.md)

초기 검토는 2026-10-30이며, 2026-11-03에 소규모 수집과 불확실성 관련 외부 문헌을 추가 확인했다. 로봇이 자신의 움직임과 명령을 연결해 배우는 연구를 중심으로, 논문의 실험 조건과 MotionBrain에 적용할 아이디어를 구분한다. 아래 구현 방향은 프로젝트의 설계 제안이며 해당 논문의 성능을 재현했다는 의미가 아니다.

## 명령에 따른 움직임 관계

**Li 외, Controlling diverse robots by inferring Jacobian fields with deep networks, Nature, 2025.**

명령 변화에 로봇의 각 공간점이 어떻게 반응하는지 Neural Jacobian Field로 학습하고, 여러 구조의 로봇에서 영상 기반 제어를 시연했다. 학습에는 여러 시점의 영상(실험 구성 12대), 제어에는 단일 카메라를 사용했다. 준정적 동작을 가정하며 고속 과도 동역학 전체를 모델링한 결과는 아니다. [논문](https://www.nature.com/articles/s41586-025-09170-0) · [프로젝트·영상](https://sizhe-li.github.io/publication/neural_jacobian_field/) · [저자 코드와 2D 튜토리얼](https://github.com/sizhe-li/neural-jacobian-field)

MotionBrain에서는 명령 채널별 시각 이동의 민감도를 참고한다. 우선 단일 시점의 팔 특징점 변화와 PWM·지속시간 사이의 유한 움직임 모델을 만들고, 주변의 국소 관계를 살펴본다. PWM을 관절각 변화로 대신 넣거나 단일 카메라 파일럿을 원 논문 재현이라고 표현하지 않는다. 원 저장소의 3D 학습 환경은 CUDA 계열 구성으로, 현재 Mac에 그대로 적용 가능한지 검증하지 않았다.

## 행동 후 관측 예측과 계획

**Ebert 외, Visual Foresight: Model-Based Deep Reinforcement Learning for Vision-Based Robotic Control, 2018.**

로봇이 자율적으로 수집한 상호작용으로 행동에 따른 미래 영상을 학습하고, 목표 픽셀·목표 이미지 등을 기준으로 후보 행동을 선택한다. 실물 물체 조작과 학습 때 보지 않은 물체에 대한 실험을 제시한다. [논문](https://arxiv.org/abs/1812.00568) · [저자 코드](https://github.com/SudeepDasari/visual_foresight)

가져올 핵심은 예측→선택→실행→재관찰 과정이다. 첫 버전은 전체 영상 생성 대신 팔 특징점 변화 예측으로 축소한다. 공개 코드는 수집·계획·제어를 포함하지만 영상 예측 모델 학습은 별도이며, 오래된 Python 환경을 전제로 하므로 구조를 참고한다.

## 부정확한 몸체와 상태 변화에 적응

**Kawaharazuka 외, Self-Supervised Learning of Visual Servoing for Low-Rigidity Robots Considering Temporal Body Changes.**

저강성 MyCobot에서 스스로 물체를 놓고 같은 동작으로 다시 집는 수집 방식과, 영상·명령 시계열 모델을 이용한 시각 제어를 제시한다. 몸체·카메라 상태의 변화를 작은 내부 변수로 적응시킨다. 제어 입력은 목표 관절각이며 동작 재현성을 활용한다. [논문 공개본](https://arxiv.org/abs/2405.11798) · [네트워크·수집·실험 본문](https://arxiv.org/html/2405.11798v1)

MotionBrain에서는 최근 영상과 명령의 이력, 장비 상태에 따른 보정 아이디어를 참고한다. 처음부터 놓기·재파지 자동 수집을 전제하지 않고, 짧은 동작의 재현성이 확인된 뒤 검토한다. 목표 관절각 입력과 DC 모터 PWM 입력의 차이를 별도로 모델링해야 한다.

## 기존 제어와 학습한 보정의 결합

**Johannink 외, Residual Reinforcement Learning for Robot Control, ICRA 2019.**

기존 피드백 제어가 처리하는 부분에 강화학습의 잔차 출력을 더해 접촉·마찰의 영향을 보완한다. 실제 로봇의 블록 조립 실험을 제시한다. [논문](https://arxiv.org/abs/1812.03201) · [저자 프로젝트·영상](https://residualrl.github.io/)

MotionBrain에서는 기구학이나 간단한 영상 제어로 기본 움직임을 구하고, 예측과 실제 이동의 차이를 학습해 보정하는 후속 구조로 활용한다. 처음부터 전체 제어를 강화학습에 맡기는 계획은 아니다. 저자 프로젝트에서 즉시 사용할 통합 코드까지 확인한 것은 아니다.

## 적은 상호작용에서 예측 불확실성 활용

**Chua 외, Deep Reinforcement Learning in a Handful of Trials using Probabilistic Dynamics Models, NeurIPS 2018 — PETS.**

확률적 동작 모델 앙상블과 궤적 샘플링으로 예측 불확실성을 전달하고, 후보 행동을 계획한다. 벤치마크에서 적은 표본으로 학습하는 효과를 비교했다. 제목의 적은 시도 수를 MotionBrain 실물 팔의 필요 데이터량으로 환산하지 않는다. [논문](https://arxiv.org/abs/1805.12114) · [저자 코드](https://github.com/kchua/handful-of-trials)

팔 특징점·사용 가능한 센서·명령 이력을 작은 상태 벡터로 만들고, 작은 모델 여러 개의 예측과 불일치를 비교하는 방법을 검토한다. 기존 공개 환경은 오래된 MuJoCo 기반이므로 알고리즘을 참고하며, 불일치 자체를 충돌 방지 보증으로 쓰지 않는다.

## 팔 전체 형태를 예측하는 자기 모델

**Hu, Lin, Lipson, Teaching Robots to Build Simulations of Themselves, Nature Machine Intelligence, 2025.**

로봇의 형태와 기구학을 학습한 자기 모델로 움직임 계획과 몸체 변화 대응을 연구했다. 공개 자료에는 고정 RGB 카메라와 관절 엔코더를 사용했다고 명시돼 있다. [논문 공개본](https://arxiv.org/abs/2311.12151) · [저자 코드](https://github.com/H-Y-H-Y-H/SelfSimRobot) · [데이터 설명](https://zenodo.org/records/14539921)

장기적으로 팔 전체의 공간 점유와 자세별 이동 가능성을 다루는 참고다. 현재 모든 관절각을 측정하지 못하는 MotionBrain에 곧바로 적용되는 카메라 전용 학습법으로 소개하지 않는다. 첫 2D 행동 모델 이후 시각 자세 추정과 결합할 후보로 둔다.

## 소규모 수집에서 가져올 판단 (2026-11-03)

**Deisenroth·Rasmussen·Fox, Learning to Control a Low-Cost Manipulator using Data-Efficient Reinforcement Learning, RSS 2011.** 자세 피드백이 없는 저가 로봇팔에서 깊이 카메라 관측과 확률 동작 모델로 제어를 학습했다. 초록·하드웨어·모델 절을 확인했다. 팔은 목표 서보 구성을 펄스 폭으로 명령하며, 물체를 집게에 미리 넣고 시작·목표 위치를 고정한다. MotionBrain의 직접 DC PWM·단일 RGB 영상과 다르므로 출력값이나 필요한 시도 수를 옮기지 않는다. 현재 상태와 명령에서 상태 변화를 배우되 관측·동작 불확실성을 함께 다룬다는 구조를 참고한다. [원논문](https://www.roboticsproceedings.org/rss07/p08.pdf)

**Sukhija 외, Optimistic Active Exploration of Dynamical Systems, NeurIPS 2023 — OPAX.** 초록·문제 설정·가정 절을 확인했다. 잘 보정된 확률 동작 모델의 불확실성을 사용해 정보 획득을 목표로 탐색하며, 로봇 시뮬레이션에서 평가했다. 현재 MotionBrain에는 이 전제를 확인한 불확실성 모델이 없으므로 자동 탐색기를 바로 도입하지 않는다. 적은 수집에서도 서로 다른 후보의 반응을 구별할 수 있어야 한다는 설계 동기로만 사용한다. [원논문](https://proceedings.neurips.cc/paper_files/paper/2023/file/77b5aaf2826c95c98e5eb4ab830073de-Paper-Conference.pdf)

**Memmel 외, ASID: Active Exploration for System Identification in Robotic Manipulation, ICLR 2024.** 학회 공식 초록을 확인했다. 초기 시뮬레이터로 탐색을 설계하고 소량의 실물 자료로 시뮬레이터를 보정한다. 현재 팔의 검증된 시뮬레이터를 전제할 수 없어 알고리즘을 직접 채택하지 않는다. 실물 시도 수보다 얻을 정보와 후속 제어 목적을 함께 고려할 근거로 참고한다. 본문 PDF는 열람 도구의 크기 제한으로 확인하지 못했으므로 세부 성능 비교에 사용하지 않았다. [학회 초록·원문 링크](https://proceedings.iclr.cc/paper_files/paper/2024/hash/ae8afec5dabb07091d8f85128250f2ff-Abstract-Conference.html)

이 문헌들은 현재 측정 흔들림을 없애거나 특정 PWM의 이동·안전을 보장하지 않는다. 첫 묶음에서는 단순한 기준선과 소수 후보의 반응을 먼저 비교하고, 작은 신경망·가우시안 프로세스·자동 정보성 탐색의 추가는 별도 자료에서 이득을 확인할 후속 선택으로 둔다. 실행 전 예정 프로파일과 실행 뒤 실제 드라이버 기록도 구분한다. 새 수집 범위는 [유한 출력 프로파일 설계](../finite-output-profile.md#학습용-첫-수집-설계)에서 관리한다.

## 적용 계획

논문별 아이디어를 적용할 순서와 평가 기준은 [동작 학습 설계의 단계](../visual-action-learning.md#단계와-판단-기준)에서 관리한다.

코드 공개 여부는 다운로드·설치·재배포 허용과 별개다. 외부 구현을 실제 편입할 때 해당 revision의 라이선스와 의존성을 확인한다.
