# 로봇 모델과 학습 결과를 눈으로 확인하기

[문서 색인](README.md) · [프로젝트](../README.md) · [실물 보정](arm-calibration.md) · [학습 설계](visual-action-learning.md)

다섯 도구를 연결해 팔 모델과 저장된 실제 평가를 확인한다. RViz는 관절·좌표계, Gazebo와 MuJoCo는 같은 URDF에서 변환한 가상 팔, Rerun은 실제 평가의 두 카메라와 사전 예측·실측, W&B는 모델·기준선 지표를 담당한다. W&B의 로컬 지표 미리보기는 W&B 웹 대시보드와 구분한다. 실물 제어·실시간 자세 추종·가상 데이터 수집·정책 학습은 이 연결의 검증 범위에 포함하지 않는다.

현재 카메라 영상·관측점·실측 M4를 계속 갱신하려면 별도의 [읽기 전용 실시간 표시](live-observation.md)를 사용한다. 아래 저장 평가 재생과는 다른 입력 경로이며, 실물 제어·학습·전체 팔 자세 추종을 수행하지 않는다.

## 설치와 모델 표시

Mac에서는 Docker Desktop을 실행한다. 컨테이너는 호스트 플랫폼의 ROS2 Jazzy 이미지를 사용한다. Apple Silicon에서 x86 이미지의 Qt GUI가 정지하면 네이티브 arm64 이미지를 다시 빌드한다. Mesa 소프트웨어 OpenGL을 사용하되 Gazebo의 Qt Quick 백엔드를 `software`로 바꾸지 않는다. Gazebo 장면 플러그인에는 OpenGL이 필요하다.

저장소 루트에서:

```bash
python3 -m venv --system-site-packages .local/venvs/visual-tools
.local/venvs/visual-tools/bin/python -m pip install -r tools/visualization/requirements.txt
.local/venvs/visual-tools/bin/rerun analytics disable
.local/venvs/visual-tools/bin/python tools/visualization/prepare.py \
  --models-only --output .local/visual-tools
docker compose -f tools/visualization/compose.yaml build rviz
docker compose -f tools/visualization/compose.yaml up -d
.local/venvs/visual-tools/bin/mjpython tools/visualization/mujoco_viewer.py \
  .local/visual-tools/model.xml
```

`--output`은 기존 폴더를 덮어쓰지 않는다. 전체 평가 준비로 이미 같은 폴더에 모델이 있다면 `--models-only`는 생략한다. Mac 이외에서는 MuJoCo 실행기에 `python`을 사용할 수 있다.

| 도구 | 여는 방법 | 조작과 해석 |
| --- | --- | --- |
| RViz | [로컬 화면](http://127.0.0.1:6081/vnc.html?autoconnect=true&resize=scale) | 별도 `joint_state_publisher_gui` 창의 슬라이더로 가상 관절값 변경. 실측 팔 상태가 아님 |
| Gazebo | [로컬 화면](http://127.0.0.1:6082/vnc.html?autoconnect=true&resize=scale) | 가상 팔·바닥. 보기용 관절 목표 자세로 시작하는 가상 물리 실행이며 일시정지 가능 |
| MuJoCo | 위 명령의 네이티브 창 | 마우스로 시점 이동, `Control`에서 가상 위치 구동기 목표 변경 |

컨테이너에는 실물 HTTP 미러나 모터 실행기를 시작하지 않는다. ROS는 별도 도메인·localhost 발견, Gazebo는 별도 partition을 사용한다. Compose는 전용 bridge 네트워크를 사용하고 GUI 포트는 Mac의 loopback에만 공개한다. 컨테이너의 네트워크 기능 전체를 차단하는 구성은 아니지만 실물 연결 노드·출력 경로를 시작하지 않는다. 종료는 `docker compose -f tools/visualization/compose.yaml stop`으로 하며 네이티브 창은 닫는다.

## 외형과 보기용 자세

기본 화면은 OWI-535 사진·조립도를 참고해 직접 작성한 외형 근사다. 노랑·검정 베이스 층판, 평행 링크, 관절 케이스, 실제 손목 힌지와 구분한 M2 모터 케이스, 양쪽 집게, 흰 카메라 지지대를 같은 URDF의 여러 visual로 표현한다. 제조사 CAD나 정밀 스캔은 아니며, 케이스 지름·두께·나사·카메라 장착 치수는 외형 추정이다. 실측한 관절 중심 간 치수와 구분한다.

베이스 옆에 떠 있던 카메라 상자, 편심 0을 나타내던 얇은 파란 원판은 제거했다. 카메라 외형은 집게 몸체에 연결했지만 외부 좌표 보정을 뜻하지 않는다. 기존 `camera_link`는 호환용 미보정 프레임으로 남고 외형은 그리지 않는다. RViz 좌표축은 기본으로 숨기며 필요할 때 Displays의 TF를 켠다.

일부 RViz 버전은 [한 링크의 여러 visual에 첫 재질을 공유하는 문제](https://github.com/ros2/rviz/issues/1293)가 있다. RViz 입력만 색별 고정 표시 링크로 나누어 색을 보존한다. 원래 관절·도구 프레임과 모든 형상의 위치는 그대로이며 추가 `rviz_visual_*` 프레임은 외형 표시용이다. 이 파생물은 MuJoCo·Gazebo 물리 모델 생성에 사용하지 않는다.

세 Viewer의 보기용 관절 목표는 [`display-pose.json`](../tools/visualization/display-pose.json)에 둔다. 현재는 기존 치수와 측면 사진에서 추정한 어깨 63°·팔꿈치 −109°·손목 32°의 정적 근사이며, yaw와 집게 각도는 가정이다. 접힌 자세를 재현하는 표시용 팔꿈치 범위와 실물 제어 범위의 차이는 [사진 근사 절차](arm-calibration.md#사진과-기존-치수로-근사-자세-맞추기)를 참고한다. 이 기본 파일 자체가 실물 센서를 읽지는 않으며, 아래의 명시적 갱신 경로로 마지막 M4 실측값을 반영할 수 있다. 표시용 팔에는 중력을 보상한다(MuJoCo body `gravcomp=1`, Gazebo robot link `gravity=false`)고 지정하되 실제 엔진의 지원 여부와 별개로 물리 보정은 미완료다. Gazebo는 속도 명령과 표시 전용 effort 한도 10을 사용하는 임시 위치 추종기로 목표를 유지한다. 소스 URDF의 effort 계약은 바꾸지 않는다. 이는 실물 PWM·토크·하중 반응이 아니며 동역학 식별에는 사용할 수 없다. 선택적 시험 물체의 중력은 유지한다. 노랑·검정 부품이 구분되도록 배경·조명을 조정하고, 측면 사진과 비교하기 쉽게 기본 시점을 맞췄다.

두 집게는 하나의 개방 입력을 반대 방향으로 따르는 단순 회전 연동이다. ROS는 URDF mimic, MuJoCo는 joint equality, Gazebo Harmonic은 설치된 Bullet Featherstone의 mimic constraint를 사용한다. 실제 집게의 다절 링크·개구폭·접촉·마찰 모델을 식별한 것은 아니다. 외형 개수로 임시 질량이 늘지 않도록 링크별 질량·관성을 한 번만 지정한다.

마지막으로 확인한 M4 센서값을 세 표시 도구에 반영하려면, 검토한 단일 근사 기준점과 측정 근거를 명시한다. 실측값·출처·환산 자세는 `.local/`에만 저장하며 나머지 네 관절은 기존 가정값을 유지한다.

```bash
python tools/visualization/update_pose.py \
  --sensor-deg <마지막_M4_센서값> --anchor .local/<근사_기준점.json> \
  --evidence .local/<측정_근거> --output .local/<현재_표시_자세.json> --apply-viewers
.local/venvs/visual-tools/bin/mjpython tools/visualization/mujoco_viewer.py \
  .local/visual-tools/model.xml --pose-file .local/<현재_표시_자세.json>
```

`--apply-viewers`는 이미 실행 중인 `motionbrain-visual-rviz`와 `motionbrain-visual-gazebo`만 갱신한다. RViz는 첫 적용 때 표시용 발행기로 전환하며 컨테이너를 한 번 재시작한다. 이후 같은 자세 파일을 갱신하면 MuJoCo와 RViz가 다시 읽고, Gazebo에는 저장 자세를 다시 전달한다. 화면에는 마지막 M4 값의 근사 환산·다른 관절 미확인·실시간 추종 아님·동역학 미보정을 표시한다. 이 경로는 실물 ROS 노드·센서 통신·모터 명령을 사용하지 않으며, 기준점의 비활성 제어 보정을 활성화하지 않는다.

초록 상자는 독립 낙하 진단 물체였으며 기본 화면에는 없다. 필요하면 `prepare.py --models-only --physics-test-object --output .local/physics-diagnostic`처럼 별도로 생성한다. 그 물체의 낙하·바닥 접촉은 로봇의 집기 성공을 뜻하지 않는다.

## 수집 파일 추종과 M1 평가 표시

수집기가 저장한 외부 영상 파일을 따라 표시하는 경로는 추가 카메라 요청 없이
동작한다. 저장 중·수신 지연·수집 종료 후 마지막 기록을 구분하며, 광학 노출에
동기화된 실시간 영상이나 실물 관절 추종을 뜻하지 않는다.

M1 턱축 모델의 전용 표시기는 원본·사전 예측·고정 모델과 평가 분모를 검증해
Rerun으로 보낸다. 아래 500ms 들기 평가 어댑터와는 다른 관측 계약이다.
두 경로의 명령 예시와 검증은 [실험 파이프라인](experiment-pipeline.md#다섯-도구의-역할)을
따르고, M1 전용 표시기는 [고정 후 평가 절](experiment-pipeline.md#m1-shaft-전용-수집고정-후-평가)을
참고한다. 여러 화면의 같은 원본을 새 평가 표본으로 세지 않는다.

## 실제 고정 평가의 재생과 비교

공개 저장소에는 비공개 원본 영상이 없다. 이 절의 어댑터는 `motionbrain.fixed_scene_lift.short500.v1` 형식의 모델과 평가 원본을 지원한다. 다른 관측 계약이나 M1 원호 모델을 자동 혼합하지 않는다. `EXPERIMENT_DIR`에는 해당 형식의 로컬 원본 폴더를 직접 지정한다.

```bash
.local/venvs/visual-tools/bin/python tools/visualization/prepare.py \
  --experiment "$EXPERIMENT_DIR" --output .local/visual-evaluation
.local/venvs/visual-tools/bin/python tools/visualization/serve.py .local/visual-evaluation
```

[실험실](http://127.0.0.1:8790)에서 두 영상의 전후 스냅샷, 사전 예측·실측·오차, 모델과 명령 평균·무변화 기준을 비교한다. [실제 Rerun Viewer](http://127.0.0.1:9090/?url=http%3A%2F%2F127.0.0.1%3A8790%2Fevaluation.rrd)도 같은 기록을 연다. `replay_step`은 전후 재생 순번이다. 카메라·제어기 시계를 합친 실시간 축이나 동시 촬영의 증거가 아니다. 서로 다른 평가 사례의 곡선도 분리한다.

준비 도구는 모델·사전 결정·결과·이미지의 해시와 관측 계약을 확인하고, 평가 분모와 기준선 오차를 다시 계산한다. 원본을 변경하거나 재학습하지 않는다. `.rrd`와 복사된 영상은 비공개 `.local/`에만 저장한다. 서버는 허용한 파일만 제공하며 디렉터리·경로 이탈·symlink 이탈을 차단한다. 종료는 서버 터미널의 `Ctrl+C`다. 8790 또는 9090 포트가 사용 중이면 기존 서버를 덮어쓰지 않는다.

W&B SDK는 지표·평가표를 `offline` 모드로 기록한다. 로컬 비교 화면은 같은 검증 지표를 표시한다. 웹 게시 후보에는 모델 해시, 학습·평가 개수, 방법별 MAE, 사례별 예측·실측·모델 선택 여부만 포함한다. 원본 이미지·장치 주소·카메라 식별자·소스 코드는 게시하지 않는다.

```bash
# 외부 전송 없는 게시 내용 미리보기
.local/venvs/visual-tools/bin/python tools/visualization/publish_wandb.py \
  --experiment "$EXPERIMENT_DIR"
```

별도 게시 승인과 사용자의 정상 W&B 로그인이 완료된 경우에만 같은 명령에 `--publish`와 필요한 `--entity`를 추가한다. 프로젝트 공개 범위·요금·계정 조건은 게시 전에 확인한다. 오프라인 실행 폴더를 통째로 동기화하는 대신 위 허용 목록으로 새 실행을 만든다. 포털에 웹 링크를 연결하려면 `--receipt .local/visual-evaluation/wandb-publication.json`처럼 새 영수증 경로를 지정한다. 영수증의 모델 해시·지표·평가 사례가 현재 실험과 일치할 때만 링크와 검증 문구를 표시한다.

## 검증 범위와 한계

실물 크기·관절 기준은 [치수 원본과 보정 절차](arm-calibration.md)를 따른다. 공통 JSON에서 FK 기본값과 5개 URDF 치수를 대조하고, 별도 후보 URDF·MuJoCo·Gazebo 모델을 생성할 수 있다. 어깨 높이·링크 길이의 수동 판독과 사진 기반 표시 자세를 반영했으며, 촬영별 첫 프레임 등록과 M4 읽기 전용 근사 기준점 후보는 별도 도구로 확인한다. 이 근거가 화면을 실제 팔의 현재 자세로 자동 갱신하지 않으며 전체 기하·관절 영점 보정은 미완료다.

URDF·FK·IK의 위치·방향 일치와 MuJoCo 변환 후 `tool0` 일치를 소프트웨어로 검사한다. Gazebo SDF의 문법과 GUI 표시, 실제 SDK의 Rerun 기록·W&B 오프라인 기록은 별도로 확인한다.

시뮬레이션 질량·관성·위치 구동기는 임시값이다. 팔 링크의 충돌과 양턱 접촉·파지 모델은 없으며, 선택적으로 추가한 독립 시험 물체와 바닥의 접촉만 지원한다. 물리 모델 식별·실물 보정·정책 학습·집기 성공을 주장하지 않는다. 실제 평가 두 건의 지표를 여러 조건의 일반화나 전체 팔 연속 집기 성과로 확대하지 않는다.
