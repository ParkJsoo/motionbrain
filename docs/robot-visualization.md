# 로봇 모델과 학습 결과를 눈으로 확인하기

[프로젝트](../README.md) · [학습 설계](visual-action-learning.md)

다섯 도구의 로컬 연결은 실물 제어와 분리한다. RViz는 관절·좌표계, Gazebo와 MuJoCo는 같은 URDF에서 변환한 가상 팔, Rerun은 실제 평가의 두 카메라와 사전 예측·실측, W&B는 모델·기준선 지표를 담당한다. W&B의 로컬 지표 미리보기는 W&B 웹 대시보드와 구분한다.

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
| Gazebo | [로컬 화면](http://127.0.0.1:6082/vnc.html?autoconnect=true&resize=scale) | 가상 팔·바닥·낙하 시험 물체. 재생 버튼으로 가상 물리 실행 |
| MuJoCo | 위 명령의 네이티브 창 | 마우스로 시점 이동, `Control`에서 가상 위치 구동기 목표 변경 |

컨테이너에는 실물 HTTP 미러나 모터 실행기를 시작하지 않는다. ROS는 별도 도메인·localhost 발견, Gazebo는 별도 partition을 사용한다. Compose는 전용 bridge 네트워크를 사용하고 GUI 포트는 Mac의 loopback에만 공개한다. 컨테이너의 네트워크 기능 전체를 차단하는 구성은 아니지만 실물 연결 노드·출력 경로를 시작하지 않는다. 종료는 `docker compose -f tools/visualization/compose.yaml stop`으로 하며 네이티브 창은 닫는다.

## 실제 고정 평가의 재생과 비교

공개 저장소에는 비공개 원본 영상이 없다. 현재 어댑터는 `motionbrain.fixed_scene_lift.short500.v1` 형식의 모델과 평가 원본을 지원한다. 다른 관측 계약이나 M1 원호 모델을 자동 혼합하지 않는다. `EXPERIMENT_DIR`에는 해당 형식의 로컬 원본 폴더를 직접 지정한다.

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

URDF·FK·IK의 위치·방향 일치와 MuJoCo 변환 후 `tool0` 일치를 소프트웨어로 검사한다. Gazebo SDF의 문법과 GUI 표시, 실제 SDK의 Rerun 기록·W&B 오프라인 기록은 별도로 확인한다.

시뮬레이션 질량·관성·위치 구동기는 임시값이다. 팔 링크의 충돌과 양턱 접촉·파지 모델은 없으며, 독립 시험 물체와 바닥의 접촉만 포함한다. 물리 모델 식별·실물 보정·정책 학습·집기 성공을 주장하지 않는다. 실제 평가 두 건의 지표를 여러 조건의 일반화나 전체 팔 연속 집기 성과로 확대하지 않는다.
