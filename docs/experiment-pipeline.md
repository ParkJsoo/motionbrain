# 반복 수집·관측·학습 파이프라인

단발 수집과 사후 처리를 분리한다. 수집기는 정해진 사전·사후 구간의 원본을 먼저
닫는다. 이후 관측기가 점을 찾지 못해도 원본의 수집 결과를 바꾸지 않는다.
관측 계약이나 주석을 수정한 뒤 같은 원본을 다시 처리할 수 있다.

이 경로는 자료 수집과 개발 재생용이다. 모델을 생성했다는 이유로 실물 제어에
적격하다고 판단하지 않는다. 장치 연결·촬영·실물 반복 운영의 검증은 아래 소프트웨어
검사 및 과거 자료 재생과 구분한다.

## 상태를 구분하는 방법

| 단계 | 판단 내용 | 불합격 처리 |
| --- | --- | --- |
| 실행 증거 | 명령·실제 출력·단일 관절·OFF·빈 큐·boot·HTTP 순서 | 미실행과 결과 불명을 구분하며 재전송하지 않음 |
| 원본 녹화 | 모든 프레임·메타데이터 해시, 시각, 종료 기록, 누락 | 실행 증거를 지우지 않고 녹화 상태만 거절 |
| 관측창 | 명령 전 사용 가능했던 영상과 OFF 후 고정 구간 | 자료 부족을 표시하고 보간하지 않음 |
| 센서 | 각 창의 유효 표본·원본 시각 범위 | 영상 전이와 별도로 거절 사유 기록 |
| 관측 | 두 프레임의 물체·양턱과 동일한 관측 계약 | 부분점·모호함을 기록하고 학습 전이 생성 보류 |
| 학습·평가 | 구도·관절별 학습, 명시적 분할, 기준선·이전 모델 비교 | 부족한 명령·범위·평가 자료를 각각 보고 |

기존 실패 원본은 실패로 보존한다. `allowFailedDevelopment: true`는 정상적인
출력·영상 감사를 다시 통과한 자료를 **개발 재가공**에 쓰는 옵션이다. 부족한
사후 녹화나 잘못된 픽셀을 복구된 것처럼 취급하지 않는다.

## 단발 원본 수집

기존 [수집 경로](manual-data-collection.md)와 [Mac 카메라 도구](mac-direct-camera.md)를
사용한다. `tools/collect_pipeline_episode.py`는 기본 실행에서 계획만 검사한다.
계획은 다음 필드를 갖는 비공개 JSON이다.

```json
{
  "schemaVersion": "motionbrain.pipeline_collection_plan.v1",
  "helperPath": ".local/bin/mac-camera-stream",
  "helperSha256": "실제 도구 파일의 SHA256",
  "controllerUrl": "컨트롤러 HTTP origin",
  "deviceId": "선택한 외부 카메라 ID",
  "viewId": "고정 구도 식별자",
  "sessionRoot": ".local/pipeline/sessions",
  "requestRoot": ".local/pipeline/requests",
  "poseContext": "실제 고정 자세·물체·부하 조건",
  "command": {"joint": "gripper", "direction": "close", "percent": 40, "durationMs": 350}
}
```

예시 문자열은 실제 값으로 바꾼다. 계획 검사에는 모터·카메라 접근이 없다.

```bash
python3 tools/collect_pipeline_episode.py --plan .local/pipeline/collection-plan.json
```

실제 단발 수집은 현재 현장 조건을 확인한 뒤 별도로 실행한다.

```bash
python3 tools/collect_pipeline_episode.py \
  --plan .local/pipeline/collection-plan.json --plan-sha '<검사한 계획 SHA256>' \
  --request-id '<이 단발의 고유 ID>' --execute --onsite-confirmed
```

인증값은 비공개 TTY에서 입력하며 계획이나 명령 인수에 넣지 않는다. 같은 요청 ID는
동일 결과 또는 미완료 예약을 반환하고 다시 움직이지 않는다. 같은 ID에 다른 명령을
넣으면 거절한다. 실패·불명 동작을 새 ID로 자동 대체하는 반복 기능은 없다.

현재 구현은 수동 자료 수집용이다. 사전 3.5초, OFF 이후 6.5초까지 수집을 유지하며
기존 출력·방향·저장·정지 검사와 공유 실행 journal을 사용한다. 모델 선택 제어기와
접촉·깊이·파지 성공 판단을 대신하지 않는다.

집게 카메라 전후 사진이 필요하면 계획에 `gripper: {"url": "카메라 HTTP origin",
"bootId": "확인한 카메라 부팅 ID"}`를 추가한다. 카메라의 지원 기한 6초로
사전 사진을 저장한 뒤 3.5초 동안 센서와 외부 영상을 수집하고 명령을 실행한다.
OFF 후 6.5초 동안 센서와 외부 영상을 수집한 다음 사후 사진을 요청한다. 외부
녹화는 사후 사진 요청 종료까지 유지해 느린 요청이 센서의 고정 관측창을 막지 않는다.
사전 사진 실패는 구동 전에 멈추고, 사후 사진 실패는 별도 상태로 보존하며 외부
녹화를 조기 종료하지 않는다. 원본은 세션의 `gripper-camera/`에, 기존
`bind_gripper_pair` 입력은 `gripper-pair/`에 같은 세션 ID와 원본 메타데이터로 남긴다.
두 카메라의 노출 동기화를 증명하는 것은 아니다. 관측 manifest에는 해당
`gripper-pair/`의 asset 루트 상대 경로를 `observer.gripperPairRoot`로 지정한다.

Python API의 선택적 `before_command(manager, gripper_pair)`는 사전 수집 구간이
끝난 뒤 실제 명령 직전에 호출한다. 관측·고정 모델·예측의 해시와 시각을 먼저
저장하는 용도이며, 반환값으로 예약된 명령을 바꾸지 않는다. 예외가 발생하면
구동 전에 중단하고 원본·요청 결과를 보존한다. 콜백 사용 여부도 요청 해시에
결합하며, 기존 콜백 없는 요청의 중복 조회 형식은 유지한다.

## 원본 재처리

`tools/run_learning_pipeline.py`에 manifest 하나와 비공개 출력 경로를 준다.

```bash
python3 tools/run_learning_pipeline.py \
  --manifest .local/pipeline/manifest.json --output .local/pipeline/output
```

manifest의 최상위 필드는 다음과 같다.

| 필드 | 의미 |
| --- | --- |
| `schemaVersion` | `motionbrain.experiment_pipeline.v1` |
| `artifactRoot` | 원본·등록·참조 파일을 읽을 명시적 루트. 비공개 자료는 `.local` 디렉터리를 루트로 지정 |
| `sources` | 고유 `id`, 루트 상대 `path`, `session.json`의 `sha256`, 개발 재가공 옵션, `observer` |
| `split` | `trainGroups`, `evaluationGroups`, 개발 재가공 학습과 OOD 진단의 명시적 옵션 |
| `previousModels` | 선택 사항. 비교할 기존 모델과 canonical JSON SHA256 |

source 경로는 정규화한 뒤 `artifactRoot` 내부인 경우에만 허용한다. 관측 asset의
상대 경로에서는 `..`와 심볼릭 링크 우회를 거절한다. 누락·변조된 source도 보고서
분모에서 삭제하지 않는다. 매 실행에서 원본을 다시 감사한다.

관측기의 `kind`는 `registered_scene`, `registered_rigid`, `registered_gripper`,
`analyst_points`다. 등록 관측은 등록·참조 파일 경로와 각각의 SHA256을 요구한다.
`context`에는 실제 `physicalView`, `object`, `poseRegime`을 지정하고 `captureGroup`으로
분할 그룹을 연결한다. M4 전용 강체 턱 관측을 M1 명령에 사용하면 거절한다.

수동 주석은 별도 관측 계약이다. 각 프레임의 `frameId`, `pixelSha256`,
`sourceFrameIds: [그 프레임 ID]`, `inputFrameOnly: true`, `accepted: true`,
`method: "analyst_points"`, 물체·양턱의 세 점을 요구한다. 자동 관측 결과와 같은
계약인 것처럼 섞지 않는다. 출처 선언이 분석가의 물리 계측 정확도 증명은 아니다.

카메라 이동·관절 고정 상태·물체·부하·점의 의미가 바뀌면 구도와 관측 계약을
다시 선언한다. 장치 ID나 영상 크기가 같다는 사실만으로 같은 구도로 간주하지 않는다.
서로 다른 관측 계약과 관절은 별도 모델로 분리한다. 같은 동작의 두 카메라는 같은
분할 안에서 각각 사용할 수 있지만 train/eval 사이의 원본·동작·세션·프레임 중복은 거절한다.

## 학습·평가 산출물

`output/latest.json`은 해시가 붙은 보고서를 가리킨다. `artifacts/`에는 단계별
불변 결과를, `models/`에는 canonical JSON 해시로 고정한 모델을 저장한다.
동일 입력과 처리 버전의 재실행은 같은 결과를 만들고 원본·기존 모델을 덮어쓰지 않는다.
수집·관측·학습 코드 및 주요 라이브러리 버전도 보고서에 포함한다.

명령당 두 전이는 학습의 최소 지원 수이며 실물 성능 보증이 아니다. 평가에는
상태 ridge·명령 평균·무변화의 분모를 표시한다. 이전 모델과의 비교는 같은 평가
자료에서 두 모델이 모두 지원하는 사례에 한정한다. 개선·동률·회귀·비교 불가를 구분한다.

기존 예측기의 입력 범위 거절은 유지한다. `allowOodDiagnostics: true`는 범위 밖의
오프라인 예측 오차를 별도로 보여줄 뿐, 제어 지원 범위나 평가 합격을 넓히지 않는다.
사후 재가공의 평가는 `developmentReplay`이며 `evaluationPassed`를 만들지 않는다.
새로운 고정 후 평가에는 실행 전에 고정된 관측·모델·입력·예측의 시간 증거가 별도로 필요하다.

## 다섯 도구의 역할

```bash
python3 tools/visualization/pipeline.py --latest .local/pipeline/output/latest.json \
  --output .local/pipeline/replay.rrd
```

Rerun은 검증된 전후 원본과 유효점·부분 관측·실패 사유·기준선 비교를 표시한다.
기본은 로컬 파일 저장이며, 기존 로컬 서버 연결은 `--connect`로 명시한다.
재생 영상은 실시간 영상으로 표시하지 않고 다른 카메라의 점을 겹치지 않는다.

수집 중에는 수집기가 이미 저장한 외부 영상 파일을 다음 표시기로 따라볼 수 있다.
카메라나 제어기에 추가 요청하지 않으며, 이 표시기의 실행은 모터를 움직이지 않는다.

```bash
python3 tools/visualization/collection.py \
  --sessions .local/pipeline/sessions --output .local/pipeline/file-viewer \
  --connect rerun+http://127.0.0.1:9876/proxy --seconds 0
```

`--sessions`는 읽을 수집 폴더를 명시하며 임의 실험 경로를 검색하지 않는다.
`--output`에는 새 비공개 출력 폴더를 사용한다. `--connect`를 생략하면 로컬 RRD에만
저장한다. `--seconds 0`은 Ctrl-C까지 표시하며, 양수는 지정한 초 뒤 표시기를 종료한다.
파일 갱신 중·지연·누락·검증 오류·수집 종료 후 마지막 기록을 구분한다. 이 화면은
저장 파일을 따라 표시하므로 실제 카메라 노출과 동기화된 실시간 추종이 아니다.
이 표시기는 외부 영상을 표시하고, 집게 카메라의 전후 관측은 사후 파이프라인에서
확인한다. 센서가 있으면 마지막 파일에 기록된 값으로 표시하며 현재 센서 직접 조회로
해석하지 않는다.

W&B에는 별도 평가·모델 버전 비교처럼 후속 판단에 필요한 결과만 선별해 게시한다.
이 파이프라인은 원본이나 준비 진단을 자동으로 웹에 올리지 않는다.
MuJoCo·Gazebo·RViz는 마지막 실측 기반의 정적 근사 자세·좌표·기구 구조 확인에
사용한다. 과거 자료 재생으로 현재 자세를 덮어쓰거나 미보정 동역학·접촉을 실측
학습 정답으로 사용하지 않는다.

## 검증 범위

모듈 테스트는 원본 변조, 부분 자료, 중복 요청, 중단, OFF 불명, 호출 중 명령 변경,
학습·평가 누출과 잘못된 카메라 overlay를 검사한다. 통합 테스트는 합성 native
패킷과 가짜 장치를 쓰며, 실제 원본의 재생 검증과 구분한다. 소프트웨어 검사와
과거 원본 재생만으로 새 카메라 구도의 자동 관측이나 실물 파지·들기를 검증했다고
해석하지 않는다.

2026-11-09 새 구도 실물 연결 검증에서는 고정된 집게 카메라 관측기가 별도 정지
영상 5/5와 M1 열기 단발 1회의 전후 사진을 통과했다. 외부 원본 99장과 집게 전후
사진 2장을 저장했고, OFF·원본·시간 감사를 거쳐 유효 전이 1건을 만들었다.
같은 요청 재조회는 장치에 접근하지 않았고, 원본 재처리는 동일 결과를 만들었다.
전이 1건은 학습 최소 지원 수에 부족하며 별도 평가 자료도 없다. 이 결과는
집기·들기나 반복 실물 성공률의 증명이 아니다. 이후 느린 집게 촬영이 M4 센서의
고정 관측창을 막지 않도록 대기 순서를 수정했으며, 수정한 순서의 센서 검증은
가짜 장치 회귀로 확인했다. 해당 M4 경로의 새 실물 동작은 실행하지 않았다.
