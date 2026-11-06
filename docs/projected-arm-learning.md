# 물체 없이 외부 영상의 팔 상태 학습하기

[수집 안내](manual-data-collection.md) · [상대 상태 학습](relative-action-selection.md)

외부 카메라의 고정 배경점과 같은 링크 위 두 기준점으로 움직임을 표현한다. 물체 중심과 집게 턱점을 쓰는 기존 상대 상태 모델과 별도 관측 계약이다. 기존 원본을 이용하더라도 새 기준점으로 다시 주석하고 별도 모델을 학습해야 한다. 두 표현의 좌표나 오차를 같은 값으로 취급하지 않는다.

## 상태와 정답

`background`는 고정 배경점, `toolFront`와 `toolRear`는 링크의 원위·근위 기준점이다. 논리 키의 이름이 실제 공구 끝이나 보정된 관절 중심을 뜻하지 않는다. 녹색 테이프 패치를 기준점으로 사용할 수 있다.

| 상태 성분 | 정의 | 단위 |
| --- | --- | --- |
| 원위점의 상대 x | `toolFront.x - background.x` | 영상 px |
| 원위점의 상대 y | `toolFront.y - background.y` | 영상 px |
| 링크 벡터 x | `toolRear.x - toolFront.x` | 영상 px |
| 링크 벡터 y | `toolRear.y - toolFront.y` | 영상 px |

입력은 명령 전 영상에서 얻은 네 상태와 예정 명령이다. 실제 출력·동작 후 센서는 사후 근거로 남기며 입력에 넣지 않는다. 정답은 OFF 이후 관측창에서 고른 영상 상태와의 차이다. 기존 `build_visual_session`으로 원본 해시·저장 시점·출력 정지·영상 관측창을 다시 검사한다. 사전 검토용 화면이 아닌 이 감사기가 선택한 입력·정답 프레임에 주석을 묶는다.

각 영상의 세 점을 독립적으로 확인하고 프레임 ID·픽셀 해시에 연결한다. 색 분할 등의 좌표 제안을 사용했다면 분석가 검토와 구분해 기록한다. 정수 반올림이나 분할 기준을 바꾸면 학습·평가에 같은 규칙을 적용한다. 작은 수치 변화나 계산상 소수점 정밀도를 물리 측정 정확도로 해석하지 않는다.

## 학습과 고정 후 평가

명세는 `motionbrain.projected_arm_manifest.v1`이며 `records`와 `evaluationGroups`를 포함한다. 각 레코드는 `sessionRoot`, `sessionSha256`, `annotation`을 가진다. 주석에는 `sessionId`, `captureGroup`, `sceneCondition`, `context`, `before`, `after`를 기록한다.

- `context`의 키는 `physicalView`, `poseRegime`, `landmarks`다. 같은 관절·카메라·영상 크기·기준점 정의를 유지한다.
- `sceneCondition`은 `object_present` 또는 `object_absent`다. 물체 제거 같은 조건 변화는 입력 특징으로 몰래 넣지 않고 평가 결과에 구분해 남긴다. 접촉·하중이 같았다는 보증은 아니다.
- `before`와 `after`에는 해당 영상의 `frameId`, `pixelSha256`, `accepted: true`, `method: "analyst_points"`, `inputFrameOnly: true`, `points`를 넣는다. `points`는 위 세 기준점의 `[x, y]`다.

```sh
python3 tools/replay_projected_arm_states.py \
  --manifest .local/projected-arm/train-manifest.json \
  --output-dir .local/projected-arm/train-v1
```

명령 조건별로 두 표본 이상일 때만 ridge 모델을 만든다. 반복 수가 모자란 조건과 거절된 원본은 지원·평가 성공으로 올리지 않는다. 모델을 고정한 뒤 새 평가 자료를 수집하고 학습·평가 세션, 명령 에피소드, 프레임이 겹치지 않는지 확인한다.

```sh
python3 tools/replay_projected_arm_states.py \
  --manifest .local/projected-arm/evaluation-manifest.json \
  --output-dir .local/projected-arm/evaluation-v1 \
  --frozen-model .local/projected-arm/train-v1/model.json \
  --model-sha256 '<고정한 모델 파일의 SHA-256>'
```

고정 모델 평가는 재학습하지 않는다. 학습 범위 밖 상태는 거절하고 같은 예측 가능 부분집합에서 명령별 평균·무변화 기준과 비교한다. 원위점 위치 오차와 링크 벡터 오차는 영상 px이며, 전체 상태 벡터 오차도 물리 거리·관절각 정확도가 아니다.

이 도구는 장치를 구동하지 않는다. 수집용 수동 명령, 오프라인 예측, 학습 모델의 실물 제어 성공을 구분한다. 물체 없는 팔 움직임 예측만으로 집기·들기·충돌 여유를 입증하지 않는다.
