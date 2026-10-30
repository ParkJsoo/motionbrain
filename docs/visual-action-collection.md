# 시각 행동 기록 수집기

[시각 행동 학습 설계](visual-action-learning.md) · [운영](../OPERATIONS.md)

명령 하나와 그 전·중·후 원본 프레임, 실행 응답, 실제 출력 기록을 같은 행동 ID로 묶는 최소 수집 코어다. **현재는 합성 입력과 이미 저장된 로컬 자료를 처리한다.** 실제 카메라·제어기 생산자 연결, 특징점 추적기, 학습과 예측 제어는 별도 구현·실물 확인이 필요하다.

기존 물체 박스·필수 M4 실측 계약과 다른 `motionbrain.visual_action_session.v1`을 사용한다. M4 센서값이나 물체 검출 결과 없이도 팔 영상을 기록할 수 있다. 기존 수집·학습 도구의 조건이나 적격 판정은 바꾸지 않는다. 이번 수집기의 모든 결과는 `trainingEligible=false`, `controlEligible=false`, 실물 학습 적격 전이 0건이다.

## 장치 없이 실행하기

Python 표준 라이브러리와 저장소 모듈만 사용한다. 카메라 권한, 네트워크, 영상 라이브러리 설치, 모델 가중치가 필요하지 않다. 출력은 항상 새 디렉터리로 지정하며 기존 실행을 덮어쓰거나 재개하지 않는다.

```sh
# 작은 합성 BGRA 프레임과 가상 명령·응답을 기록하고 검사
python3 tools/collect_visual_actions.py dry-run --output-dir .local/visual-actions/demo
python3 tools/collect_visual_actions.py audit .local/visual-actions/demo

# 결과 불명도 실패 자료로 보존. 이 예제와 감사의 종료코드는 2
python3 tools/collect_visual_actions.py dry-run --scenario unknown_result --output-dir .local/visual-actions/unknown
python3 tools/collect_visual_actions.py audit .local/visual-actions/unknown

# 전체 입력 형식을 담은 합성 예제 생성 → 저장 자료 재생
python3 tools/collect_visual_actions.py example --output-dir .local/visual-actions/example
python3 tools/collect_visual_actions.py replay --bundle .local/visual-actions/example/bundle.json --output-dir .local/visual-actions/replayed

python3 -m unittest discover -s tests -p test_visual_action_collection.py
```

예제의 PWM·시간·모터·영상 크기는 소프트웨어 검사용 값이며 실물 실행 계획이 아니다. `complete`, `no_motion`, `missing_after`, `unknown_result`, `controller_failed`, `missing_frame`, `corrupt_frame`, `interrupted`를 합성 시나리오로 제공한다. 명령을 보내는 옵션은 없다.

정상 기록·감사는 종료코드 0, 실패·불완전 기록·감사 오류는 2, 실제 사용자 중단은 130이다. 합성 `interrupted` 시나리오는 이미 끊긴 이벤트 목록을 재생하므로 2를 반환한다. `recordingComplete=true`는 기록 형식과 원본의 연결이 완결됐다는 뜻이며, 작업 성공·영상 정착·학습 적격을 뜻하지 않는다. `no_visible_motion`도 삭제하지 않고 정상 관측 결과로 보존한다.

## 저장 구조와 영상 구간

| 파일 | 내용 |
| --- | --- |
| `session.json` | 세션·구도·카메라 스트림·두 시계의 ID, 영상 형식, 유한 저장 예산 |
| `raw/*.json` | 받은 원본 이벤트. 잘못된 ID·시각·해시도 진단을 위해 보존 |
| `frames/*.bgra` | 원본 BGRA 바이트. 원본 해시와 저장 후 해시를 비교 |
| `events/*.json` | 연속된 영속 이벤트와 원본 파일의 경로·SHA-256·바이트 수 |
| `summary.json` | 시도 분모, 요청/실행 결과, 프레임 구간, 실패 사유와 적격 상태 |

구간 추출은 재인코딩한 MP4가 아니라 **원본 프레임을 참조하는 순서 있는 목록**이다. 각 프레임의 PTS, 호스트 수신 시각, 특징점과 원본 경로를 유지한다.

- `before`: 명령 전송 의도 기록보다 먼저 호스트에 도착한 프레임.
- `during`: 전송 의도부터 실행 결과 수신까지 도착한 프레임. 실제 모터 동작 시간과 같지 않다.
- `after`: 종료 응답과 소프트웨어 출력 OFF를 확인한 이후 도착한 프레임.
- `unresolved`: 결과 불명 또는 미전송 결과를 받은 뒤의 관찰. 출력 종료 후 영상으로 해석하지 않는다.

**호스트 수신 순서는 광학 노출 순서의 증명이 아니다.** 지연된 프레임이 종료 응답 뒤에 도착할 수 있다. 카메라 PTS와 제어기 밀리초는 원래 시계 영역을 유지하고, 서로 직접 빼거나 광학 노출 시각으로 변환하지 않는다. 카메라 스트림·구도·시계 ID 변경, PTS 반복·역행·epoch 변경은 새 세션으로 다룬다. 소프트웨어 OFF와 `settlingObservation`의 관찰 선언은 물리 정착 인증이 아니다.

## 생산자 입력 계약

전체 JSON 예제는 위 `example` 명령으로 생성한다. 번들은 `session`과 `events`를 가지며, 각 항목은 `event`와 번들 디렉터리에 상대적인 `imagePath`다. 영상이 없는 이벤트의 `imagePath`는 null이다. 절대경로·상위 경로·심볼릭 링크를 통해 원본을 가져오지 않는다. 실패한 재생에서 아직 읽지 않은 꼬리는 수집 폴더에 복사되지 않으므로 입력 번들도 원본으로 보존한다.

세션에는 다음 필드를 명시한다.

| 필드 | 의미 |
| --- | --- |
| `schemaVersion` | `motionbrain.visual_action_session.v1` |
| `mode` | `synthetic` 또는 `offline_import`. 선언만으로 실물 출처를 인증하지 않음 |
| `sessionId`, `viewId`, `streamId` | 한 수집 세션·고정 구도·카메라 스트림의 식별자 |
| `hostClockId`, `sourceClockId` | 이벤트 수신의 호스트 단조 시계와 원래 카메라 PTS 시계 |
| `width`, `height`, `rowBytes` | BGRA8 원본 형식. 패딩을 포함해 `rowBytes × height` 바이트를 저장 |
| `videoMirrored`, `videoRotationAngle`, `cameraViewFixed` | 생산자가 확인한 영상 방향·구도 선언 |
| `maxAttempts`, `maxFrames`, `maxDurationSeconds` | 수집기의 유한 처리 예산. 모터 실행 허용 범위와 다름 |

한 세션은 최대 100시도·10,000프레임·이벤트 시각 기준 3,600초, 한 원본 프레임은 최대 32MiB다. 더 작은 예산을 지정할 수 있다. 파일 저장은 동기식이므로 실시간 고해상도 연속 저장 처리량을 보장하지 않는다. BGRA 외의 영상·압축 파일 자동 디코딩은 제공하지 않는다.

모든 이벤트는 `type`, `at`(호스트 단조 시계의 초), `hostClockId`, `actionId`를 포함한다.

| 이벤트 | 추가 필드·역할 |
| --- | --- |
| `begin` | `episodeId`, `startPoseId`, `request`. 요청의 `commandId`와 `actionId`가 같아야 함 |
| `frame` | `frameId`, `viewId`, `streamId`, `sourceClockId`, `pts`, `timestampSemantics`, `pixelFormat`, `sha256`, `features` |
| `dispatch` | 실제 전송 직전에 생산자가 남길 의도. 이 수집기는 전송하지 않음 |
| `result` | `executionState`, `record`, `baselineOutputs`, `outputLog`. 요청과 응답·출력 연결을 검사 |
| `end` | `motionObservation`: `observed`/`no_visible_motion`/`unknown`, `settlingObservation`: `observed_still`/`unverified` |
| `failure` | `reason`: 실패 코드. 예: `camera_disconnected`. 주소·토큰·임의 예외 문자열을 넣지 않음 |

`request`는 기존 제한 행동 API의 필드를 그대로 쓴다. non-M4는 부호 있는 PWM·요청 지속시간, M4는 목표각·출력 비율·시간 한도이며 서로 바꾸어 해석하지 않는다. `result`의 `known_terminal`은 실제 명령과 일치하는 종료 응답을 요구하고, 출력 기록의 누락·순번·OFF를 검사한다. 같은 제어기 부팅 ID에서 실행 ordinal과 출력 sequence를 재사용하지 않는다. 제어기 32비트 시각의 정상 wrap과 역행을 구분한다.

프레임의 `pts`는 정수 `value`, 양의 정수 `timescale`, `epoch`를 가진다. `timestampSemantics`는 `presentation_time`, `pixelFormat`은 `BGRA8`이다. `features`는 빈 배열도 가능하다. 각 점은 `id`, 픽셀 좌표 `x`/`y`, `visible`, 0–1 `confidence`를 가진다. 가려진 점은 `visible=false`, 좌표는 null로 남긴다. 수집기가 점을 자동 검출하거나 가려진 좌표를 복원하지 않는다.

모든 시도는 세션 ID를 `leakageGroup`으로 유지하고 시작 자세·에피소드를 보존한다. 향후 학습 시 연속 영상이나 같은 세션을 무작위 프레임 분할하지 않는다. 실제 출력 trace·종료 응답·사후 영상은 예측 정답/검증 근거이며 사전 입력에 넣지 않는다.

## 실패와 중단

진행 순서는 `begin → before frame → dispatch → during frame → result → after frame → end`다. 각 구간에 여러 프레임을 받을 수 있다. ID 중복·시계 불일치·해시 오류·잘못된 입력은 원본을 남기고 추가 입력을 중단한다. 시도 분모는 영속화된 유효 `begin`의 수이며 거부된 입력은 별도 `errors`로 남는다.

정상 형식의 실패/결과 불명 응답이나 `failure` 이벤트는 **다음 명령을 막되 현재 시도의 후속 관찰·종료 기록은 받는다.** 결과 불명 뒤 영상은 `unresolved`에 들어간다. 종료 후 새 시도를 이어가지 않으며 재전송·자동 원점 복귀·실행 재개는 없다.

저장 오류가 나면 후속 입력을 거부하고 남은 파일을 검사한다. 전송 의도 후 프로세스가 죽고 종료 응답이 없으면 `outcome_unknown`으로 남는다. 시작 로그만 있고 종료 로그가 없으면 감사는 미완료로 보고한다. 감사는 원본 바이트·해시와 이벤트를 재생해 요약을 비교하며 파일을 수정하거나 동작을 재개하지 않는다. 해시는 우발적 손상을 검사하는 수단이지 모든 파일을 함께 변조한 경우의 출처 인증은 아니다.

## 다음 실물 연결

`VisualActionCollector.record(event, pixels)`가 생산자 연결 지점이다. 전송 의도를 영속화한 뒤에만 기존 실행기가 명령을 보낼 수 있도록 연결해야 한다. 호출은 한 소유자가 순서대로 수행하며, 카메라 수신 시각을 저장 완료 시각으로 다시 찍지 않는다. 실행기의 저널·확인·신선도 검사·취소 경로는 별도 책임이다. 수집기의 실패 기록이 실제 모터를 정지시키지는 않는다.

현장에서 먼저 고정 구도와 첫 관절·관측점을 정하고, 카메라 생산자와 제한 행동 실행기를 연결한다. 출력/시간·시도수·관측 지연·정착 기준을 확인한 후 실물 수집을 진행한다. 이 단계 전에는 합성 테스트 통과를 실물 실행 준비 완료나 행동 모델 성능으로 표현하지 않는다.
