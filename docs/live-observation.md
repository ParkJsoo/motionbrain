# 두 카메라와 M4를 Rerun에서 실시간으로 보기

[운영 안내](../OPERATIONS.md) · [저장 평가와 시뮬레이터](robot-visualization.md)

`tools/visualization/live.py`는 외부 카메라·집게 카메라의 최신 영상과 등록 관측점, 실측 M4 센서값을 Rerun에 보낸다. 모터 API·학습·W&B 게시 기능은 없다. 이 경로는 화면 확인용이며 수집기나 제어 피드백으로 사용하지 않는다.

## 준비와 실행

macOS의 외부 카메라는 정확한 Continuity Camera 식별자로 선택한다. 해당 장치가 없으면 다른 카메라로 전환하지 않는다. 기존 영상에서 확인한 두 카메라의 `RegisteredSceneLandmarks` 등록과 참조 원본, 외부 촬영의 설정 메타데이터가 필요하다. 새 구도에 옛 등록을 자동 적용하지 않는다.

시각화 환경의 [의존성](../tools/visualization/requirements.txt)과 Xcode 명령줄 도구를 준비한 뒤 저장소 루트에서 실행한다.

```bash
mkdir -p .local/bin
bash tools/macos/build_camera_probe.sh .local/bin/mac-camera-preview --preview
# 별도 터미널에서 기존 로컬 포털/Rerun 서버를 실행한 상태로 사용한다.
python tools/visualization/live.py --config .local/live-preview-config.json
```

설정의 장치별 값·참조 경로는 현재 설치 환경에서 확인해 채운다. 비공개 설정은 `.local/`에 둔다. `output`은 매 실행마다 새 비공개 폴더를 지정한다.

```json
{
  "helper": ".local/bin/mac-camera-preview",
  "rerunUrl": "rerun+http://127.0.0.1:9876/proxy",
  "output": ".local/live-preview-01",
  "external": {
    "deviceId": "<현재 외부 카메라 식별자>",
    "registration": ".local/<외부 등록.json>",
    "reference": ".local/<외부 참조.png>",
    "metadata": ".local/<확인한 외부 프레임 메타데이터.json>"
  },
  "gripper": {
    "url": "http://<카메라 주소 또는 로컬 경유 포트>",
    "bootId": "<현재 카메라 boot ID>",
    "registration": ".local/<집게 등록.json>",
    "reference": ".local/<집게 참조.jpg>"
  },
  "controller": {
    "url": "http://<현재 제어기 주소>",
    "bootId": "<현재 제어기 boot ID>"
  }
}
```

외부 메타데이터에서 `deviceUniqueId`, `configurationSha256`, `width`, `height`, `videoMirrored`, `videoRotationAngle`을 확인한다. 현재 설정이 이 값과 다르면 표시를 중단하며 카메라 설정을 자동 변경하지 않는다. [로컬 Rerun](http://127.0.0.1:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy)에서 새 실시간 기록을 선택한다. 종료는 실행 터미널의 `Ctrl+C`다. 유한 점검에는 `--seconds 10`처럼 실행 시간을 지정할 수 있다.

외부 영상은 최대 2Hz, 집게 영상은 요청 처리 후 0.5초 간격, M4는 요청 처리 후 1초 간격으로 갱신한다. 화면 확인용 주기이며 모터 제어 주기나 두 카메라의 동기 촬영을 의미하지 않는다. `host_elapsed`는 같은 Mac의 단조 시계에 따른 수신·프레임 시각이며 광학적 동기화나 지연 없는 영상을 보장하지 않는다.

## 상태와 기록의 해석

- `live`: 최신 입력을 받았으며 해당 관측 검사를 통과했다.
- `rejected`: 최신 원본 영상은 표시하지만 특징점 관측은 거절됐다. 이전 관측점을 남기지 않는다.
- `stale`: 영상 1.5초, M4 2.5초의 표시 유효 시간을 벗어났다. 과거 영상은 현재 화면에서 지운다.
- `failed`: 연결·원본·설정·부팅 검사에서 실패했다. 해당 입력을 자동 재시도하거나 재등록하지 않는다.

미리보기는 원본을 계속 쌓지 않는다. 외부 영상은 메모리에서 전달하며 집게 영상은 기존 fresh/boot/픽셀 검증에 필요한 한 장만 임시 저장하고 지운다. 실행 폴더에는 최신 `status.json`과 오류·종료 진단만 남는다. 화면 영상은 표시용으로 압축되며 학습 자료나 실험 원본을 대신하지 않는다. Rerun 서버의 버퍼·저장 정책은 별도이며, 실험 재현용 원본은 기존 유한 수집기로 수집한다.

M4는 펌웨어의 `angleDeg` 센서 좌표다. 미확인 관절을 측정값으로 만들거나 시뮬레이터의 전체 팔 자세를 실시간으로 갱신하지 않는다. 특징점 일치가 깊이·접촉·파지·측정 정확도나 기존 모델의 적용 가능성을 입증하지도 않는다.

현재 미리보기와 수집기는 별도 카메라 세션을 연다. 같은 외부 카메라로 동작 자료를 수집하기 전에 미리보기를 종료한다. 수집 중의 실시간 표시는 수집기가 사용하는 프레임 스트림에 연결해야 하며, 이 미리보기를 병렬 실행해 대체하지 않는다.

## 수동 조작·동작 수집 중 같은 영상 보기

수집기의 새 실행 폴더를 지정한다. `--source`는 `external-stream`과 `gripper`가 생길 상위 폴더이며, 생성 전부터 기다릴 수 있다. 양안 실험에서는 외장 볼륨을 가리키는 해당 실행의 `source` 링크도 사용할 수 있다. 수동 수집에서는 수집기 자체의 출력 폴더를 외장 볼륨에 지정한다.

```bash
python tools/visualization/collector_live.py --source "$CAPTURE_ROOT" --rerun-url "$RERUN_URL"
```

이 프로세스는 카메라 연결 없이 수집 도중 확정된 최신 원장과 그 원본 바이트를 읽는다. 화면용 외부 JPEG와 집게 원본 JPEG를 각각 갱신한다. 집게 영상은 수집기가 실제로 요청한 횟수만큼만 생긴다. 수집이 멈춰 원본 시각 하한이 1.5초 이상 지난 영상은 다음 점검에서 지운다. 표시가 건너뛴 프레임도 외장 저장소의 원장에 남는다. 표준 출력의 프레임 ID·원장 경로·제출 시각과 수집 원장을 비교하면 표시 간격과 건너뛴 수를 계산할 수 있다. 제출 시각은 Mac 단조 시계로 기록하며, 화면에 픽셀이 나타난 시각과 광학적 촬영 시각은 별도 기준 시계로 측정해야 한다.

단발 명령 수집기의 `native-camera`·`gripper-camera` 구조에는 같은 명령에 `--single-command`를 추가한다.

## 구동 중 양안 판단

집게 열기·닫기, 손목·팔꿈치·어깨 상하, 베이스 좌우에 같은 실행기를 사용한다. 시연에서 사람이 명령을 누르기 전에 볼 수 있었던 두 영상을 각각 현재 영상과 비교한다. 두 카메라가 같은 동작을 지지하고 각 영상의 오차가 계획의 기준 이내일 때 그 동작을 선택한다. 시연에 없는 방향의 정확도나 새로운 배치에서의 파지 성공률은 별도 평가 대상이다.

```bash
python tools/run_dual_view_policy.py --plan "$PLAN"
python tools/run_dual_view_policy.py --plan "$PLAN" --observe --rerun-url "$RERUN_URL"
python tools/run_dual_view_policy.py --plan "$PLAN" --execute --confirm-plan-sha256 "$PLAN_SHA256" --rerun-url "$RERUN_URL"
```

첫 명령으로 실제 시연 원본, 시연에 있는 행동, 등록된 구동 프로필과 계획 해시를 확인한다. 두 번째 명령은 카메라와 외장 기록만 사용한다. 세 번째 명령에는 현재 부팅 ID, 이미 ARM 상태인 전용 제어기와 기존 비공개 환경변수 `MOTIONBRAIN_HTTP_TOKEN`이 필요하다. 실행기가 ARM을 전송하지 않는다. 기본 펌웨어와 구분되는 PlatformIO 빌드 환경은 `esp32dev_dual_view_policy`다.

계획 JSON에는 다음 값이 필요하다.

| 필드 | 내용 |
| --- | --- |
| `examples` | 각 사람 시연의 `episode`, `episodeSha256`, 1부터 시작하는 `actionIndex` |
| `external`, `helper`, `helperSha256` | 촬영 당시의 외부 카메라 설정, 수집 실행 파일 경로와 SHA-256 |
| `sideSampleFps` (선택) | 측면 원본 수집 목표 속도. 정수 4(기본) 또는 8. 실제 양안 판단 주기는 두 카메라의 새 원본 ID와 판단 시각으로 측정한다. |
| `gripperUrl`, `gripperBootId`, `scene` | 집게 카메라 주소와 현재 부팅 ID, 촬영 구도·물체·자세 범위·특징점 설명 |
| `mount`, `volumeUuid`, `captureRoot` | 원본을 저장할 외장 볼륨과 기존 수집 상위 디렉터리 |
| `maxAge`, `maxSkew`, `maxViewRmse` | 영상 나이와 카메라 간 시차의 최대 허용 초, 외부·집게 영상별 정규화 RGB 오차 한도 |
| `controllerUrl`, `controllerBootId`, `seconds`, `maxActions` | 제어기, 현재 부팅 ID, 수집 시간과 최대 명령 횟수 |
| `profiles` | 행동별 검증된 `shoulderRangeDeg`와 장치 `command`. 관측 전용은 빈 객체도 허용 |

프로필 이름은 예를 들어 `elbow/down`이다. 각 `command`에는 `kind`, `motor`, 부호 있는 원시 출력 `pwm`, 장치 기한 `durationMs`, `targetDegrees`, `percent`를 명시한다. 어깨는 절대 각도 목표, 나머지 축은 유한 펄스를 사용한다. 기존 수동 UI의 퍼센트나 가감속 파형을 원시 PWM으로 대체해 검증된 명령으로 취급할 수 없다.

다섯 출력의 OFF와 출력 원장의 일련번호를 확인한 뒤 촬영된 연속 두 영상 쌍으로 시작한다. 구동 중에는 별도 수집 프로세스가 계속 추론한다. 실행기는 반대 판단, 영상 불일치 또는 관측 만료 때 진행 중인 명령 ID로 취소한다. 다음 방향과 관절을 선택하려면 종료 기록·드라이버 OFF와 새 영상 쌍이 필요하다. 동시에 출력할 수 있는 관절은 하나다.

외장 실행 폴더의 `decisions.jsonl`에는 원본 프레임 ID와 양쪽 판단, `actions.jsonl`에는 실행기가 소비한 프레임과 판단, 취소·종료 근거가 남는다. 짧은 동작은 새 양안 쌍을 받기 전에 끝날 수 있다. 이 경우 `observedRunningPairs`는 0이다. 이 수치는 호스트의 구동 응답 이후에 얻은 영상 쌍 수이며 모터의 물리적 이동은 영상과 실제 출력 시각을 대조해 판정한다.

### 학습된 관절 반응으로 행동 선택하기

상대 위치 정책을 사용하려면 계획의 `responsePolicy`에 로컬 학습 산출물 경로 `artifact`와 원본 바이트의 `sha256`을 지정한다. 각 구동 프로필의 명령은 학습에 사용한 명령과 일치해야 한다. 기존 RGB 시연 비교 모드는 이 필드가 없는 계획에 적용된다.

이 정책은 두 영상에서 턱과 물체의 상대 위치를 측정하고, 관절별로 관측한 이동량을 적용해 파지 직전 시연에 가까워지는 후보를 고른다. 움직임을 가르치는 왕복 자료와 닫기·들기의 목표 상태를 가르치는 시연은 별도로 사용한다. 진행 중인 명령을 장치가 수락하면 양쪽 모두 수락 이후에 촬영된 영상 쌍으로 실제 변화량을 다시 계산한다. 해당 명령의 개발자료에서 식별된 이동량에 도달하면 정책이 보류를 선택해 새 관측을 요청할 수 있다.

닫기 이후의 들기에는 같은 실행에서 확인한 닫기 명령 ID, 집게 드라이버의 음수 출력과 마지막 0 쓰기, OFF 이후에 촬영된 두 영상이 필요하다. 힘이나 접촉 센서가 없으므로 출력 기록과 영상 형상을 각각 점검한다. 실제 들기 성공은 모터 OFF 이후 연속된 독립 영상 쌍에서 물체 위·아래 경계의 상승량으로 평가한다. 로그에서 정책이 낸 `modelAction`, 신선도 검사를 거친 `action`, 동작 ID `activeCommandId`를 함께 보면 모델 전환과 시간 만료를 구분할 수 있다.
