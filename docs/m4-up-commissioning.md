# M4 위 방향의 전용 단발 시운전

`learning/m4_up_commissioning.py`는 탁자에 지지된 물체에 양턱이 접촉한 자세에서 **수동 식별용 명령 하나**를 시험하기 위한 별도 코어다. 일반 수집기의 M4 위 방향 거절과 `PROFILES`는 유지한다. 생성·범위 검증은 장치에 접속하지 않으며, 실제 전송은 관측 어댑터와 비공개 토큰을 명시적으로 주입한 `run()`에서만 가능하다. 자동 장치 검색은 제공하지 않는다. 두 카메라 연결과 명시적 실행 진입점은 아래에 설명한다.

소프트웨어 검증은 모의 제어기, 합성 영상과 원본 메타데이터로 수행한다. 실제 두 카메라의 무구동 전후 저장·종료 점검도 완료했다. 이 점검에서는 제어기 GET만 사용했으며 전축 OFF와 출력 이력 무변화를 확인했다. 실제 M4 위 구동·이동 거리·물체 보유·들기 성능은 검증하지 않았다. 기존 `/shoulder` 절대각 제어와 자동 보정, `/actions` 유한 출력 프로파일로 우회하지 않는다.

## 고정한 범위

| 항목 | 전용 경로의 값 |
| --- | --- |
| 명령 | M4(`shoulder`, 모터4) 위 방향 하나 |
| 출력 | `defaultSpeed=255`, `percent=16`, 요청 드라이버 값 `+40` |
| 요청 기간·횟수 | `200ms`, 최대1명령 |
| 시작각 | 현재 조회·검토해 범위에 고정한 각도, 허용 차이 `±0.5°` |
| 관측 중 중단 기준 | 시작각 대비 절댓값 `1°` 초과 또는 반대 방향 `−0.25°` 미만 |
| 시간 | 전체 본문60초, 사전 영상 최대10초 경과, OFF 확인 후 최소3초에 촬영한 사후 영상 |
| RUN 직전 제어기 근거 | 마지막 상태·부팅 ID가 결합된 출력 이력의 요청 시작부터 각각250ms 이내 |

16%는 위의 기본 출력255에 대한 백분율이다. 기본 출력100에서의 40%와 혼동하지 않는다. 명목200ms는 실제 출력 기간이나 작은 이동을 보장하지 않는다. 각도 기준은 호스트가 관측했을 때 실패·정지하는 조건이며 기계적 이동 한계나 독립적인 실시간 정지 장치가 아니다. 통신·호스트 정지 지연과 구동 후 이동이 있을 수 있다. 원래 센서 신선도·수동 제어 보호·넓은 각도 제한도 그대로 검사한다.

출력이 작아 반응이 관측되지 않아도 자동 증량·재전송·원위치 복귀하지 않는다. 실패·불명확한 결과는 별도 실패로 남긴다. 같은 시험 ID를 새 객체로 다시 만들더라도 재실행할 수 없다. 새 ID의 다음 시험은 새로운 현장 판단과 실행 범위 결정이 필요하며 자동 대체가 아니다.

## 범위 파일과 오프라인 검사

범위 파일은 `.local/` 아래에 둔다. `schemaVersion=motionbrain.m4_up_commissioning.v1`이며 아래 항목 외의 필드는 거절한다.

- `trialId`, `controllerUrl`, `controllerBootId`, `hostClockId`: 새 시험·제어기·현재 부팅·호스트 시계 식별자. 주소에 인증정보를 넣지 않는다.
- `implementationSha256`: 이 코어·공통 전송 코드·두 카메라 어댑터의 현재 해시. 변경 후에는 다시 검토한다.
- `context`: `physicalView`, `poseRegime`, `object`, `supportMode=table_supported`, `externalDeviceId`, `externalViewId`, `gripperCameraBootId`.
- `pulse`: `{ "joint":"shoulder", "motor":4, "direction":"up", "percent":16, "durationMs":200, "expectedPwm":40 }`.
- `defaultSpeed=255`, `maxCommands=1`, 현장에서 확인한 `expectedStartAngleDeg`, `startAngleToleranceDeg=0.5`, `maxObservedDeltaDeg=1`, `maxSeconds=60`, `maxObservationAgeSeconds=10`, `postOffSeconds=3`.

```sh
python3 tools/validate_m4_up_commissioning.py --implementation-sha256
python3 tools/validate_m4_up_commissioning.py --scope .local/m4-up/scope.json
```

이 CLI는 파일·구현 해시와 범위만 검사한다. 토큰을 읽거나 저널을 점유하거나 제어기·카메라에 접속하지 않는다. `hardwareCalls=0`, `generalM4UpEnabled=false`, `liveReadinessVerified=false`를 출력한다. 범위 해시 일치나 오프라인 통과 자체가 사용자 실행 승인·현재 장치 확인을 대신하지 않는다.

## 전송과 두 카메라 근거

`M4UpCommissioning(scope, observer, token, store, evidence_root=..., clock=...)`에 `.local/`의 새 증거 저장소, 정상적인 비공개 입력으로 받은 런타임 토큰, 현장 검토와 촬영을 수행하는 관측 어댑터를 주입한다. `run(confirmed_scope_sha256=...)`은 정확한 범위 해시를 요구한다. 실행 전에 기존 공용 명령 저널을 독점하고 시험 ID를 영구 소비한다. 다른 수집기의 점유나 미해결 `pending`·`off_unverified` 기록이 있으면 장치 호출 전 차단한다. 별도 저널을 만들어 우회하지 않는다. 기존 수집 서비스에서 정상적으로 제어권을 넘겨받아야 한다.

관측 어댑터 계약은 `start(deadline=...)`, `observe(after_off=..., deadline=..., tick=...)`, `latest()`, `check()`, `close()`다. 관측 대기 중에는 `tick`으로 다른 출력·상태 변화를 검사한다. `latest`·`check`는 비차단 검사만 하며 촬영·HTTP를 수행하지 않는다. `close`는 `{status:"completed"}`를 반환해야 한다. `learning/m4_up_observer.py`의 `M4DualCameraObserver`는 기존 `ActionRecorder`와 `GripperObservationSource`를 연결한다. 네이티브 스트림·신선 요청2회·원본·전후 미리보기를 보존한다. 촬영 실패나 검토 거절 후 재촬영하지 않는다. 정상적인 조기 종료의 네이티브 원본 `partial/operator_stop`을 그대로 보존하고 시계 마감 근거와 두 소스의 정상 종료를 확인한 경우에만 어댑터 마감을 완료로 처리한다.

### 촬영 점검과 단발 실행

`tools/m4_up_trial.py` 설정은 `.local/`의 `{scope, camera}` JSON이다. `scope`는 위 범위 전체이며, `camera`는 `{helperPath,helperSha256,cameraUrl}`다. 실행 파일 해시를 생성·시작 시 확인한다. 주소·부팅 ID·구도는 현장에서 확인하고, 인증정보는 설정에 넣지 않는다.

```sh
python3 tools/m4_up_trial.py capture-check \
  --config .local/m4-up/config.json --output-dir .local/m4-up/camera-check-01
python3 tools/m4_up_trial.py run \
  --config .local/m4-up/config.json --output-dir .local/m4-up/trial-01 \
  --confirm-scope-sha256 '<현재 범위 해시>' --wait-for-start
```

`capture-check`는 제어기 쓰기를 STOP까지 모두 차단한다. 두 카메라의 전후 저장과 마감, OFF 및 출력 이력 무변화를 검사한다. 접촉·현장 참석 등 긍정 판단을 자동 생성하지 않으며 실물 실행 승인을 부여하지 않는다. 촬영 중인 다른 작업을 덮어쓰지 않는다.

`run`은 터미널에서 토큰을 비공개 입력받는다. 에코 억제를 보장할 수 없는 터미널에서는 입력 전에 실패하며 환경·다른 프로세스·브라우저에서 토큰을 가져오지 않는다. `--wait-for-start`는 토큰 입력 뒤 별도의 Enter를 기다린다. 이 대기 중에는 저널·시험 디렉터리를 생성하거나 촬영·장치 조회를 시작하지 않는다. 분석가는 토큰 입력 완료 채팅을 받은 뒤 영상 검토를 수행할 준비를 마치고 시작한다. 채팅 지연을 영상 신선도 완화나 재촬영으로 보상하지 않는다. 실제 시험 전에 기존 수집기의 제어권을 정상 이관하고, 현재 조건과 사용자 실행 범위를 확인해야 한다.

전후 영상이 저장되면 실행기는 `<label>-review-request.json`과 미리보기 경로를 출력한다. 분석가는 원본 영상과 현장 확인에 근거해 `{requestSha256,decision:{context,stable,onsite}}` 형식의 `<label>-review-decision.json`을 새로 저장한다. 요청 파일 해시가 같아야 하며 `context`·`onsite` 계약은 아래와 같다. 실행 전 검토는 원래 영상의9초 이내에 응답해야 한다. 늦거나 불명확한 판단은 RUN 없이 종료하고 자동으로 대체 자료를 찍지 않는다. 사후 검토는 원래 전체 기한 내에 수행한다. 이 검토는 방향·출력을 고르는 절차가 아니다.

전후 관측에는 다음을 묶은 변경 불가 검토 JSON과 해시가 필요하다.

- 고정 외부 카메라의 네이티브 무손실 프레임·메타데이터·호스트 수신 기록·시계 변환 원본. 장치·구도·실제 설정 해시·픽셀 해시와 원래 촬영 시각을 검사한다.
- ESP32-CAM의 새 요청 이후 프레임과 JPEG·응답 원본. 카메라 부팅 ID·프레임 식별자·실제 촬영 시각·수신·저장 시각을 검사한다.
- 두 영상이 같은 현재 구도·물체·자세 배치라는 검토와 현장 참석·이동 여유 확인. 실행 전에는 양턱 접촉·탁자 지지도 확인한다. 사후 접촉·지지 여부는 실제 관측대로 기록한다. 범위의 `table_supported`는 시작 조건이며 사후 상태를 강제하지 않는다.

`observe` 결과의 필드는 `hostClockId`, `context`, `external`, `gripper`, `stable`, `onsite`, `evidenceFile`, `evidenceSha256`다. `external`은 `{root,ingressFile,ingressSha256,clockSha256}`, `gripper`는 `{root,label,metadataSha256,imageSha256}`다. 경로는 증거 루트 아래의 상대 경로이며 원본 형식은 기존 네이티브 수집기와 집게 카메라 수집기의 형식을 재사용한다. `onsite`는 `operatorPresent`, `clearanceVerified`, `jawContactVerified`, `tableSupportVerified`의 불리언이다. 검토 JSON은 마지막 두 증거 필드를 제외한 제출 내용과 같아야 한다. 해시는 변경 검사용이며 물리 상태나 사람 판단의 정확도를 인증하지 않는다.

RUN 전에는 소유권·구현·범위·출력·센서·두 카메라 원본·입력 변경을 다시 검사하고 판단을 저장한다. 마지막 원본 재검사와 저장을 포함한 검사 시간이100ms를 넘으면 RUN하지 않는다. 판단 저장 뒤에도 두 영상의 원래 촬영 시각 기준10초 한도와 제어기 근거의250ms 한도를 다시 계산한다. 느린 RUN 의도 저장으로 제어기 근거가 오래되면 재조회·자동 재시도 없이 거절한다. 이 시간 기준은 실물 정지·이동량의 보장이 아니다. RUN 응답이 유실돼도 다시 보내지 않는다. 실행 결과 불명은 OFF를 증명해도 공용 저널의 `pending`으로 남겨 새 ID의 실행까지 차단한다. 정지나 OFF 증명 실패도 자동 재전송하지 않고 `off_unverified`로 후속 실행을 막는다. 실패와 마지막 OFF 증명은 별개로 보존한다. 촬영 종료 뒤의 예상치 못한 출력도 완료를 막는다.

## 전체 팔의 집기·들기로 연결

이 시험은 새 위 방향의 반응을 알아보는 수동 식별이며 모델 선택이나 들기 성공으로 세지 않는다. 결과는 기본적으로 `trainingEligible=false`, `modelSelected=false`, `graspVerified=false`, `liftVerified=false`다. 실측 후 출력 이력과 관측 품질을 검토해 학습 적격 여부를 별도로 결정한다.

다음 관측 계약은 고정 외부 카메라에서 **탁자 기준 도구·물체의 이동과 분리**, 집게 카메라에서 **물체와 턱의 상대 미끄럼**, M4 센서에서 **각도 변화**를 함께 다룬다. 기존 M1 상대 세 좌표는 도구·물체가 함께 상승하는지 판단할 수 없으므로 자동 혼합하지 않는다. 현재 자세·하중의 필요한 관절 자료를 최소 수집하고 모델 고정→별도 평가→후보 결과 비교로 작은 들기 명령을 선택하는 순서로 이어간다. 무반응·관측 실패에는 다음 범위를 재검토하며 M3 전환이나 추가 M1 수집을 자동으로 시작하지 않는다.

[현재 모델 선택 집기 코어](model-grasp-trial.md)는 접근·닫기 뒤 보유 확인까지이며 들기 단계는 아직 포함하지 않는다. 이 단발 경로의 완료만으로 모델 선택 들기가 구현됐다고 표현하지 않는다. 전체 팔의 기구학은 실측 링크·축·영점과 카메라 프레임을 대조해 연결한다. 미측정 관절을0으로 채워 실제 끝단 높이를 주장하지 않는다. RViz·Rerun의 상태·근거 표시와 W&B의 고정 모델·평가 비교는 병행할 수 있으며, Gazebo·MuJoCo의 접촉·동역학 실험은 전체 팔 모델의 보정 뒤 별도 비교한다. 시뮬레이터 완성을 첫 국소 실측의 선행 조건으로 두지 않는다.

```sh
python3 -m unittest tests.test_m4_up_commissioning tests.test_m4_up_observer tests.test_supervised_transport \
  tests.test_supervised_collection tests.test_manual_collection tests.test_model_grasp_trial
```
