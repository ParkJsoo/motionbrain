# 현장 감독하의 단일 관절 수집

이 수집기는 한 관절·한 방향의 짧은 동작 묶음을 실행하고, 영상 기록과 사후 측정을 분리한다. 사용자는 현장을 계속 관찰하며 묶음의 시작·중단·다음 방향을 결정한다. 무인 수집이나 위치 제어기가 아니다.

## 동작과 현재 한계

- 묶음당 1~3개 유한 명령을 한 번에 하나씩 실행한다. 실제 출력 OFF와 3초 관측 대기 뒤 다음 명령을 보낸다. 정확한 자세 맞춤·자동 왕복·명령 재전송은 없다.
- M1·M2·M3·M5 양방향과 M4 아래 방향의 과거 출력 상한을 구분한다. M4는 묶음당1회이며 위 방향은 과거 큰 이동 사례 때문에 비활성이다. 이 상한은 현재 자세에서의 안전한 이동 범위를 보장하지 않는다.
- 아이폰 무손실 압축 프레임과 PTS를 저장한다. 실시간 특징점 추적·정착 판정은 하지 않는다. 한 프레임 누락·느린 저장은 기록하고, 지속 소스 중단·저장/시각 오류에는 새 명령을 막는다.
- 네트워크 결과 불명은 부분 실행일 수 있다. STOP·출력 확인 후 종료하고, 같은 sessionId는 다시 실행하지 않는다. 중간 종료 후 자동 재개하지 않는다.
- 예외적으로 ARM 전 출력 OFF가 확인된 큐 조회(`GET /sequence`)의 `RemoteDisconnected`만 기존 요청 제한 시간 안에서 한 번 재조회한다. 두 시도는 모두 기록하며, 명령 전송·출력 중 조회·정리 과정에는 이 복구를 적용하지 않는다.
- 명목 명령 기간과 실제 출력 기간은 다르다. 드라이버 PWM 이력을 보존하지만 전류·토크 실측이나 실제 관절 회전이라고 해석하지 않는다.
- 신규 경로는 소프트웨어·합성 자료 검증 단계다. 실제 구도·표식·통신·구동은 별도 현장 검증이 필요하다. 기존 모델의 실물 성공을 이 경로의 성공으로 간주하지 않는다.

## 준비·오프라인 검증

빌드에는 카메라 접근이 없다. 원본·계획·로그는 Git 제외 경로에 둔다.

```sh
mkdir -p .local/supervised/bin
bash tools/macos/build_action_recorder.sh .local/supervised/bin/mac-action-recorder
python3 tools/collect_supervised_actions.py profiles
```

계획에는 현재 장치 식별자·부팅 ID·구도·자세 배치·촬영 그룹·데이터 분할을 명시한다. 다음은 자리표시자를 바꿔 쓰는 **계획 생성 예시**다. 이 명령은 촬영·구동하지 않는다. 출력과 기간은 현재 현장에 맞게 검토해야 한다.

```sh
python3 tools/collect_supervised_actions.py prepare \
  --session-id session-001 --capture-group group-001 --split development \
  --device-id '<camera-device-id>' --view-id wrist-side --pose-context middle \
  --boot-id '<32-hex-current-boot-id>' --controller-url 'http://<controller-host>' \
  --joint wrist --direction up --percent 63 --durations-ms 550 \
  --helper .local/supervised/bin/mac-action-recorder \
  --output-plan .local/supervised/session-001-plan.json
python3 tools/collect_supervised_actions.py validate \
  --plan .local/supervised/session-001-plan.json \
  --helper .local/supervised/bin/mac-action-recorder
```

계획과 helper 해시를 고정한다. 60초·8fps·최대500프레임의 압축 최악 예산과 추가2GiB 여유 공간을 실행 전 확인한다. helper가 실제1920×1080을 제공하지 않으면 초기 관측에서 중단한다. 이전 날짜의 추적점을 재등록하지 않는다.

## 명시적인 실제 실행

현장 확인과 해당 묶음의 실행 범위를 정한 뒤에만 실행한다. 제어 토큰은 기존 안전한 메모리 전달 방식으로 `MOTIONBRAIN_HTTP_TOKEN`에 제공하며, 파일·쉘 이력·로그에 값을 남기지 않는다.

```sh
python3 tools/collect_supervised_actions.py run \
  --plan .local/supervised/session-001-plan.json \
  --helper .local/supervised/bin/mac-action-recorder \
  --confirm-plan-sha256 '<validated-plan-sha256>' --onsite \
  --output-dir .local/supervised/session-001
```

터미널 Ctrl-C/SIGTERM은 정지·마감 경로로 연결된다. 현장 물리적 정지 수단도 유지한다. STM32 데드맨은 HTTP sequence의 보호 스위치가 아니며 원격조작과 동시에 사용하지 않는다. 동일 제어기를 다른 호스트에서 동시에 조작하지 않는다. 로컬 실행기는 기존 공용 저널 잠금을 공유하지만 다른 장치의 조작까지 배제하지 못한다.

명령 실행 중 호스트가 갑자기 죽으면 pending 저널을 남기고 다음 실행을 차단한다. 미확인 출력을 수동으로 확인·복구하기 전 저널을 삭제하거나 새 경로로 우회하지 않는다. 정상 마감·OFF 확인 뒤에는 새 sessionId와 새 계획으로 다음 묶음을 시작한다. 실패한 명령 ID나 기존 출력 디렉터리를 재사용하지 않는다.

## 사후 측정

`tools/measure_supervised_collection.py`는 장치를 호출하지 않는다. 전후 구간에서 각 프레임의 표식을 독립 검출한다. 색상 범위·관측 영역은 해당 구도에서 정하며, 없는 표식·여러 후보·영역 경계 접촉은 해당 사례의 제외 사유다. 현재 자동 측정은 표식의 상대 방향·거리·한 축 상대 위치를 지원한다. M4 센서 원본은 HTTP 기록에 보존되지만 이 도구의 자동 라벨은 영상 좌표다.

측정 설정의 형태:

```json
{
  "schemaVersion": "motionbrain.endpoint_measurement.v1",
  "width": 1920, "height": 1080,
  "stabilityThreshold": 0.5,
  "points": [
    {"id":"reference", "roi":[100,100,200,200], "hsvLower":[40,100,80], "hsvUpper":[80,255,255], "minAreaPx":20, "maxAreaPx":500},
    {"id":"moving", "roi":[400,100,200,200], "hsvLower":[100,100,80], "hsvUpper":[130,255,255], "minAreaPx":20, "maxAreaPx":500}
  ],
  "representation":{"kind":"projected_position","moving":"moving","reference":"reference","axis":"x"}
}
```

좌표·색상은 형식 예시이며 실물용 확정 설정이 아니다.

```sh
python3 tools/measure_supervised_collection.py \
  --report .local/supervised/session-001/report.json \
  --config .local/supervised/measurement.json \
  --output .local/supervised/session-001-measurement
```

출력은 `measurements.json`과 `measurements.csv`다.

동작 전은 전송 시작 전0.75~0.10초, 결과는 OFF확인 후2.25~3초 구간을 사용한다. 최소3프레임·최대0.3초 간격과 설정한 안정성 기준을 검사한다. 고정3초 대기는 기계적 정착을 보장하지 않는다. 사례별 상태·변화량·원본 프레임·출력 이력과 제외 사유를 남긴다.

단위는 영상상 상대 방향(도) 또는 픽셀이다. 실제3D 관절각으로 표현하지 않는다. 기존 명령 시작+4초 계약과 다른 v3 데이터이며 자동으로 섞거나 학습하지 않는다. 한 촬영 그룹을 학습/검증/시험으로 분할하지 않는다. 다른 관절 배치·카메라 구도는 별도 맥락으로 기록한다.

블록 후반이 실패해도 앞선 사례에 완전한 전후 관측과 출력 근거가 있으면 심사한다. 이전/다음 명령이 측정창을 침범하거나 출력이 불명확하면 해당 사례를 제외한다. 정상 종료 플래그나 고정120개 사례 수를 데이터 생성의 필수 조건으로 삼지 않는다.
