# 원본·receipt 저장 지연 분해와 병렬화 — 2026-10-15

**파일별 fsync·hash 재검증을 유지하며, 같은 실제 원본 12개의 저장 구간 중앙값을 42.72→34.93ms(18.2%) 줄였다.** 디렉터리는 GET 전에 준비하고, 독립적인 receipt/JPEG 쓰기를 겹친 뒤 두 쓰기가 끝나야 측정 결과를 저장한다. 다음 프레임을 백그라운드에 쌓지 않는다.

최종 전체 소프트웨어 **481개 테스트 통과(18.249초)**. 실제 카메라 GET 12회에서는 저장 구간 중앙값 35.20ms, 원본·측정 저장 후 나이 **134~180ms로 모두 100ms 초과**였다. 계산 완료 당시에는 2/12이 수락됐지만 저장 후 사용 가능하다는 의미는 아니다. 모터 명령 0회, 출력 로그 불변, 제어/학습 적격성 false다.

## 비용 분해와 선택

Pi `/tmp`는 RAM tmpfs가 아닌 `/dev/mmcblk0p2`의 ext4다. 운영 서비스/설정을 유지한 채 별도 프로세스에서 이전 실행 원본을 새 임시 디렉터리에 저장했다. 기존 방식의 16회 표본에서 receipt·JPEG·measurement 저장 중앙값은 41.02ms, 그중 순차 fsync 합계 중앙값은 36.15ms였다. 이미지별 디렉터리만 미리 준비한 비교는 37.53ms, 준비 비용은 5.36ms였다. 초기 [계측 코드](capture-persistence-2026-10-15/profile.py)와 [표본](capture-persistence-2026-10-15/profile.json)은 이전 `4e3313e` 임시 소스를 사용한 진단이다. 이때의 fsync wrapper는 별도 벤치마크 프로세스에만 존재하며 제품 코드에 추가하지 않았다.

주된 비용이 디스크 확정 쓰기여서, fsync를 제거하거나 성공으로 일찍 보고하지 않고 독립 파일의 저장을 병렬화했다. JPEG 저장 worker는 현재 프레임 한 개만 받는다. receipt는 호출 스레드에서 쓰며 기존 recorder callback의 스레드도 유지한다. 공유 run/event 기록이나 다음 capture를 동시에 실행하지 않는다.

## 변경 파일과 내구성 경계

- `learning/artifacts.py`: `prepare_parent()`는 안전한 상대 경로의 부모 디렉터리만 미리 생성·fsync한다. 경로 탈출·symlink는 기존 no-follow 규칙으로 거부한다. 빈 준비 디렉터리는 촬영/저장 성공 증거가 아니다. 기존 `_write()`의 임시 파일, file fsync, 원자적 create-if-absent hardlink, parent fsync, readback hash 검증은 그대로다.
- `learning/fast_probe.py`: `_persist_capture()`는 receipt/JPEG 저장을 겹치고 종료를 기다린다. 두 저장이 성공해야 image reference가 포함된 measurement를 기록한다. artifact 목록 갱신과 callback은 호출 스레드에서 수행한다. 오류·중단 시에도 진행 중인 JPEG 저장이 끝날 때까지 기다리고, 저장된 원본을 검증 목록에 수거한 뒤 기존 오류/중단을 전달한다. 저장 성공을 얻지 못하면 새 촬영이나 성공 measurement를 만들지 않는다.
- `tools/probe_fast_observation.py`: 기본은 병렬 저장, `--serial-persistence`는 이전 순차 방식 비교용이다. session에 `parallelPersistence`를 기록한다. source hash·원본/JPEG/receipt/measurement 경로와 내용 계약은 유지한다.
- persistence sidecar의 `storageTiming`은 GET 전 준비, receipt/JPEG 각각의 경과시간, 둘을 합친 wall time, measurement 쓰기, 계산 완료→저장 완료 시간을 구분한다. 겹치는 두 쓰기 시간의 합을 전체 wall time으로 부르지 않는다.

저장 완료 시각은 **원본·receipt·measurement의 내구성 확인 직후**다. 이 시각을 기록하는 persistence sidecar 자체, measured event, 추가 진단 GET/파일 쓰기는 그 뒤에 수행한다. 따라서 이 숫자는 최종 report를 나중에 읽는 순간의 나이가 아니다. 실제 소비자는 자신의 현재 시각으로 다시 검사해야 한다. 이 작업에는 물리 제어 consumer가 없고 모든 결과는 제어 부적격이다.

실제 SIGINT가 `Future.result()` 대기 중 들어오면 파일은 저장돼도 descriptor 수거가 건너뛰어지는 P2를 독립 리뷰에서 발견했다. receipt 실패 후 executor 종료 대기에 SIGINT가 오는 복합 경로도 같은 문제가 있었다. 현재 쓰기 완료까지 중단을 보존·지연하고, worker 성공 결과 슬롯을 정리 경로에서 수거하도록 고쳤다. 두 경로 모두 별도 자식 프로세스에 실제 SIGINT를 보내는 회귀로 검증했다. 강제 kill/전원 차단 시 원본이 반드시 남는다는 보장은 하지 않는다. durable capture_requested 이후 terminal/measurement가 없으면 미완료로 취급한다. 실제 전원 차단 시험은 수행하지 않았다.

## 최종 소스의 동일 원본 비교

[벤치마크](capture-persistence-2026-10-15/benchmark.py)는 이전 실물 실행 `95c43314228b470c9770fc1106ac348b`의 12개 원본·receipt hash를 먼저 확인하고, 최종 `_persist_capture()`를 직접 호출한다. 각 프레임의 직렬/병렬 순서를 교차했다. 전처리·ROI 계산을 포함하지 않는 저장 비교이며, 실제 probe처럼 기존 raw/images/measurement 디렉터리가 있는 정상 반복 구간을 재현했다. 첫 생성 비용은 별도 준비 단계에 포함한다.

| 항목 | 중앙값 | 최소~최대 |
|---|---:|---:|
| 직렬 receipt+JPEG+measurement | 42.72ms | 37.51~48.96ms |
| 병렬 receipt+JPEG 이후 measurement | 34.93ms | 31.52~37.85ms |
| 병렬 방식의 GET 전 디렉터리 준비 | 5.12ms | 4.45~6.58ms |

**18.2%는 촬영 이후 저장 구간의 개선**이며 전체 작업량이나 실제 영상 나이가 18.2% 줄었다는 주장이 아니다. 디렉터리 생성 비용 일부를 촬영 전으로 옮겼고, 준비 시간은 숨기지 않고 별도 기록한다. 원본/JPEG 내용과 최종 artifact hash를 양쪽에서 검사했다. 입력 디스크 읽기·run 생성·공통 preflight 준비는 비교 구간에서 제외했다. fsync는 운영체제/저장장치의 동기화 계약이며 이번 시험이 갑작스러운 물리 전원 차단 내구성을 입증하지는 않는다.

[전체 표본·실행 소스 hash](capture-persistence-2026-10-15/benchmark.json). 원본이 있는 환경에서 아래 명령으로 재현할 수 있다.

```sh
python3 docs/evidence/capture-persistence-2026-10-15/benchmark.py \
  --source-dir datasets/fast_probes/95c43314228b470c9770fc1106ac348b \
  --output-dir /tmp/motionbrain-storage-benchmark
```

## 실제 비구동 GET 결과

Pi 별도 `/tmp/motionbrain-storage-20261015`, 배터리함 분리·USB/Pi 연결·정지 컵 상태에서 `--samples 12 --capture-timing --connection-timing`으로 실행했다. run `173d984ee2ac424f8605d14aba868ae4`다. 운영 repo `559db54`·3개 서비스·카메라 설정/펌웨어는 유지했다.

| 구간 | 중앙값 | 최소~최대 |
|---|---:|---:|
| GET 전 디렉터리 준비 | 5.61ms | 4.86~10.68ms |
| receipt/JPEG 병렬 저장 wall time | 21.56ms | 17.68~25.99ms |
| measurement 저장 | 11.84ms | 10.16~14.26ms |
| 계산 완료→원본·측정 저장 완료 | 35.20ms | 28.73~37.67ms |
| 계산 완료 나이 상한 | 107.78ms | 97.73~142.94ms |
| 원본·측정 저장 후 나이 상한 | 142.44ms | 133.69~179.82ms |

계산 시점 2/12 수락, 나머지 10개 나이 초과; 저장 후 0/12 통과다. 이전 실험의 6/12와 이번 2/12는 서로 다른 GET 원본·네트워크/스케줄링 표본이므로 저장 최적화가 계산 정확도를 악화시켰다는 비교로 사용하지 않는다. 최적화 자체의 비교는 앞의 같은 원본 시험이다. 100ms·ROI score/margin/translation/anchor 정책과 source-age 계산식은 변경하지 않았다.

원본 JPEG 13개, artifact 108개, event 26개, 종료 report 및 현재 실행 소스 hash를 Mac에서 검증했다. 원본은 Git 제외 `datasets/fast_probes/173d984ee2ac424f8605d14aba868ae4`에 보존했다. IDLE·motorEnabled=false·5축 속도 0, 시작/종료 출력 로그 동일, CAM boot 유지·captureFailures=0, Pi 3개 서비스 active·NRestarts=0을 확인했다.

[실제 결과](capture-persistence-2026-10-15/live-results.json), [저장 구간 전체 표본](capture-persistence-2026-10-15/persistence.json), [환경·소스 hash](capture-persistence-2026-10-15/live-session.json), [무결성·통계](capture-persistence-2026-10-15/validation.json), [시작 상태](capture-persistence-2026-10-15/before-status.json), [종료 상태](capture-persistence-2026-10-15/final-status.json).

## 테스트·리뷰와 다음 단계

신규 10개 테스트는 사전 생성/경로/fsync 실패, 병렬 겹침·호출 스레드 유지, receipt 실패·중단 시 원본 수거, image 중단 시 receipt 보존, 다음 촬영 차단, 실제 SIGINT 및 receipt 실패 복합 경로, 직렬/병렬 측정·이미지 동일성을 검사한다. 최종 [전체 481개 테스트](capture-persistence-2026-10-15/tests.log)가 통과했다. 독립 리뷰에서 발견한 P2는 수정·재검토했다. 리뷰 에이전트가 최종 helper 벤치마크를 원본 12개로 독립 재실행하고, 실측 통계·원본/종료/실행 소스 hash·문서를 검증했다. 최종 미해결 P0/P1/P2 없음.

저장 최적화만으로 100ms 문제는 해결되지 않았다. **다음은 카메라에서 더 최근에 생성된 프레임을 받는 선택형 캡처 경로의 가능성을 검토·가상 테스트하는 단계**다. 기존 `/capture` 기본 동작과 원본 ID/시각 의미를 보존하고, 새 프레임 대기·timeout·중복 timestamp·boot 변경을 검증해야 한다. 현재 자료만으로 노출/readout와 버퍼 체류 시간을 분리하거나 버퍼가 원인이라고 확정하지 않는다. 새 카메라 드라이버나 추가 센서를 선행조건으로 삼지 않는다.

이후 펌웨어 변경·실물 업로드가 필요하면 최소 변경과 비구동 비교 절차를 먼저 준비해 별도 승인을 받는다. 컵/카메라 위치·배터리 분리·USB/Pi 연결은 유지하면 된다. 실제 소비 시각 재검사, 광학/clock/M4 정합성, 제한 동작 실물 검증과 학습용 행동 수집은 여전히 남아 있다. main 병합·운영 배포·모터 구동은 이번 범위 밖이다.
