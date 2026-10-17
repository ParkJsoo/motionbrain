# Mac 카메라 계측

[데이터 계약](data-pipeline.md) · [운영 안내](../OPERATIONS.md)

`tools/probe_mac_camera.py`는 명시적으로 선택한 연속성 카메라에서 유한한 프레임을 수집하고 픽셀·메타데이터·영속 저장을 검사한다. macOS 14 이상과 맞는 Xcode SDK가 필요하며 Python 감사는 플랫폼과 무관하다. 모터 명령·모델 학습은 수행하지 않는다.

## 사용

```sh
mkdir -p .local/bin
bash tools/macos/build_camera_probe.sh .local/bin/mac-camera-probe
python3 tools/probe_mac_camera.py list --helper .local/bin/mac-camera-probe
python3 tools/probe_mac_camera.py capture --helper .local/bin/mac-camera-probe --device-id '<Continuity Camera ID>' --view-id '<new-view-id>' --output 'datasets/mac_camera/<new-run>'
python3 tools/probe_mac_camera.py audit 'datasets/mac_camera/<new-run>'
```

Python 래퍼가 독립 프로세스 기한을 적용한다. 기존 디렉터리 덮어쓰기·재개·자동 재시도·다른 카메라 대체는 없다. iPhone은 고정·잠금·Mac 신뢰 설정이 필요하다. 케이블 연결만으로 실제 USB 영상 전송을 증명하지 못하므로 `transportVerifiedUSB=false`다. [Apple 요구사항](https://support.apple.com/en-us/102546)

## 고정 계약

- 시작 최대 30초, 준비 1초 또는 콜백 30회 중 먼저 도달하는 값.
- 기준 한 장 + 평가 20장, 측정 최대 10초, 프로세스 45초 + 종료 유예 2초.
- 요청 형식 640×480 BGRA. 실제 크기·행 배치·회전·반전·프레임 형식을 기록하며 변경 시 중단한다. 늦은 콜백은 버리고 개수를 기록한다.
- 행 패딩 없는 BGRA를 보존하고 밝기 `(77*R + 150*G + 29*B)/256`를 재계산한다. 센서 RAW나 의미 기반 물체 검출이 아니다.
- 이미지·메타데이터 생성, `fsync`, 재읽기, 디렉터리 `fsync`의 전체 트랜잭션을 250ms로 제한한다. 이후 최종 기록·오프라인 감사 시간과 구분한다.
- 저장 실패, 잘못되거나 중복·역행하는 PTS, 장치 단절, 구성 변경, 기한 초과는 실패로 남기며 부분 파일도 보존한다.

CoreMedia 시계에서 PTS를 변환하고 callback·처리·저장 시각을 기록한다. `captureProcessingPassed`는 21장 무결성·저장 통과와 평가 20장 완료, 그중 최소 16장의 PTS→처리 100ms 및 기준 PTS 나이 15초 조건을 요구한다. 기준 프레임은 의미 기반 물체 앵커가 아니다. 표시 시각과 callback으로 광학 노출 시각을 입증하지 못하므로 `overallPassed`, `captureHostBoundsCalibrated`, `controlEligible`, `trainingEligible`은 false다.

## 2026-10-16 실측 결과

최초 권한 획득 실패는 촬영 0장이며 지연 측정 표본에 포함하지 않는다. 이후 서로 다른 구도의 두 실행 결과는 다음과 같다.

| 진단 | 첫 완료 실행 | 계측을 추가한 새 구도 |
| --- | ---: | ---: |
| 실제 전달 형식 | 1920×1080 BGRA | 1920×1080 BGRA |
| PTS→처리 ≤100ms | 12/20, 기준 미달 | 12/20, 기준 미달 |
| PTS→처리 최소 / 중앙값 / 최대 ms | 75.453 / 81.840 / 115.418 | 66.481 / 79.440 / 115.662 |
| 영속 저장 트랜잭션 최대 ms | 12.162 | 18.910 |
| 보고된 누락 프레임 | 0 | 0 |

두 실행 모두 프레임 21장의 무결성은 통과했지만 처리 진단은 실패했다. 새 구도에서 실패한 여덟 장은 callback 도착 때 이미 PTS로부터 100ms를 넘었다. 상위 경로 지연을 시사하지만 ISP·전송·OS 큐 중 원인을 특정하지 못한다. 광학 시각은 미검증이며 실물 학습 적격 데이터는 0건이다.

## 제어기 상태와 함께 보관

```sh
python3 tools/probe_mac_controller.py --controller-url 'http://<controller-host>' --helper .local/bin/mac-camera-probe --device-id '<Continuity Camera ID>' --view-id '<new-view-id>' --output '.local/observation/<new-run>'
python3 tools/probe_mac_controller.py --audit '.local/observation/<saved-run>'
```

이 도구는 새 촬영 한 번과 읽기 전용 제어기 응답을 묶는다. 촬영 전 네 GET(`/status`, `/sequence`, `/actions`, `/action_outputs`), 첫 프레임 파일 확인 후 카메라 프로세스가 살아 있는 동안 `/status` 여섯 번, 완료 후 같은 네 GET으로 최대 14회다. 요청마다 800ms, 관측 단계 50초, 카메라 래퍼 47초를 적용한다. 프로세스 정리는 TERM·KILL 각각 최대 2초이며 최종 오프라인 감사·봉인과 함께 관측 시간에서 구분한다. 요청을 반복하거나 기존 실행을 재개하지 않는다. 권한은 실행한 터미널 앱에 따라 달라질 수 있으며, 권한 실패는 프레임 0장의 별도 실패로 보존한다.

IDLE·모터 다섯 개 OFF·빈 시퀀스·센서 준비 상태를 검사하고, 전후 부팅 ID와 출력 이력이 같아야 `status=recorded`다. 저장 실패나 250ms 초과, 통신 실패, 구동 상태, 조기 촬영 종료는 후속 요청을 중단한다. 카메라 래퍼와 자식 프로세스도 종료하며 부분 파일은 남긴다. 이 관측 도구에는 ARM·모터 명령·OFF 명령 경로가 없다. 다른 실행기가 구동한 상태를 감지했을 때도 자체적으로 로봇을 정지시키는 도구는 아니다.

원래 응답 본문·요청/수신 시각·해시와 영상 원본을 보관한다. M4의 보고 나이에 요청 왕복 시간을 더한 진단값과 20ms 이내 개수를 기록하지만, CoreMedia·Python 단조 시계·MCU 시각을 합치거나 각 영상에 센서 값을 대응시키지 않는다. 카메라 **프로세스**와 조회가 겹친 사실은 광학 노출과 센서 샘플의 동시성을 증명하지 않는다. 과거 누적 출력 누락 수가 변하지 않았더라도 학습용 출력 이력 적격 판정은 아니다. 감사는 원본 해시·응답·순서·부팅·출력·M4 요약과 카메라 진단을 재검사하며 학습·구동 적격은 항상 false다.

## 2026-10-17 새 구도 실측

권한이 없는 실행 환경에서는 30초 후 촬영 0장으로 종료했다. 카메라 권한이 있는 터미널 앱에서 별도 새 실행을 시작해 1920×1080 BGRA 21장을 보관했다. 평가 프레임의 PTS→처리 진단은 20/20, 최소/중앙값/최대 57.273/65.302/75.374ms, 전체 21장 영속 저장 최대 14.278ms, 보고된 누락은 0장이었다. 처리 진단은 통과했지만 전날과 구도·실행 환경이 달라 개선율로 비교하지 않는다. 광학 노출 시각·USB 전송 경로·의미 기반 립밤 앵커는 여전히 미검증이며 실물 학습 적격 전이는 0건이다.

이어 영상·제어기 응답 보관 도구로 한 번 더 유한 관측했다. 4.089초에 GET 14회와 영상 21장을 기록했고 원본 감사가 통과했다. PTS→처리 진단 20/20, 최소/중앙값/최대 58.914/62.728/66.409ms, 영속 저장 최대 16.730ms, 보고된 누락 0장이었다. 여섯 M4 값은 모두 219.81°였으나 보고 나이 9–13ms에 요청 왕복 시간을 더한 값은 55.051–175.123ms로 **20ms 기준 0/6**이었다. 전후 IDLE·모터 다섯 개 OFF, 같은 부팅 ID·출력 이력을 확인했다. ARM·모터 명령은 보내지 않았다. 관측 보관 성공이며, 전체 팔 정착·광학 동기화·행동 전이 적격을 입증한 결과는 아니다.

## 고정 오프라인 검출 평가

```sh
python3 tools/evaluate_mac_camera_detection.py --capture-dir 'datasets/mac_camera/<saved-run>' --model .local/models/yolov5s.onnx --labels config/coco80.labels --output '.local/detector-evaluation/<new-run>'
```

입력·모델·라벨·코드 해시를 고정하고 CPU 작업자 하나, 60초 기한·2초 종료 유예, 준비 한 번·21평가를 실행한다. 설정은 YOLOv5s ONNX 입력640, cup, confidence0.20, NMS0.45, 가장 큰 대상, 별칭 없음이다. 새 촬영·학습·파라미터 탐색은 없다.

첫 구도 21장에서 컵 검출은 0/21이었다. 검출기 호출 중앙값 268.809ms, 입력 준비 포함 전체 중앙값 320.146ms였다. 정답 박스가 없어 정확도 평가가 아니며, 재생 시간은 실시간 촬영→검출 지연이 아니다. 새 구도의 검출 성능은 평가하지 않았다. 원본 장면과 세션 기록은 비공개로 보관한다.
