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

## 고정 오프라인 검출 평가

```sh
python3 tools/evaluate_mac_camera_detection.py --capture-dir 'datasets/mac_camera/<saved-run>' --model .local/models/yolov5s.onnx --labels config/coco80.labels --output '.local/detector-evaluation/<new-run>'
```

입력·모델·라벨·코드 해시를 고정하고 CPU 작업자 하나, 60초 기한·2초 종료 유예, 준비 한 번·21평가를 실행한다. 설정은 YOLOv5s ONNX 입력640, cup, confidence0.20, NMS0.45, 가장 큰 대상, 별칭 없음이다. 새 촬영·학습·파라미터 탐색은 없다.

첫 구도 21장에서 컵 검출은 0/21이었다. 검출기 호출 중앙값 268.809ms, 입력 준비 포함 전체 중앙값 320.146ms였다. 정답 박스가 없어 정확도 평가가 아니며, 재생 시간은 실시간 촬영→검출 지연이 아니다. 새 구도의 검출 성능은 평가하지 않았다. 원본 장면과 세션 기록은 비공개로 보관한다.
