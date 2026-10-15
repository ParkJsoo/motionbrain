# 빠른 관측 경로 착수: 기존 모델 설정 비교 — 2026-10-15

**기존 모델의 스레드·입력 크기 설정만으로 수집 신선도 100ms를 만족하지 못했다. 운영 설정은 변경하지 않았다.** 다음 구현은 객체 선택과 최신 영상의 빠른 위치 측정을 분리하는 쪽으로 좁힌다. 이 문서는 성능 측정 결과이며 새 추적기 구현이나 학습 개선 완료 기록이 아니다.

## 방법

- 앞선 비구동 수집에서 보존한 서로 다른 JPEG 2개를 사용했다. 모든 설정에 동일한 bytes를 주고 hash를 기록했다. 새 영상 획득이나 모터 명령은 없다.
- Pi의 기존 `vision_detection.py`와 `yolov5s.onnx`를 그대로 사용했다. 실제 서비스와 같은 OpenCV 가상환경(4.12.0), confidence 0.20, NMS 0.45, target cup, 기존 aliases chair/toilet, largest 정책이다. 입력 크기 640/320과 OpenCV 스레드 1/2만 비교했다.
- 각 설정에서 첫 이미지 1회 warm-up을 제외하고, 이미지 2개 × 2회 = 4회 측정했다. 순서는 640/1 → 320/1 → 640/2 → 320/2다. 모델 load 시간은 측정에서 제외되며 JPEG decode·전처리·추론·후처리는 포함한다.
- 운영 perception은 그대로 실행했다. 같은 Pi CPU를 공유하므로 통제된 하드웨어 벤치마크나 안정적인 지연 상한으로 해석하지 않는다. 저장 이미지 둘의 검출 여부는 별도 정답 데이터셋 정확도를 대신하지 않는다.

## 결과

| 설정 | 측정 중앙값 | 최대 | 해당 저장 이미지의 결과 |
|---|---:|---:|---|
| 640 / 1스레드 | 2,867.32ms | 3,047.85ms | 4/4 검출, 모두 실제 classId 41(cup) |
| 640 / 2스레드 | 1,986.36ms | 2,086.63ms | 4/4 검출, 모두 실제 classId 41(cup) |
| 320 / 1스레드 | 측정 불가 | — | 현재 ONNX의 Reshape 단계 shape 불일치 |
| 320 / 2스레드 | 측정 불가 | — | 같은 오류 |

640의 2스레드가 이 표본에서는 더 빨랐지만, 추론 경로만 약 2초이므로 100ms 수집 정책을 충족하지 않는다. 320은 CLI 숫자를 바꾸는 것만으로 적용할 수 없다. 이를 지원하려면 별도 모델 export/호환성·검출 품질 검증이 필요하다.

별도 JPEG decode 10회는 0.81~2.60ms, 중앙값 1.02ms였다. 이는 decode만 측정한 값이다. 검출/추적·네트워크 수신·원본 source age·시간 교정 오차를 모두 포함한 관측 지연은 아니다.

최초 시스템 Python/OpenCV 4.6.0 실행은 모델의 FLOAT16을 지원하지 않아 모든 설정에서 load 실패했다. 이를 모델 성능 결과에 섞지 않았다. 정상 운영 서비스의 별도 가상환경을 확인한 뒤 위 시험을 다시 수행했다.

## 다음 구현의 계약

1. 느린 객체 인식은 대상 선택의 근거로 유지한다. 선택에 쓴 원본 frame ID·JPEG/hash·ROI를 보존한다.
2. 최신 프레임에서 실제로 계산한 ROI/영상 특징의 위치 변화만 빠른 피드백 후보로 사용한다. 과거 좌표의 hold/republication을 새로운 측정으로 표시하지 않는다. 초기 대상 근거와 현재 측정 frame ID를 따로 기록한다.
3. 카메라 재부팅, 대상 소실, 모호한 후보, 큰 외관/크기 변화에서는 측정을 거부하고 재확인한다. 추적 결과가 새로운 객체 분류나 파지 성공을 입증한다고 주장하지 않는다.
4. 우선 비구동·저장 영상/가상 이동으로 위치 정확도·실패 거부를 검증하고, 실제 최신 프레임에서 전체 지연을 측정한다. 100ms/20ms 기준을 몰래 완화하지 않는다. 현재 집게 근처 전방 카메라 장착을 유지하며 집게가 화면에 보일 것을 요구하지 않는다.

운영 서비스·모델·펌웨어는 변경하지 않았다. 종료 확인에서 메인 IDLE·모터 출력 0, CAM captureFailures=0, Pi perception fresh/ok·연속 오류 0, 서비스 3개 active·NRestarts=0을 확인했다.

## 원본과 재현

- [운영 환경 측정 JSON](fast-observation-2026-10-15/runtime-results.json): 모델·검출 소스·벤치마크·입력 이미지 hash, 설정, 모든 개별 결과.
- [실행 로그와 320 오류](fast-observation-2026-10-15/runtime.log), [최초 시스템 Python 실패](fast-observation-2026-10-15/system-python-failure.json), [종료 상태](fast-observation-2026-10-15/final-status.json).
- [측정 스크립트](fast-observation-2026-10-15/benchmark.py). 저장 이미지 경로는 앞선 수집 원본이며 원본 사진·모델 바이너리는 저장소에 추가하지 않는다. 재실행 시 새로운 output 파일을 지정한다.

```sh
/home/motionbrain/.cache/motionbrain/opencv-venv/bin/python \
  docs/evidence/fast-observation-2026-10-15/benchmark.py \
  --repo /home/motionbrain/develop/arduino/motionbrain \
  --model /home/motionbrain/.cache/motionbrain/models/yolov5s.onnx \
  --labels /home/motionbrain/develop/arduino/motionbrain/config/coco80.labels \
  --frames /path/to/first-original.jpg /path/to/second-original.jpg \
  --output /tmp/new-detector-benchmark.json
```
