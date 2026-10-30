# 저장 체크포인트의 가변 이미지 평가

`tools/evaluate_cv.py`는 확정 라벨이 있는 매니페스트의 선택 분할 전체를 저장 모델로 한 번 추론하고, [기존 채점 규칙](offline-cv.md#계산-규칙)으로 검출·상대 위치·방향을 평가한다. 6장이나 특정 클래스 개수에 고정되지 않는다. 임의의 이미지 수·원본 크기·클래스 부재 조합과 여러 세션을 허용하며, 선택된 ID와 순서는 실행 전에 고정한다.

소프트웨어 검증은 합성 이미지·가짜 프로세스·작은 tensor 모듈로 수행한다. 별도로 실제 체크포인트 복원과 새 촬영 세션의 사진 평가를 수행했으며, 조건·실패·이후 학습 편입 여부는 [인식 개발 결과](evidence/2026-10-30-cv-development.md)에 정리했다. 테스트 통과와 실제 모델 성능은 구분한다. 설치·다운로드·학습·촬영·장비 제어 기능은 없다.

## 입력과 모델 복원

입력은 [승인 데이터 계약](offline-cv.md#승인-데이터-매니페스트)을 따른다. 두 클래스의 존재·부재와 사람 승인 상태가 모두 필요하다. 미확정 라벨, 중복 이미지, 분할을 넘는 세션/누수 그룹은 실행 전에 거부한다. 원본은 공개 fixture에 포함하지 않는다.

`motionbrain.prospective_evaluation_lock.v1` 잠금 파일에는 체크포인트 바이트 SHA-256과 전체 tensor 상태 SHA-256, 전처리·클래스·필터·운영점·실행환경·기존 소스 해시·학습 자료 목록을 기록한다. 체크포인트와 소스 경로는 저장소 루트 기준이다. `sourceFiles`에는 기존 데이터/학습/채점/표시 모듈과 실제 설치된 torchvision의 `faster_rcnn.py`, `rpn.py`, `roi_heads.py`, `transform.py`가 포함되어야 한다. 새 실행기 세 파일의 해시는 별도 실행 계획에 고정하므로 기존 잠금을 조용히 갱신하지 않는다.

현재 지원 계약은 Faster R-CNN MobileNetV3 320, 배경0·립밤1·집게2다. `weights=None`, `weights_backbone=None`, 3클래스로 구성하고 backbone의 정규화를 `FrozenBatchNorm2d`로 바꾼 뒤 `weights_only=True`, CPU 안전 로딩과 `strict=True` 복원을 적용한다. 복원 후 predictor를 교체하지 않는다. 저장 tensor와 로딩 직후 전체 tensor의 해시를 각각 검사한다.

이미지는 Pillow RGB float32 CHW/255, EXIF 회전·미러·증강 없이 처리한다. 내부 resize 320/640, ImageNet 평균·표준편차, RPN 후보 상한150·score0.05·NMS0.7, ROI score0.05·NMS0.5·출력상한100의 실효값을 확인한다. 외부 score/NMS/IoU는 각각0.5, 상대 위치 오차와 방향 중립대는 960×540 기준10px다. 현재 지원 버전은 torch2.14.1·torchvision0.29.1·numpy2.2.6·Pillow12.2.0이며 이미 준비된 환경에서 사용한다. 버전·코드·기준 변경은 새 검토된 계약이 필요하다.

## 실행 계획

다음은 형식 예시이며 해시 자리표시자는 실제 값으로 바꿔야 한다. `lock`, `manifest`, `output_dir`는 계획 파일 폴더 기준 상대경로 또는 절대경로다. 출력 폴더는 새 경로여야 한다. `frame_ids`는 선택 분할의 모든 ID를 매니페스트 순서로 나열한다. 임의의 일부 사진만 사후 선택할 수 없다.

```json
{
  "schema": "motionbrain.cv_checkpoint_plan.v1",
  "lock": "model-and-criteria-lock-v1.json",
  "lock_sha256": "잠금 파일의 SHA-256",
  "manifest": "dataset/manifest.json",
  "manifest_sha256": "매니페스트의 SHA-256",
  "split": "train_development_pilot",
  "frame_ids": ["example-01", "example-02"],
  "runner_sources": {
    "learning/cv_checkpoint.py": "파일 SHA-256",
    "learning/cv_checkpoint_display.py": "파일 SHA-256",
    "tools/evaluate_cv.py": "파일 SHA-256"
  },
  "output_dir": "evaluation-01",
  "scope": "separate_session_development",
  "seconds": 120
}
```

`scope=development`는 기존 자료의 개발 진단이다. `separate_session_development`는 학습 원본 해시·세션·누수 그룹과 중복을 거부한다. 재인코딩·근접중복·잘못 선언한 세션까지 판별하지 못하므로 `sessionIndependenceProven=false`를 보고한다. 어느 모드도 최종 봉인 test나 집계 품질 합격을 선언하지 않는다. `split=test`가 들어 있어도 평가 범위는 계획의 개발 역할로 명시된다.

```sh
# 읽기 전용: 모델 생성·역직렬화·이미지 디코딩 없이 계약과 파일 해시 검사
python tools/evaluate_cv.py check --plan .local/cv-evaluation/plan.json

# 실제 실행 범위를 정한 뒤에만 사용. 이 명령은 모델을 실행한다.
python tools/evaluate_cv.py run --plan .local/cv-evaluation/plan.json --execute
```

`check`는 실제 모델 복원이나 이미지 읽기 성공을 보장하지 않는다. 이미지 파일 누락·해시 변조·디코딩 실패도 실행에서 해당 프레임의 실패 분모로 남긴다. 선택적 의존성은 `run`의 모델 작업 프로세스에서만 불러오며 `check`는 torch 없이 동작한다.

## 실패·중단·출력

실행은 계획 옆에 `.consumed` 파일을 배타적으로 만든 뒤 새 출력 폴더를 만든다. 사용된 계획·기존 출력에는 이어하기·덮어쓰기가 없다. 모델 로딩 실패나 중단도 자동 재시도하지 않는다. 재실행하려면 원인과 범위를 검토한 새 계획이 필요하며 기존 기록은 보존한다.

모델 작업 프로세스는 CPU float32·스레드4·배치1·평가 모드·추론 모드에서 이미지만 입력한다. 정답을 forward에 넘기거나 optimizer를 만들지 않는다. 프레임마다 읽기·추론 시도 의도·완료를 디스크에 기록하며 첫 실패에서 중단한다. 후속 프레임도 `not_evaluated`로 남긴다. 정상 무검출은 성공적으로 처리한 `ok`와 빈 검출 목록이며 처리 실패와 구분한다.

`seconds`는 1–900초의 모델 작업 프로세스 상한이다. 제한 초과·사용자 중단 시 부모가 작업 프로세스를 종료하며, 종료 유예는 최대 약2초다. 모델이 종료된 뒤 부모의 JSON 채점·비교 그림 생성 시간은 이 상한 밖이다. 강제 종료로 마지막 tensor 해시 확인을 못 했거나 상태가 달라졌으면 모든 예측을 미검증으로 채점하고 원시 기록은 보존한다. 정상 종료 또는 처리 실패 후에도 모델 상태 불변이 확인된 앞부분은 부분 결과로 남길 수 있으나 전체 실행 상태와 모든 실패 분모를 함께 보고한다.

- `run.json`: 계획·잠금/모델 해시·실행 상태·상태 불변 검사·실행 시간.
- `predictions.json`: 원본 좌표의 모델 후처리 출력, 외부 운영점 적용 전 검출과 프레임 오류·시도 상태.
- `progress.json`: 작업 프로세스의 마지막 저장 상태.
- `report.json`: 전체/세션/방향 경계별 TP/FP/FN·부재 오탐·쌍·위치·방향 분자/분모. 분모0은 null.
- `comparisons/`, `comparisons.json`: 전체 적격 프레임의 GT/선택 예측 비교 그림. 원본을 못 읽으면 실패 화면을 남긴다. 그림 생성 실패는 추론 성공과 별도로 표시한다.

채점은 작업 프로세스에서 해시·크기를 확인한 저장 예측을 사용하며 모델을 재실행하지 않는다. 실패·미시도 프레임을 분모에서 빼지 않고, 부재 프레임의 처리 실패를 정상 음성으로 세지 않는다. 모델 출력과 비교 그림은 이미지 좌표 진단이며 물리적 거리·영상 시간 안정성·도달·파지 성공을 의미하지 않는다.

종료코드0은 계약 검사 또는 모든 처리/저장이 완료된 실행이다. 낮은 검출 성능 자체는 실행 오류가 아니다. 시간 초과·처리 실패·미검증 상태·그림 생성 실패는 종료코드2다. 프로세스 강제 종료나 저장장치 쓰기 실패로 최종 보고서를 못 남기면 기존 부분 파일과 `.consumed` 기록을 보존한다.

## 소프트웨어 검사

```sh
python -m unittest discover -s tests -p 'test_cv_checkpoint.py' -v
```

기본 Python에서는 선택적 torch 테스트를 건너뛴다. 이미 의존성이 준비된 환경에서도 같은 명령을 실행해 합성 파일·작은 모듈로 복원/상태 변조·가변 분모·중단 경로를 검증한다. 실제 detector 생성은 테스트에서 대체하며 실제 체크포인트·사진을 사용하지 않는다.
