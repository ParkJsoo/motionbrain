# 대상 물체·집게 오프라인 표시와 평가

저장된 이미지와 라벨로 방향 표시를 검토하고, 저장된 예측값의 검출·상대 위치 결과를 계산한다. 모델·카메라·서버를 실행하지 않으며 가중치 다운로드와 학습도 하지 않는다. 기존 단일 대상 `evaluate_object_detector.py`와 별도 계약이다.

[현재 동작 학습 계획](visual-action-learning.md)에서 이 도구는 물체·집게 관측과 결과 분석을 지원한다. [실제 인식 개발 학습 결과](evidence/2026-10-30-cv-development.md)는 별도로 정리하며, 정지사진 검출 완성을 행동 파일럿의 선행 조건으로 삼지 않는다.

이 문서의 대상 물체는 기존 실험에서 사용한 소형 원통형 물체 한 종류다. 데이터 계약의 클래스 식별자 `lip_balm`은 그대로 사용하며, 임의의 물체 종류를 지원하는 범용 검출 계약은 아니다.

## 기능과 한계

- `learning/cv_dataset.py`: 승인 라벨·이미지 해시·좌표·세션 분할 검사와 RGB 이미지/검출 target 로딩.
- `learning/cv_tensors.py`: 선택적 PyTorch CPU tensor 어댑터와 가변 크기 배치 구성.
- `learning/cv_evaluation.py`: 점수 문턱·클래스별 NMS 이후 검출의 TP/FP/FN, 두 대상 검출률, 상대벡터 오차·방향 일치와 경계별·세션별 결과.
- `learning/cv_display.py`: 승인 라벨 또는 미확정 후보의 정적 박스·중심·연결선·방향 예시. 모델 출력으로 표시하지 않는다.
- `tools/offline_cv.py`: 위 기능의 파일 입출력 명령.

실제 모델 성능·동영상 처리·시간축 안정성·COCO AP는 이 도구의 구현 범위가 아니다. tensor 변환은 선택적 어댑터로 제공하며 모델 실행과 학습은 포함하지 않는다. 이미지 상의 가시 bbox 중심은 집게 입구·실제 거리·깊이·도달 가능성·접촉·파지 성공을 뜻하지 않는다. 합성 예측으로 얻은 수치는 계산 검증이며 학습 성과가 아니다.

## 승인 데이터 매니페스트

`schema="motionbrain.cv_dataset.v1"` JSON을 사용한다. 기존 개발용 `motionbrain-p1-six-frame-development-export-v1`도 같은 필드 계약으로 읽는다. 복제된 `targets.json`은 신뢰 입력으로 읽지 않고 아래 라벨에서 target을 재계산한다.

최상위 필드:

| 필드 | 계약 |
|---|---|
| `classIds` | 정확히 `{"background_reserved_not_annotation":0,"lip_balm":1,"gripper":2}` |
| `records` | 비어 있지 않은 프레임 목록. `id`, `imageId`, 원본 해시는 각각 고유 |
| `splits` | 역할별 프레임 ID 목록. 모든 프레임을 정확히 한 번 포함 |

프레임 필드:

| 필드 | 계약 |
|---|---|
| `id` / `imageId` | 문자열 ID / 양의 정수 ID |
| `image` | 매니페스트 폴더 기준 상대경로. 상위 탈출·외부 심볼릭 링크 금지 |
| `sourceSha256` | 실제 이미지 파일의 소문자 SHA-256 |
| `width`, `height` | 원본의 양의 정수 크기. 읽을 때 실제 디코딩 크기와 비교 |
| `sessionId`, `leakageGroup` | 비어 있지 않은 문자열. 같은 세션 또는 누수 그룹은 분할을 넘을 수 없음 |
| `split` | `train`, `train_development_pilot`, `validation`, `test` 중 하나 |
| `coordinates` | `original_xyxy_no_rotation_no_mirror` |
| `labels` | 두 클래스 각각 하나의 라벨. 한쪽 없음도 명시해야 함 |

라벨은 `class`, `presence`, `bbox_xyxy`, `bbox_valid`, `occlusion`, `truncated`, `humanApproved`, `reviewStatus`를 가진다. `humanApproved=true`, `reviewStatus="human_approved"`가 필수다. 이 선언은 출처를 보관하는 계약이며 사람이 승인했다는 사실을 도구가 독립 인증하는 것은 아니다.

- `present`: 유한 원본 픽셀 `0≤x0<x1≤width`, `0≤y0<y1≤height`, `bbox_valid=true`, `occlusion=none|partial`, `truncated`는 bool.
- `absent`: `bbox_xyxy=null`, `bbox_valid=false`. 가림/잘림은 비적용 값으로 보존 가능하다.
- unknown·미승인·상자 없는 present·중복/누락 클래스는 매니페스트 로딩을 중단한다. 배경이나 음성으로 바꾸지 않는다.

`CVDataset(path)[index]`는 `(RGB uint8 H×W×3 배열, target 딕셔너리)`다. target은 `image_id`, `boxes`, `labels`, `area`, `iscrowd`를 포함한다. bbox 면적은 `(x1-x0)*(y1-y0)`, `iscrowd=0`이며 현재 클래스별 단일 인스턴스 계약이다. 두 클래스 모두 없는 사진도 허용하고 빈 목록을 반환한다. 선택적 `TorchCVDataset`은 빈 상자를 `(0,4)`로 변환하고 float32/int64 자료형을 명시한다. 데이터 변환 검증과 실제 모델 학습·성능 검증은 별개다.

이미지는 매 접근마다 해시와 디코딩을 검사하며 RGB 변환 외에 회전·미러·크롭·리사이즈를 하지 않는다. EXIF 자동회전을 끈 원본 좌표를 사용하므로 입력 방향은 라벨 계약과 일치해야 한다. 분할 검사는 `--split`으로 필터하기 전 전체 매니페스트에서 수행한다. 해시/세션 식별자가 같지 않은 근접중복이나 잘못된 세션 선언까지 자동 판별하지는 못한다.

## 저장 예측값 계약

평가 입력은 다음 구조다. 아래 좌표와 ID는 형식 설명용이며 실제 평가에는 해당 이미지 해시·원본 좌표를 넣는다.

```json
{
  "schema": "motionbrain.cv_predictions.v1",
  "coordinates": "original_xyxy_no_rotation_no_mirror",
  "evidenceKind": "synthetic",
  "frames": [
    {
      "id": "example-01",
      "sourceSha256": "실제 이미지의 64자리 소문자 SHA-256",
      "status": "ok",
      "detections": [
        {"class": "lip_balm", "score": 0.9, "bbox_xyxy": [30, 110, 60, 150]},
        {"class": "gripper", "score": 0.8, "bbox_xyxy": [100, 60, 130, 100]}
      ]
    }
  ]
}
```

`evidenceKind=synthetic|model`을 명시한다. 이는 입력 작성자의 선언이며 모델 실행 증명이 아니다. `synthetic` 결과의 `modelAccuracyMeasured`는 false다. 정상 미검출은 `status=ok, detections=[]`, 처리 실패는 `status=error`다. 누락된 프레임도 처리 실패로 남긴다. 예측의 원본 해시 불일치·무효 좌표·무효 score는 해당 프레임 실패다. 중복 ID·선택 데이터 밖의 ID·지원하지 않는 전체 스키마는 평가 자체를 거부한다.

## 계산 규칙

1. 유효 score의 범위는 0–1이며 문턱 이상을 유지한다. 클래스별로 score 내림차순, 동점이면 bbox 좌표 오름차순으로 정렬한다. NMS는 기존 유지 상자와 IoU가 문턱을 **초과**하면 제거한다. GT로 선택 순서를 바꾸지 않는다.
2. 운영점 TP/FP/FN은 클래스별 GT 최대 1개와 일대일 매칭한다. IoU≥0.5인 예측은 최대 1개만 TP, 나머지 예측은 FP, 미검출 GT는 FN이다. NMS에서 제거된 상자는 운영점의 출력에 포함하지 않는다.
3. 상대 위치는 각 클래스의 **최고 score 예측**으로만 계산한다. 낮은 score의 정답에 가까운 상자가 있어도 최고 score의 잘못된 상자를 대체하지 않는다. 따라서 검출 TP가 있어도 두 대상 상대 위치 평가에서는 실패할 수 있다.
4. `대상 물체 중심−집게 중심` 벡터를 기준 영상 960×540에 균일 축척 `min(960/W,540/H)`로 환산한다. 두 클래스 모두 올바르게 매칭된 뒤 상대벡터 L2≤10px면 위치 조건 충족이다. 원본 픽셀의 축별/거리 오차와 x/W,y/H 정규화 거리도 별도 기록한다.
5. 방향은 축별 −10 미만: 왼쪽/위, −10부터 +10: 중앙 부근, +10 초과: 오른쪽/아래다. 방향 일치는 위치 조건과 별도 계산한다. 현재 CLI의 10px와 IoU 0.5는 버전이 있는 작업값이며 실제 용도에 대한 성능 보장이 아니다.
6. GT의 경계 거리 `min(||dx|-10|,||dy|-10|)≤10`이면 인접층이다. 인접층을 전체 분모에 포함하면서 층별 결과도 보고한다. 설정된 오차 허용량보다 경계 거리가 큰 경우의 방향 보존은 수학적 충분조건이며 모델이 그 오차를 달성한다는 뜻은 아니다.
7. 두 GT 상자가 모두 유효한 완전 검토 프레임을 분모에 유지한다. 읽기/추론 실패·누락은 성공 0으로 남는다. 조건부 오차는 올바르게 매칭된 두 대상만 계산하며 그 프레임 수와 전체 예상 프레임 수를 함께 표시한다. absent 오탐수/전체 absent 수와 처리 실패 수를 함께 보며, 실패를 `absentClearFrames`에 넣지 않는다. 분모 0인 비율은 null이다.

전체 합격률·validation용 confidence/NMS 선택·독립 세션 성능은 이 도구가 결정하지 않는다. 개발 자료의 평가를 최종 test 결과로 바꾸지 않는다. 보고서에 프레임별 split·세션·누수 그룹·입력 매니페스트 해시와 운영점을 남긴다.

## 명령 예시

기존 호스트 의존성 `numpy`, `opencv-python-headless`를 사용한다. 아래 명령은 설치나 모델 실행을 하지 않는다. 출력 폴더는 새 경로여야 하며 기존 결과를 덮어쓰지 않는다. 중간 실패의 부분 출력도 자동 삭제하지 않는다.

```sh
python3 tools/offline_cv.py inspect \
  --manifest .local/cv-data/manifest.json --output-dir .local/cv-check

python3 tools/offline_cv.py render \
  --manifest .local/cv-data/manifest.json --output-dir .local/cv-preview

# 0.5는 명령 형식 예시다. 실제 모델 운영점은 별도 validation에서 정한다.
python3 tools/offline_cv.py evaluate \
  --manifest .local/cv-data/manifest.json --predictions .local/predictions.json \
  --score-threshold 0.5 --nms-threshold 0.5 --output-dir .local/cv-evaluation
```

`--split`을 생략하면 매니페스트의 모든 프레임을 처리한다. 여러 역할이 있으면 목적에 맞는 분할을 명시하고 결과의 split을 확인한다. 처리 실패가 있는 평가·표시는 결과를 보존하고 종료코드 2를 반환한다. 데이터 검사에서 이미지 검증이 실패하면 전체 검사를 중단한다. 낮은 검출 성능 자체는 실행 오류와 구분하며 종료코드 0일 수 있다. 표시 축척 후 한 축이 1픽셀보다 작아지는 극단적인 종횡비는 실패 화면과 보고서를 남기고 다음 프레임을 처리한다.

기존 측면 라벨 작업본의 후보 표시는 별도 경로다. `motionbrain-p1-reviewed-working-v1` 라벨과 동일 ID·세션·해시·크기를 가진 촬영 목록을 입력한다. 원본 유효 bbox를 우선하고 없는 경우 보존된 `candidate_bbox_xyxy`를 표시한다. 후보를 승인 라벨로 바꾸거나 집게의 가려진 외형을 복원하지 않는다.

```sh
python3 tools/offline_cv.py render-reference \
  --labels .local/archived-labels.json --captures .local/archived-captures.json \
  --source-root . --output-dir .local/candidate-preview
```

후보 이미지에는 `CANDIDATE REFERENCE - GEOMETRY UNRESOLVED`, 모든 이미지에는 `NOT MODEL OUTPUT`을 명시한다. 후보 프레임의 target 생성과 평가 입력은 거부한다. absent면 연결선·방향을 숨기며 이미지 읽기 실패도 새 빈 화면으로 표시한다. 정적 승인 라벨의 `ANNOTATED ABSENT`를 미래 모델의 미검출 문구로 사용하면 안 된다. 이 도구는 모델의 실시간 상태 표시를 구현하지 않는다.

## 소프트웨어 검증

```sh
python3 -m unittest discover -s tests -p test_offline_cv.py -v
```

생성한 합성 이미지와 가짜 예측으로 라벨 미완료·좌표 오류·세션 누수·파일 변조·디코딩 실패·예측 누락·음성 실패·최고 score 선택·중복/NMS·방향 경계·후보 격리·기존 출력 보존을 검증한다. 실제 사진·사용자 라벨·개인 원본을 테스트 fixture로 공개하지 않는다.

## 선택적 PyTorch 입력 어댑터

`learning.cv_tensors.TorchCVDataset`은 기존 매니페스트의 승인·해시·좌표·세션/누수 그룹·분할 검사를 상속한다. 이미지마다 해시를 다시 검사하고 Pillow로 디코딩한다. OpenCV와 독립적으로 `torch`, `numpy`, `Pillow`가 필요하며 패키지 설치나 가중치 다운로드를 수행하지 않는다. 모듈 import와 매니페스트 검사는 torch 없이도 가능하고, 실제 항목 접근 때만 선택적 의존성을 불러온다.

- 이미지: 원본 크기의 RGB `float32[3,H,W]`, 범위0–1, 연속 메모리, CPU. EXIF 회전·미러·리사이즈·증강·장치 이동을 하지 않는다.
- 대상: `boxes=float32[N,4]`, `labels=int64[N]`, `area=float32[N]`, `iscrowd=int64[N]`, `image_id=int64[1]`. 클래스는 배경0 예약·대상 물체1·집게2를 유지하며 실제 target에는 배경상자를 넣지 않는다.
- 두 대상이 모두 없으면 boxes는 `(0,4)`, labels/area/iscrowd는 `(0,)`다. 읽기 실패·미승인·unknown을 빈 target으로 바꾸지 않고 오류를 전달한다.
- 8비트 `RGB`, `RGBA`, `L`만 허용한다. RGBA의 alpha는 합성하지 않고 버리며 L은 세 채널로 복제한다. 팔레트·16비트·CMYK·애니메이션은 거부한다. 해시와 크기가 맞아도 디코더 오류는 그대로 실패다.
- float32 변환으로 폭/높이/면적이0 또는 비유한 값이 되는 상자, int64를 넘는 imageId를 거부한다. 면적은 변환된 상자에서 계산한다. 반환 tensor를 수정해도 다음 읽기나 원본 라벨에 영향을 주지 않는다.
- Pillow와 OpenCV가 모든 압축 이미지에서 동일하게 디코딩된다고 가정하지 않는다. 사용하는 원본에서 색상·방향·좌표를 확인해야 한다.

```python
from torch.utils.data import DataLoader
from learning.cv_tensors import TorchCVDataset, detection_collate

# 실제 존재하는 승인 매니페스트와 분할을 지정한다.
dataset = TorchCVDataset(".local/cv-data/manifest.json", split="train_development_pilot")
loader = DataLoader(dataset, batch_size=2, shuffle=False, num_workers=0,
                    collate_fn=detection_collate)
images, targets = next(iter(loader))
# 서로 다른 영상 크기/상자 개수를 보존하는 list 두 개다. 모델은 실행하지 않는다.
```

`detection_collate`는 이미지를 같은 크기로 강제 stack하지 않는다. 빈 배치는 오류다. 파일 읽기는 지연되므로 반복 중 실패가 나면 호출자가 실패를 기록해야 하며 이 어댑터가 프레임을 자동 생략하지 않는다. 세션·원본 해시·split은 `dataset.frames`, 매니페스트 해시는 `dataset.manifest_sha256`에 유지된다.

검증 명령(이미 의존성이 있는 Python 사용):

```sh
python -m unittest discover -s tests -p 'test_cv_tensors.py' -v
```

torch가 없는 환경에서는 tensor 테스트가 명시적으로 건너뛰어지며 이를 변환 검증 통과로 세지 않는다. torchvision·모델·optimizer는 어댑터 테스트에 필요하지 않다. 이 모듈에는 학습 실행기나 체크포인트 저장 경로가 없다.

학습 실행은 별도 [제한된 CV 개발 학습 실행기](bounded-cv-training.md)에 분리돼 있다. 이 문서의 inspect/render/evaluate와 tensor 어댑터는 학습을 자동 실행하지 않는다.
