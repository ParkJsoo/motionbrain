# 제한된 CV 개발 학습 실행기

`tools/train_cv.py`는 승인된 한 세션의 개발용 6장에 대해 실행 1회를 제한하는 선택적 도구다. **실행기 검증·개발용 학습 완료와 일반화 성능 검증은 다르다.** 이 도구의 개발용 학습만으로 새로운 장면의 검출 정확도를 확인할 수 없다. 가중치 사용 조건과 다운로드·학습 승인은 별도로 확인해야 한다.

## 고정 범위

- 모델: `fasterrcnn_mobilenet_v3_large_320_fpn`. 다운로드 기능은 없으며 두 자동 가중치 옵션을 끄고, 로컬 state_dict를 SHA-256 검사 후 `weights_only=True`, CPU로 읽는다. 다운로드 없는 생성 경로의 backbone BatchNorm을 COCO 생성 경로와 같은 FrozenBatchNorm으로 바꾸고, 기존 91클래스 모델에 strict 로딩한 뒤 배경0·립밤1·집게2 predictor로 교체한다. 호환되는 파일이라는 사실만으로 출처·사용 권한이 인증되는 것은 아니다.
- 입력: [승인 매니페스트와 tensor 어댑터](offline-cv.md)의 `train_development_pilot` 분할. 정확히 한 세션6장·상자10개·립밤 없음2장·집게 존재6장이어야 한다. 다른 분할의 데이터는 학습하지 않으며 전체 매니페스트의 누수 검사는 필터 전에 적용한다. 각 이미지의 해시는 사전 검사와 실제 읽기마다 확인한다.
- 환경: Python에서 torch2.14.1, torchvision0.29.1, NumPy, Pillow가 필요하다. 자동 설치·업그레이드·MPS 전환은 없다. CPU/float32, CPU스레드4, batch2, shuffle, workers0, 증강없음으로 고정한다.
- 업데이트: `roi_heads.box_predictor`만 학습 가능하게 하고 나머지는 동결한다. backbone은 eval 상태로 유지하여 BatchNorm 통계 변경도 막는다. SGD의 lr0.005·momentum0.9·weight_decay0.0005는 개발 진단을 위한 시작값이며 성능 보장값이 아니다.
- 상한: 10epoch, optimizer 시도30회, 900초. 계획에서 더 작은 양의 정수로 줄일 수 있지만 늘릴 수 없다. 시간은 새 실행 디렉터리를 만든 뒤 시작하므로 import·해시 검사·입력 읽기·모델 초기화·저장도 포함한다.

## 검토용 계획 파일

계획은 Git 제외 경로에 둔다. 아래는 형식 예시이며 실제 경로·전체 해시로 바꾸기 전에는 실행할 수 없다. 상대 경로는 계획 파일의 폴더를 기준으로 해석한다.

```json
{
  "schema": "motionbrain.cv_training_plan.v1",
  "manifest": "data/manifest.json",
  "manifest_sha256": "실제 매니페스트의 소문자 SHA-256 64자리",
  "weights": "weights/source.pth",
  "weights_sha256": "사용 범위를 확인한 로컬 가중치의 소문자 SHA-256 64자리",
  "output_dir": "runs/development-01",
  "epochs": 10,
  "steps": 30,
  "seconds": 900,
  "seed": 20261027
}
```

정확한 hash로 입력과 계획을 결합하지만 사람의 승인이나 가중치 라이선스를 인증하지 않는다. `check`는 계획·라벨·분할·프레임/가중치 바이트 해시를 읽기 전용으로 확인한다. 이미지 디코딩, tensor 생성, 가중치 역직렬화, 모델 실행은 하지 않으므로 실제 실행 성공을 보장하지 않는다. 모델 파일이 없으면 검사를 통과시키기 위해 임의 다운로드하지 않는다.

```sh
python tools/train_cv.py check --plan .local/cv-training/plan.json
```

## 명시적 실행과 중단

다음 명령은 실제 optimizer 업데이트를 수행한다. 해당 입력·계획에 대한 사용자 승인을 받은 뒤에만 사용한다.

```sh
python tools/train_cv.py run --plan .local/cv-training/plan.json --execute
```

`--execute`가 없으면 모델을 불러오거나 실행 디렉터리를 만들지 않는다. 실행 시 계획 옆의 `plan.json.consumed`를 배타적으로 생성하며, 한 번 생성되면 성공·실패·중단 여부와 무관하게 자동 해제하지 않는다. 이미 있는 출력 디렉터리도 덮어쓰지 않는다. 재시작·resume·자동 재시도는 없다. 이 장치는 동일 계획 경로의 실수로 인한 중복 실행을 막는 것이며, 계획을 복사하거나 소비 기록을 지운 사람이 새로 실행하는 것까지 막는 인증/전역 사용량 시스템은 아니다. 새 실험은 별도로 승인·기록해야 한다.

부모 프로세스가 별도 worker를 감시한다. 시간 상한 도달 또는 Ctrl-C/SIGTERM이면 종료를 요청하고 필요하면 강제 종료한다. 종료 확인에 최대 약2초의 여유가 추가될 수 있다. OS 스케줄링·입출력 지연까지 포함한 실시간 제어 보장은 아니다. worker 내부에서도 긴 작업 전후와 업데이트 직전에 시간을 검사한다. 네트워크 요청 진입점을 차단하지만 이 기능을 OS 수준 네트워크 격리로 해석하지 않는다. 카메라·장치·서비스 경로는 없다.

각 업데이트 전에 `step_intent`, 완료 반환 후 `step_complete`를 동기 기록한다. `stepIntents`는 `stepsCompleted`보다 클 수 있다. 업데이트 중단이나 업데이트 후 오류가 있으면 마지막 시도가 실제로 적용됐는지 불확실하므로 성공으로 올려 세지 않고 자동 재시도하지 않는다. epoch/step 제한은 진단 완료의 종료 사유일 뿐 정확도 합격이 아니다. `epochsCompleted`는 배치 반복을 끝까지 빠져나온 epoch만 세므로 마지막 배치에서 step 제한에 도달하면 더 작을 수 있다.

## 산출물과 실패 해석

- `run.json`: 최종 상태, 실제 적용 계획, 계획 해시, 완료/시도 횟수, 경과 시간. 종료된 `step_limit` 또는 `epoch_limit`만 정상 제한 종료다.
- `events.jsonl`, `progress.json`: 실행 순서와 마지막 진행 상태. 갑작스러운 전원 종료·부모 강제 종료 시 최종 보고서가 남지 않을 수 있다. `starting`/`running`을 완료로 해석하지 않는다.
- `checkpoint.pt`: 정상 제한 종료 후 동결된 파라미터·buffer 해시를 확인하고 최대1개를 저장한다. state_dict와 입력 해시·seed·고정 설정·개발용 결과를 포함하며 optimizer 상태를 저장하지 않는다. 런타임 버전은 일반 문자열로 저장하여 `torch.load(..., weights_only=True)`로 읽을 때 별도 사용자 정의 타입 허용이 필요하지 않게 한다. 재개 기능이 없다.
- `checkpoint.pt.partial`: 저장 도중 실패·종료되면 남을 수 있는 불완전 파일. 자동 삭제하지 않으며 최종 checkpoint로 쓰지 않는다. timeout 후 최종 파일이 남더라도 run 상태를 우선하고 성공 실험으로 채택하지 않는다.

실패·중단도 실험 이력에 남긴다. `runConsumed`는 실행 시도 소비, `trainingMayHaveStarted`는 optimizer 시도 기록 여부다. 첫 업데이트 이전 실패와 실제 optimizer에 도달한 시도를 구분하며, 뒤의 시도는 학습 횟수 예산에 포함한다. 파일 경로·프레임 식별자·가중치와 checkpoint는 비공개 자료로 보관한다.

이 도구는 validation/test 평가, AP, 동영상 추론, 예측 시각화, checkpoint 선택을 수행하지 않는다. 개발 자료 손실이 낮아져도 일반화 성능을 증명하지 못한다. 사전학습 원본 및 미세조정 가중치의 공개 배포 가능 여부도 실행 성공과 별도로 확인한다.

## 소프트웨어 검증 범위

```sh
python -m unittest discover -s tests -p 'test_cv_training.py' -v
```

제어 흐름은 가짜 업데이트 카운터·작업 프로세스로 검증하고, 실제 잠자는 자식 프로세스로 시간 종료를 검사한다. 선택적 torch 검사는 작은 합성 모듈과 모의 optimizer를 사용하며 실제 optimizer.step을 호출하지 않는다. head만 학습 가능함, backbone 통계 동결, 비정상 손실 거부, 저장 전 동결 상태 검사를 확인한다. torch 없는 환경에서는 해당 검사를 명시적으로 건너뛴다. 이 결과를 실제 Faster R-CNN의 승인 영상 학습 성능으로 표현하지 않는다.
