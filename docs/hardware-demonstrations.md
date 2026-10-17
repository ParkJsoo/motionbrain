# 실물 시연 원본 보관

`tools/archive_hardware_demo.py SOURCE NEW_DESTINATION`은 네트워크 연결이나 장치 명령 없이 수동 로봇 시연 자료를 비공개 사본으로 보관한다. 입력 단계는 `step-01`, `step-02` 등의 폴더로 구성한다. 복사 대상은 `plan.json`, `execution/result.json`, `after.png` 또는 `after.jpg`, 선택 항목인 `camera.jpg`와 `camera-receipt.json`뿐이다. 인증정보·스크립트·무관한 파일은 제외한다. 원본 경로에는 심볼릭 링크가 없어야 하며, 보관 작업 중 원본을 변경하지 않는다.

```sh
python3 tools/archive_hardware_demo.py .local/example-phase datasets/raw-demo-001 \
  --observation placed --note 'Object remained standing while open jaws lifted.'
```

출력 디렉터리는 새로 만들어야 한다. 파일은 비공개·불변 자료로 저장하고, `fsync`와 SHA-256 재읽기 검증을 마친 뒤 최종 `demo.json`을 기록한다. 실패하거나 누락된 시도도 목록에 남긴다. `archiveComplete`는 복사 완료를 뜻한다. `sourceRecordsComplete`는 연속된 모든 단계에 계획·결과·최소 한 장의 사후 이미지가 있다는 뜻이며, 실행이나 실물 작업의 성공을 뜻하지 않는다.

처리 가능한 쓰기 오류가 발생하면 최종 매니페스트를 제거한다. 프로세스나 전원이 갑자기 종료되면 정리 작업도 중단될 수 있으므로, 남은 바이트를 감사한 뒤 사용한다. 기존 보관 폴더에서 작업을 재개하지 않는다. 나중의 감사 통과는 과거 `fsync`의 영속성을 증명하지 않는다. `audit_demo()`는 바이트 누락·변경과 원본 전용 계약의 변경을 탐지한다. 파일과 매니페스트를 함께 고친 경우까지 해시가 보호하지는 않는다.

`--observation`은 보관된 사후 이미지에 연결하는 관찰자 주석이다. ACK·STOP·모터 OFF만으로 시각적 결과를 추론하지 않는다. 입력 결과 JSON은 바이트 그대로 보존하며, 그 안의 주장이 보관 자료에 학습 자격을 부여하지 않는다. 이미지는 관찰 시점의 스냅샷이며, 행동 시작·종료에 동기화한 프레임이 아니다. 호스트의 화면 획득 시각은 광학 노출 시각이 아니다.

기존 기준은 원본 영상 나이 **100ms**, 앵커 나이 **15s**, 원본 촬영 자료의 영속 저장 **250ms**다. 이는 정책 기준이며 측정 통과값이 아니다. 세 검사는 모두 `unknown`이다. 오프라인 복사 속도로 원본 촬영 시점의 저장 시간을 증명할 수 없다. `trainingEligible`과 `controlEligible`은 항상 false이며, 별도 형식인 `motionbrain.hardware_demo.v1`을 학습 데이터셋 생성기의 capture-session 입력으로 사용할 수 없다.

이 자료는 시각 주석, 반복성 분석, 디버깅에 사용한다. 학습용 전이로 인정하려면 기존 광학 시각·의미 기반 앵커·제한된 행동 실행·영속 저장 근거가 필요하다. 이미지·응답 기록·보관 자료는 Git에서 제외한 `.local/` 또는 `datasets/`에 두며 Git에 올리지 않는다.

이미지 확장자는 실제 PNG/JPEG 시그니처와 일치해야 한다. 화면 제공기가 PNG 파일명으로 JPEG를 반환했다면 별도 입력 디렉터리에 바이트가 동일한 `.jpg` 사본을 준비한다. 원본을 재인코딩하거나 버리지 않는다.
