# 고정 배경 기준 좌표 관측의 개발 연결

이 기능은 외부 카메라의 작은 영상 변형을 고정 배경만으로 추정하는 **오프라인 M1 비접촉 관측 개발 모드**다. 현재 실물 수집·모델 제어에 사용할 자격은 없다. `motionbrain.batch_background_registered_observer.v1`은 기존 관측 schema와 구분하며, `BatchRuntime`은 기존 motion/collection admission이 있더라도 이 schema의 실물 진입을 무조건 거부한다. 원래 정지 관측기의 실패·앵커·모델·자료는 변경하지 않는다.

## 좌표와 증거

기준 PNG 및 픽셀 해시, 공개 알고리즘 구현 해시와 OpenCV 버전, 고정 배경 영역·매개변수를 새 관측 계약에 고정한다. 사용한 배경은 해당 구도의 왼쪽 벽과 위쪽 벽 두 영역이다. 로봇·물체·연결선 근처의 받침 영역과 영상 아래쪽은 fit에서 제외했다. 이 마스크가 다른 관절 자세에서도 배경이라는 주장은 하지 않는다.

기준→현재 affine은 배경 특징의 양방향 LK 대응과 RANSAC으로 계산한다. 현재 영상을 기준 좌표로 역변환한 뒤 기존 template/HSV 관측을 재사용한다. 양쪽 endpoint와 LK·피라미드의 영상 지지 영역을 배경 안에 제한한다. 전경의 관측값·목표값·예상 명령 결과로 변환을 맞추지 않는다. 고정 앵커 중심은 원본 reference의 선언된 point다. 현재 프레임 평균이나 이전 운영용 acceptance 중심을 따라가지 않는다.

결과에는 다음을 구분해 보존한다.

- `frameBinding`: 실제 원본 영상의 해시·frame ID·시각.
- `backgroundFit`: 기준→현재/현재→기준 변환, 대응점·inlier·잔차·기하 제한 결과.
- `landmarks`, `states`: 정규화된 기준 좌표 관측.
- `rawCurrentLandmarks`: 원본 영상에 새 기준 중심의 기존 detector를 별도로 적용한 값과 거부 상태. 과거 acceptance-window 계약의 판정과는 다르다.
- `mappedCurrentLandmarks`: 정규화 검출을 현재 좌표로 역매핑한 값. 독립 원본 검출이 아니다.
- `normalizedFrameBinding`: 같은 원본 frame ID에서 파생한 픽셀의 별도 해시. 새 촬영이나 원본 해시가 아니다.

기존 landmark/state의 선언 불확실성은 affine 추정·재표본화 오차를 포함하지 않는다. **총오차는 미보정·미상**이다. 배경 잔차는 전경의 실제 정확도·깊이·접촉·물리 자세의 증명이 아니다. 원본 수동 라벨을 동일 추정 변환으로 옮긴 비교 역시 변환 정확도의 독립 검증이 아니다.

## 제한과 재생

최종 개발 설정은 half-resolution, LK window 21·pyramid 1·배경 경계 안쪽 52 px다. 최소 대응 30개, 영역별 inlier 4개, inlier 비율 0.90, hull 면적 100,000 px², 잔차 p95 0.75 px를 요구한다. scale 0.985–1.015, 회전 ±0.15°, 중심 이동 2 px, 모서리 이동 20 px, shear 0.003을 넘으면 거부한다. 이 수치는 현재 개발 자료를 바탕으로 선언한 제한이며 물리적으로 검증한 안전 영역이 아니다.

```sh
python3 tools/replay_background_observation.py --manifest .local/background-replay/manifest.json --output .local/background-replay/new-output
```

manifest는 `legacySpec`의 경로·해시와 `sets`를 가진다. 각 set에는 `name`, `role`(`development` 또는 `diagnostic_development`), 원본 `folder`, `frames` 목록 파일의 경로·해시, `expectedViewConfigurationId`, `expectedConfigurationSha256`을 고정한다. 원본 metadata/image 해시·장치·shape·구성 ID를 검사하고 폴더 밖 경로를 거부한다. 새 촬영·인증·장치 요청은 없다.

## 개발 재생 결과와 미완료 조건

처음 window 31·pyramid 1·inset 72는 높이 155 px인 위쪽 배경에서 유효 폭이 부족해 190/190장을 영역별 inlier 부족으로 거부했다. pyramid 0·inset 40은 합성의 알려진 작은 카메라 변형에서도 대응 실패를 보였고, 실제 자료에서는 111/190장만 통과했다. 이 두 실패 결과는 보존했다. inlier·잔차·변형 허용 기준을 완화하지 않고, 다중 해상도를 유지하면서 window를 21로 줄인 최종 설정을 평가했다.

| 자료 구분 | 통과/전체 | 거부 |
|---|---:|---:|
| 기존 동작 개발 영상 | 29/106 | 77 |
| 이전 정지 영상 | 0/23 | 23 |
| 최근 차단된 정지 영상 | 5/23 | 18 |
| 별도 광학 진단 개발 영상 | 38/38 | 0 |
| 합계 | 72/190 | 118 |

최종 거부 118장은 모두 영역별 inlier 부족이다. 통과한 자료의 배경 잔차 p95는 약 0.206–0.443 px였다. 38장도 설계 과정에서 확인한 **개발 자료**이며 고정 후 holdout이 아니다. 정지 38/38 통과를 새 실물 움직임 검증이나 전체 운영 범위 확보로 확대하지 않는다.

합성 시험에서는 카메라 변형과 독립적인 전경 이동을 동시에 넣어 전경 이동이 남는지, 선언한 배경 밖 픽셀을 극단적으로 바꿔도 대응점·변환이 동일한지 검사했다. 과도한 변형·배경 결손·계약 변조·실물 진입은 거부한다. 소프트웨어 연결과 실패 시 차단은 검증했지만, **실제 운영 관측 영역은 아직 성립하지 않았다.** 새 실물 명령·자동 수집·고정 후 모델 평가·파지 성공은 이 기능의 결과가 아니다.
