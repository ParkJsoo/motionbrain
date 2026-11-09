# 두 카메라와 M4를 Rerun에서 실시간으로 보기

[운영 안내](../OPERATIONS.md) · [저장 평가와 시뮬레이터](robot-visualization.md)

`tools/visualization/live.py`는 외부 카메라·집게 카메라의 최신 영상과 등록 관측점, 실측 M4 센서값을 Rerun에 보낸다. 모터 API·학습·W&B 게시 기능은 없다. 이 경로는 화면 확인용이며 수집기나 제어 피드백으로 사용하지 않는다.

## 준비와 실행

macOS의 외부 카메라는 정확한 Continuity Camera 식별자로 선택한다. 해당 장치가 없으면 다른 카메라로 전환하지 않는다. 기존 영상에서 확인한 두 카메라의 `RegisteredSceneLandmarks` 등록과 참조 원본, 외부 촬영의 설정 메타데이터가 필요하다. 새 구도에 옛 등록을 자동 적용하지 않는다.

시각화 환경의 [의존성](../tools/visualization/requirements.txt)과 Xcode 명령줄 도구를 준비한 뒤 저장소 루트에서 실행한다.

```bash
mkdir -p .local/bin
bash tools/macos/build_camera_probe.sh .local/bin/mac-camera-preview --preview
# 별도 터미널에서 기존 로컬 포털/Rerun 서버를 실행한 상태로 사용한다.
python tools/visualization/live.py --config .local/live-preview-config.json
```

설정의 장치별 값·참조 경로는 현재 설치 환경에서 확인해 채운다. 비공개 설정은 `.local/`에 둔다. `output`은 매 실행마다 새 비공개 폴더를 지정한다.

```json
{
  "helper": ".local/bin/mac-camera-preview",
  "rerunUrl": "rerun+http://127.0.0.1:9876/proxy",
  "output": ".local/live-preview-01",
  "external": {
    "deviceId": "<현재 외부 카메라 식별자>",
    "registration": ".local/<외부 등록.json>",
    "reference": ".local/<외부 참조.png>",
    "metadata": ".local/<확인한 외부 프레임 메타데이터.json>"
  },
  "gripper": {
    "url": "http://<카메라 주소 또는 로컬 경유 포트>",
    "bootId": "<현재 카메라 boot ID>",
    "registration": ".local/<집게 등록.json>",
    "reference": ".local/<집게 참조.jpg>"
  },
  "controller": {
    "url": "http://<현재 제어기 주소>",
    "bootId": "<현재 제어기 boot ID>"
  }
}
```

외부 메타데이터에서 `deviceUniqueId`, `configurationSha256`, `width`, `height`, `videoMirrored`, `videoRotationAngle`을 확인한다. 현재 설정이 이 값과 다르면 표시를 중단하며 카메라 설정을 자동 변경하지 않는다. [로컬 Rerun](http://127.0.0.1:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy)에서 새 실시간 기록을 선택한다. 종료는 실행 터미널의 `Ctrl+C`다. 유한 점검에는 `--seconds 10`처럼 실행 시간을 지정할 수 있다.

외부 영상은 최대 2Hz, 집게 영상은 요청 처리 후 0.5초 간격, M4는 요청 처리 후 1초 간격으로 갱신한다. 화면 확인용 주기이며 모터 제어 주기나 두 카메라의 동기 촬영을 의미하지 않는다. `host_elapsed`는 같은 Mac의 단조 시계에 따른 수신·프레임 시각이며 광학적 동기화나 지연 없는 영상을 보장하지 않는다.

## 상태와 기록의 해석

- `live`: 최신 입력을 받았으며 해당 관측 검사를 통과했다.
- `rejected`: 최신 원본 영상은 표시하지만 특징점 관측은 거절됐다. 이전 관측점을 남기지 않는다.
- `stale`: 영상 1.5초, M4 2.5초의 표시 유효 시간을 벗어났다. 과거 영상은 현재 화면에서 지운다.
- `failed`: 연결·원본·설정·부팅 검사에서 실패했다. 해당 입력을 자동 재시도하거나 재등록하지 않는다.

미리보기는 원본을 계속 쌓지 않는다. 외부 영상은 메모리에서 전달하며 집게 영상은 기존 fresh/boot/픽셀 검증에 필요한 한 장만 임시 저장하고 지운다. 실행 폴더에는 최신 `status.json`과 오류·종료 진단만 남는다. 화면 영상은 표시용으로 압축되며 학습 자료나 실험 원본을 대신하지 않는다. Rerun 서버의 버퍼·저장 정책은 별도이며, 실험 재현용 원본은 기존 유한 수집기로 수집한다.

M4는 펌웨어의 `angleDeg` 센서 좌표다. 미확인 관절을 측정값으로 만들거나 시뮬레이터의 전체 팔 자세를 실시간으로 갱신하지 않는다. 특징점 일치가 깊이·접촉·파지·측정 정확도나 기존 모델의 적용 가능성을 입증하지도 않는다.

현재 미리보기와 수집기는 별도 카메라 세션을 연다. 같은 외부 카메라로 동작 자료를 수집하기 전에 미리보기를 종료한다. 수집 중의 실시간 표시는 수집기가 사용하는 프레임 스트림에 연결해야 하며, 이 미리보기를 병렬 실행해 대체하지 않는다.
