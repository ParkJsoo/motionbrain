# ESP32-CAM 주소·포트·재부팅·충돌 진단 — 2026-10-15

최신 후속: [부팅 watchdog 조사와 별도 네트워크 오류 수정](2026-10-15-camera-boot-watchdog.md)을 완료했다. watchdog은 반복 부팅에서 재현되지 않아 원인 미확정이며, 아래 기록은 그 수정 이전의 시험 결과다.

**주소와 포트는 맞다. 재부팅으로 복구됐지만, Wi-Fi 송신 경로의 실제 펌웨어 충돌과 지속 무응답을 재현했다. 근본 원인 수정은 아직 완료하지 않았다.** 앞선 짧은 비구동 검증 통과를 장시간 안정성으로 확대해서 해석하지 않는다.

후속: 사용자의 카메라 위치 고정 후 같은 SDK·같은 요청 부하에서 변경 전후 각각 10분(각 520/520 요청), 현재 버전 복원·Pi 재개 후 5분(260/260 요청)을 통과했다. 최초 부팅 watchdog 1회는 별도 보존했고 근본 원인은 미확정이다. 최신 결과는 [장시간 비교 기록](2026-10-15-camera-long-comparison.md)을 참고한다. 아래는 그 이전 장애 재현 기록이다.

## 조건과 범위

- 현재 코드 `559db54`, CAM의 최근 변경은 `ca48f04`의 원본 프레임 식별/응답 헤더 추가다.
- 실제 칩 MAC `30:76:f5:e5:ea:60`, CH340 `/dev/cu.usbserial-1120`. 메인 ESP32·STM32 펌웨어와 구동 상태는 변경하지 않았다. 이 진단에서 모터 명령을 보내지 않았다.
- 사용자는 ESP32·STM32·CAM 모두 Mac USB 허브에 연결되어 있다고 확인했다. 초기 진단 때는 배터리함→XL4015가 연결돼 있었고, 진단 중 사용자가 분리한 이후에도 시험했다. 앞선 문서의 전원 분리 조건을 이 전체 진단에 소급 적용하지 않는다.
- Pi perception은 기존 설정으로 동작하도록 유지했다. 별도 클라이언트 시험은 Mac에서 2개 클라이언트가 각각 3초 간격으로 `/capture`를 12회 요청하는 조건이다. 브라우저 등 모든 외부 요청과 RF 환경을 완전히 통제한 시험은 아니다.
- 비교 코드는 `ca48f04^`의 CAM 소스이며 임시 디렉터리에서 같은 PlatformIO espressif32 6.12.0 / Arduino 2.0.17 / ESP-IDF 4.4.7로 빌드했다. 과거 flash 백업 전체나 과거 SDK 조합을 그대로 복원한 시험은 아니다. 저장소 구현 코드는 수정하지 않았다.

## 가설별 관찰

| 가능성 | 관찰과 판단 |
|---|---|
| 잘못된 IP·IP 변경 | 재부팅 직후 같은 `192.168.219.116`에서 복구. Pi ARP도 실제 CAM MAC과 일치. 무응답 때 다른 주소 검색에서도 카메라를 찾지 못했다. 주소 변경이 이번 장애의 설명은 아니다. |
| 잘못된 포트 | 펌웨어는 `WebServer(80)`이며 재부팅 후 같은 80번에서 `/`, `/status`, `/capture` 모두 HTTP 200. 무응답 때 80·443 연결 모두 타임아웃. HTTPS나 별도 영상 포트를 요구하는 화면이 아니다. |
| 토큰·브라우저만의 문제 | Pi에서도 ping 실패·ARP FAILED·perception No route to host가 발생했다. 카메라 root/capture GET에는 조작 토큰이 필요하지 않는다. 브라우저 밖에서도 재현된다. |
| 재부팅으로 해결 | 카메라 RTS 리셋 후 약 5.1초에 HTTP 서버가 복귀했다. 하지만 이후 충돌 및 지속 무응답이 재발해 일시 복구만 확인했다. |
| Wi-Fi 송신 중 충돌 | UART에서 Core 0 `LoadProhibited`를 포착했고, 동일 ELF로 Wi-Fi 송신 `ppProcTxSecFrame` 경로를 확인했다. 아래 증거 참조. 충돌 위치는 확인했지만 SDK 자체 결함인지 다른 곳에서 손상된 메모리가 드러난 것인지는 미확정이다. |
| 여러 이미지 요청의 영향 | 첫 2-client 시험 중 충돌, 반복 시험은 정상. 배터리 분리 후 동일 시험에서는 지속 무응답. 트래픽과 함께 재현됐지만 동시 요청만으로 반드시 발생하는 것은 아니다. |
| 모터 전원·USB 허브 | 배터리 분리 후에도 재현되어 모터 전원 연결만을 원인으로 볼 수 없다. USB 허브·케이블·보드 5V/3.3V는 분리 시험/실측하지 않아 제외하지 못했다. 이번 CAM 충돌에는 brownout 메시지가 없었다. 앞선 메인 ESP32의 brownout 증거와 혼동하지 않는다. |
| 무선 환경 | 현재 코드 관찰 RSSI는 -78~-48dBm으로 변동했다. RF 불안정이 영향을 주는지는 미확정. 이전 코드 시험은 대체로 더 나은 신호 조건이어서 완전한 통제 비교가 아니다. |
| 단순 heap 고갈·카메라 캡처 실패 | 충돌 직전 heapFree 약 191KB, captureFailures/slowCaptures/clientWriteFailures=0. 관찰된 값은 단순 고갈·캡처 실패를 뒷받침하지 않는다. heap 손상·스택/메모리 오류까지 배제하는 것은 아니다. |
| 최근 카메라 변경 회귀 | 같은 SDK의 변경 전 소스에서는 약 179초 시점까지 uptime 유지, 107 captures 및 captureFailures=0. 최근 변경의 영향은 배제하지 못하지만, 짧고 순차적인 A/B 및 달라진 RF 조건만으로 인과를 확정하지 않는다. |

## 재현한 충돌

리셋 후 약 129.2초에 다음 오류가 발생했다. 약 134.4초에는 자동 재부팅 후 서버가 다시 시작됐고, HTTP에서도 boot ID 변경 및 `resetReason=panic`을 확인했다.

```text
Guru Meditation Error: Core 0 panic'ed (LoadProhibited)
PC: 0x4016995f
EXCVADDR: 0x80169ad7
Backtrace:
  ppProcTxSecFrame
  ppTxPkt
  ppTxPktForceWaked
  ieee80211_output_process
  ppTask
```

UART의 ELF hash prefix `501c263d72221fa9`와 실제 업로드 빌드 ELF SHA-256
`501c263d72221fa988e7f3184ebc1d7ef682a2add83cb18a0644dc9d16fcf8c9`가 일치한다.
link map도 해당 심벌을 SDK `libpp.a(pp.o)`로 연결한다. 디버그 소스 행 번호는 제공되지 않았다.

`LoadProhibited`는 유효하지 않은 메모리 읽기 오류를 뜻한다. 이 해석은 [Espressif의 해당 버전 공식 오류 문서](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-guides/fatal-errors.html#loadprohibited-storeprohibited)에 따른다. 드라이버 함수에서 발생했다는 사실만으로 SDK가 최초 손상 원인이라고 단정하지 않는다.

## 실물 시험 결과

| 코드 / 조건 | 이미지 요청 | 결과 |
|---|---:|---|
| 현재 / 배터리 연결 / 1차 | 23/24 성공 | UART panic 1회와 요청 타임아웃, 자동 복귀 |
| 현재 / 배터리 연결 / 반복 | 24/24 성공 | 해당 구간 재충돌 없음 |
| 현재 / 배터리 분리 | 4/24 성공 | 약 5~6초 후부터 네트워크 무응답. 약 75초 관찰까지 지속. 즉시 추가 panic 출력은 없었으며 장기 무응답이 위 panic과 같은 원인인지는 미확정 |
| 메타데이터 추가 전 소스 / 배터리 분리 / 1차 | 22/24 성공 | 시작 시점 두 요청 타임아웃; UART 정상 부팅과 그 뒤 정상 응답 확인 |
| 같은 이전 소스 / 2차 | 24/24 성공 | 정상 |
| 같은 이전 소스 / 부팅 후 약 100초의 3차 | 24/24 성공 | 정상. 종료 후 uptime 약 179초, resetReason=poweron 유지 |

현재 코드의 첫 약 5분 네트워크 표본은 59회 중 57회 응답, 2회 오류(부팅 중 1회·panic 구간 1회)였다. 평균 성공률만으로 간헐적인 장치 다운을 숨기지 않는다. 이후 배터리 분리 조건에서는 별도로 지속 무응답이 발생했다.

## 후속 판단

비교 시험 후 현재 펌웨어로 복원했고 업로드 hash 검증에 성공했다. 복원한 ELF SHA-256은 위 충돌 분석 대상과 동일하다. 마지막 확인에서 `/`, `/status`, `/capture`는 모두 200이며 원본 frame ID 헤더가 복귀했다. Pi exact JPEG/hash·누락 ID 404·stale 차단 검사도 다시 통과했다. 이는 복원 확인이며 간헐 장애 수정 완료를 뜻하지 않는다.

1. 현재 주소·포트 변경은 해결책이 아니다. 재부팅은 임시 복구 수단이다.
2. USB 허브를 제외한 CAM 단독 전원/연결 조건과 현재 조건을 비교하고, 가능하면 5V/3.3V를 실측한다. 배터리를 분리한 결과만으로 USB 전원 문제까지 배제하지 않는다.
3. 동일한 전원·RF·트래픽 조건에서 변경 전/후 코드를 더 길게 비교한다. Wi-Fi disconnect reason, loop heartbeat, stack 여유·heap integrity 계측을 추가해 지속 무응답과 panic을 분리한다.
4. 반복 결과에 따라 최근 HTTP 응답 변경·메모리 사용 또는 CAM 전용 SDK 비교를 좁혀 수행한다. 전체 메인 펌웨어 재설계나 추가 관절 센서는 필요하지 않다.
5. 기존 HTTP stall 자동 재시작은 기본 OFF이며 이번에 변경하지 않았다. SDK 자동 reconnect는 일부 disconnect 사유에 반응하지만 앱에서 원인 로그·복구 실패 감시는 부족하다. 자동 재부팅을 켜는 것만으로 근본 해결을 주장하지 않는다.

## 보존한 실물 증거

- [현재 코드 UART 및 panic](camera-diagnostics-2026-10-15/uart-panic.txt)
- [ELF 대조·심벌 해석](camera-diagnostics-2026-10-15/backtrace-decoded.txt)
- [현재 코드 네트워크 관찰](camera-diagnostics-2026-10-15/network-current.jsonl)
- [배터리 분리 후 무응답](camera-diagnostics-2026-10-15/network-battery-off.jsonl)
- [클라이언트별 전체 요청 결과](camera-diagnostics-2026-10-15/client-results.json)
- [이전 코드 UART](camera-diagnostics-2026-10-15/old-firmware-uart.txt)
- [이전 코드 네트워크 관찰](camera-diagnostics-2026-10-15/old-firmware-network.jsonl)
- [현재 펌웨어 복원 후 상태](camera-diagnostics-2026-10-15/restored-current-status.json), [원본 프레임 계약 재검증](camera-diagnostics-2026-10-15/restored-source-contract.json)

이 기록은 실제 카메라 통신 진단이다. 모터 조작·학습 데이터 수집·학습 성능 검증 결과는 아니다.
