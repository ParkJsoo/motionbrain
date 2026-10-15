# ESP32-CAM 부팅 watchdog 후속 진단 — 2026-10-15

**watchdog의 근본 원인은 아직 확정하지 못했다. 반복 부팅에서 재현되지 않았으며, 별도로 확인된 호스트명 적용 순서와 소켓 timeout 단위 오류를 수정했다. 이 두 수정으로 기존 watchdog을 해결했다고 주장하지 않는다.**

[이전 25분 비교](2026-10-15-camera-long-comparison.md)에서 최초 부팅 중 관찰한 `TG0WDT_SYS_RESET` 1회의 후속 조사다. 카메라 전용 시험이며 모터 명령/ARM, 메인 ESP32·STM32 업로드는 수행하지 않았다.

## 조건과 방법

- 사용자가 고정한 카메라 위치와 기존 USB 허브/케이블을 유지했다. 배터리함→XL4015는 마지막 사용자 보고상 분리 상태다. 전압 실측이나 USB 직결 대조는 수행하지 않았다.
- CAM MAC `30:76:f5:e5:ea:60`, 당시 `/dev/cu.usbserial-1120`, `192.168.219.116:80`. QVGA / JPEG quality 15, SDK는 앞선 비교와 같은 Arduino 2.0.17 / ESP-IDF 4.4.7이다.
- Pi perception은 계속 실행했다. 각 회차에서 CAM의 RTS 리셋을 수행하고 UART를 연속 기록했다. 최소 20초 관찰 후 `/status`, `/capture`를 요청했다. 서버 준비가 없으면 최대 45초까지 관찰하고 시험을 중단하도록 했다.
- 이 시험은 **EN/RTS 리셋 반복**이다. USB 전원을 뽑았다 연결하는 cold power-cycle 시험으로 해석하지 않는다. 로그의 `poweron`은 장치가 보고한 reset reason이다.
- 기준 버전은 `559db54`의 CAM 소스다. 임시 진단 버전은 기능 순서를 유지하면서 각 Wi-Fi API 전후 시각, heap 무결성, 가용 스택, 이벤트 번호/이유, 실제 STA 인터페이스 호스트명을 기록했다. Wi-Fi 자격 증명은 기록하지 않았다.
- 수정 진단 버전에서는 실제 `SO_RCVTIMEO`, `SO_SNDTIMEO`를 `getsockopt`로 읽고 Stream timeout도 확인했다. heap 전체 검사·이벤트 출력·소켓 설정값 출력은 최종 제품 펌웨어에서 제거했다. 짧은 부팅 단계 로그 4개는 재발 시 중단 지점을 식별하기 위해 제품 코드에 남긴다.

## 확정한 오류와 수정

### 1. 초기 연결에 잘못된 호스트명 적용

기존 순서는 `WiFi.mode(WIFI_STA)` 다음 `WiFi.setHostname(...)`였다. 이 SDK는 `mode()`에서 현재 캐시된 이름을 네트워크 인터페이스에 적용하고, `setHostname()`은 캐시만 바꾼다. 실제 진단에서도 10회 모두 `actual=esp32-E5EA60`, `requested=motionbrain-cam`이 관찰됐다.

설정을 `mode()` 앞으로 옮긴 뒤 수정 진단의 5회 모두 실제 이름이 `motionbrain-cam`으로 일치했다. mDNS에 명시적으로 같은 이름을 등록하는 기존 경로와 DHCP/STA 호스트명은 서로 다른 설정이다. 이 오류를 이전 숫자 IP 장애나 watchdog의 확정 원인으로 해석하지 않는다.

### 2. 750밀리초가 750초로 해석되는 timeout

`WiFiClient::setTimeout(uint32_t seconds)`는 초 단위이며 내부에서 Stream 값에 `seconds * 1000`, 소켓 `timeval.tv_sec`에 전달값을 사용한다. 기존 `client.setTimeout(CLIENT_IO_TIMEOUT_MS)`는 의도한 750ms 대신 750초를 설정했다.

수정은 Stream timeout을 명시적으로 750ms로 설정하고, 소켓 송수신 timeout을 `tv_sec=0`, `tv_usec=750000`으로 설정한다. 설정 실패 시 정상 JPEG 헤더/본문 전송을 진행하지 않고 기존 실패 카운터·프레임 반환·연결 종료 경로를 거친다. 수정 진단의 실제 설정값 20개 표본 모두 일치했다.

이 변경은 **timeout 설정 단위의 수정**이다. SDK의 `WiFiClient::write()`는 자체 select/retry와 `MSG_DONTWAIT`를 사용하므로, 소켓 timeout만으로 모든 전송이 750ms 또는 기존 외부 deadline 2초 이내에 끝난다고 보장하지 않는다. 기존 이미지 응답 실패가 이 단위 오류 때문이었다고도 확정하지 않는다.

### 3. 다음 재발을 위한 최소 로그

제품 코드에 호스트명 설정, station mode, 절전 설정, connection begin 직전의 `millis()` 로그를 추가했다. SSID/비밀번호는 출력하지 않는다. watchdog 설정, 재시작 정책, HTTP stall supervisor 기본 OFF는 그대로다.

## watchdog에 대해 확인한 범위

설치 SDK에는 Task WDT 5초, CPU0 idle 감시, Interrupt WDT 300ms가 설정돼 있다. Arduino loop task의 별도 WDT 구독은 기본 비활성이다. 원래 오류는 ROM의 Timer Group 0 watchdog reset 및 앱의 `task_watchdog` 분류까지만 남았고, 원인 태스크/PC를 특정할 panic backtrace가 없었다.

Task WDT는 CPU 시간을 오래 양보하지 못한 태스크를 감지하고 Timer Group 0을 사용한다. 이는 [ESP-IDF 4.4.7 공식 watchdog 문서](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/system/wdts.html)에 따른다. reset reason만으로 특정 사용자 함수, 전원 저하, SDK 오류 중 하나를 원인으로 확정하지 않는다.

기준/진단 반복에서 watchdog을 재현하지 못했다. 진단 로그의 heap 무결성 실패는 0이며 초기화 중 Arduino setup 태스크(core 1)의 가용 스택 최솟값은 6,268 bytes였다. 이는 Wi-Fi 태스크(core 0)의 스택을 측정한 결과가 아니다. 계측을 넣으면 타이밍과 메모리 배치도 바뀌므로 정상 진단 결과가 원래 간헐 장애를 배제하지는 않는다.

## 검증 결과

| 단계 | 부팅/서버 준비 | 관찰 창 내 비의도 재부팅/panic | 회차 말 이미지 요청 |
|---|---:|---:|---:|
| 변경 전 제품 | 10/10 | 0 | 9/10: 5회차 timeout |
| 변경 전 + 진단 로그 | 10/10 | 0 | 9/10: 2회차 본문 일부만 수신 |
| 수정 + 진단 로그 | 5/5 | 0 | 5/5 |
| 최종 제품 | 5/5 | 0 | 5/5 |

초기 진행 설명에서 기준 버전 이미지까지 모두 정상이라고 한 표현은 전체 집계 후 정정했다. 부팅 성공과 이미지 응답 성공을 구분한다. 수정 전 2건의 통신 오류 시점에 watchdog 재부팅은 관찰되지 않았다. 회차 말 HTTP 검사 직후 다음 의도적 리셋으로 넘어가므로 그 이후의 자연 복구 시간까지 측정한 시험은 아니다.

소프트웨어 전체 회귀는 최종 코드 기준 **372개 통과, 8.913초**다. 최종 제품 빌드·업로드(63.66초) 및 flash hash 검증에 성공했다. 소프트웨어 테스트 결과는 위의 실물 부팅 결과와 별도다. Pi 통합 검사는 120초 동안 이미지 **80/80**, 상태 **24/24** 성공, UART panic/비의도 재부팅 0으로 완료했다. 원본 ID로 JPEG 재조회·hash 일치, 누락 ID 404, 약 17.72초 지난 관측의 stale/제어 불가 판정을 재확인했다. 미교정 시계는 계속 미검증 상태다.

최종 점검에서 CAM uptime 154,060ms, captureFailures=0, Pi 영상 `ok=true`·`fresh=true`·`consecutiveErrors=0`, 서비스 3개 active·NRestarts=0, Pi throttled=0x0이었다. 메인 제어기는 IDLE, motorEnabled=false, M1~M5 출력 모두 0이며 마지막 명령 ID 18은 기존 사용자의 disarm으로 유지됐다. 테스트가 모터 명령을 보낸 결과가 아니다.

반복 부팅 30회 외 최종 영상 검사 시작 시 CAM 리셋 1회를 추가했다. 업로드 종료의 자동 RTS 리셋은 반복 부팅 성공률에 섞지 않았다. 최종 배포물의 source/ELF/bin SHA-256 및 진단 패치는 증거 디렉터리에 보존했다. 진단 당시에는 커밋/푸시하지 않았고, 이후 사용자가 코드리뷰 후 main 반영을 요청했다.

기존 정적 계약 테스트가 잘못된 `client.setTimeout(CLIENT_IO_TIMEOUT_MS)` 문자열 자체를 요구해 최초 실행에서 실패했다. 해당 문자열 강제 검사를 제거했고, timeout의 실제 의미는 위 보드 설정값 검사로 검증한다. 다른 회귀 테스트를 삭제하거나 비활성화하지 않았다.

## 커밋 전 코드리뷰

- 설치된 Arduino 2.0.17 소스와 대조해 hostname 적용 시점, Stream/소켓 timeout 단위, 설정 실패 시 JPEG 전송 차단·프레임 반환·연결 종료를 검토했다. 커밋을 막을 추가 결함은 발견하지 않았다.
- 전체 소프트웨어 회귀 재실행: **372개 통과, 8.727초**. 첫 제한 환경 실행의 로컬 서버 소켓 권한 오류 9건은 권한을 허용한 재실행에서 해소됐다. CAM 빌드도 성공했다(RAM 51,836 bytes, flash 903,745 bytes).
- 보존 JSON/JSONL 파싱, Python 문법, 문서 상대 링크, 소스 hash와 배포 증거 일치, 두 분석 스크립트 재집계와 저장 결과 일치를 확인했다. 문서의 현재 상태 및 미커밋 표기를 정리했다.
- 이 리뷰에서는 추가 업로드·장치 리셋·실물 시험·모터 명령을 수행하지 않았다. 위 실물 결과는 앞선 진단에서 얻은 증거다.
- 사용자가 연결 문제로 보고 조사를 종료하도록 요청했다. 아래 원인 분리 항목은 재발 시 참고사항이며, 연결/전원 원인 확정이나 watchdog 해결 완료 판정은 아니다.

## 남은 원인 분리

- 현재 재현되지 않은 watchdog을 임의 delay 추가, watchdog 해제/기간 연장, 자동 재부팅 활성화로 덮지 않았다.
- 재발하면 새 단계 로그와 UART reset/panic 내용을 먼저 확보한다. 필요 시 같은 계측 패치를 적용해 Wi-Fi API 안쪽 구간을 더 좁힌다.
- CAM 단독 USB 직결과 현재 허브 조건의 비교, 부팅/RF 시작 순간의 보드 5V/3.3V 실측, 실제 cold power-cycle은 별도 물리 확인으로 남는다. 재현이 없는 두 조건의 성공률만으로 전원 원인을 단정하지 않는다.
- 이전 Wi-Fi TX panic, 지속 무응답, 이번 부팅 watchdog, 이번 단발 이미지 실패가 모두 같은 원인인지 미확정이다.

[원본 증거·진단 패치·집계](camera-boot-watchdog-2026-10-15/)에 보관한다. 펌웨어 바이너리, NVS 자격 증명, 이미지 원본은 저장소에 넣지 않는다. 반복 부팅 스크립트의 장치 포트는 당시 CAM에 고정돼 있으므로 재실행 시 실제 장치 MAC/포트를 확인해야 한다.
