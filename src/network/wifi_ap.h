#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <Arduino.h>
#include <WiFi.h>

/**
 * MotionBrain Wi-Fi AP Module
 * 
 * Phase 1.5-1: Wi-Fi AP 입력 채널
 * - ESP32를 Wi-Fi Access Point로 설정
 * - 외부 인터넷 연결 없이 로컬 네트워크 구성
 * - 무선으로 명령 전달 경로 확보
 * 
 * 목표:
 * - 안전한 무선 입력 경로 확보
 * - 접속 로그 출력
 * - 부팅 시 자동 arm 금지
 */

/**
 * WiFiAP 클래스
 * 
 * ESP32 Wi-Fi AP 모드 관리
 * - AP 모드 설정
 * - 클라이언트 접속 관리
 * - 접속 상태 모니터링
 */
class WiFiAP {
public:
  /**
   * 생성자
   */
  WiFiAP();

  /**
   * 초기화
   * Wi-Fi AP 모드 시작
   * @param ssid AP 이름 (SSID)
   * @param password AP 비밀번호 (nullptr면 공개 AP)
   * @param ip AP IP 주소 (기본값: 192.168.4.1)
   * @return 초기화 성공 여부
   */
  bool init(const char* ssid, const char* password = nullptr, IPAddress ip = IPAddress(192, 168, 4, 1));

  /**
   * 업데이트 (주기적으로 호출)
   * 클라이언트 접속 상태 확인
   * loop()에서 주기적으로 호출해야 함
   */
  void update();

  /**
   * AP 활성화 여부 확인
   * @return AP 활성화 여부
   */
  bool isActive() const;

  /**
   * 연결된 클라이언트 수 확인
   * @return 연결된 클라이언트 수
   */
  uint8_t getClientCount() const;

  /**
   * AP IP 주소 가져오기
   * @return AP IP 주소
   */
  IPAddress getIP() const;

  /**
   * AP SSID 가져오기
   * @return AP SSID
   */
  const char* getSSID() const;

private:
  // Phase 4 MVP: Mac host + ESP32-CAM + phone viewer 동시 접속 허용.
  // 명령 권한은 HTTP CSRF 헤더와 host-side 정책으로 제한한다.
  static const uint8_t MAX_CLIENTS = 3;

  bool active_;                   // AP 활성화 여부
  IPAddress apIP_;                // AP IP 주소
  char ssid_[33];       // SSID 버퍼 (최대 32자 + null) — 댕글링 포인터 방지
  char password_[65];   // 비밀번호 버퍼 (최대 64자 + null) — 댕글링 포인터 방지
  uint32_t lastCheckTime_;        // 마지막 접속 체크 시간
  uint8_t lastClientCount_{255};  // 이전 클라이언트 수 (255: 초기값, 항상 다르게)

  /**
   * 클라이언트 접속 상태 체크 (private)
   */
  void checkClients();
};

#endif // WIFI_AP_H
