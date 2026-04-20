#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>  // ESP32 WebServer 라이브러리
#include <stdint.h>      // uint16_t를 위해 추가

// 전방 선언 (순환 참조 방지)
class SystemStateManager;
class MotorControl;
class RobotArm;
class MotionSequence;
class SearchLight;
class CommandBus;
class Dispatcher;
struct MotionEvent;
struct Command;
struct CommandResult;

/**
 * MotionBrain Web Server Module
 * 
 * Phase 1.5-2: 웹 UI (대시보드 + 명령)
 * - ESP32를 HTTP 웹 서버로 설정
 * - 웹 브라우저로 시스템 상태 확인
 * - 웹에서 명령 전송
 * 
 * 목표:
 * - 상태 가시화
 * - 명령 전달 실험
 * - UI 이벤트 → 모터 직접 제어 금지 (내부 명령 객체 생성까지만)
 */

/**
 * MotionBrainWebServer 클래스
 * 
 * ESP32 HTTP 웹 서버 관리
 * - HTTP 요청 처리
 * - HTML 대시보드 제공
 * - RESTful API 제공 (상태 조회, 명령 실행)
 * 
 * 주의: ESP32 라이브러리의 WebServer 클래스와 이름 충돌을 피하기 위해
 * MotionBrainWebServer로 명명했습니다.
 */
class MotionBrainWebServer {
public:
  /**
   * 생성자
   */
  MotionBrainWebServer();

  /**
   * 초기화
   * HTTP 웹 서버 시작
   * @param systemState SystemStateManager 참조 (상태 조회 및 명령 처리용)
   * @param motorControl MotorControl 참조 (명령 처리용)
   * @param port HTTP 서버 포트 (기본값: 80)
   * @return 초기화 성공 여부
   */
  bool init(SystemStateManager* systemState, MotorControl* motorControl,
            RobotArm* robotArm = nullptr,
            MotionSequence* motionSequence = nullptr,
            SearchLight* searchLight = nullptr,
            CommandBus* commandBus = nullptr,
            Dispatcher* dispatcher = nullptr,
            uint16_t port = 80);

  /**
   * 업데이트 (주기적으로 호출)
   * HTTP 요청 처리
   * loop()에서 주기적으로 호출해야 함
   */
  void update();

  /**
   * 웹 서버 활성화 여부 확인
   * @return 웹 서버 활성화 여부
   */
  bool isActive() const;

private:
  // ESP32 WebServer 객체 (포트 80으로 명시적 초기화)
  ::WebServer server_{80};  // ::WebServer는 전역 네임스페이스의 WebServer 클래스

  bool active_;         // 웹 서버 활성화 여부
  uint16_t port_;       // HTTP 서버 포트

  // 외부 객체 참조 (상태 조회 및 명령 처리용)
  SystemStateManager* systemState_;
  MotorControl* motorControl_;
  RobotArm* robotArm_;
  MotionSequence* motionSequence_;
  SearchLight* searchLight_;
  CommandBus* commandBus_;
  Dispatcher* dispatcher_;

  bool submitCommand(const Command& command, CommandResult& result);
  void appendStateSummaryJson(String& json) const;
  void sendErrorJson(int statusCode, const char* error, const String& details = "");
  void sendCommandResult(const CommandResult& result, const String& extraJson = "");

  // ===== HTTP 라우트 핸들러 (private) =====
  // Step 2에서 구현 예정

  /**
   * GET / 처리
   * HTML 대시보드 페이지 반환
   */
  void handleRoot();

  /**
   * GET /status 처리
   * JSON 형식으로 현재 상태 반환
   */
  void handleStatus();

  /**
   * GET /events 처리
   * 최근 이벤트 목록 JSON 반환
   */
  void handleEvents();

  /**
   * POST /command 처리
   * 명령 실행 (arm, disarm, stop 등)
   */
  void handleCommand();
  
  /**
   * POST /motor 처리
   * 모터 제어 (forward, reverse, stop, default)
   */
  void handleMotor();

  /**
   * POST /joint 처리
   * 관절 제어 (gripper/wrist/elbow/shoulder/base)
   */
  void handleJoint();

  /**
   * POST /base 처리
   * base 상대각 제어 및 정지
   */
  void handleBase();

  /**
   * POST /sequence 처리
   * 시퀀스 제어 (add, run, stop, clear)
   */
  void handleSequence();

  /**
   * GET /sequence 처리
   * 시퀀스 상태 JSON 반환
   */
  void handleSequenceStatus();

  /**
   * 공통 favicon 응답
   */
  void handleFavicon();

  /**
   * 공통 Apple touch icon 응답
   */
  void handleAppleTouchIcon();

  /**
   * POST /light 처리
   * 서치라이트 on/off/toggle
   */
  void handleLight();

  /**
   * 404 Not Found 처리
   * 존재하지 않는 경로 접근 시
   */
  void handleNotFound();
};

#endif // WEB_SERVER_H
