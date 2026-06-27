#ifndef MOTION_SEQUENCE_H
#define MOTION_SEQUENCE_H

#include <Arduino.h>
#include <stdint.h>

// 전방 선언
class RobotArm;
class SystemStateManager;
class AngleController;
class ShoulderAngleController;

/**
 * 모션 관절 종류
 */
enum class MotionJoint : uint8_t {
  GRIPPER  = 1,
  WRIST    = 2,
  ELBOW    = 3,
  SHOULDER = 4,
  BASE     = 5
};

/**
 * 모션 방향
 * OPEN/CLOSE: gripper 전용
 * UP/DOWN: wrist, elbow, shoulder 전용
 * LEFT/RIGHT: base 전용
 */
enum class MotionDirection : uint8_t {
  OPEN  = 0,
  CLOSE = 1,
  UP    = 2,
  DOWN  = 3,
  LEFT  = 4,
  RIGHT = 5
};

/**
 * 단일 모션 명령
 *
 * 기본은 duration 기반 step이며, base 전용 폐루프 step은
 * `joint=BASE` + `targetDegrees>0` 조합으로 표현한다.
 */
struct MotionCommand {
  MotionJoint     joint;
  MotionDirection direction;
  uint8_t         speed;      // 1~100 (%)
  uint32_t        durationMs; // 동작 지속 시간 (ms), 최소 1
  float           targetDegrees;
};

/**
 * 시퀀스 상태
 */
enum class SequenceState : uint8_t {
  IDLE      = 0,  // 대기 (명령 추가 가능)
  RUNNING   = 1,  // 실행 중
  COMPLETED = 2,  // 완료 (모든 명령 실행됨)
  STOPPED   = 3   // 중단됨 (stop() 호출)
};

/**
 * MotionSequence — Phase 2-B
 *
 * 시간 기반 비차단(non-blocking) 모션 시퀀스 큐.
 * 인코더 없이 durationMs로 각 스텝을 제어.
 *
 * 사용 흐름:
 *   1. addCommand(...) × N
 *   2. run()
 *   3. update() — loop()에서 주기 호출
 *   4. 완료 시 getState() == COMPLETED
 */
class MotionSequence {
public:
  /** 최대 명령 수 (고정 배열) */
  static const uint8_t MAX_COMMANDS = 16;

  MotionSequence();

  /**
   * 초기화
   * @param robotArm       RobotArm 참조 (관절 제어용)
   * @param systemState    SystemStateManager 참조 (ARMED 체크 및 timeout 리셋)
   * @param angleController base 상대각 폐루프 제어기 참조
   */
  void init(RobotArm* robotArm, SystemStateManager* systemState,
            AngleController* angleController = nullptr,
            ShoulderAngleController* shoulderAngleController = nullptr);

  /**
   * 업데이트 — loop()에서 주기 호출 필수
   * RUNNING 상태일 때 스텝 진행 처리
   */
  void update();

  /**
   * 명령 추가
   * @param joint      대상 관절
   * @param direction  방향
   * @param speed      속도 (1~100%)
   * @param durationMs 지속 시간 (ms, 최소 1)
   * @return 추가 성공 여부 (full이거나 durationMs==0이면 false)
   */
  bool addCommand(MotionJoint joint, MotionDirection direction, uint8_t speed, uint32_t durationMs);

  /**
   * base 상대각 step 추가
   * @param direction     left/right
   * @param speed         속도 (1~100%)
   * @param targetDegrees 목표 상대각 (3~180 deg)
   */
  bool addBaseAngleCommand(MotionDirection direction, uint8_t speed, float targetDegrees);

  /**
   * 시퀀스 실행 시작
   * 조건: IDLE/COMPLETED/STOPPED 상태 + count > 0 + 시스템 ARMED
   * @return 실행 시작 성공 여부
   */
  bool run();

  /**
   * 시퀀스 중단 (현재 관절 즉시 정지)
   */
  void stop();

  /**
   * 큐 초기화 (RUNNING이면 stop() 후 초기화)
   */
  void clear();

  // ===== 상태 조회 =====

  SequenceState getState()        const;
  uint8_t       getCurrentIndex() const;
  uint8_t       getTotalCount()   const;

  /**
   * 현재 스텝의 남은 시간 (ms)
   * RUNNING이 아니면 0 반환
   */
  uint32_t getRemainingMs() const;

  bool isFull() const;

  // ===== 파싱 유틸 (시리얼/웹 핸들러용) =====

  static const char* stateToString(SequenceState state);

  /**
   * 문자열 → MotionJoint 변환
   * "gripper","wrist","elbow","shoulder","base" 지원 (대소문자 무시)
   */
  static bool parseJoint(const char* str, MotionJoint& joint);

  /**
   * 문자열 → MotionDirection 변환 (관절 종류에 따라 유효한 방향이 다름)
   * gripper: open/close, wrist/elbow/shoulder: up/down, base: left/right
   * + 모든 관절: "stop" → false 반환 (stop은 별도 처리)
   */
  static bool parseDirection(MotionJoint joint, const char* str, MotionDirection& direction);

private:
  MotionCommand       queue_[MAX_COMMANDS];
  uint8_t             count_;
  uint8_t             currentIndex_;
  SequenceState       state_;
  uint32_t            stepStartMs_;
  RobotArm*           robotArm_;
  SystemStateManager* systemState_;
  AngleController*    angleController_;
  ShoulderAngleController* shoulderAngleController_;

  bool isBaseAngleCommand(const MotionCommand& cmd) const;
  bool executeCommand(const MotionCommand& cmd, char* errorMessage, size_t errorMessageSize);
  bool startCurrentCommand();
  void advanceToNextStep();
  void stopWithReason(const char* reason);

  /** 현재 관절 정지 */
  void stopCurrentJoint(const MotionCommand& cmd);
};

#endif // MOTION_SEQUENCE_H
