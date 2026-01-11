#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <stdint.h>
#include "system/system_init.h"

// 전방 선언 (순환 참조 방지)
class SystemStateManager;

/**
 * MotionBrain Motor Control Module
 * 
 * Phase 1-5: TB6612FNG 연동
 * - 3개 TB6612FNG로 5개 모터 제어 (로봇팔용)
 * - 실제 PWM 출력 및 STBY 핀 제어
 * - 안전 규칙 강제
 * 
 * 모터 번호 체계 (Gripper Robot Arm):
 * - M1: 그리퍼 (Gripper) - 물체 잡기/놓기
 * - M2: 손목 관절 (Wrist Tilt) - 손목 상하 움직임
 * - M3: 팔꿈치 관절 (Elbow Joint) - 팔꿈치 상하 움직임
 * - M4: 어깨 관절 (Shoulder Joint) - 어깨 상하 움직임
 * - M5: 베이스 회전 (Base Rotation) - 로봇팔 전체 좌우 회전
 * 
 * TB6612FNG 드라이버 매핑:
 * - TB6612FNG #1: M1 (모터 A), M2 (모터 B)
 * - TB6612FNG #2: M3 (모터 A), M4 (모터 B)
 * - TB6612FNG #3: M5 (모터 A), 미사용 (모터 B)
 * 
 * 안전 규칙:
 * - SystemState가 ARMED일 때만 제어 가능
 * - BOOT/IDLE/FAULT 상태에서는 모든 명령 거부
 * - 부팅 시 STBY = LOW (차단)
 * - ARMED 상태에서만 STBY = HIGH
 */
class MotorControl {
public:
  // 모터 및 드라이버 개수
  static const uint8_t NUM_MOTORS = 5;  // 로봇팔: 5개 모터
  static const uint8_t NUM_DRIVERS = 3;
  
  // 모터 번호 상수 (1-based: M1 ~ M5)
  static const uint8_t MOTOR_1 = 1;  // 그리퍼 (Gripper) - TB6612FNG #1 - A
  static const uint8_t MOTOR_2 = 2;  // 손목 관절 (Wrist Tilt) - TB6612FNG #1 - B
  static const uint8_t MOTOR_3 = 3;  // 팔꿈치 관절 (Elbow Joint) - TB6612FNG #2 - A
  static const uint8_t MOTOR_4 = 4;  // 어깨 관절 (Shoulder Joint) - TB6612FNG #2 - B
  static const uint8_t MOTOR_5 = 5;  // 베이스 회전 (Base Rotation) - TB6612FNG #3 - A
  
  /**
   * 생성자
   */
  MotorControl();
  
  /**
   * 모터 제어 모듈 초기화
   * @return 초기화 성공 여부
   */
  bool init();
  
  /**
   * 기본 속도 설정
   * forward()/reverse() 호출 시 이 속도를 사용
   * @param speed 기본 속도 (0 ~ 255)
   * @return 성공 여부
   */
  bool setDefaultSpeed(uint8_t speed);
  
  /**
   * 기본 속도 가져오기
   * @return 기본 속도 (0 ~ 255)
   */
  uint8_t getDefaultSpeed() const;
  
  /**
   * 개별 모터 속도 설정
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @param speed 속도 (-255 ~ 255, 음수: 역방향)
   * @return 성공 여부 (상태 체크 실패 시 false)
   */
  bool setSpeed(uint8_t motorId, int16_t speed);
  
  /**
   * 모든 모터 속도 설정 (동일한 속도)
   * @param speed 속도 (-255 ~ 255, 음수: 역방향)
   * @return 성공 여부 (상태 체크 실패 시 false)
   */
  bool setSpeedAll(int16_t speed);
  
  /**
   * 모터 정방향 구동 (기본 속도 사용)
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @return 성공 여부
   */
  bool forward(uint8_t motorId);
  
  /**
   * 모터 정방향 구동 (속도 비율 지정)
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @param percent 속도 비율 (0 ~ 100, 기본 속도의 퍼센트)
   * @return 성공 여부
   */
  bool forward(uint8_t motorId, uint8_t percent);
  
  /**
   * 모터 역방향 구동 (기본 속도 사용)
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @return 성공 여부
   */
  bool reverse(uint8_t motorId);
  
  /**
   * 모터 역방향 구동 (속도 비율 지정)
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @param percent 속도 비율 (0 ~ 100, 기본 속도의 퍼센트)
   * @return 성공 여부
   */
  bool reverse(uint8_t motorId, uint8_t percent);
  
  /**
   * 개별 모터 정지
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @return 성공 여부 (상태 체크 실패 시 false)
   */
  bool stop(uint8_t motorId);
  
  /**
   * 모든 모터 정지
   * @return 성공 여부 (상태 체크 실패 시 false)
   */
  bool stopAll();
  
  /**
   * 비상 정지 (상태 무시하고 즉시 정지)
   * 모든 모터 정지 및 STBY = LOW
   */
  void emergencyStop();
  
  /**
   * 모터 활성화 여부 확인
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @return 활성화 여부
   */
  bool isEnabled(uint8_t motorId) const;
  
  /**
   * 모든 모터 활성화 여부 확인 (하나라도 활성화되어 있으면 true)
   * @return 활성화 여부
   */
  bool isEnabled() const;
  
  /**
   * 현재 모터 속도 가져오기
   * @param motorId 모터 번호 (1 ~ 5, M1 ~ M5)
   * @return 현재 속도 (-255 ~ 255)
   */
  int16_t getSpeed(uint8_t motorId) const;
  
  /**
   * 모터 제어 업데이트 (주기적으로 호출)
   * 점진적 속도 변경 처리
   * loop()에서 주기적으로 호출해야 함
   */
  void update();

private:
  // TB6612FNG 핀 정의
  // TB6612FNG #1 (모터 M1, M2)
  static const uint8_t PIN_STBY_1 = 4;      // Standby 핀
  static const uint8_t PIN_AIN1_1 = 16;     // 모터 A 방향 1
  static const uint8_t PIN_AIN2_1 = 17;     // 모터 A 방향 2
  static const uint8_t PIN_PWMA_1 = 18;     // 모터 A PWM
  static const uint8_t PIN_BIN1_1 = 19;     // 모터 B 방향 1
  static const uint8_t PIN_BIN2_1 = 21;     // 모터 B 방향 2
  static const uint8_t PIN_PWMB_1 = 22;     // 모터 B PWM
  
  // TB6612FNG #2 (모터 M3, M4)
  static const uint8_t PIN_STBY_2 = 5;      // Standby 핀
  static const uint8_t PIN_AIN1_2 = 23;     // 모터 A 방향 1
  static const uint8_t PIN_AIN2_2 = 25;     // 모터 A 방향 2
  static const uint8_t PIN_PWMA_2 = 26;     // 모터 A PWM
  static const uint8_t PIN_BIN1_2 = 27;     // 모터 B 방향 1
  static const uint8_t PIN_BIN2_2 = 32;     // 모터 B 방향 2
  static const uint8_t PIN_PWMB_2 = 33;     // 모터 B PWM
  
  // TB6612FNG #3 (모터 M5, M6은 미사용)
  static const uint8_t PIN_STBY_3 = 2;      // Standby 핀
  static const uint8_t PIN_AIN1_3 = 12;     // 모터 A 방향 1 (M5)
  static const uint8_t PIN_AIN2_3 = 13;     // 모터 A 방향 2 (M5)
  static const uint8_t PIN_PWMA_3 = 14;     // 모터 A PWM (M5)
  static const uint8_t PIN_BIN1_3 = 15;     // 모터 B 방향 1 (미사용)
  static const uint8_t PIN_BIN2_3 = 0;      // 모터 B 방향 2 (미사용)
  static const uint8_t PIN_PWMB_3 = 35;     // 모터 B PWM (미사용)
  
  // PWM 채널 정의
  static const uint8_t PWM_CHANNEL_M1 = 0;  // 모터 M1 (그리퍼) PWM 채널
  static const uint8_t PWM_CHANNEL_M2 = 1;  // 모터 M2 (손목) PWM 채널
  static const uint8_t PWM_CHANNEL_M3 = 2;  // 모터 M3 (팔꿈치) PWM 채널
  static const uint8_t PWM_CHANNEL_M4 = 3;  // 모터 M4 (어깨) PWM 채널
  static const uint8_t PWM_CHANNEL_M5 = 4;  // 모터 M5 (베이스) PWM 채널
  
  // PWM 설정
  static const uint32_t PWM_FREQUENCY = 1000;  // 1kHz
  static const uint8_t PWM_RESOLUTION = 8;      // 8-bit (0-255)
  
  // 점진적 속도 변경 설정
  static const uint8_t RAMP_STEP_SIZE = 10;      // 속도 변경 단계 크기 (한 번에 변경할 속도)
  static const uint32_t RAMP_INTERVAL_MS = 50;   // 속도 변경 간격 (밀리초)
  
  // 모터 상태 배열
  bool enabled_[NUM_MOTORS];           // 각 모터 활성화 여부
  int16_t currentSpeed_[NUM_MOTORS];   // 각 모터 현재 속도 (-255 ~ 255)
  int16_t targetSpeed_[NUM_MOTORS];    // 각 모터 목표 속도 (-255 ~ 255)
  uint32_t lastUpdateTime_[NUM_MOTORS]; // 각 모터 마지막 업데이트 시간 (밀리초)
  
  // 기본 속도 설정
  uint8_t defaultSpeed_;               // 기본 속도 (0 ~ 255)
  
  // 드라이버별 오류 상태 추적
  bool driverError_[NUM_DRIVERS];      // 각 드라이버 오류 상태
  
  // STBY 핀 배열 (각 드라이버마다)
  static const uint8_t STBY_PINS[NUM_DRIVERS];
  
  /**
   * 안전 검사 (private)
   * @param action 동작 이름 (로그용)
   * @return 안전 여부 (ARMED 상태면 true)
   */
  bool checkSafety(const char* action);
  
  /**
   * 현재 시스템 상태 가져오기
   * 전역 객체 systemState 참조
   */
  SystemState getCurrentState() const;
  
  /**
   * 모터 ID 유효성 검사 (1-based: 1 ~ 5)
   * @param motorId 모터 번호 (1 ~ 5)
   * @return 유효 여부
   */
  bool isValidMotorId(uint8_t motorId) const;
  
  /**
   * 모터 번호를 배열 인덱스로 변환 (1-based → 0-based)
   * @param motorId 모터 번호 (1 ~ 5)
   * @return 배열 인덱스 (0 ~ 4)
   */
  uint8_t motorIdToIndex(uint8_t motorId) const;
  
  /**
   * 배열 인덱스를 모터 번호로 변환 (0-based → 1-based)
   * @param index 배열 인덱스 (0 ~ 4)
   * @return 모터 번호 (1 ~ 5)
   */
  uint8_t indexToMotorId(uint8_t index) const;
  
  /**
   * 개별 모터 제어 (내부 헬퍼)
   * @param motorId 모터 번호 (1 ~ 5)
   * @param speed 속도 (-255 ~ 255)
   * @return 성공 여부
   */
  bool setMotorSpeedInternal(uint8_t motorId, int16_t speed);
  
  /**
   * STBY 핀 제어
   * @param driverId 드라이버 번호 (0 ~ 2)
   * @param enable 활성화 여부 (true = HIGH, false = LOW)
   */
  void setSTBY(uint8_t driverId, bool enable);
  
  /**
   * 모든 STBY 핀 제어
   * @param enable 활성화 여부 (true = HIGH, false = LOW)
   */
  void setSTBYAll(bool enable);
  
  /**
   * 모터 번호로 드라이버 번호 가져오기
   * @param motorId 모터 번호 (1 ~ 5)
   * @return 드라이버 번호 (0 ~ 2)
   */
  uint8_t getDriverId(uint8_t motorId) const;
  
  /**
   * 모터 번호로 드라이버 내 모터 인덱스 가져오기 (A=0, B=1)
   * @param motorId 모터 번호 (1 ~ 5)
   * @return 드라이버 내 모터 인덱스 (0=A, 1=B)
   */
  uint8_t getMotorIndexInDriver(uint8_t motorId) const;
  
  /**
   * 속도 비율을 실제 속도로 변환
   * @param percent 속도 비율 (0 ~ 100)
   * @return 실제 속도 (0 ~ 255)
   */
  uint8_t percentToSpeed(uint8_t percent) const;
};

#endif // MOTOR_DRIVER_H

