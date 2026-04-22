#ifndef INPUT_TELEOP_ADAPTER_H
#define INPUT_TELEOP_ADAPTER_H

#include <Arduino.h>
#include <stdint.h>

class SystemStateManager;
class MotorControl;
class MotionSequence;
class SafetyMonitor;
class AngleController;
class CommandBus;
class Dispatcher;

enum class TeleopStopReason : uint8_t {
  NONE = 0,
  DEADMAN_RELEASE,
  FRAME_TIMEOUT,
  NOT_ARMED,
  SAFETY_BLOCK
};

struct TeleopFrame {
  uint32_t sourceTimestampMs;
  uint32_t sequence;
  uint32_t session;
  bool     deadman;
  float    reach;
  float    lift;
  float    twist;
  bool     gripOpen;
  bool     gripClose;
  uint32_t ledToggleSeq;

  TeleopFrame()
    : sourceTimestampMs(0)
    , sequence(0)
    , session(0)
    , deadman(false)
    , reach(0.0f)
    , lift(0.0f)
    , twist(0.0f)
    , gripOpen(false)
    , gripClose(false)
    , ledToggleSeq(0) {
  }
};

class TeleopAdapter {
public:
  static const uint32_t BAUD_RATE = 115200;
  static const uint32_t LINK_TIMEOUT_MS = 200;
  static const uint32_t RECOMMENDED_FRAME_PERIOD_MS = 40;  // ~25Hz
  static const int RX_PIN = 34;
  static const int TX_PIN = -1;

  TeleopAdapter();

  bool init(SystemStateManager* systemState,
            MotorControl* motorControl,
            MotionSequence* motionSequence,
            SafetyMonitor* safetyMonitor,
            AngleController* angleController,
            CommandBus* commandBus,
            Dispatcher* dispatcher);
  void update();

  bool isReady() const;
  bool isConnected() const;
  bool isDeadmanHeld() const;
  bool isControlActive() const;
  uint32_t getLastFrameAgeMs() const;
  uint32_t getPacketsReceived() const;
  uint32_t getParseErrors() const;
  uint32_t getLastSequence() const;
  uint32_t getLastSession() const;
  uint32_t getLastLedToggleSeq() const;
  float getLastReach() const;
  float getLastLift() const;
  float getLastTwist() const;
  const char* getLastStopReasonString() const;

  static const char* stopReasonToString(TeleopStopReason reason);

private:
  static const size_t LINE_BUFFER_SIZE = 256;
  static const uint8_t OUTPUT_QUANT_STEP_PERCENT = 5;
  static const uint8_t GRIPPER_BUTTON_PERCENT = 70;

  HardwareSerial*     serial_;
  SystemStateManager* systemState_;
  MotorControl*       motorControl_;
  MotionSequence*     motionSequence_;
  SafetyMonitor*      safetyMonitor_;
  AngleController*    angleController_;
  CommandBus*         commandBus_;
  Dispatcher*         dispatcher_;

  TeleopFrame lastFrame_;
  char        lineBuffer_[LINE_BUFFER_SIZE];
  size_t      lineIndex_;
  bool        overflowDropping_;
  bool        controlActive_;
  uint32_t    lastFrameReceivedMs_;
  uint32_t    packetsReceived_;
  uint32_t    parseErrors_;
  uint32_t    lastHandledLedToggleSeq_;
  TeleopStopReason lastStopReason_;

  int8_t appliedGripPercent_;
  int8_t appliedWristPercent_;
  int8_t appliedElbowPercent_;
  int8_t appliedShoulderPercent_;
  int8_t appliedBasePercent_;

  void processIncomingByte(char c);
  bool parseTeleopLine(const char* line, TeleopFrame& outFrame) const;
  void handleFrame(const TeleopFrame& frame, uint32_t now);
  void handleFreshFrame(uint32_t now);
  void stopControlledOutputs(TeleopStopReason reason, const char* detail = nullptr, bool updateReason = true);
  void updateLedToggleIfNeeded();
  void applyContinuousOutputs();
  void applyJointOutputs(int8_t gripPercent,
                         int8_t wristPercent,
                         int8_t elbowPercent,
                         int8_t shoulderPercent,
                         int8_t basePercent);
  void applyJointSemanticPercent(uint8_t motorId, int8_t requestedPercent, int8_t& appliedPercent, bool positiveMeansForward);
  bool submitLightToggle();

  static float clampUnit(float value);
  static float absf(float value);
  static int8_t quantizeNormalized(float value);
  static int8_t quantizePercentMagnitude(uint8_t percent);
};

#endif // INPUT_TELEOP_ADAPTER_H
