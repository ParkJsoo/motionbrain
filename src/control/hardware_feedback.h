#ifndef CONTROL_HARDWARE_FEEDBACK_H
#define CONTROL_HARDWARE_FEEDBACK_H

#include <Arduino.h>
#include <stdint.h>

class AngleController;

enum class HardwareFeedbackFault : uint8_t {
  READY = 0,
  NOT_INSTALLED,
  DISCONNECTED,
  STALE,
  UNREFERENCED,
  FAULTED,
  NO_PROGRESS,
  TIMEOUT,
  OVERSHOOT
};

struct BaseYawReferenceFeedback {
  bool installed;
  bool available;
  bool connected;
  bool fresh;
  bool referenced;
  bool faulted;
  bool readyForRoutineExecution;
  bool physicalRoutineExecutionAllowed;
  uint32_t ageMs;
  uint32_t lastUpdateMs;
  float positionDeg;
  float velocityDps;
  HardwareFeedbackFault fault;
  const char* blockReason;
  const char* lastStopReason;
  const char* detail;
};

class HardwareFeedback {
public:
  static const char* selectedClosureTarget();
  static BaseYawReferenceFeedback baseYawReferenceStatus(
    const AngleController* angleController = nullptr);
  static bool baseYawReferenceReadyForRoutineExecution(
    const AngleController* angleController = nullptr);
  static const char* faultToString(HardwareFeedbackFault fault);
  static const char* routineBlockReason();
  static const char* routineBlockEventCode();
  static void appendStatusJson(String& json, const AngleController* angleController = nullptr);
};

#endif // CONTROL_HARDWARE_FEEDBACK_H
