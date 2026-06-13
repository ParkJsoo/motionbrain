#ifndef CONTROL_HARDWARE_FEEDBACK_H
#define CONTROL_HARDWARE_FEEDBACK_H

#include <Arduino.h>
#include <stdint.h>

class AngleController;

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_ENABLED
#define MOTIONBRAIN_BASE_YAW_REFERENCE_ENABLED 0
#endif

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_PIN
#define MOTIONBRAIN_BASE_YAW_REFERENCE_PIN 36
#endif

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_ACTIVE_LOW
#define MOTIONBRAIN_BASE_YAW_REFERENCE_ACTIVE_LOW 1
#endif

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_DEBOUNCE_MS
#define MOTIONBRAIN_BASE_YAW_REFERENCE_DEBOUNCE_MS 25
#endif

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_STALE_MS
#define MOTIONBRAIN_BASE_YAW_REFERENCE_STALE_MS 500
#endif

#ifndef MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED
#define MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED 0
#endif

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
  bool hardwareReady;
  bool readyForRoutineExecution;
  bool physicalRoutineExecutionAllowed;
  bool signalActive;
  uint8_t pin;
  bool activeLow;
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
  static void initBaseYawReference();
  static void updateBaseYawReference();
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
