// Production driver with a separate raw command trace; fake Arduino writes only.
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <vector>

SystemStateManager systemState;
SafetyMonitor safetyMonitor;
static uint32_t nowMs = 100;
struct RawWrite { uint32_t at; uint8_t motor; uint32_t duty; };
static std::vector<RawWrite> writes;
uint32_t millis() { return nowMs; }
void pinMode(uint8_t pin, uint8_t mode) { assert(pin < 40 && mode == OUTPUT); }
void digitalWrite(uint8_t pin, uint8_t value) { assert(pin < 40 && value <= 1); }
void ledcSetup(uint8_t channel, uint32_t frequency, uint8_t resolution) {
  assert(channel < 5 && frequency == 1000 && resolution == 8);
}
void ledcAttachPin(uint8_t pin, uint8_t channel) { assert(pin < 40 && channel < 5); }
void ledcWrite(uint8_t channel, uint32_t duty) {
  assert(channel < 5 && duty <= 255);
  writes.push_back({++nowMs, static_cast<uint8_t>(channel + 1), duty});
}
static void legacy() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  assert(driver.setSpeed(2, 20)); driver.update();
  assert(driver.getSpeed(2) == 10 && driver.outputLog().count() == 6);
  driver.hardStopAll(); assert(!driver.hasPendingOutput());
}

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
using finite_profile::DriverOutputEvidence;
static uint32_t acquire(MotorControl& driver, uint32_t ordinal = 17) {
  uint32_t lease = 0; assert(driver.acquireFiniteProfileLease(ordinal, lease)); return lease;
}
static void initialize(MotorControl& driver) {
  assert(driver.init()); systemState.state = SystemState::ARMED;
}
static void bindValidation() {
  MotorControl driver; initialize(driver); const uint32_t lease = acquire(driver);
  const size_t n = writes.size();
  assert(!driver.bindFiniteProfileEvidence(0, "boot", "one", 1));
  assert(!driver.bindFiniteProfileEvidence(lease, nullptr, "one", 1));
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "bad command", 1));
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "one", 0));
  assert(driver.finiteProfileEvidenceCount() == 0 && writes.size() == n);
  assert(driver.bindFiniteProfileEvidence(lease, "boot", "one", 3));
  const auto* evidence = driver.findFiniteProfileEvidence("boot", "one");
  assert(evidence && evidence == driver.finiteProfileEvidence(lease));
  assert(evidence->requestEpoch == 3 && evidence->lease == lease && evidence->ordinal == 17);
  assert(evidence->opened && !evidence->closed && evidence->captureComplete && evidence->count == 0);
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "two", 3));
  assert(writes.size() == n && driver.finiteProfileEvidenceCount() == 1);
  assert(!driver.findFiniteProfileEvidence("other-boot", "one"));
}
static void repeatedWrites() {
  MotorControl driver; initialize(driver); const uint32_t lease = acquire(driver);
  assert(driver.bindFiniteProfileEvidence(lease, "boot", "repeated", 1));
  const size_t n = writes.size(); uint32_t at = 0;
  for (unsigned i = 0; i < 3; ++i) assert(driver.applyFiniteProfile(lease, 2, -80, at));
  const auto* evidence = driver.finiteProfileEvidence(lease);
  assert(evidence->count == 3 && evidence->totalWrites == 3 && evidence->dropped == 0);
  assert(driver.outputLog().count() == 6); // Ring intentionally deduplicates.
  for (unsigned i = 0; i < 3; ++i) {
    assert(evidence->entries[i].sequence == i + 1);
    assert(evidence->entries[i].atMs == writes[n + i].at);
    assert(evidence->entries[i].motor == 2 && evidence->entries[i].pwm == -80);
    assert(evidence->entries[i].reason == bounded::OutputReason::DriverWrite);
  }
  assert(at == evidence->entries[2].atMs);
}
static void quietAndClose() {
  MotorControl driver; initialize(driver); const uint32_t lease = acquire(driver);
  assert(driver.bindFiniteProfileEvidence(lease, "boot", "quiet", 1));
  uint32_t at = 0;
  assert(driver.applyFiniteProfile(lease, 2, 80, at));
  assert(driver.applyFiniteProfile(lease, 2, 0, at));
  driver.setActionOrdinal(999); driver.emergencyStop();
  assert(driver.stopFiniteProfile(lease, at));
  const auto* evidence = driver.finiteProfileEvidence(lease);
  assert(evidence->count == 12 && !evidence->closed && evidence->captureComplete);
  assert(evidence->entries[1].pwm == 0 && evidence->entries[1].reason == bounded::OutputReason::DriverWrite);
  for (unsigned i = 2; i < 12; ++i) {
    assert(evidence->entries[i].sequence == i + 1 && evidence->entries[i].pwm == 0);
    assert(evidence->entries[i].motor == (i - 2) % 5 + 1);
    assert(evidence->entries[i].reason == bounded::OutputReason::ForcedOff);
  }
  assert(at == evidence->entries[8].atMs); // Last scoped stop's M2, not M5.
  assert(driver.releaseFiniteProfileLease(lease));
  assert(evidence->closed && evidence->closedAt == nowMs && evidence->ordinal == 17);
  for (unsigned i = 0; i < 30; ++i) driver.hardStopAll();
  assert(driver.outputLog().dropped() > 0);
  assert(evidence->count == 12 && evidence->totalWrites == 12 && evidence->captureComplete);
}
static void overflow() {
  MotorControl driver; initialize(driver); const uint32_t lease = acquire(driver);
  assert(driver.bindFiniteProfileEvidence(lease, "boot", "full", 1));
  uint32_t at = 0;
  for (unsigned i = 0; i < DriverOutputEvidence::CAPACITY; ++i)
    assert(driver.applyFiniteProfile(lease, 2, 80, at));
  const auto* evidence = driver.finiteProfileEvidence(lease);
  assert(evidence->captureComplete && evidence->count == 64);
  const size_t n = writes.size();
  assert(!driver.applyFiniteProfile(lease, 2, 80, at));
  assert(driver.finiteProfileLeaseSealed(lease) && !driver.hasPendingOutput());
  for (size_t i = n; i < writes.size(); ++i) assert(writes[i].duty == 0);
  assert(!evidence->captureComplete && evidence->count == 64);
  assert(evidence->totalWrites == 69 && evidence->dropped == 5);
  assert(driver.stopFiniteProfile(lease, at));
  assert(evidence->totalWrites == 74 && evidence->dropped == 10);
  assert(evidence->entries[63].sequence == 64 && evidence->entries[63].pwm == 80);
}
static void capacityAndIdentity() {
  MotorControl driver; initialize(driver);
  const DriverOutputEvidence* first = nullptr;
  for (unsigned i = 0; i < 32; ++i) {
    const uint32_t lease = acquire(driver, i + 1); char command[20];
    snprintf(command, sizeof(command), "command-%u", i);
    assert(driver.bindFiniteProfileEvidence(lease, "boot", command, i + 1));
    if (i == 0) first = driver.finiteProfileEvidence(lease);
    uint32_t at = 0; assert(driver.stopFiniteProfile(lease, at));
    assert(driver.releaseFiniteProfileLease(lease));
  }
  assert(driver.finiteProfileEvidenceCount() == 32 && first->closed && first->count == 5);
  const uint32_t lease = acquire(driver, 33); const size_t n = writes.size();
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "too-many", 33));
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "command-0", 1));
  assert(writes.size() == n && driver.findFiniteProfileEvidence("boot", "command-0") == first);
}
static void duplicateIdentity() {
  MotorControl driver; initialize(driver); const uint32_t first = acquire(driver);
  assert(driver.bindFiniteProfileEvidence(first, "boot", "same", 1));
  assert(driver.releaseFiniteProfileLease(first));
  const uint32_t second = acquire(driver, 18); const size_t n = writes.size();
  assert(!driver.bindFiniteProfileEvidence(second, "boot", "same", 2));
  assert(driver.bindFiniteProfileEvidence(second, "other-boot", "same", 2));
  assert(writes.size() == n && driver.finiteProfileEvidenceCount() == 2);
  assert(driver.finiteProfileEvidence(first)->closed && !driver.finiteProfileEvidence(second)->closed);
}
static void unboundLease() {
  MotorControl driver; initialize(driver); const uint32_t lease = acquire(driver); uint32_t at = 0;
  for (unsigned i = 0; i < 80; ++i) assert(driver.applyFiniteProfile(lease, 2, 80, at));
  assert(driver.finiteProfileEvidenceCount() == 0 && !driver.finiteProfileEvidence(lease));
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "late", 1));
  assert(driver.applyFiniteProfile(lease, 2, 0, at));
  assert(!driver.bindFiniteProfileEvidence(lease, "boot", "late", 1));
}
static void saturation() {
  finite_profile::DriverEvidenceStore store;
  assert(store.bind("boot", "saturation", 1, 1, 1, 0));
  // Direct boundary injection avoids billions of writes; production access is const.
  auto* record = const_cast<DriverOutputEvidence*>(store.find(1));
  record->totalWrites = UINT32_MAX - 1; record->dropped = UINT32_MAX - 1;
  store.append(1, 1, 1, 2, 0, bounded::OutputReason::ForcedOff);
  store.append(1, 1, 2, 2, 0, bounded::OutputReason::ForcedOff);
  store.append(1, 1, 3, 2, 0, bounded::OutputReason::ForcedOff);
  assert(record->totalWrites == UINT32_MAX && record->dropped == UINT32_MAX);
  assert(!record->captureComplete && record->count == 1);
}
static void memory() {
  static_assert(sizeof(finite_profile::DriverOutputEntry) == 12, "Compact raw write layout changed");
  static_assert(sizeof(DriverOutputEvidence) == 964, "Fixed command evidence budget changed");
  printf("entry=%zu record=%zu store=%zu motor=%zu\n", sizeof(finite_profile::DriverOutputEntry),
         sizeof(DriverOutputEvidence), sizeof(finite_profile::DriverEvidenceStore), sizeof(MotorControl));
}
#endif

int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "legacy")) legacy();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
  else if (!strcmp(argv[1], "bind")) bindValidation();
  else if (!strcmp(argv[1], "repeated")) repeatedWrites();
  else if (!strcmp(argv[1], "quiet")) quietAndClose();
  else if (!strcmp(argv[1], "overflow")) overflow();
  else if (!strcmp(argv[1], "capacity")) capacityAndIdentity();
  else if (!strcmp(argv[1], "duplicate")) duplicateIdentity();
  else if (!strcmp(argv[1], "unbound")) unboundLease();
  else if (!strcmp(argv[1], "saturation")) saturation();
  else if (!strcmp(argv[1], "memory")) memory();
#endif
  else return 2;
}
