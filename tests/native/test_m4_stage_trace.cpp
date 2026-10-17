#include <cassert>
#include <cstring>
#include <string>
#include <iostream>
#include "input/m4_stage_trace.h"

static const char* BOOT = "0123456789abcdef0123456789abcdef";
static void complete(M4StageTrace& t, const char* nonce = "0123abcd") {
  t.loopStarted(100000); t.loopStarted(101000);
  assert(t.begin(BOOT, nonce, 101, 101100, true));
  t.loopStarted(104000);
  t.pollStarted(104, 104010, 104100, 104110, true);
  t.angleStarted(104200); t.angleCompleted(104400, true); t.pollCompleted(104600, true);
  t.replyEnqueued(105, 104, 2778, 58, 105010); t.writeReturned(58, 105020);
}
int main() {
  M4StageTrace t;
  char output[12288];
  assert(t.formatJson(output, sizeof(output), BOOT));
  std::cout << output << '\n';
  assert(std::strstr(output, "\"nextSequence\":1") && std::strstr(output, "\"records\":[]"));
  assert(!t.begin(BOOT, "invalid", 1, 100, true));
  assert(!t.active() && t.nextSequence() == 1);
  complete(t);
  assert(t.formatJson(output, sizeof(output), BOOT));
  std::cout << output << '\n';
  assert(!t.active() && t.count() == 1 && t.dropped() == 0 && t.nextSequence() == 2);
  const auto* record = t.at(0);
  assert(record && std::strcmp(record->status, "replied") == 0 && record->protocol == 3);
  assert(record->maxPendingLoopGapUs == 3000 && record->requestLoopGapUs == 1000);
  assert(record->pollAttempts == 1 && record->pollSucceeded && record->angleSucceeded);
  assert(record->replyRaw == 2778 && record->replyPollMarkerMs == 104 && record->replyBytes == 58);
  assert(!t.at(1));
  const std::string original(output);
  for (unsigned i = 0; i < 3; ++i) {
    assert(t.formatJson(output, sizeof(output), BOOT));
    assert(original == output); // GET snapshot cannot mutate/reset counters.
  }
  char small[16]; memset(small, '!', sizeof(small));
  assert(!t.formatJson(small, 15, BOOT) && small[15] == '!');
  assert(!t.formatJson(output, 0, BOOT));
  assert(!t.formatJson(nullptr, sizeof(output), BOOT));
  assert(!t.formatJson(output, sizeof(output), "bad"));
  assert(!t.formatJson(output, sizeof(output), "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"));

  // An existing pending request is never replaced, and unsuccessful or timed
  // out requests remain in the same bounded history.
  assert(t.begin(BOOT, "00000002", 110, 110000, false));
  assert(!t.begin(BOOT, "00000003", 111, 111000, true));
  t.pollStarted(120, 120000, 120010, 120020, false); t.pollCompleted(120050, false);
  assert(t.at(1)->pollAttempts == 1 && !t.at(1)->angleObserved && !t.at(1)->adcSampled);
  t.pollStarted(140, 140000, 140010, 140020, true);
  t.angleStarted(140030); t.angleCompleted(140040, false); t.pollCompleted(140050, false);
  assert(t.at(1)->pollAttempts == 2 && !t.at(1)->pollSucceeded);
  t.cancel(true);
  assert(!t.active() && !std::strcmp(t.at(1)->status, "expired"));
  assert(t.formatJson(output, sizeof(output), BOOT));
  std::cout << output << '\n';
  assert(t.begin(BOOT, "00000003", 1, 1, false)); t.cancel();
  assert(!std::strcmp(t.at(2)->status, "cancelled"));
  assert(t.begin(BOOT, "00000004", 1, 1, false)); t.formatFailed();
  assert(!std::strcmp(t.at(3)->status, "format_failed"));
  assert(t.begin(BOOT, "00000005", 1, 1, false));
  t.replyEnqueued(2, 1, 1, 82, 1000); t.writeReturned(40, 2000);
  assert(!std::strcmp(t.at(4)->status, "write_failed"));
  const auto countBefore = t.count();
  t.pollStarted(1, 1, 2, 3, true); t.angleStarted(4); t.angleCompleted(5, true);
  t.pollCompleted(6, true); t.replyEnqueued(2, 1, 1, 82, 7); t.writeReturned(82, 8);
  t.cancel(); t.formatFailed();
  assert(t.count() == countBefore && t.at(4)->writtenBytes == 40);

  for (unsigned i = 5; i < 20; ++i) {
    char nonce[9]; snprintf(nonce, sizeof(nonce), "%08x", i);
    complete(t, nonce);
  }
  assert(t.count() == 8 && t.dropped() == 12 && t.nextSequence() == 21);
  for (size_t i = 0; i < 8; ++i) assert(t.at(i)->sequence == 13 + i);
  assert(t.formatJson(output, sizeof(output), BOOT));
  std::cout << output << '\n';
  assert(strlen(output) < 12288);
  assert(!t.begin("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "00000000", 1, 1, true));

  // Microsecond timestamps are uint64 and do not inherit millis() wrap.
  M4StageTrace wrap;
  const uint64_t epoch = UINT64_C(4294967296000);
  wrap.loopStarted(epoch); assert(wrap.begin(BOOT, "ffffffff", 0xfffffffe, epoch + 1, true));
  wrap.pollStarted(2, epoch + 2000, epoch + 2010, epoch + 2020, true);
  wrap.angleStarted(epoch + 2030); wrap.angleCompleted(epoch + 2040, true);
  wrap.pollCompleted(epoch + 2050, true); wrap.replyEnqueued(4, 2, 4095, 58, epoch + 4000);
  wrap.writeReturned(58, epoch + 5000);
  assert(wrap.formatJson(output, sizeof(output), BOOT));
  std::cout << output << '\n';
  assert(std::strstr(output, "4294967296001"));
}
