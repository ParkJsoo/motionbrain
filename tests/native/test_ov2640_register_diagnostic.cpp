#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <initializer_list>
#include "ov2640_register_diagnostic.h"

using namespace ov2640_diag;
struct FakeBus {
  uint64_t time = 100;
  uint64_t cost = 1;
  int selected = 0;
  int calls = 0;
  int failCall = -1;
  bool failAddress = true;
  bool mutateFailedWrite = true;
  int badBankRead = -1;
  int pid = 0x26;
  uint64_t nowUs() { return time; }
  ReadResult read(uint8_t reg) {
    time += cost;
    ReadResult r;
    const int call = calls++;
    if (call == failCall) {
      r.addressStatus = failAddress ? 263 : 0;
      r.status = failAddress ? NOT_ATTEMPTED : 263;
      return r;
    }
    r.addressStatus = r.status = 0;
    r.value = reg == 0xff ? (call == badBankRead ? 2 : selected) :
        (selected == 1 && reg == 0x0a ? pid : 0x00);
    return r;
  }
  int writeBank(uint8_t value) {
    assert(value <= 1);
    time += cost;
    const int call = calls++;
    if (call != failCall || mutateFailedWrite) selected = value;
    return call == failCall ? 263 : 0;
  }
};

static void emit(const Snapshot& s) {
  printf("{\"startedUs\":%llu,\"endedUs\":%llu,\"transactions\":[",
         (unsigned long long)s.startedUs, (unsigned long long)s.endedUs);
  for (size_t i = 0; i < s.count; ++i) {
    const Transaction& t = s.transactions[i];
    printf("%s{\"op\":\"%s\",\"bank\":%d,\"register\":%u,\"value\":%d,",
           i ? "," : "", t.write ? "write_bank" : "read", t.bank, t.reg, t.value);
    if (t.write) printf("\"addressStatus\":null,");
    else printf("\"addressStatus\":%d,", t.addressStatus);
    printf("\"status\":%d,\"startedUs\":%llu,\"endedUs\":%llu}", t.status,
           (unsigned long long)t.startedUs, (unsigned long long)t.endedUs);
  }
  printf("]}\n");
}

int main(int argc, char**) {
  FakeBus success;
  Snapshot good = observe(success);
  assert(good.ok && good.bankRestored && good.count == 41 && success.selected == 0);
  if (argc > 1) { emit(good); return 0; }
  for (int original = 0; original <= 1; ++original) {
    FakeBus bus; bus.selected = original;
    const Snapshot s = observe(bus);
    assert(s.ok && s.bankRestored && bus.selected == original);
    for (size_t i = 0; i < s.count; ++i) {
      const Transaction& t = s.transactions[i];
      assert(t.endedUs >= t.startedUs);
      if (t.write) assert(t.reg == 0xff && t.value <= 1);
    }
  }
  // Exhaust every actual transfer position, including restore write/read.
  for (int failure = 0; failure < 41; ++failure) {
    for (bool address : {false, true}) {
      FakeBus bus; bus.failCall = failure; bus.failAddress = address;
      const Snapshot s = observe(bus);
      assert(!s.ok && *s.failure && s.count <= 41);
      if (failure == 0) assert(s.count == 1 && !s.bankRestored);
      else if (failure < 39) assert(s.bankRestored && bus.selected == 0);
      else assert(!s.bankRestored && strcmp(s.failure, "bank_restore_failed") == 0);
      bool preserved = false;
      for (size_t i = 0; i < s.count; ++i) {
        const Transaction& t = s.transactions[i];
        if (t.addressStatus == 263 || t.status == 263) {
          preserved = true;
          if (!t.write) assert(t.value == -1);
        }
      }
      assert(preserved);
    }
  }
  for (int read : {0, 2, 32, 40}) {
    FakeBus bus; bus.badBankRead = read;
    const Snapshot s = observe(bus);
    assert(!s.ok);
    if (read == 0) assert(s.count == 1);
    else if (read == 40) assert(!s.bankRestored);
    else assert(s.bankRestored);
  }
  FakeBus wrong; wrong.pid = 0xff;
  assert(strcmp(observe(wrong).failure, "sensor_pid_mismatch") == 0);
  FakeBus slow; slow.cost = 100000;
  Snapshot late = observe(slow);
  assert(!late.ok && late.bankRestored && slow.selected == 0);
  assert(strcmp(late.failure, "snapshot_deadline") == 0);

  TrialGate gate; gate.readyUs = 100;
  assert(*gate.begin("bad", 101) && gate.attempts == 0);
  assert(!*gate.begin("0123456789abcdef", 101));
  assert(!*gate.begin("1123456789abcdef", 102));
  assert(!*gate.begin("2123456789abcdef", 103));
  assert(*gate.begin("3123456789abcdef", 104) && gate.attempts == 3);
  TrialGate duplicate;
  assert(!*duplicate.begin("0123456789abcdef", 1));
  assert(*duplicate.begin("0123456789abcdef", 2) && duplicate.fault);
  assert(*duplicate.begin("1123456789abcdef", 3) && duplicate.attempts == 1);
  TrialGate expired;
  assert(*expired.begin("0123456789abcdef", READY_US));
  TrialGate window;
  assert(!*window.begin("0123456789abcdef", 1));
  assert(*window.begin("1123456789abcdef", SESSION_US+1));
  TrialGate regression; regression.readyUs = 100;
  assert(*regression.begin("0123456789abcdef", 99));
  TrialGate reserve;
  assert(!*reserve.begin("0123456789abcdef", 1));
  assert(*reserve.begin("1123456789abcdef", SESSION_US-FINISH_RESERVE_US+2));
  const char* request = "GET /diagnostics/ov2640?nonce=0123456789abcdef HTTP/1.1\r\nHost: camera\r\n\r\n";
  RequestParser parser(0);
  for (size_t i = 0; i < strlen(request); ++i) assert(parser.feed(request[i], i));
  char nonce[17]; assert(parser.nonce(nonce) && !strcmp(nonce, "0123456789abcdef"));
  RequestParser drip(0);
  for (size_t i = 0; i < strlen(request); ++i) drip.feed(request[i], i*10000);
  assert(drip.failed && !drip.nonce(nonce));
  RequestParser oversize(0);
  for (int i = 0; i < 1537; ++i) oversize.feed('a', i);
  assert(oversize.failed && !oversize.nonce(nonce));
  RequestParser wrongRoute(0);
  const char* capture = "GET /capture HTTP/1.1\r\n\r\n";
  for (size_t i = 0; i < strlen(capture); ++i) wrongRoute.feed(capture[i], i);
  assert(!wrongRoute.nonce(nonce));
  puts("all register failures and finite gates passed");
}
