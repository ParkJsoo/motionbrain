#include "manual_control_lease.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

namespace manual_control {
namespace {
bool bootText(const char* text) {
  if (!text || strnlen(text, 33) != 32) return false;
  for (size_t i = 0; i < 32; ++i) {
    if (!((text[i] >= '0' && text[i] <= '9') ||
          (text[i] >= 'a' && text[i] <= 'f'))) return false;
  }
  return true;
}

bool decimal(const char* text, uint32_t& out) {
  if (!text || !*text || (*text == '0' && text[1])) return false;
  uint32_t value = 0;
  for (const char* p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const uint32_t digit = *p - '0';
    if (value > (UINT32_MAX - digit) / 10) return false;
    value = value * 10 + digit;
  }
  if (!value) return false;
  out = value;
  return true;
}

bool tokenText(const char* text) {
  if (!text || strnlen(text, ID_CAPACITY) != 43 || text[32] != '-' ||
      text[34] != '-' || text[33] < '1' || text[33] > '5') return false;
  for (size_t i = 0; i < 43; ++i) {
    if (i == 32 || i == 33 || i == 34) continue;
    if (!((text[i] >= '0' && text[i] <= '9') ||
          (text[i] >= 'a' && text[i] <= 'f'))) return false;
  }
  return true;
}
}

Request::Request()
    : start(false), motor(0), forward(false), percent(0), boot{0},
      generation(0), leaseId{0}, sequence(0) {}

bool parseRequest(const char* mode, const char* boot, const char* generation,
                  const char* id, const char* sequence, uint8_t motor,
                  bool physicalForward, uint8_t percent, Request& out) {
  if (!mode || !boot || !generation || !id || !sequence || motor < 1 ||
      motor > 5 || percent < 1 || percent > 100) return false;
  Request request;
  request.motor = motor;
  request.forward = physicalForward;
  request.percent = percent;
  if (strcmp(mode, "start") == 0) {
    if (!bootText(boot) || !decimal(generation, request.generation) ||
        *id || *sequence) return false;
    request.start = true;
    memcpy(request.boot, boot, sizeof(request.boot));
  } else if (strcmp(mode, "renew") == 0) {
    if (*boot || *generation || !tokenText(id) ||
        !decimal(sequence, request.sequence)) return false;
    memcpy(request.leaseId, id, sizeof(request.leaseId));
  } else {
    // An untyped legacy RUN cannot identify a new gesture versus an old renewal.
    return false;
  }
  out = request;
  return true;
}

Leases::Leases() : boot_{0}, states_{} {
  for (auto& state : states_) state.generation = 1;
}

bool Leases::init(const char* boot, Port& port) {
  if (!bootText(boot)) return false;
  // Reinitialization cannot reset generations or resurrect prior lease IDs.
  if (*boot_) return strcmp(boot_, boot) == 0;
  memcpy(boot_, boot, sizeof(boot_));
  for (uint8_t i = 0; i < 5; ++i)
    states_[i].observedEpoch = port.controlEpoch(i + 1);
  return true;
}

const State* Leases::state(uint8_t motor) const {
  return motor >= 1 && motor <= 5 ? &states_[motor - 1] : nullptr;
}

bool Leases::anyActive() const {
  for (const auto& state : states_) if (state.active) return true;
  return false;
}

void Leases::revoke(uint8_t index) {
  State& state = states_[index];
  state.active = false;
  state.leaseId[0] = 0;
  state.lastSequence = 0;
  if (state.generation == UINT32_MAX) state.exhausted = true;
  else ++state.generation;
}

void Leases::synchronize(Port& port) {
  for (uint8_t i = 0; i < 5; ++i) {
    const uint64_t epoch = port.controlEpoch(i + 1);
    if (epoch != states_[i].observedEpoch) {
      revoke(i);
      states_[i].observedEpoch = epoch;
    }
  }
}

void Leases::invalidate(uint8_t motor, Port& port) {
  if (motor < 1 || motor > 5) return;
  revoke(motor - 1);
  states_[motor - 1].observedEpoch = port.controlEpoch(motor);
}

void Leases::clear(Port& port) {
  for (uint8_t motor = 1; motor <= 5; ++motor) invalidate(motor, port);
}

uint32_t Leases::tick(Port& port) {
  synchronize(port);
  const uint32_t now = port.now();
  for (uint8_t i = 0; i < 5; ++i) {
    if (states_[i].active &&
        static_cast<uint32_t>(now - states_[i].acceptedAt) >= TTL_MS) {
      revoke(i);
      port.expired(i + 1);
      states_[i].observedEpoch = port.controlEpoch(i + 1);
    }
  }
  return now;
}

bool Leases::execute(const Request& request, Port& port, bool& dispatched,
                     const char*& reason, uint32_t* checkedAt) {
  dispatched = false;
  reason = "manual_lease_malformed";
  if (!*boot_ || request.motor < 1 || request.motor > 5 ||
      request.percent < 1 || request.percent > 100 ||
      (request.start && (!bootText(request.boot) || !request.generation)) ||
      (!request.start && (!tokenText(request.leaseId) || !request.sequence)))
    return false;

  // Check TTL in the handler transaction even when the outer loop was delayed.
  const uint32_t now = tick(port);
  if (checkedAt) *checkedAt = now;
  State& state = states_[request.motor - 1];
  if (request.start) {
    if (strcmp(request.boot, boot_) != 0) {
      reason = "manual_lease_boot_mismatch";
      return false;
    }
    if (state.exhausted || state.generation == UINT32_MAX) {
      reason = "manual_lease_generation_exhausted";
      return false;
    }
    if (request.generation != state.generation) {
      reason = "manual_lease_generation_mismatch";
      return false;
    }
    if (state.active) {
      reason = "manual_lease_active_stop_required";
      return false;
    }
    // Consume a start snapshot even if the existing dispatcher rejects it.
    revoke(request.motor - 1);
  } else {
    if (!state.active) {
      reason = "manual_lease_inactive_or_expired";
      return false;
    }
    if (strcmp(request.leaseId, state.leaseId) != 0) {
      reason = "manual_lease_id_mismatch";
      return false;
    }
    if (request.forward != state.forward) {
      reason = "manual_lease_direction_mismatch";
      return false;
    }
    if (request.sequence <= state.lastSequence) {
      reason = "manual_lease_sequence_stale";
      return false;
    }
    // A failed/ambiguous renewal cannot be replayed with the same sequence.
    state.lastSequence = request.sequence;
  }

  dispatched = true;
  if (!port.dispatch()) {
    state.observedEpoch = port.controlEpoch(request.motor);
    reason = "manual_lease_dispatch_rejected";
    return false;
  }
  state.observedEpoch = port.controlEpoch(request.motor);
  state.active = true;
  state.forward = request.forward;
  state.acceptedAt = port.now();
  if (request.start) {
    state.lastSequence = 0;
    snprintf(state.leaseId, sizeof(state.leaseId), "%s-%c-%08lx", boot_,
             '0' + request.motor, static_cast<unsigned long>(state.generation));
  }
  reason = "accepted";
  return true;
}
}
