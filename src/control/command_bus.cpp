#include "control/finite_profile_legacy_guard.h"
#include "control/command_bus.h"

CommandBus::CommandBus()
  : head_(0)
  , tail_(0)
  , count_(0)
  , nextId_(1) {
}

uint32_t CommandBus::allocateId() {
  return nextId_++;
}

bool CommandBus::enqueue(const Command& command) {
  if (!FiniteProfileLegacy::allowed(command.type)) return false;
  if (isFull()) {
    return false;
  }

  queue_[tail_] = command;
  tail_ = (tail_ + 1) % MAX_PENDING;
  count_++;
  return true;
}

bool CommandBus::dequeue(Command& command) {
  if (isEmpty()) {
    return false;
  }

  command = queue_[head_];
  head_ = (head_ + 1) % MAX_PENDING;
  count_--;
  return true;
}

bool CommandBus::isEmpty() const {
  return count_ == 0;
}

bool CommandBus::isFull() const {
  return count_ >= MAX_PENDING;
}

uint8_t CommandBus::size() const {
  return count_;
}

bool CommandBus::hasPendingControlCommand() const {
  for (uint8_t i = 0; i < count_; ++i) {
    switch (queue_[(head_ + i) % MAX_PENDING].type) {
      case CommandType::LIGHT_ON:
      case CommandType::LIGHT_OFF:
      case CommandType::LIGHT_TOGGLE:
      case CommandType::ROUTINE_DRY_RUN:
        break;
      default:
        return true;
    }
  }
  return false;
}
