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
