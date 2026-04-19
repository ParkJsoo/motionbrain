#ifndef CONTROL_COMMAND_BUS_H
#define CONTROL_COMMAND_BUS_H

#include <Arduino.h>
#include <stdint.h>
#include "control/command.h"

class CommandBus {
public:
  static const uint8_t MAX_PENDING = 16;

  CommandBus();

  uint32_t allocateId();
  bool enqueue(const Command& command);
  bool dequeue(Command& command);
  bool isEmpty() const;
  bool isFull() const;
  uint8_t size() const;

private:
  Command  queue_[MAX_PENDING];
  uint8_t  head_;
  uint8_t  tail_;
  uint8_t  count_;
  uint32_t nextId_;
};

#endif // CONTROL_COMMAND_BUS_H
