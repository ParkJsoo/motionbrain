#pragma once
#include "control/command.h"
class Dispatcher { public: bool execute(const Command&, CommandResult& result) { result.success = true; return true; } };
