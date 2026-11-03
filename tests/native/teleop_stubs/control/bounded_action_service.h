#pragma once
namespace bounded { enum class Reason { OperatorOverride }; }
namespace BoundedActions { inline void cancel(bounded::Reason) {} inline bool active() { return false; } }
