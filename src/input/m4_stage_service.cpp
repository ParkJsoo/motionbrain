#include "input/m4_stage_service.h"
#include <esp_timer.h>
namespace M4Diagnostics {
M4StageTrace& trace() { static M4StageTrace instance; return instance; }
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
}
