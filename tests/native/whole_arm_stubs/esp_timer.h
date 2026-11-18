#pragma once
#include <cstdint>

struct FakeTimer { void (*callback)(void*); };
using esp_timer_handle_t = FakeTimer*;
using SemaphoreHandle_t = bool*;
struct esp_timer_create_args_t { void (*callback)(void*); const char* name; };
constexpr int ESP_OK = 0;
constexpr int portMAX_DELAY = -1;
int64_t esp_timer_get_time();
int esp_timer_create(const esp_timer_create_args_t*, esp_timer_handle_t*);
int esp_timer_start_periodic(esp_timer_handle_t, uint64_t);
void esp_timer_delete(esp_timer_handle_t);
SemaphoreHandle_t xSemaphoreCreateMutex();
void xSemaphoreTake(SemaphoreHandle_t, int);
void xSemaphoreGive(SemaphoreHandle_t);
