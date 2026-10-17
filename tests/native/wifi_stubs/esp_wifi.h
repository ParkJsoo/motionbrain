#pragma once
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
enum wifi_ps_type_t { WIFI_PS_NONE, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM };
enum wifi_mode_t { WIFI_MODE_NULL, WIFI_MODE_STA, WIFI_MODE_AP, WIFI_MODE_APSTA };
esp_err_t esp_wifi_get_ps(wifi_ps_type_t*);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t);
esp_err_t esp_wifi_get_mode(wifi_mode_t*);
