#pragma once
#define WIFI_MODE_STA 1
#define WL_CONNECTED 3
struct WiFiFake{int connection=3;int getMode(){return 1;}int status(){return connection;}};
extern WiFiFake WiFi;
