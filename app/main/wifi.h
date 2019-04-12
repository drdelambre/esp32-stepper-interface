#pragma once
#include "esp_system.h"
#include "esp_blufi_api.h"

void start_wifi(void);
void stop_wifi(void);
void scan_wifi(void);
bool isWifiConnected(void);
void set_wifi_ssid(char *ssid, uint8_t len);
void set_wifi_pass(char *pass, uint8_t len);
bool status_wifi(wifi_mode_t *mode, esp_blufi_extra_info_t *info);
void init_wifi(void);
