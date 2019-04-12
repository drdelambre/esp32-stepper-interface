#pragma once

void initNVR(void);
char* fetchNVR(const char *key);
esp_err_t putNVR(const char *key, const char *val);
