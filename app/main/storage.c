#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "storage.h"

#define STORAGE_NAMESPACE "WIFICREDS"

const char ALPHA[] = "abcdefghijklmnopqrstuvwxyz0123456789";
static char appName[] = "ddlabs-xxxxxx";

char* fetchNVR(const char *key) {
    nvs_handle my_handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return "";

    size_t required_size = 0;
    err = nvs_get_str(my_handle, key, NULL, &required_size);

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return "";

    if (required_size == 0) {
        nvs_close(my_handle);
        return "";
    }

    char *resp = malloc(required_size);
    err = nvs_get_str(my_handle, key, resp, &required_size);

    if (err == ESP_OK) {
        nvs_close(my_handle);
        return resp;
    }

    nvs_close(my_handle);
    return "";
}

esp_err_t putNVR(const char *key, const char *val) {
    nvs_handle my_handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    err = nvs_set_str(my_handle, key, val);

    if (err != ESP_OK) return err;

    err = nvs_commit(my_handle);
    nvs_close(my_handle);

    return err;
}

void initNVR(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    char *_uuid = fetchNVR("uuid");
    if (!strlen(_uuid)) {
        for (uint8_t ni = 0; ni < 6; ni++) {
            appName[ni + 7] = ALPHA[esp_random() % (sizeof(ALPHA) - 1)];
        }
        putNVR("uuid", appName);
    }
}

