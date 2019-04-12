#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "ble.h"
#include "wifi.h"
#include "storage.h"

static const char *TAG = "BLE";
static uint8_t service_uuid[32] = {
    0x64, 0x64, 0x6c, 0x61, 0x62, 0x73, 0x45, 0x6b, 0xac, 0xb9, 0xad, 0x97, 0x67, 0x0b, 0xfc, 0x34
};
static void event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);
static char *appName;
static uint8_t server_if;
static uint16_t conn_id;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid,
    .flag = 0x6,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_blufi_callbacks_t ble_callbacks = {
    .event_cb = event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static void event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param) {
    switch (event) {
        case ESP_BLUFI_EVENT_INIT_FINISH:
            esp_ble_gap_set_device_name(appName);
            esp_ble_gap_config_adv_data(&adv_data);
            ESP_LOGI(TAG, "Init finished");
            break;
        case ESP_BLUFI_EVENT_DEINIT_FINISH:
            ESP_LOGI(TAG, "Deinit finished");
            break;
        case ESP_BLUFI_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "BLE Connected");
            server_if = param->connect.server_if;
            conn_id = param->connect.conn_id;
            stop_advertise();
            blufi_security_init();
            break;
        case ESP_BLUFI_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "BLE Disconnected");
            blufi_security_deinit();
            if (!isWifiConnected()) {
                start_advertise();
            }
            break;
        case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
            ESP_LOGI(TAG, "Request wifi connection");
            /* there is no wifi callback when the device has already connected to this wifi
                so disconnect wifi before connection.
            */
            start_wifi();
            break;
        case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
            ESP_LOGI(TAG, "Request wifi disconnection");
            stop_wifi();
            break;
        case ESP_BLUFI_EVENT_REPORT_ERROR:
            ESP_LOGI(TAG, "Report error: %d", param->report_error.state);
            esp_blufi_send_error_info(param->report_error.state);
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
            wifi_mode_t mode;
            esp_blufi_extra_info_t info;

            if (status_wifi(&mode, &info)) {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
            }

            break;
        }
        case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
            ESP_LOGI(TAG, "GATT Connection closed");
            esp_blufi_close(server_if, conn_id);
            break;
        case ESP_BLUFI_EVENT_RECV_STA_SSID:
            set_wifi_ssid((char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
            break;
        case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
            set_wifi_pass((char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
            scan_wifi();
            break;
        }
        case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
            ESP_LOGI(TAG, "Recieved custom data: %d", param->custom_data.data_len);
            esp_log_buffer_hex("Custom Data", param->custom_data.data, param->custom_data.data_len);
            break;
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            // esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

void start_advertise(void) {
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    ESP_LOGI(TAG, "Started Advertising: %s", appName);
}

void stop_advertise(void) {
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "Stoped Advertising");
}

void ble_disable(void) {
    ESP_ERROR_CHECK(esp_blufi_profile_deinit());
    esp_bluedroid_disable();
    ESP_LOGI(TAG, "Radio Disabled");
}

void ble_enable(void) {
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_blufi_register_callbacks(&ble_callbacks));
    ESP_ERROR_CHECK(esp_blufi_profile_init());
    ESP_LOGI(TAG, "Radio Enabled");
}

void init_ble(void) {
    appName = fetchNVR("uuid");

    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize bt controller failed: %s", __func__, esp_err_to_name(ret));
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable bt controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&ble_callbacks);
    if (ret) {
        ESP_LOGE(TAG, "%s blufi register failed, error code = %x", __func__, ret);
        return;
    }

    esp_blufi_profile_init();
}
