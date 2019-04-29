/**************************************************************************\

  @file     wifi.c
  @author   drdelambre

  @section LICENSE

  Software License Agreement (BSD License)

  Copyright (c) 2019, Section 31
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  3. Neither the name of the copyright holders nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_blufi_api.h"
#include "mdns.h"
#include "mqtt.h"
#include "wifi.h"
#include "ble.h"
#include "storage.h"

#define WIFI_LIST_NUM   10
static const char *TAG = "WIFI";

static wifi_config_t sta_config;

static EventGroupHandle_t wifi_event_group;

const int CONNECTED_BIT = BIT0;

static bool gl_sta_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;

static uint8_t autoConnectFails = 0;

void start_wifi() {
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool isWifiConnected() {
    return (bool) CONNECTED_BIT;
}

void stop_wifi() {
    stop_mqtt();
    mdns_free();
    esp_wifi_disconnect();
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
}

void scan_wifi(void) {
    wifi_scan_config_t scanConf = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
}

void set_wifi_ssid(char *ssid, uint8_t len) {
    strncpy((char *)sta_config.sta.ssid, ssid, len);
    sta_config.sta.ssid[len] = '\0';
    putNVR("ssid", (char *)sta_config.sta.ssid);
}

void set_wifi_pass(char *pass, uint8_t len) {
    strncpy((char *)sta_config.sta.password, pass, len);
    sta_config.sta.password[len] = '\0';
    putNVR("passwd", (char *)sta_config.sta.password);
}

bool status_wifi(wifi_mode_t *mode, esp_blufi_extra_info_t *info) {
    esp_wifi_get_mode(mode);

    if (!gl_sta_connected) {
        return false;
    }

    memset(info, 0, sizeof(esp_blufi_extra_info_t));
    memcpy(info->sta_bssid, gl_sta_bssid, 6);
    info->sta_bssid_set = true;
    info->sta_ssid = gl_sta_ssid;
    info->sta_ssid_len = gl_sta_ssid_len;

    return true;
}

esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    wifi_mode_t mode;

    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP: {
            system_event_sta_got_ip_t *got_ip = &event->event_info.got_ip;
            esp_blufi_extra_info_t info;

            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            esp_wifi_get_mode(&mode);

            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
            char addr[15];
            sprintf(
                addr,
                "%d.%d.%d.%d",
                (uint8_t)(*(&got_ip->ip_info.ip.addr)),
                (uint8_t)(*(&got_ip->ip_info.ip.addr) >> 8),
                (uint8_t)(*(&got_ip->ip_info.ip.addr) >> 16),
                (uint8_t)(*(&got_ip->ip_info.ip.addr) >> 24)
            );
            mdns_service_txt_item_set("_ddlabs", "_tcp", "ip", addr);
            mdns_service_txt_item_set("_ddlabs", "_tcp", "wifi", (char *)sta_config.sta.ssid);
            stop_advertise();
            ble_disable();
            start_mqtt();
            break;
        }
        case SYSTEM_EVENT_STA_CONNECTED:
            gl_sta_connected = true;
            memcpy(gl_sta_bssid, event->event_info.connected.bssid, 6);
            memcpy(gl_sta_ssid, event->event_info.connected.ssid, event->event_info.connected.ssid_len);
            gl_sta_ssid_len = event->event_info.connected.ssid_len;
            autoConnectFails = 0;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            if (autoConnectFails > 10) {
                autoConnectFails = 0;
                stop_wifi();
                ble_enable();
                start_advertise();
                break;
            }
            autoConnectFails++;
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            gl_sta_connected = false;
            memset(gl_sta_ssid, 0, 32);
            memset(gl_sta_bssid, 0, 6);
            gl_sta_ssid_len = 0;
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_SCAN_DONE: {
            uint16_t apCount = 0;
            esp_wifi_scan_get_ap_num(&apCount);

            if (apCount == 0) {
                ESP_LOGI(TAG, "No Access Points found");
                break;
            }

            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);

            if (!ap_list) {
                ESP_LOGI(TAG, "malloc error, ap_list is NULL");
                break;
            }

            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
            esp_blufi_ap_record_t * blufi_ap_list = (esp_blufi_ap_record_t *)malloc(apCount * sizeof(esp_blufi_ap_record_t));

            if (!blufi_ap_list) {
                if (ap_list) {
                    free(ap_list);
                }

                ESP_LOGI(TAG, "malloc error, blufi_ap_list is NULL");
                break;
            }

            for (int i = 0; i < apCount; ++i) {
                blufi_ap_list[i].rssi = ap_list[i].rssi;
                memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
            }

            esp_blufi_send_wifi_list(apCount, blufi_ap_list);
            esp_wifi_scan_stop();
            free(ap_list);
            free(blufi_ap_list);
            break;
        }
        default:
            break;
    }

    mdns_handle_system_event(ctx, event);
    return ESP_OK;
}


void init_mdns(void) {
    char *_uuid = fetchNVR("uuid");
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(_uuid));

    mdns_txt_item_t serviceTxtData[7] = {
        { "wifi", "" },
        { "ip", "" },
        { "version", "0.0.0" },
        { "type", "saw_fence" },
        { "name", "My Fence" },
        { "mqtt", "" },
        { "status", "OK" }
    };

    ESP_ERROR_CHECK(mdns_service_add("ddlabs status info", "_ddlabs", "_tcp", 9000, serviceTxtData, 7));

    char *_display = fetchNVR("name");
    if (strlen(_display)) {
        ESP_ERROR_CHECK( mdns_service_txt_item_set("_ddlabs", "_tcp", "name", _display) );
    }

    char *_broker = fetchNVR("mqtt");
    if (strlen(_broker)) {
        ESP_ERROR_CHECK( mdns_service_txt_item_set("_ddlabs", "_tcp", "mqtt", _broker) );
    }

    ESP_LOGI(TAG, "mdns service started");
}

void init_wifi(void) {
    init_mdns();
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );

    char *_ssid = fetchNVR("ssid");
    char *_pw = fetchNVR("passwd");

    if (!strlen(_ssid) || !strlen(_pw)) {
        start_advertise();
        return;
    }

    strcpy((char *)sta_config.sta.ssid, _ssid);
    strcpy((char *)sta_config.sta.password, _pw);
    start_wifi();
}
