#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mdns.h"
#include "mqtt_client.h"

#include "mqtt.h"
#include "storage.h"
#include "register.h"

static TaskHandle_t _conn;
esp_mqtt_client_handle_t client;
static const char *TAG = "MQTT";

static char m_prefix[120];
static char m_msg_topic[120];
static char m_json_buff[256];
static char m_json_res_buff[256];
static uint16_t m_json_buff_len;

static void setup_mqtt() {
    char* prefix = fetchNVR("prefix");
    char* id = fetchNVR("uuid");

    if (strlen(prefix)) {
        strcpy(m_prefix, prefix);
    } else {
        strcpy(m_prefix, id);
    }

    // information about the device
    esp_mqtt_client_subscribe(client, "/edge_node/info", 1);

    strcpy(m_msg_topic, "/");
    strcat(m_msg_topic, id);
    strcat(m_msg_topic, "/info");
    esp_mqtt_client_subscribe(client, m_msg_topic, 1);

    strcpy(m_msg_topic, "/");
    strcat(m_msg_topic, id);
    strcat(m_msg_topic, "/prefix");
    esp_mqtt_client_subscribe(client, m_msg_topic, 1);

    strcpy(m_msg_topic, "/");
    strcat(m_msg_topic, prefix);
    strcat(m_msg_topic, "/info");
    esp_mqtt_client_subscribe(client, m_msg_topic, 1);

    strcpy(m_msg_topic, "/");
    strcat(m_msg_topic, prefix);
    strcat(m_msg_topic, "/prefix");
    esp_mqtt_client_subscribe(client, m_msg_topic, 1);

    register_sub_nodes(m_prefix);
}

void mqtt_sub(char* topic) {
    if (!client) {
        return;
    }

    esp_mqtt_client_subscribe(client, topic, 1);
}

void mqtt_unsub(char* topic) {
    if (!client) {
        return;
    }

    esp_mqtt_client_unsubscribe(client, topic);
}

static void handle_mqtt_evt(esp_mqtt_event_handle_t evt) {
    char* id = fetchNVR("uuid");

    if (
        (strncmp(evt->topic, "/edge_node/info", evt->topic_len) == 0) ||
        (evt->topic_len == strlen(id) + 5 && strncmp(&evt->topic[strlen(id) + 1], "/info", evt->topic_len - (strlen(id) + 1)) == 0) ||
        (evt->topic_len == strlen(m_prefix) + 5 && strncmp(&evt->topic[strlen(m_prefix) + 1], "/info", evt->topic_len - (strlen(m_prefix) + 1)) == 0)
    ) {
        /*
         * needs to return enough info to allow for system provisioning
         * on /edge_node/info/result
         *     id: string to uniquely identify controller
         *     prefix:  string to uniquely identify controller group
         *     nodes: [
         *         {
         *             code:    // unique id for the controller
         *             type:    // identifier of node type
         *             ...      // type specific information
         *         },
         *         ...
         *     ]
         *
         */
        register_nodes_to_string(m_json_res_buff);
        m_json_buff_len = snprintf(m_json_buff, sizeof(m_json_buff), "{\"id\":\"%s\",\"prefix\":\"%s\",\"nodes\":%s}", id, m_prefix, m_json_res_buff);
        strncpy(m_msg_topic, evt->topic, evt->topic_len);
        m_msg_topic[evt->topic_len] = '\0';
        strcat(m_msg_topic, "/result");
        esp_mqtt_client_publish(client, m_msg_topic, m_json_buff, m_json_buff_len, 0, 0);
        return;
    }

    if (
        (evt->topic_len == strlen(id) + 8 && strncmp(&evt->topic[strlen(id) + 1], "/prefix", evt->topic_len - (strlen(id) + 1)) == 0) ||
        (evt->topic_len == strlen(m_prefix) + 8 && strncmp(&evt->topic[strlen(m_prefix) + 1], "/prefix", evt->topic_len - (strlen(m_prefix) + 1)) == 0)
    ) {
        register_unsub_nodes(m_prefix);

        strcpy(m_msg_topic, "/");
        strcat(m_msg_topic, m_prefix);
        strcat(m_msg_topic, "/prefix");
        esp_mqtt_client_unsubscribe(client, m_msg_topic);

        memcpy(m_prefix, evt->data, evt->data_len);
        memcpy(&m_prefix[evt->data_len], "\0", 1);
        putNVR("prefix", m_prefix);

        strcpy(m_msg_topic, "/");
        strcat(m_msg_topic, m_prefix);
        strcat(m_msg_topic, "/prefix");
        esp_mqtt_client_subscribe(client, m_msg_topic, 1);

        register_sub_nodes(m_prefix);

        return;
    }

    register_handle_nodes(evt, m_prefix);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            setup_mqtt();

            break;
        case MQTT_EVENT_DISCONNECTED:
            client = NULL;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            break;
        case MQTT_EVENT_PUBLISHED:
            break;
        case MQTT_EVENT_DATA:
            handle_mqtt_evt(event);
            break;
        case MQTT_EVENT_ERROR:
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }

    return ESP_OK;
}

static bool get_server_ip(mdns_result_t *r, ip4_addr_t *out) {
    mdns_ip_addr_t *a = r->addr;
    while(a) {
        if (a->addr.type == IPADDR_TYPE_V4) {
            *(out) = a->addr.u_addr.ip4;
            return true;
        }

        a = a->next;
    }

    return false;
}

static void mqtt_server_task(void *pvParameters) {
    mdns_result_t *results = NULL;
    ip4_addr_t broker;

    while(1) {
        esp_err_t err = mdns_query_ptr("_mqtt", "_tcp", 3000, 20,  &results);

        if (!err && results) {
            while (results) {
                // do a better job filtering
                if (get_server_ip(results, &broker)) {
                    break;
                }
                results = results->next;
            }
            mdns_query_results_free(results);

            if (results) {
                break;
            }
        }

        if (err) {
            ESP_LOGE(TAG, "Query Failed: %s", esp_err_to_name(err));
        }

        ESP_LOGI(TAG, "No results found!");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    char ip[16];

    sprintf(ip, "%u.%u.%u.%u", ((char *)(&broker.addr))[0], ((char *)(&broker.addr))[1], ((char *)(&broker.addr))[2], ((char *)(&broker.addr))[3]);
    char *uri = malloc(strlen(ip) + 7);
    sprintf(uri, "%s%s", "mqtt://", ip);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = uri,
        .event_handle = mqtt_event_handler
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    vTaskDelete(_conn);
    _conn = NULL;
}

void start_mqtt(void) {
    xTaskCreate(mqtt_server_task, "mqtt_server", 3072, NULL, 5, &_conn);
}

void stop_mqtt(void) {
    if (_conn) {
        vTaskDelete(_conn);
    }
}

void mqtt_node_publish(char code, char* channel, char* data, uint16_t len) {
    if (!client) {
        return;
    }

    esp_mqtt_client_publish(client, gen_topic(m_prefix, code, channel), data, len, 0, 0);
}
