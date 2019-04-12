#include "node_temp.h"
#include "temp_controller.h"
#include "mqtt.h"
#include "register.h"

void to_string(void* node, char* buff, uint16_t* offset) {
    TempNode* tmp = (TempNode*)node;

    *offset += sprintf(&buff[*offset], "{\"code\":\"%c\",\"type\":\"temperature\"}", tmp->code);
}

void subscribe(char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    // sets the target temperature for the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/temp"));
    // a read request for the target and current temperatures from the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/read"));

    // sets the ramp speed of the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/ramp"));
    // sets the pid variables of the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/tune"));
    // kills the heating element
    mqtt_sub(gen_topic(prefix, _node->code, "/disable"));
}

void unsubscribe(char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    mqtt_unsub(gen_topic(prefix, _node->code, "/temp"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/read"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/ramp"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/tune"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/disable"));
}

bool handle(esp_mqtt_event_handle_t evt, char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    uint16_t _prefix_len = strlen(gen_topic(prefix, _node->code, ""));
    uint16_t _topic_len = evt->topic_len - _prefix_len;
    char str[18];
    uint16_t str_len;

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/temp", _topic_len) == 0) {
        temp_set_temp(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/read", _topic_len) == 0) {
        str_len = snprintf(str, sizeof(str), "%f", temp_read_temp(_node->driver));
        mqtt_node_publish(_node->code, "/read/result", str, str_len);
        return true;
    }

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/ramp", _topic_len) == 0) {
        return true;
    }

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/tune", _topic_len) == 0) {
        return true;
    }

    if (_topic_len == 8 && strncmp(&evt->topic[_prefix_len], "/disable", _topic_len) == 0) {
        temp_disable(_node->driver);
        return true;
    }

    return false;
}

void node_temp_init(TempController* temp, char code) {
    temp_init(temp);

    TempNode* node = malloc(sizeof(TempNode));
    node->code = code;
    node->driver = temp;

    NodeDescription* des = malloc(sizeof(NodeDescription));

    des->node = (void*)node;
    des->to_string = to_string;
    des->sub = subscribe;
    des->unsub = unsubscribe;
    des->handle = handle;

    register_node(des);
}
