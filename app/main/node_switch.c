#include "node_switch.h"
#include "switch_driver.h"
#include "register.h"

void to_string(void* node, char* buff, uint16_t* offset) {
    SwitchNode* tmp = (SwitchNode*)node;

    *offset += sprintf(&buff[*offset], "{\"code\":\"%c\",\"type\":\"switch\"}", tmp->code);
}

void subscribe(char* prefix, void* node) {
    SwitchNode* _node = (SwitchNode*)node;

    // enables the switch
    mqtt_sub(gen_topic(prefix, _node->code, "/on"));

    // disables the switch
    mqtt_sub(gen_topic(prefix, _node->code, "/off"));
}

void unsubscribe(char* prefix, void* node) {
    SwitchNode* _node = (SwitchNode*)node;

    mqtt_unsub(gen_topic(prefix, _node->code, "/on"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/off"));
}

bool handle(esp_mqtt_event_handle_t evt, char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    uint16_t _prefix_len = strlen(gen_topic(prefix, _node->code, ""));
    uint16_t _topic_len = evt->topic_len - _prefix_len;

    if (_topic_len == 3 && strncmp(&evt->topic[_prefix_len], "/on", _topic_len) == 0) {
        switch_open(_node);
        return true;
    }

    if (_topic_len == 4 && strncmp(&evt->topic[_prefix_len], "/off", _topic_len) == 0) {
        switch_close(_node);
        return true;
    }

    return false;
}

void node_switch_init(Switch* device, char code) {
    switch_init(device);

    SwitchNode* node = malloc(sizeof(SwitchNode));
    node->code = code;
    node->driver = device;

    NodeDescription* des = malloc(sizeof(NodeDescription));

    des->node = (void*)node;
    des->to_string = to_string;
    des->sub = subscribe;
    des->unsub = unsubscribe;
    des->handle = handle;

    register_node(des);
}
