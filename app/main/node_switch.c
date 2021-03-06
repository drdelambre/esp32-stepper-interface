/**************************************************************************\

  @file     node_switch.c
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
#include "node_switch.h"
#include "switch_driver.h"
#include "mqtt.h"
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
    SwitchNode* _node = (SwitchNode*)node;

    uint16_t _prefix_len = strlen(gen_topic(prefix, _node->code, ""));
    uint16_t _topic_len = evt->topic_len - _prefix_len;

    if (_topic_len == 3 && strncmp(&evt->topic[_prefix_len], "/on", _topic_len) == 0) {
        switch_open(_node->driver);
        return true;
    }

    if (_topic_len == 4 && strncmp(&evt->topic[_prefix_len], "/off", _topic_len) == 0) {
        switch_close(_node->driver);
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
