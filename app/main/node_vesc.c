/**************************************************************************\

  @file     node_vesc.c
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
#include "node_vesc.h"
#include "vesc_driver.h"
#include "register.h"
#include "mqtt.h"

void subscribe(char* prefix, void* node) {
    VescNode* _node = (VescNode*)node;
    mqtt_sub(gen_topic(prefix, _node->code, "/info"));
    mqtt_sub(gen_topic(prefix, _node->code, "/position"));
    mqtt_sub(gen_topic(prefix, _node->code, "/duty"));
    mqtt_sub(gen_topic(prefix, _node->code, "/current"));
    mqtt_sub(gen_topic(prefix, _node->code, "/rpm"));
}

void unsubscribe(char* prefix, void* node) {
    VescNode* _node = (VescNode*)node;
    mqtt_unsub(gen_topic(prefix, _node->code, "/info"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/position"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/duty"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/current"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/rpm"));
}


bool handle(esp_mqtt_event_handle_t evt, char* prefix, void* node) {
    VescNode* _node = (VescNode*)node;
    uint16_t _prefix_len = strlen(gen_topic(prefix, _node->code, ""));
    uint16_t _topic_len = evt->topic_len - _prefix_len;

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/info", _topic_len) == 0) {
        vesc_read_stats(_node->driver);
        // serialize and publish
        return true;
    }

    if (_topic_len == 9 && strncmp(&evt->topic[_prefix_len], "/position", _topic_len) == 0) {
        vesc_set_position(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/duty", _topic_len) == 0) {
        vesc_set_duty(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 8 && strncmp(&evt->topic[_prefix_len], "/current", _topic_len) == 0) {
        vesc_set_current(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 4 && strncmp(&evt->topic[_prefix_len], "/rpm", _topic_len) == 0) {
        vesc_set_rpm(_node->driver, atoi(evt->data));
        return true;
    }

    return false;
}

void to_string(void* node, char* buff, uint16_t* offset) {
    VescNode* _node = (VescNode*)node;

    *offset += sprintf(&buff[*offset], "{\"code\":\"%c\",\"type\":\"vesc\"}", _node->code);
}

void node_vesc_init(VescDriver* dev, char code) {
    vesc_init(dev);

    VescNode* node = malloc(sizeof(VescNode));
    node->code = code;
    node->driver = dev;

    // TODO: put this in a linked list so we can dealloc memory (dd)
    NodeDescription* des = malloc(sizeof(NodeDescription));

    des->node = (void*)node;
    des->to_string = to_string;
    des->sub = subscribe;
    des->unsub = unsubscribe;
    des->handle = handle;

    register_node(des);
}

