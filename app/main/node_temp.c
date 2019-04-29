/**************************************************************************\

  @file     node_temp.c
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

#include "node_temp.h"
#include "temp_controller.h"
#include "mqtt.h"
#include "register.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void process_interrupt_task(void *pvParam) {
    TempNode* node = (TempNode *)pvParam;

    while (1) {
        xSemaphoreTake(node->driver->m_change, portMAX_DELAY);

        if (node->driver->m_active) {
            mqtt_node_publish(node->code, "/pid/on", "", 0);
        } else {
            mqtt_node_publish(node->code, "/pid/off", "", 0);
        }
    }
}

void process_reach_task(void *pvParam) {
    TempNode* node = (TempNode *)pvParam;

    while (1) {
        xSemaphoreTake(node->driver->m_reached, portMAX_DELAY);

        mqtt_node_publish(node->code, "/temp/reached", "", 0);
    }
}

void to_string(void* node, char* buff, uint16_t* offset) {
    TempNode* tmp = (TempNode*)node;

    double target = 0.0;

    if (tmp->driver->m_enabled) {
        target = tmp->driver->m_target;
    }

    *offset += sprintf(
        &buff[*offset],
        "{\"code\":\"%c\",\"type\":\"temperature\",\"set\":%.2f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f}",
        tmp->code,
        ((signed long)(target * 100) * 0.01f),
        ((signed long)(tmp->driver->pid->kp * 100) * 0.01f),
        ((signed long)((tmp->driver->pid->ki / (tmp->driver->pid->sample_time / 1000.0)) * 100) * 0.01f),
        ((signed long)((tmp->driver->pid->kd * (tmp->driver->pid->sample_time / 1000.0)) * 100) * 0.01f)
    );
}

void subscribe(char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    // sets the target temperature for the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/temp"));
    // a read request for the target and current temperatures from the controller
    mqtt_sub(gen_topic(prefix, _node->code, "/read"));

    // sets the pid variables of the controller
    // format:   "p:i:d"
    mqtt_sub(gen_topic(prefix, _node->code, "/tune"));
    // enables the heating element
    mqtt_sub(gen_topic(prefix, _node->code, "/on"));
    // kills the heating element
    mqtt_sub(gen_topic(prefix, _node->code, "/off"));
}

void unsubscribe(char* prefix, void* node) {
    TempNode* _node = (TempNode*)node;

    mqtt_unsub(gen_topic(prefix, _node->code, "/temp"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/read"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/tune"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/on"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/off"));
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

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/tune", _topic_len) == 0) {
        char* data[3];
        uint8_t ni;
        strncpy(str, evt->data, evt->data_len);

        for (ni = 0; ni < 3; ni++) {
            if (ni == 0) {
                data[ni] = strtok(str, ":");
            } else {
                data[ni] = strtok(NULL, ":");
            }
        }

        pid_tune(
            atof(data[0]),
            atof(data[1]),
            atof(data[2]),
            _node->driver->pid->POn,
            _node->driver->pid
        );

        return true;
    }

    if (_topic_len == 3 && strncmp(&evt->topic[_prefix_len], "/on", _topic_len) == 0) {
        temp_enable(_node->driver);
        return true;
    }

    if (_topic_len == 4 && strncmp(&evt->topic[_prefix_len], "/off", _topic_len) == 0) {
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
    xTaskCreate(process_interrupt_task, "interrupt", 2048, node, 10, &node->m_conn);
    xTaskCreate(process_reach_task, "destination", 2048, node, 10, &node->m_conn2);

    NodeDescription* des = malloc(sizeof(NodeDescription));

    des->node = (void*)node;
    des->to_string = to_string;
    des->sub = subscribe;
    des->unsub = unsubscribe;
    des->handle = handle;

    register_node(des);
}
