#include "node_stepper.h"
#include "stepper_driver.h"
#include "mqtt.h"
#include "register.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void process_interrupt_task(void *pvParam) {
    StepperNode* node = (StepperNode *)pvParam;

    char str[18];
    uint16_t str_len;

    while (1) {
        xSemaphoreTake(node->driver->m_stall, portMAX_DELAY);

        switch (node->m_stall_wait) {
            case STALL_WAIT_HOME:
                node->m_stall_wait = STALL_WAIT_NONE;

                // NOTE: this also stops the motor
                stepper_set_position(node->driver, 0);
                stepper_disable_stall(node->driver);

                str_len = snprintf(str, sizeof(str), "%f", 0.0);
                mqtt_node_publish(node->code, "/home/result", str, str_len);
                break;
            case STALL_WAIT_MEASURE:
                node->m_stall_wait = STALL_WAIT_NONE;

                stepper_stop(node->driver);
                stepper_disable_stall(node->driver);

                str_len = snprintf(str, sizeof(str), "%f", stepper_position(node->driver));
                mqtt_node_publish(node->code, "/measure/result", str, str_len);
                break;
            case STALL_WAIT_POSITION:
                node->m_stall_wait = STALL_WAIT_NONE;

                str_len = snprintf(str, sizeof(str), "%f", stepper_position(node->driver));
                mqtt_node_publish(node->code, "/position/result", str, str_len);
                break;
            default:
                break;
        }
    }
}

void subscribe(char* prefix, void* node) {
    StepperNode* _node = (StepperNode*)node;

    // enables the motor
    mqtt_sub(gen_topic(prefix, _node->code, "/enable"));
    // disables the motor
    mqtt_sub(gen_topic(prefix, _node->code, "/disable"));
    // homes the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/home"));
    // measures the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/measure"));
    // returns the current position of the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/position"));
    // move the axis to a position in mm
    mqtt_sub(gen_topic(prefix, _node->code, "/position/mm"));
    // move the axis to a position in inches
    mqtt_sub(gen_topic(prefix, _node->code, "/position/inch"));

    // set the max speed (in mm/s) for the motor
    mqtt_sub(gen_topic(prefix, _node->code, "/speed"));
    // set the max acceleration for the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/accel"));
    // set the steps per mm for the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/steps/per/mm"));
    // set the steps per inch for the axis
    mqtt_sub(gen_topic(prefix, _node->code, "/steps/per/inch"));
    // adjust the stall detector sensitivity
    mqtt_sub(gen_topic(prefix, _node->code, "/stall/sensitivity"));
}

void unsubscribe(char* prefix, void* node) {
    StepperNode* _node = (StepperNode*)node;

    mqtt_unsub(gen_topic(prefix, _node->code, "/enable"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/disable"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/home"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/measure"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/position"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/position/mm"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/position/inch"));

    mqtt_unsub(gen_topic(prefix, _node->code, "/speed"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/accel"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/steps/per/mm"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/steps/per/inch"));
    mqtt_unsub(gen_topic(prefix, _node->code, "/stall/sensitivity"));
}

bool handle(esp_mqtt_event_handle_t evt, char* prefix, void* node) {
    StepperNode* _node = (StepperNode*)node;

    uint16_t _prefix_len = strlen(gen_topic(prefix, _node->code, ""));
    uint16_t _topic_len = evt->topic_len - _prefix_len;

    if (_topic_len == 7 && strncmp(&evt->topic[_prefix_len], "/enable", _topic_len) == 0) {
        stepper_enable(_node->driver);
        return true;
    }

    if (_topic_len == 8 && strncmp(&evt->topic[_prefix_len], "/disable", _topic_len) == 0) {
        stepper_disable(_node->driver);
        return true;
    }

    if (_topic_len == 5 && strncmp(&evt->topic[_prefix_len], "/home", _topic_len) == 0) {
        stepper_enable_stall(_node->driver);
        stepper_move(_node->driver, -3000.0);
        _node->m_stall_wait = STALL_WAIT_HOME;
        return true;
    }

    if (_topic_len == 8 && strncmp(&evt->topic[_prefix_len], "/measure", _topic_len) == 0) {
        stepper_enable_stall(_node->driver);
        stepper_move(_node->driver, 3000.0);
        _node->m_stall_wait = STALL_WAIT_MEASURE;
        return true;
    }

    if (_topic_len == 9 && strncmp(&evt->topic[_prefix_len], "/position", _topic_len) == 0) {
        char str[18];
        uint16_t str_len = snprintf(str, sizeof(str), "%f", stepper_position(_node->driver));
        mqtt_node_publish(_node->code, "/position/result", str, str_len);
        return true;
    }

    if (_topic_len == 12 && strncmp(&evt->topic[_prefix_len], "/position/mm", _topic_len) == 0) {
        if (_node->m_stall_wait == STALL_WAIT_POSITION) {
            stepper_move(_node->driver, atof(evt->data));
            return true;
        }

        stepper_move(_node->driver, atof(evt->data));
        _node->m_stall_wait = STALL_WAIT_POSITION;
        return true;
    }

    if (_topic_len == 14 && strncmp(&evt->topic[_prefix_len], "/position/inch", _topic_len) == 0) {
        if (_node->m_stall_wait == STALL_WAIT_POSITION) {
            stepper_move(_node->driver, atof(evt->data) * 25.4);
            return true;
        }

        stepper_move(_node->driver, atof(evt->data) * 25.4);
        _node->m_stall_wait = STALL_WAIT_POSITION;
        return true;
    }

    if (_topic_len == 6 && strncmp(&evt->topic[_prefix_len], "/speed", _topic_len) == 0) {
        stepper_set_speed(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 6 && strncmp(&evt->topic[_prefix_len], "/accel", _topic_len) == 0) {
        stepper_set_accel(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 13 && strncmp(&evt->topic[_prefix_len], "/steps/per/mm", _topic_len) == 0) {
        stepper_set_steps_per_mm(_node->driver, atof(evt->data));
        return true;
    }

    if (_topic_len == 15 && strncmp(&evt->topic[_prefix_len], "/steps/per/inch", _topic_len) == 0) {
        stepper_set_steps_per_mm(_node->driver, atof(evt->data) / 25.4);
        return true;
    }

    if (_topic_len == 18 && strncmp(&evt->topic[_prefix_len], "/stall/sensitivity", _topic_len) == 0) {
        stepper_stall_level(_node->driver, atoi(evt->data));
        return true;
    }

    return false;
}

void to_string(void* node, char* buff, uint16_t* offset) {
    StepperNode* _node = (StepperNode*)node;

    *offset += sprintf(&buff[*offset], "{\"code\":\"%c\",\"type\":\"stepper\"}", _node->code);
}

void node_stepper_init(StepperAxis *axis, char code) {
    stepper_init(axis);

    StepperNode* node = malloc(sizeof(StepperNode));
    node->code = code;
    node->driver = axis;
    node->m_stall_wait = STALL_WAIT_NONE;
    xTaskCreate(process_interrupt_task, "interrupt", 2048, NULL, 10, node->m_conn);

    // TODO: put this in a linked list so we can dealloc memory (dd)
    NodeDescription* des = malloc(sizeof(NodeDescription));

    des->node = (void*)node;
    des->to_string = to_string;
    des->sub = subscribe;
    des->unsub = unsubscribe;
    des->handle = handle;

    register_node(des);
}

