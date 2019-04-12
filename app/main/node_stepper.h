#pragma once

#include "stepper_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    STALL_WAIT_NONE = 0,
    STALL_WAIT_HOME,
    STALL_WAIT_MEASURE,
    STALL_WAIT_POSITION
} StallWait;

typedef struct stepper_node {
    char code;
    StepperAxis* driver;

    StallWait m_stall_wait;
    TaskHandle_t m_conn;
} StepperNode;

void node_stepper_init(StepperAxis* axis, char code);
