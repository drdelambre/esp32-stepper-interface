#pragma once

#include "temp_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct temp_node {
    char code;
    TempController* driver;

    TaskHandle_t m_conn;
} TempNode;

void node_temp_init(TempController* con, char code);
