#pragma once

#include "temp_controller.h"

typedef struct temp_node {
    char code;
    TempController* driver;
} TempNode;

void node_temp_init(TempController* con, char code);
