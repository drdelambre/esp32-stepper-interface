#pragma once

#include "switch_driver.h"

typedef struct switch_node {
    char code;
    Switch* driver;
} SwitchNode;

void node_switch_init(Switch* driver, char code);
