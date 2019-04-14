#pragma once

typedef enum {
    CLOSED = 0,
    OPEN = 1
} ToggleState;

typedef struct SwitchDef {
    uint8_t pin_switch;

    ToggleState state;
} Switch;

void switch_open(Switch* device);
void switch_close(Switch* device);
void switch_init(Switch* device);
