#include "switch_driver.h"
#include "driver/gpio.h"

void switch_open(Switch* device) {
    gpio_set_level(device->pin_switch, 1);
}

void switch_close(Switch* device) {
    gpio_set_level(device->pin_switch, 0);
}

void switch_init(Switch* device) {
    gpio_pad_select_gpio(device->pin_switch);
    gpio_set_direction(device->pin_switch, GPIO_MODE_OUTPUT);
}
