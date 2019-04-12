#include "wifi.h"
#include "ble.h"
#include "storage.h"
#include "spi.h"
#include "node_temp.h"
#include "temp_controller.h"
#include "pid.h"

PIDDef pid = {
    .min = 0.0,
    .max = 5000000.0,         // same as window_size
    .sample_time = 1000000,   // 1s in us

    .kp = 850.0,
    .ki = 0.5,
    .kd = 0.1
};

TempController x_axis = {
    .pin_cs = 32,
    .pin_relay = 13,
    .window_size = 5000000,   // 5s in us
    .pid = &pid
};

void app_main() {
    initNVR();
    init_ble();
    init_wifi();

    spi_init_bus();

    node_temp_init(&x_axis, 'X');

    while(1) {
    }
}
