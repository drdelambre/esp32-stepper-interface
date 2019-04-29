#include "wifi.h"
#include "ble.h"
#include "storage.h"
#include "spi.h"
#include "node_temp.h"
#include "temp_controller.h"
#include "pid.h"

PIDDef pid = {
    .min = 0.0,
    .max = 1000.0,         // same as window_size
    .sample_time = 500.0,      // 1s in ms

    .kp = 300.0,
    .ki = 50.0,
    .kd = 600.0
};

TempController x_axis = {
    .pin_cs = 4,
    .pin_relay = 14,
    .window_size = 1000.0,      // 5s in ms
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
