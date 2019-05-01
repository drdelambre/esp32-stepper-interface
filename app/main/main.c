#include "wifi.h"
#include "ble.h"
#include "storage.h"
#include "spi.h"
#include "node_temp.h"

void app_main() {
    initNVR();
    init_ble();
    init_wifi();

    spi_init_bus();

    node_temp_init(4, 14, 'X');
}
