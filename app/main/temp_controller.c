#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "gll.h"
#include "temp_controller.h"
#include "pid.h"
#include "max31855.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static TaskHandle_t _conn;
static const char *TAG = "TEMP";
static gll_t m_controllers;

void temp_update_task() {
    while (1) {
        gll_node_t *curr = m_controllers.first;
        uint16_t ni = 0;
        TempController* data;
        int64_t now = esp_timer_get_time();

        while (ni < m_controllers.size) {
            data = (TempController *)curr->data;

            data->m_current = max31855_read(&data->m_interface);

            if (data->m_enabled) {
                pid_calculate(data->m_current, data->m_target, &data->m_output, data->pid);

                if (now - data->m_window_start > data->window_size) {
                    data->m_window_start += data->window_size;
                }

                if (data->m_output > 100 && data->m_output > (now - data->m_window_start)) {
                    gpio_set_level(data->pin_relay, 1);
                } else {
                    gpio_set_level(data->pin_relay, 0);
                }
            } else {
                gpio_set_level(data->pin_relay, 0);
            }

            ni++;
            curr = curr->next;
        }
    }

    vTaskDelete(_conn);
    _conn = NULL;
}

void temp_set_temp(TempController* tmp, double target) {
    if (tmp->m_target == target) {
        return;
    }

    tmp->m_target = target;

    if (tmp->m_enabled) {
        return;
    }

    tmp->m_enabled = 1;
    tmp->m_window_start = esp_timer_get_time();
}

double temp_read_temp(TempController* tmp) {
    return tmp->m_current;
}

void temp_disable(TempController* tmp) {
    tmp->m_enabled = 0;
}

void temp_init(TempController* con) {
    gll_pushBack(&m_controllers, (void *)con);

    gpio_pad_select_gpio(con->pin_relay);
    gpio_set_direction(con->pin_relay, GPIO_MODE_OUTPUT);

    // Add it to the spi bus
    spi_device_interface_config_t devcfg={
        .command_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .clock_speed_hz = 4000000,
        .mode = 0,                               //SPI mode 0
        .spics_io_num = con->pin_cs,             //CS pin
        .queue_size = 7                          //We want to be able to queue 7 transactions at a time
    };

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &con->m_interface));

    con->m_current = max31855_read(&con->m_interface);
    con->m_last_read = esp_timer_get_time();

    if (!_conn) {
        xTaskCreate(temp_update_task, "temp_controller", 3072, NULL, 5, &_conn);
    }
}
