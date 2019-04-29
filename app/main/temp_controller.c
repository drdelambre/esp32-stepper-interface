/**************************************************************************\

    @file     temp_controller.c
    @author   drdelambre

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019 Section 31
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
    THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**************************************************************************/

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

#define THRESHOLD   0.3

static TaskHandle_t _conn;
static const char *TAG = "TEMP";
static gll_t m_controllers;

void temp_update_task() {
    gll_node_t *curr = m_controllers.first;
    TempController* data;
    double now;

    while(1) {
        now = esp_timer_get_time() / 1000.0;
        curr = m_controllers.first;

        while (curr) {
            data = (TempController *)curr->data;
            data->m_current = max31855_read(&data->m_interface);

            if (data->m_enabled) {
                if (!data->m_has_hit && abs(data->m_current - data->m_target) < THRESHOLD) {
                    data->m_has_hit = true;
                    xSemaphoreGive(data->m_reached);
                }

                pid_calculate(data->m_current, data->m_target, &data->m_output, data->pid);

                if (now - data->m_window_start > data->window_size) {
                    data->m_window_start += data->window_size;
                }

                if (data->m_output > 100.0 && data->m_output > now - data->m_window_start) {
                    if (!data->m_active) {
                        data->m_active = true;
                        xSemaphoreGive(data->m_change);
                        gpio_set_level(data->pin_relay, 1);
                    }
                } else {
                    if (data->m_active) {
                        data->m_active = false;
                        xSemaphoreGive(data->m_change);
                        gpio_set_level(data->pin_relay, 0);
                    }
                }
            } else if (data->m_active) {
                data->m_active = false;
                xSemaphoreGive(data->m_change);
                gpio_set_level(data->pin_relay, 0);
            }

            curr = curr->next;
        }

        vTaskDelay(100);
    }

    vTaskDelete(_conn);
    _conn = NULL;
}

void temp_set_temp(TempController* tmp, double target) {
    if (tmp->m_target == target) {
        return;
    }

    tmp->m_target = target;

    tmp->m_has_hit = false;

    if (tmp->m_enabled) {
        return;
    }

    tmp->m_enabled = 1;
    tmp->m_window_start = esp_timer_get_time() / 1000.0;
}

double temp_read_temp(TempController* tmp) {
    return tmp->m_current;
}

void temp_enable(TempController* tmp) {
    tmp->m_enabled = 0;
    gpio_set_level(tmp->pin_relay, 1);
}

void temp_disable(TempController* tmp) {
    tmp->m_enabled = 0;
    gpio_set_level(tmp->pin_relay, 0);
}

void temp_init(TempController* con) {
    gll_pushBack(&m_controllers, (void *)con);

    gpio_pad_select_gpio(con->pin_relay);
    gpio_set_direction(con->pin_relay, GPIO_MODE_OUTPUT);
    gpio_set_level(con->pin_relay, 0);

    con->m_change = xSemaphoreCreateBinary();
    con->m_reached = xSemaphoreCreateBinary();

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

    pid_tune(con->pid->kp, con->pid->ki, con->pid->kd, con->pid->POn, con->pid);

    if (!_conn) {
        xTaskCreate(temp_update_task, "temp_controller", 3072, NULL, 5, &_conn);
    }
}
