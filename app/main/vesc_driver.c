/**************************************************************************\

  @file     vesc_driver.c
  @author   drdelambre

  @section LICENSE

  Software License Agreement (BSD License)

  Copyright (c) 2019, Section 31
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
#include "vesc_driver.h"
#include "esp32/rom/crc.h"
#include "driver/uart.h"

/** Helper functions for parsing messages **/
int8_t buffer_get_int8(const uint8_t *buffer, int32_t *index) {
    int8_t res =   ((uint16_t) buffer[*index]) << 8 |
                    ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res =   ((uint16_t) buffer[*index]) << 8 |
                    ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res =   ((uint32_t) buffer[*index]) << 24 |
                    ((uint32_t) buffer[*index + 1]) << 16 |
                    ((uint32_t) buffer[*index + 2]) << 8 |
                    ((uint32_t) buffer[*index + 3]);
    *index += 4;

    return res;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int16(buffer, index) / scale;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int32(buffer, index) / scale;
}
/** End helper functions **/

int send_payload(int uart_num, uint8_t *payload, int len) {
    uint16_t crc = crc16_be(0, payload, len);
    int count = 0;
    uint8_t msg[256];

    if (len <= 256) {
        msg[count++] = 2;
        msg[count++] = len;
    } else {
        msg[count++] = 3;
        msg[count++] = (uint8_t)(len >> 8);
        msg[count++] = (uint8_t)(len & 0xFF);
    }

    memcpy(&msg[count], payload, len);

    count += len;
    msg[count++] = (uint8_t)(crc >> 8);
    msg[count++] = (uint8_t)(crc & 0xFF);
    msg[count++] = 3;

    //Sending package
    uart_write_bytes(uart_num, (char*)msg, count);

    return (0);
}

int parse_stats_packet(uint8_t *msg, struct vesc_stats *stats, int len) {
    COMM_PACKET_ID packet_id;
    int32_t idx = 0;

    packet_id = (COMM_PACKET_ID) msg[0];
    msg++; //Eliminates the message id
    len--;

    if (packet_id != COMM_GET_VALUES) {
        return (1);
    }

    stats->temp_mosfet = buffer_get_float16(msg, 1e1, &idx);
    stats->temp_motor = buffer_get_float16(msg, 1e1, &idx);
    stats->current_motor = buffer_get_float32(msg, 1e2, &idx);
    stats->current_in = buffer_get_float32(msg, 1e2, &idx);
    idx += 8; // skipping id and iq vals
    stats->duty_cycle_now = buffer_get_float16(msg, 1e3, &idx);
    stats->rpm = buffer_get_float32(msg, 1e0, &idx);
    stats->voltage_input = buffer_get_float16(msg, 1e1, &idx);
    stats->amp_hours = buffer_get_float32(msg, 1e4, &idx);
    stats->amp_hours_charged = buffer_get_float32(msg, 1e4, &idx);
    stats->watt_hours = buffer_get_float32(msg, 1e4, &idx);
    stats->watt_hours_charged = buffer_get_float32(msg, 1e4, &idx);
    stats->tachometer = buffer_get_int32(msg, &idx);
    stats->tachometer_abs = buffer_get_int32(msg, &idx);
    stats->fault_code = (uint16_t) msg[idx++];

    if (len - idx > 4) {
        stats->position = buffer_get_float32(msg, 1e6, &idx);
    } else {
        stats->position = -1.0;
    }

    return (0);
}

int vesc_read_stats(VescDriver *dev) {
    uint8_t msg[1] = { COMM_GET_VALUES };
    int rc;

    rc = send_payload(dev->uart_num, &msg[0], 1);
	assert(rc == 0);


	if (xSemaphoreTake(dev->m_read_evt, 500 / portTICK_PERIOD_MS) == pdFALSE) {
        return -2;
    }

	//rc = parse_stats_packet(itf->rx_data, &dev->stats, );

	return (rc);
}

int vesc_set_duty(VescDriver *dev, float duty_cycle) {
    uint8_t msg[5] = {0};
    int32_t idx = 0;
    int rc;

    msg[idx++] = COMM_SET_DUTY;
    buffer_append_int32(msg, (int32_t)(duty_cycle * 1e5), &idx);

    rc = send_payload(dev->uart_num, &msg[0], 5);

    return (rc);
}

int vesc_set_current(VescDriver *dev, float current) {
    uint8_t msg[5];
    int32_t idx = 0;
    int rc;

    msg[idx++] = COMM_SET_CURRENT;
    buffer_append_int32(msg, (int32_t)(current * 1e3), &idx);

    rc = send_payload(dev->uart_num, &msg[0], 5);

    return (rc);
}

int vesc_set_rpm(VescDriver *dev, int32_t rpm) {
    uint8_t msg[5];
    int32_t idx = 0;
    int rc;

    msg[idx++] = COMM_SET_RPM;
    buffer_append_int32(msg, rpm, &idx);

    rc = send_payload(dev->uart_num, &msg[0], 5);

    return (rc);
}

int vesc_set_position(VescDriver *dev, float position) {
    uint8_t msg[5];
    int32_t idx = 0;
    int rc;

    msg[idx++] = COMM_SET_POS;
    buffer_append_int32(msg, (int32_t)(position * 1e6), &idx);

    rc = send_payload(dev->uart_num, &msg[0], 5);

    return (rc);
}

void uart_event_task(void *pvParam) {
    VescDriver* dev = (VescDriver*)pvParam;
    dev->m_uart_buff = (uint8_t*) malloc(RD_BUF_SIZE);
    dev->m_uart_len = 0;
    uart_event_t event;
    uint16_t crc;
    uint16_t pkg_len;

    for(;;) {
        if(!xQueueReceive(dev->m_queue_uart, (void*)&event, (portTickType)portMAX_DELAY)) {
            continue;
        }

        switch(event.type) {
            case UART_DATA:
                uart_read_bytes(dev->uart_num, &dev->m_uart_buff[dev->m_uart_len], event.size, portMAX_DELAY);
                dev->m_uart_len += event.size;

                if (dev->m_uart_len > 2 && dev->m_uart_buff[0] == 2) {
                    pkg_len = dev->m_uart_buff[1] + 5;
                    if (pkg_len <= dev->m_uart_len) {
                        crc = crc16_be(0, dev->m_uart_buff, pkg_len);

                        if (crc == ((uint16_t) dev->m_uart_buff[pkg_len - 2] << 8 | (uint16_t) dev->m_uart_buff[pkg_len - 1])) {
                            xSemaphoreGive(dev->m_read_evt);
                            dev->m_uart_len = 0;
                            uart_flush_input(dev->uart_num);
                            xQueueReset(dev->m_queue_uart);
                        }
                    }
                }

                break;

            case UART_FIFO_OVF:
                uart_flush_input(dev->uart_num);
                xQueueReset(dev->m_queue_uart);
                break;

            case UART_BUFFER_FULL:
                uart_flush_input(dev->uart_num);
                xQueueReset(dev->m_queue_uart);
                break;

            default:
                break;
        }
    }

    free(dev->m_uart_buff);
    vTaskDelete(dev->m_conn);
    dev->m_conn = NULL;
}

int vesc_init(VescDriver *dev) {
    dev->m_read_evt = xSemaphoreCreateBinary();
    // const int uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(dev->uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(dev->uart_num, dev->pin_tx, dev->pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(dev->uart_num, 2048, 0, 0, &dev->m_queue_uart, 0));
    xTaskCreate(uart_event_task, "uart_event_task", 2048, (void*)dev, 12, &dev->m_conn);

    return 0;
}

