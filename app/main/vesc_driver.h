/**************************************************************************\

    @file     vesc.h
    @author   drdelambre

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Nombot Inc (somethings.cooking)
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

#pragma once

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define RD_BUF_SIZE 1024*2

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
} COMM_PACKET_ID;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR
} vesc_fault_code;

typedef struct vesc_stats {
	float voltage_input;
	float temp_mosfet;
	float temp_motor;
	float current_motor;
	float current_in;
	float rpm;
	float duty_cycle_now;
	float amp_hours;
	float amp_hours_charged;
	float watt_hours;
	float watt_hours_charged;
	int32_t tachometer;
	int32_t tachometer_abs;
	float position;
	vesc_fault_code fault_code;
} VescStats;

typedef struct vesc_driver {
    int uart_num;

    uint8_t    pin_tx;
    uint8_t    pin_rx;
    VescStats  stats;

    QueueHandle_t m_queue_uart;
    TaskHandle_t  m_conn;
    SemaphoreHandle_t m_read_evt;

    uint8_t* m_uart_buff;
    uint16_t m_uart_len;
} VescDriver;

/**
 * Initialize the VESC driver
 *
 * @param dev  Pointer to the vesc_dev device descriptor
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_init(VescDriver *dev);

/**
 * Fetch VESC motor controller readouts
 *
 * @param dev  Pointer to the VescDriver device descriptor
 * @param stats  Pointer to a vesc_stats buffer
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_read_stats(VescDriver *dev);

/**
 * Set the motor duty cycle
 *
 * @param dev  Pointer to the VescDriver device descriptor
 * @param duty_cycle  The duty cycle TODO: figure out the range (drdelambre)
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_set_duty(VescDriver *dev, float duty_cycle);

/**
 * Set the motor current output
 *
 * @param dev  Pointer to the VescDriver device descriptor
 * @param current  The current TODO: figure out the range (drdelambre)
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_set_current(VescDriver *dev, float current);

/**
 * Set the motor rpm
 *
 * @param dev  Pointer to the VescDriver device descriptor
 * @param rpm  The rpm TODO: figure out the range (drdelambre)
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_set_rpm(VescDriver *dev, int32_t rpm);

/**
 * Set the motor position
 *
 * @param dev  Pointer to the VescDriver device descriptor
 * @param position  The position TODO: figure out the range (drdelambre)
 *
 * @return 0 on success, non-zero on failure
 */
int vesc_set_position(VescDriver *dev, float position);
