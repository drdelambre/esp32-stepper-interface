/**************************************************************************\

    @file     temp_controller.h
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

#pragma once

#include "pid.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct temp_con {
    uint8_t pin_cs;
    uint8_t pin_relay;

    uint16_t window_size;     // Time Proportional Output window

    PIDDef* pid;

    double m_target;            // target temperature
    double m_current;           // current temperature
    double m_target_rate;       // max rate of increase in degC/sec
    double m_current_rate;      // measured rate of increase in degC/sec

    spi_device_handle_t m_interface;
    SemaphoreHandle_t m_change;
    bool m_active;
    uint8_t m_enabled;
    double m_output;
    int64_t m_last_read;
    int64_t m_window_start;
} TempController;

void temp_init(TempController* con);
void temp_set_temp(TempController* con, double target);
double temp_read_temp(TempController* con);
void temp_disable(TempController* con);

#ifdef __cplusplus
}
#endif
