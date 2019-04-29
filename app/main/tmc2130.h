/**************************************************************************\

    @file     tmc2130.h
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

#include "esp_system.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TMC2130_READ       0x00
#define TMC2130_WRITE      0x80

#define REG_GCONF          0x00
#define REG_GSTAT          0x01
#define REG_IOIN           0x04
#define REG_IHOLD_IRUN     0x10
#define REG_TPOWERDOWN     0x11
#define REG_TSTEP          0x12
#define REG_TPWMTHRS       0x13
#define REG_TCOOLTHRS      0x14
#define REG_THIGH          0x15
#define REG_XDIRECT        0x2D
#define REG_VDCMIN         0x33
#define REG_CHOPCONF       0x6C
#define REG_COOLCONF       0x6D
#define REG_DRV_STATUS     0x6F
#define REG_PWMCONF        0x70
#define REG_PWM_SCALE      0x71
#define REG_ENCM_CTRL      0x72
#define REG_LOST_STEPS     0x73

#define VSENSE_bm          0x20000UL
#define VSENSE_bp          17
#define IHOLD_bm           0x1FUL
#define IHOLD_bp           0
#define IRUN_bm            0x1F00UL
#define IRUN_bp            8
#define TOFF_bm            0xFUL
#define TOFF_bp            0
#define TBL_bm             0x18000UL
#define TBL_bp             15
#define EN_PWM_MODE_bm     0x4UL
#define EN_PWM_MODE_bp     2
#define MRES_bm            0xF000000UL
#define MRES_bp            24
#define DIAG1_STALL_bm     0x100UL
#define DIAG1_STALL_bp     8
#define DIAG1_PUSHPULL_bm  0x2000UL
#define DIAG1_PUSHPULL_bp  13
#define SGT_bm			   0x7F0000UL
#define SGT_bp			   16
#define SEMIN_bm           0xFUL
#define SEMIN_bp           0
#define SEMAX_bm           0xF00UL
#define SEMAX_bp           8
#define SEDN_bm            0x6000UL
#define SEDN_bp            13
#define HSTRT_bm           0x70UL
#define HSTRT_bp           4
#define HEND_bm            0x780UL
#define HEND_bp            7
#define SG_RESULT_bm       0x3FFUL
#define SG_RESULT_bp       0
#define CS_ACTUAL_bm       0x1F0000UL
#define CS_ACTUAL_bp       16
#define LOST_STEPS_bm      0xFFFFFUL
#define LOST_STEPS_bp      0

typedef struct loadMeasure {
    uint16_t current;
    uint16_t load;
} TMC2130Load;

void tmc2130_set_current(uint16_t mA, spi_device_handle_t* spi);
void tmc2130_set_stall_current(int8_t scale, spi_device_handle_t* spi);
void tmc2130_set_microsteps(uint16_t ms, spi_device_handle_t* spi);
void tmc2130_test(spi_device_handle_t* spi);
void tmc2130_init(spi_device_handle_t* spi);
void tmc2130_disable_stall(spi_device_handle_t* spi);
void tmc2130_enable_stall(spi_device_handle_t* spi);
uint32_t tmc2130_lost_steps(spi_device_handle_t* spi);
TMC2130Load tmc2130_measure(spi_device_handle_t* spi);

#ifdef __cplusplus
}
#endif
