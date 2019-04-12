#include "tmc2130.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>
#include <esp_err.h>
#include "spi.h"

static const char *TAG = "TMC2130";
static uint32_t GCONF_sr = 0x00000000UL,
                IHOLD_IRUN_sr   = 0x00000000UL,
                TPOWERDOWN_sr   = 0x00000000UL,
                TPWMTHRS_sr     = 0x00000000UL,
                TCOOLTHRS_sr    = 0x00000000UL,
                THIGH_sr        = 0x00000000UL,
                XDIRECT_sr      = 0x00000000UL,
                VDCMIN_sr       = 0x00000000UL,
                CHOPCONF_sr     = 0x00000000UL,
                COOLCONF_sr     = 0x00000000UL,
                PWMCONF_sr      = 0x00050480UL,
                ENCM_CTRL_sr    = 0x00000000UL;

void send2130(uint8_t addressByte, uint32_t *config, spi_device_handle_t *spi) {
    if (addressByte >> 7) { // Check if WRITE command
        uint8_t sender[4];
        sender[0] = (*config >> 24) & 0xFF;
        sender[1] = (*config >> 16) & 0xFF;
        sender[2] = (*config >>  8) & 0xFF;
        sender[3] = *config & 0xFF;

        spi_send_addressed(addressByte, sender, spi);
    } else { // READ command
        uint8_t buff[4];

        spi_read_addressed(addressByte, buff, spi);
        spi_read_addressed(addressByte, buff, spi);

        *config = buff[0];
        *config <<= 8;
        *config |= buff[1];
        *config <<= 8;
        *config |= buff[2];
        *config <<= 8;
        *config |= buff[3];
    }
}

void tmc2130_push(spi_device_handle_t* spi) {
    send2130(TMC2130_WRITE|REG_GCONF, &GCONF_sr, spi);
    send2130(TMC2130_WRITE|REG_IHOLD_IRUN, &IHOLD_IRUN_sr, spi);
    send2130(TMC2130_WRITE|REG_TPOWERDOWN, &TPOWERDOWN_sr, spi);
    send2130(TMC2130_WRITE|REG_TPWMTHRS, &TPWMTHRS_sr, spi);
    send2130(TMC2130_WRITE|REG_TCOOLTHRS, &TCOOLTHRS_sr, spi);
    send2130(TMC2130_WRITE|REG_THIGH, &THIGH_sr, spi);
    send2130(TMC2130_WRITE|REG_XDIRECT, &XDIRECT_sr, spi);
    send2130(TMC2130_WRITE|REG_VDCMIN, &VDCMIN_sr, spi);
    send2130(TMC2130_WRITE|REG_CHOPCONF, &CHOPCONF_sr, spi);
    send2130(TMC2130_WRITE|REG_COOLCONF, &COOLCONF_sr, spi);
    send2130(TMC2130_WRITE|REG_PWMCONF, &PWMCONF_sr, spi);
    send2130(TMC2130_WRITE|REG_ENCM_CTRL, &ENCM_CTRL_sr, spi);
}

void tmc2130_set_current(uint16_t mA, spi_device_handle_t* spi) {
    float multiplier = 0.5,
          RS = 0.11;
    uint8_t CS = 32.0*1.41421*mA/1000.0*(RS+0.02)/0.325 - 1;
    if (CS < 16) {
        CHOPCONF_sr &= ~VSENSE_bm;
        CHOPCONF_sr |= ((uint32_t)true<<VSENSE_bp)&VSENSE_bm;
        send2130(TMC2130_WRITE|REG_CHOPCONF, &CHOPCONF_sr, spi);
        CS = 32.0*1.41421*mA/1000.0*(RS+0.02)/0.180 - 1;
    } else if((bool)((CHOPCONF_sr&VSENSE_bm) >> VSENSE_bp)) { // If CS >= 16, turn off high_sense_r if it's currently ON
        CHOPCONF_sr &= ~VSENSE_bm;
        CHOPCONF_sr |= ((uint32_t)false<<VSENSE_bp)&VSENSE_bm;
        send2130(TMC2130_WRITE|REG_CHOPCONF, &CHOPCONF_sr, spi);
    }
    IHOLD_IRUN_sr &= ~IRUN_bm;
    IHOLD_IRUN_sr |= ((uint32_t)CS<<IRUN_bp)&IRUN_bm;
    IHOLD_IRUN_sr &= ~IHOLD_bm;
    IHOLD_IRUN_sr |= ((uint32_t)(CS*multiplier)<<IHOLD_bp)&IHOLD_bm;
    send2130(TMC2130_WRITE|REG_IHOLD_IRUN, &IHOLD_IRUN_sr, spi);
}

void tmc2130_set_stall_current(int8_t mA, spi_device_handle_t* spi) {
    COOLCONF_sr &= ~SGT_bm;
    COOLCONF_sr |= ((uint32_t)mA<<SGT_bp)&SGT_bm;
    send2130(TMC2130_WRITE|REG_COOLCONF, &COOLCONF_sr, spi);
    uint32_t beans;
    send2130(TMC2130_READ|REG_CHOPCONF, &beans, spi);
}

void tmc2130_set_microsteps(uint16_t ms, spi_device_handle_t* spi) {
    uint8_t res;

    switch(ms) {
        case 256: res = 0; break;
        case 128: res = 1; break;
        case  64: res = 2; break;
        case  32: res = 3; break;
        case  16: res = 4; break;
        case   8: res = 5; break;
        case   4: res = 6; break;
        case   2: res = 7; break;
        case   0: res = 8; break;
        default: return;
    }

    CHOPCONF_sr &= ~MRES_bm;
    CHOPCONF_sr |= ((uint32_t)res<<MRES_bp)&MRES_bm;
    send2130(TMC2130_WRITE|REG_CHOPCONF, &CHOPCONF_sr, spi);
}

uint16_t rms_current(uint8_t CS) {
  return (float)(CS+1)/32.0 * (true?0.180:0.325)/(0.11+0.02) / 1.41421 * 1000;
}

void tmc2130_test(spi_device_handle_t* spi) {
    uint32_t drv_status;
    send2130(TMC2130_READ|REG_DRV_STATUS, &drv_status, spi);

    ESP_LOGI(TAG, "Driver Status:    %08x\n"
            "    standstill .......... %s\n"
            "    openload on phase A . %s\n"
            "    openload on phase B . %s\n"
            "    short on phase A .... %s\n"
            "    short on phase B .... %s\n"
            "    overtemp warning .... %s\n"
            "    overtemp error ...... %s\n"
            "    stall detected ...... %s\n\n"

            "    motor current: %d\n"
            "    stall load: %d",
            drv_status,
            (drv_status >> 31 & 0x1) ? "true" : "false",
            (drv_status >> 29 & 0x1) ? "true" : "false",
            (drv_status >> 30 & 0x1) ? "true" : "false",
            (drv_status >> 27 & 0x1) ? "true" : "false",
            (drv_status >> 28 & 0x1) ? "true" : "false",
            (drv_status >> 26 & 0x1) ? "true" : "false",
            (drv_status >> 25 & 0x1) ? "true" : "false",
            (drv_status >> 24 & 0x1) ? "true" : "false",
            (int)(int)rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp),
            (int)(drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
}

void tmc2130_init(spi_device_handle_t* spi) {
    //toff(3);
    CHOPCONF_sr &= ~TOFF_bm;
    CHOPCONF_sr |= ((uint32_t)3<<TOFF_bp)&TOFF_bm;

    //tbl(1);
    CHOPCONF_sr &= ~TBL_bm;
    CHOPCONF_sr |= ((uint32_t)1<<TBL_bp)&TBL_bm;

    // driver.hysteresis_start(4);
    CHOPCONF_sr &= ~HSTRT_bm;
    CHOPCONF_sr |= ((uint32_t)3<<HSTRT_bp)&HSTRT_bm;

    // driver.hysteresis_end(-2);
    CHOPCONF_sr &= ~HEND_bm;
    CHOPCONF_sr |= ((uint32_t)1<<HEND_bp)&HEND_bm;

    //enable stealthchop
    //GCONF_sr &= ~EN_PWM_MODE_bm;
    //GCONF_sr |= ((uint32_t)true<<EN_PWM_MODE_bp)&EN_PWM_MODE_bm;

    // configure stall detection
    // GCONF_sr &= ~DIAG1_STALL_bm;
    // GCONF_sr |= ((uint32_t)1<<DIAG1_STALL_bp)&DIAG1_STALL_bm;

    GCONF_sr &= ~DIAG1_PUSHPULL_bm;
    GCONF_sr |= ((uint32_t)1<<DIAG1_PUSHPULL_bp)&DIAG1_PUSHPULL_bm;

    // TCOOLTHRS_sr = (uint32_t)0xFFFFF;
    //driver.THIGH(0);
    THIGH_sr = (uint32_t)0;

    //driver.semin(5);
    COOLCONF_sr &= ~SEMIN_bm;
    COOLCONF_sr |= ((uint32_t)5<<SEMIN_bp)&SEMIN_bm;

    //driver.semax(2);
    COOLCONF_sr &= ~SEMAX_bm;
    COOLCONF_sr |= ((uint32_t)2<<SEMAX_bp)&SEMAX_bm;

    //driver.sedn(0b01);
    COOLCONF_sr &= ~SEDN_bm;
    COOLCONF_sr |= ((uint32_t)1<<SEDN_bp)&SEDN_bm;

    tmc2130_push(spi);

    uint32_t gstat;
    send2130(TMC2130_READ|REG_GSTAT, &gstat, spi);
}

void tmc2130_disable_stall(spi_device_handle_t* spi) {
    TCOOLTHRS_sr = (uint32_t)0x00000;
    send2130(TMC2130_WRITE|REG_TCOOLTHRS, &TCOOLTHRS_sr, spi);
    GCONF_sr &= ~EN_PWM_MODE_bm;
    GCONF_sr |= ((uint32_t)true<<EN_PWM_MODE_bp)&EN_PWM_MODE_bm;
    GCONF_sr &= ~DIAG1_STALL_bm;
    GCONF_sr |= ((uint32_t)0<<DIAG1_STALL_bp)&DIAG1_STALL_bm;
    send2130(TMC2130_WRITE|REG_GCONF, &GCONF_sr, spi);
}

void tmc2130_enable_stall(spi_device_handle_t* spi) {
    TCOOLTHRS_sr = (uint32_t)0xFFFFF;
    send2130(TMC2130_WRITE|REG_TCOOLTHRS, &TCOOLTHRS_sr, spi);
    GCONF_sr &= ~EN_PWM_MODE_bm;
    GCONF_sr |= ((uint32_t)false<<EN_PWM_MODE_bp)&EN_PWM_MODE_bm;
    GCONF_sr &= ~DIAG1_STALL_bm;
    GCONF_sr |= ((uint32_t)1<<DIAG1_STALL_bp)&DIAG1_STALL_bm;
    send2130(TMC2130_WRITE|REG_GCONF, &GCONF_sr, spi);
}

uint32_t tmc2130_lost_steps(spi_device_handle_t* spi) {
    uint32_t ret;
    send2130(TMC2130_READ|REG_LOST_STEPS, &ret, spi);

    return ret;
}

TMC2130Load tmc2130_measure(spi_device_handle_t* spi) {
    uint32_t drv_status;
    send2130(TMC2130_READ|REG_DRV_STATUS, &drv_status, spi);
    TMC2130Load value = {
        .current = (int)(int)rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp),
        .load = (int)(drv_status & SG_RESULT_bm)>>SG_RESULT_bp
    };

    return value;
}
