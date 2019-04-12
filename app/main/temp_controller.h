#pragma once

#include "pid.h"
#include "driver/spi_master.h"

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
    uint8_t m_enabled;
    double m_output;
    int64_t m_last_read;
    int64_t m_window_start;
} TempController;

void temp_init(TempController* con);
void temp_set_temp(TempController* con, double target);
double temp_read_temp(TempController* con);
void temp_disable(TempController* con);

