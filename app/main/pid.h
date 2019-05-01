#pragma once

#include "esp_system.h"

typedef enum {
    P_ON_E,
    P_ON_M
} POMSwitch;

typedef struct piddef {
    double min;
    double max;
    uint32_t sample_time;

    double kp;
    double ki;
    double kd;
    POMSwitch POn;

    int64_t m_last_time;
    double m_sum;
    double m_last_input;
} PIDDef;

void pid_init(PIDDef* def);
void pid_calculate(double current, double target, double* output, PIDDef* def);
void pid_tune(double kp, double ki, double kd, POMSwitch pon, PIDDef* def);
void pid_sample_rate(uint32_t sample_time, PIDDef* def);
void pid_pause(PIDDef* def);
void pid_resume(PIDDef* def);
