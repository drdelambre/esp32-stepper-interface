#include "esp_system.h"
#include "esp_timer.h"
#include "pid.h"

void pid_init(PIDDef* def) {
    def->m_last_time = esp_timer_get_time() - def->sample_time;
    def->m_sum = def->min;

    pid_tune(def->kp, def->ki, def->kd, def->POn, def);
}

void pid_calculate(double current, double target, double* output, PIDDef* def) {
    int64_t now = esp_timer_get_time();
    int64_t diff = now - def->m_last_time;

    if (diff < def->sample_time) {
        return;
    }

    double error = target - current;
    double diff_input = current - def->m_last_input;
    def->m_sum += def->ki * error;

    if (def->POn != P_ON_E) {
        def->m_sum -= def->kp * diff_input;
    }

    if (def->m_sum > def->max) {
        def->m_sum = def->max;
    } else if (def->m_sum < def->min) {
        def->m_sum = def->min;
    }

    double out = 0;
    if (def->POn == P_ON_E) {
        out = def->kp * error;
    }

    out += def->m_sum - def->kd * diff_input;

    if (out > def->max) {
        out = def->max;
    } else if (out < def->min) {
        out = def->min;
    }

    *output = out;

    def->m_last_input = current;
    def->m_last_time = now;
}

void pid_tune(double kp, double ki, double kd, POMSwitch pon, PIDDef* def) {
    if (kp < 0 || ki < 0 || kd < 0) {
        return;
    }

    def->POn = pon;

    double sample_time_sec = def->sample_time / 1000000.0;

    def->kp = kp;
    def->ki = ki * sample_time_sec;
    def->kd = kd / sample_time_sec;
}

void pid_sample_rate(uint32_t sample_time, PIDDef* def) {
    double sample_time_ratio = (double)sample_time / (double)def->sample_time;

    def->ki *= sample_time_ratio;
    def->kd /= sample_time_ratio;

    def->sample_time = sample_time;
}
