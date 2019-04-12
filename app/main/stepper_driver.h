#pragma once
#include "esp_system.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    DIRECTION_CCW = 0,
    DIRECTION_CW = 1
} Direction;

typedef struct AxisDef {
    uint8_t pin_en;
    uint8_t pin_step;
    uint8_t pin_dir;
    uint8_t pin_cs;
    uint8_t pin_stall;

    float speed;
    float max_accel;
    uint16_t microsteps;
    float steps_per_mm;
    float deg_per_step;
    int8_t stall_level;

    // these are all internal variables
    uint16_t m_min_pulse_width;
    spi_device_handle_t m_interface;

    SemaphoreHandle_t m_stall;

    float m_speed_interval;
    float m_speed;
    float m_accel;
    float m_interval;
    int64_t m_last_time;
    Direction m_dir;
    float m_n;
    float m_c0;
    float m_cn;
    float m_cmin;
    bool m_enabled;
    bool m_should_step;

    int32_t m_current_pos;
    int32_t m_target_pos;
} StepperAxis;

void stepper_enable(StepperAxis* axis);
void stepper_enable_stall(StepperAxis* axis);
void stepper_disable_stall(StepperAxis* axis);
void stepper_disable(StepperAxis* axis);
void stepper_stop(StepperAxis* axis);
void stepper_set_speed(StepperAxis* axis, float speed);
void stepper_set_accel(StepperAxis* axis, float accel);
float stepper_get_accel(StepperAxis* axis);
void stepper_set_steps_per_mm(StepperAxis* axis, float steps);
void stepper_move(StepperAxis* axis, float position);
float stepper_position(StepperAxis* axis);
StepperAxis* stepper_from_code(char code);
void stepper_stall_level(StepperAxis* axis, int8_t stall_level);

void stepper_init(StepperAxis* axis);

void stepper_run();

// NOTE: this function doesnt move the motor,
// but stops it's movement and sets the internal counter
void stepper_set_position(StepperAxis* axis, int32_t position);
