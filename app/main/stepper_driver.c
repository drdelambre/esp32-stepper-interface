#include <string.h>
#include "stepper_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "tmc2130.h"
#include "spi.h"

#include "driver/gpio.h"

#include "gll.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define NOP() asm volatile ("nop")
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define max(a,b) ((a)>(b)?(a):(b))

static gll_t m_axii;
static xQueueHandle gpio_evt_queue = NULL;

void computeNewSpeed(StepperAxis *axis) {
    int32_t distance = axis->m_target_pos - axis->m_current_pos;
    int32_t stepsLeft = (int32_t)((axis->m_speed * axis->m_speed) / (2.0 * axis->m_accel));

    if (distance == 0 && stepsLeft <= 1) {
        axis->m_interval = 0;
        axis->m_speed = 0.0;
        axis->m_n = 0;

        xSemaphoreGive(axis->m_stall);

        return;
    }

    if (distance > 0) {
        if (axis->m_n > 0) {
            if ((stepsLeft >= distance) || axis->m_dir == DIRECTION_CCW) {
                axis->m_n = -stepsLeft;
            }
        } else if (axis->m_n < 0) {
            if ((stepsLeft < distance) && axis->m_dir == DIRECTION_CW) {
                axis->m_n = -axis->m_n;
            }
        }
    } else if (distance < 0) {
        if (axis->m_n > 0) {
            if ((stepsLeft >= -distance) || axis->m_dir == DIRECTION_CW) {
                axis->m_n = -stepsLeft;
            }
        } else if (axis->m_n < 0) {
            if ((stepsLeft < -distance) && axis->m_dir == DIRECTION_CCW) {
                axis->m_n = -axis->m_n;
            }
        }
    }

    if (axis->m_n == 0.0) {
        axis->m_cn = axis->m_c0;
        axis->m_dir = (distance > 0) ? DIRECTION_CW : DIRECTION_CCW;
    } else {
        axis->m_cn = axis->m_cn - ((2.0 * axis->m_cn) / ((4.0 * axis->m_n) + 1)); // Equation 13
        axis->m_cn = max(axis->m_cn, axis->m_cmin);
    }

    axis->m_n++;
    axis->m_interval = axis->m_cn;
    axis->m_speed = 1000000.0 / axis->m_cn;

    if (axis->m_dir == DIRECTION_CCW) {
        axis->m_speed = -axis->m_speed;
    }
}

void m_run(StepperAxis* axis) {
    int64_t time = esp_timer_get_time();

    if (!axis->m_interval) {
        return false;
    }

    time = esp_timer_get_time();

    if (time - axis->m_last_time < axis->m_interval) {
        return false;
    }

    if (time - axis->m_last_time < axis->m_speed_interval) {
        return false;
    }

    axis->m_should_step = true;
    axis->m_last_time = time;

    computeNewSpeed(axis);
}

void stepper_run() {
    gll_each(&m_axii, (void *) m_run);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void process_stall(void* arg) {
    uint32_t io_num;
    gll_node_t *curr;
    uint8_t ni;
    StepperAxis *axis;

    for(;;) {
        if(!xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            continue;
        }

        if(gpio_get_level(io_num) != 1) {
            continue;
        }

        curr = m_axii.first;
        ni = 0;

        while (ni < m_axii.size) {
            axis = (StepperAxis*)curr->data;

            if (axis->pin_stall == io_num) {
                xSemaphoreGive(axis->m_stall);
            }

            curr = curr->next;
            ni++;
        }
    }
}

void m_step_callback(StepperAxis *axis) {
    if (!axis->m_should_step) {
        return;
    }

    axis->m_should_step = false;

    if (axis->m_dir == DIRECTION_CW) {
        axis->m_current_pos += 1;
    } else {
        axis->m_current_pos -= 1;
    }

    gpio_set_level(axis->pin_dir, (axis->m_dir == DIRECTION_CW) ? 1 : 0);
    gpio_set_level(axis->pin_step, 1);
    int32_t timer = esp_timer_get_time() + axis->m_min_pulse_width;
    while (esp_timer_get_time() < timer) {
        NOP();
    }

    gpio_set_level(axis->pin_step, 0);
}

static void step_callback(void* arg) {
    gll_each(&m_axii, (void *) m_step_callback);
}

void stepper_stop(StepperAxis* axis) {
    axis->m_target_pos = axis->m_current_pos;
    axis->m_n = 0;
    axis->m_interval = 0;
}

void stepper_move(StepperAxis* axis, float position) {
    axis->m_target_pos = axis->steps_per_mm * axis->microsteps * position;

    if (!axis->m_enabled) {
        gpio_set_level(axis->pin_en, 0);
        axis->m_enabled = true;
    }

    computeNewSpeed(axis);
}

void stepper_enable(StepperAxis* axis) {
    if (axis->m_enabled) {
        return;
    }

    gpio_set_level(axis->pin_en, 0);
    axis->m_enabled = true;
}

void stepper_enable_stall(StepperAxis* axis) {
    tmc2130_enable_stall(&axis->m_interface);
}

void stepper_disable_stall(StepperAxis* axis) {
    tmc2130_disable_stall(&axis->m_interface);
}

void stepper_disable(StepperAxis* axis) {
    if (!axis->m_enabled) {
        return;
    }

    gpio_set_level(axis->pin_en, 1);
    axis->m_enabled = false;
}

void stepper_set_position(StepperAxis* axis, int32_t position) {
    axis->m_target_pos = axis->m_current_pos = position;
    axis->m_n = 0;
    axis->m_interval = 0;
}

void stepper_set_speed(StepperAxis* axis, float speed) {
    axis->speed = speed;
    // convert to microseconds per microstep
    axis->m_speed_interval = 1000000.0 / (speed * axis->microsteps * axis->steps_per_mm);
}

void stepper_set_accel(StepperAxis* axis, float accel) {
    if (accel == 0.0) {
        return;
    }

    if (accel < 0.0) {
        accel = -accel;
    }

    accel = constrain(accel, 0.0, axis->max_accel);

    if (axis->m_accel == accel) {
        return;
    }

    axis->m_n = axis->m_n * (axis->m_accel / accel);
    axis->m_c0 = 0.676 * sqrt(2.0 / accel) * 1000000.0;
    axis->m_accel = accel;
    computeNewSpeed(axis);
}

float stepper_get_accel(StepperAxis* axis) {
    return axis->m_accel;
}

void stepper_set_steps_per_mm(StepperAxis* axis, float steps) {
    axis->steps_per_mm = steps;
    axis->m_speed_interval = 1000000.0 / (axis->speed * axis->microsteps * axis->steps_per_mm);
}

float stepper_position(StepperAxis* axis) {
    return axis->m_current_pos / (axis->steps_per_mm * axis->microsteps);
}

void stepper_stall_level(StepperAxis* axis, int8_t stall_level) {
    axis->stall_level = stall_level;
    tmc2130_set_stall_current(axis->stall_level, &axis->m_interface);
}

void stepper_init(StepperAxis *axis) {
    gll_pushBack(&m_axii, axis);

    // set up pins, set enable high
    axis->m_enabled = false;
    axis->m_cmin = 1.0;
    axis->m_min_pulse_width = 10;
    axis->m_last_time = 0;

    axis->m_stall = xSemaphoreCreateBinary();

    gpio_pad_select_gpio(axis->pin_en);
    gpio_pad_select_gpio(axis->pin_step);
    gpio_pad_select_gpio(axis->pin_dir);
    gpio_pad_select_gpio(axis->pin_stall);

    gpio_set_direction(axis->pin_en, GPIO_MODE_OUTPUT);
    gpio_set_direction(axis->pin_step, GPIO_MODE_OUTPUT);
    gpio_set_direction(axis->pin_dir, GPIO_MODE_OUTPUT);
    gpio_set_direction(axis->pin_stall, GPIO_MODE_INPUT);

    if (!gpio_evt_queue) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        xTaskCreate(process_stall, "stall_detector", 2048, NULL, 10, NULL);

        const esp_timer_create_args_t periodic_timer_args = {
            .callback = &step_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "step_timer"
        };

        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 100));
    }

    gpio_set_intr_type(axis->pin_stall, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(axis->pin_stall, gpio_isr_handler, (void*) axis->pin_stall);

    gpio_set_level(axis->pin_en, 1);

    stepper_set_position(axis, 0);

    gpio_set_level(axis->pin_step, 0);

    // initialize device
    spi_device_interface_config_t devcfg={
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .flags = SPI_TRANS_VARIABLE_ADDR,
        .clock_speed_hz = 2000000,
        .mode = 3,                               //SPI mode 3
        .spics_io_num = axis->pin_cs,            //CS pin
        .queue_size = 7                          //We want to be able to queue 7 transactions at a time
    };

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &axis->m_interface));

    tmc2130_init(&axis->m_interface);
    tmc2130_set_current(1000, &axis->m_interface);
    tmc2130_set_microsteps(axis->microsteps, &axis->m_interface);
    tmc2130_set_stall_current(axis->stall_level, &axis->m_interface);

    stepper_set_speed(axis, axis->speed);
    stepper_set_accel(axis, axis->max_accel);
}
