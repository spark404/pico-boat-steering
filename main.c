#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/pwm.h"

volatile uint64_t steer_pulse_start = 0;
volatile int32_t steer_pulse_us = 0;
volatile uint64_t throttle_pulse_start = 0;
volatile int32_t throttle_pulse_us = 0;

typedef struct {
    uint32_t speed;
    uint8_t direction;
} track_control_t;

typedef struct {
    track_control_t left;
    track_control_t right;
} control_t;

long constrain(long x, long min, long max);
long map(long x, long in_min, long in_max, long out_min, long out_max);
long deadzone(long x, long center, long width);

#define GPIO_STEER 18
#define GPIO_THROTTLE 19
#define GPIO_PIN_LEFT_PWM 10
#define GPIO_PIN_LEFT_DIR 11
#define GPIO_PIN_RIGHT_PWM 12
#define GPIO_PIN_RIGHT_DIR 13

// Typical settings for Spektrum servo pulse width in microseconds
#define MIN_US 1100
#define MAX_US 1900
#define NEUTRAL_US 1500
#define DEADZONE_US 20

#define PWM_WRAP 1249

/**
 * Callback for GPIO pin events. On rising edge the current time
 * in microseconds is recorded. On falling edge the delta is
 * calculated and the result is stored as the length of the pulse.
 *
 * @param gpio
 * @param event
 */
void gpio_callback(uint gpio, uint32_t event) {
    if (gpio == GPIO_THROTTLE) {
        if (event == GPIO_IRQ_EDGE_RISE) {
            throttle_pulse_start = time_us_64();
        } else {
            throttle_pulse_us = (int32_t)(time_us_64() - throttle_pulse_start);
        }
    } else {
        if (event == GPIO_IRQ_EDGE_RISE) {
            steer_pulse_start = time_us_64();
        } else {
            steer_pulse_us = (int32_t)(time_us_64() - steer_pulse_start);
        }
    }
}

/**
 * This function takes the measured pulse width in microseconds from
 * the Spektrum receiver and mixes the signal into target speed and
 * directions for the left and right engine.
 */
void mixer(int32_t throttle_us, int32_t steer_us, control_t *control) {
    int throttle = constrain(throttle_us, MIN_US, MAX_US);
    int steer = constrain(steer_us, MIN_US, MAX_US);

    throttle = deadzone(throttle, NEUTRAL_US, DEADZONE_US);
    steer = deadzone(steer, NEUTRAL_US, DEADZONE_US);

    int throttle_value = map(throttle, MIN_US, MAX_US, -127, 127);
    int steer_value = map(steer, MIN_US, MAX_US, -127, 127);

    int turn_direction = steer_value < 0;
    int proportional_steer = map(steer_value, -127, 127, -throttle_value, throttle_value);

#ifdef PICO_DEBUG
    printf("input: throttle %ld, steer %ld\n", throttle_us, steer_us);
    printf("mapped: throttle %d, steer %d\n", throttle_value, steer_value);
    printf("steer: proportional %d, direction %d\n", proportional_steer, turn_direction);
#endif

    int target_speed_outside = abs(throttle_value);
    int target_direction_outside = throttle_value < 0 ? 1 : 0;
    int target_speed_inside = target_speed_outside - abs(proportional_steer);
    int target_direction_inside = target_direction_outside;
    if (target_speed_inside < 0) {
        target_direction_inside = !target_direction_inside;
        target_speed_inside = abs(target_speed_inside);
    }

    if (turn_direction) {
        control->left.speed = target_speed_inside;
        control->left.direction = target_direction_inside;
        control->right.speed = target_speed_outside;
        control->right.direction = target_direction_outside;
    } else {
        control->left.speed = target_speed_outside;
        control->left.direction = target_direction_outside;
        control->right.speed = target_speed_inside;
        control->right.direction = target_direction_inside;
    }
}

long deadzone(long x, long center, long width) {
    return abs(x-center) > width ? x : center;
}

long constrain(long x, long min, long max) {
    return x < min ? min : x > max ? max : x;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    bi_decl(bi_program_description("Interprets pwm signals from a radio receiver and translates them to pwm and direction signals for moto"));
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "On-board LED"));
    bi_decl(bi_2pins_with_names(GPIO_PIN_LEFT_DIR, "Skid Left Direction", GPIO_PIN_LEFT_PWM, "Skid Left PWM"));
    bi_decl(bi_2pins_with_names(GPIO_PIN_RIGHT_PWM, "Skid Right Direction", GPIO_PIN_RIGHT_PWM, "Skid Right PWM"));

    stdio_init_all();

    // Init the built in LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // Init the servo inputs
    gpio_init(GPIO_STEER);
    gpio_set_dir(GPIO_STEER, false);
    gpio_pull_down(GPIO_STEER);
    gpio_set_irq_enabled_with_callback(GPIO_STEER, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(GPIO_THROTTLE);
    gpio_set_dir(GPIO_THROTTLE, false);
    gpio_pull_down(GPIO_THROTTLE);
    gpio_set_irq_enabled(GPIO_THROTTLE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Configure the output pins
    gpio_init(GPIO_PIN_LEFT_DIR);
    gpio_set_dir(GPIO_PIN_LEFT_DIR, true);
    gpio_init(GPIO_PIN_RIGHT_DIR);
    gpio_set_dir(GPIO_PIN_RIGHT_DIR, true);
    gpio_set_function(GPIO_PIN_LEFT_PWM, GPIO_FUNC_PWM);
    gpio_set_function(GPIO_PIN_RIGHT_PWM, GPIO_FUNC_PWM);

    // Set PWM config to create a 10kHz pwm signal
    pwm_config left_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&left_config, 10);
    pwm_config_set_clkdiv_mode(&left_config, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&left_config, PWM_WRAP); // 125 Mhz / 10 / 1249 = 10.000,08 hz

    uint left_slice_num = pwm_gpio_to_slice_num(GPIO_PIN_LEFT_PWM);
    pwm_init(left_slice_num, &left_config, false);

    uint right_slice_num = pwm_gpio_to_slice_num(GPIO_PIN_RIGHT_PWM);
    pwm_init(right_slice_num, &left_config, false);

    pwm_set_enabled(pwm_gpio_to_slice_num(GPIO_PIN_LEFT_PWM), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(GPIO_PIN_RIGHT_PWM), true);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    control_t control;
    int left_error = 0;
    int left_setpoint = 0;
    int right_error = 0;
    int right_setpoint = 0;

    for(;;) {
        mixer(throttle_pulse_us, steer_pulse_us, &control);

#ifdef PICO_DEBUG
        printf("targets: left speed %lu, direction %d, right speed %lu / direction %d\n",
               control.left.speed, control.left.direction,
               control.right.speed, control.right.direction);
#endif

        int left_pwm = map(control.left.speed, 0, 127, 0, PWM_WRAP);
        int right_pwm = map(control.right.speed, 0, 127, 0, PWM_WRAP);

        left_error = (left_pwm * (control.left.direction ? 1 : -1)) - left_setpoint;
        if (abs(left_error) > 50) {
            left_error = 50 * (left_error < 0 ? -1 : 1);
        }
        left_setpoint += left_error;

        right_error = (right_pwm * (control.right.direction ? 1 : -1)) - right_setpoint;
        if (abs(right_error) > 50) {
            right_error = 50 * (right_error < 0 ? -1 : 1);
        }
        right_setpoint += right_error;

#ifdef PICO_DEBUG
        printf("setpoint: left %d, error %d, right %d / error %d\n",
               left_setpoint, left_error,
               right_setpoint, right_error);
#endif

        gpio_put(GPIO_PIN_LEFT_DIR, left_setpoint < 0 ? 0 : 1);
        gpio_put(GPIO_PIN_RIGHT_DIR, right_setpoint < 0 ? 0 : 1);
        pwm_set_gpio_level(GPIO_PIN_LEFT_PWM, abs(left_setpoint) > 1240 ? 1250 : abs(left_setpoint));
        pwm_set_gpio_level(GPIO_PIN_RIGHT_PWM, abs(right_setpoint)> 1240 ? 1250 : abs(right_setpoint));

#ifdef PICO_DEBUG
        printf("control: left pwm %d, direction %d, right pwm %d / direction %d\n",
               left_pwm, control.left.direction,
               right_pwm, control.right.direction);
#endif
        
        sleep_ms(20);
    }
#pragma clang diagnostic pop
}