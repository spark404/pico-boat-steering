#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/pwm.h"

volatile uint64_t steer_high = 0;
volatile int32_t steer_delta = 0;
volatile uint64_t throttle_high = 0;
volatile int32_t throttle_delta = 0;

uint32_t left_speed;
uint32_t right_speed;
uint8_t left_direction;
uint8_t right_direction;

long constrain(long x, long min, long max);
long map(long x, long in_min, long in_max, long out_min, long out_max);
long deadzone(long x, long center, long width);

#define GPIO_STEER 18
#define GPIO_THROTTLE 19
#define GPIO_PIN_LEFT_PWM 10
#define GPIO_PIN_LEFT_DIR 11
#define GPIO_PIN_RIGHT_PWM 12
#define GPIO_PIN_RIGHT_DIR 13

#define DELTA_MIN 1100
#define DELTA_MAX 1900
#define DELTA_NEUTRAL 1500
#define DELTA_DEADZONE 5

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
            throttle_high = time_us_64();
        } else {
            throttle_delta = (int32_t)(time_us_64() - throttle_high);
        }
    } else {
        if (event == GPIO_IRQ_EDGE_RISE) {
            steer_high = time_us_64();
        } else {
            steer_delta = (int32_t)(time_us_64() - steer_high);
        }
    }
}

/**
 * This function takes the measured pulse width in microseconds from
 * the Spektrum receiver and mixes the signal into target speed and
 * directions for the left and right engine.
 */
void mixer() {
    int throttle = constrain(throttle_delta, DELTA_MIN, DELTA_MAX);
    int steer = constrain(steer_delta, DELTA_MIN, DELTA_MAX);

    throttle = deadzone(throttle, DELTA_NEUTRAL, DELTA_DEADZONE);
    steer = deadzone(steer, DELTA_NEUTRAL, DELTA_DEADZONE);

    int throttle_value = map(throttle, DELTA_MIN, DELTA_MAX, -127, 127);
    int steer_value = map(steer, DELTA_MIN, DELTA_MAX, -127, 127);

    int turn_direction = steer_value < 0;
    int proportional_steer = map(steer_value, -127, 127, -throttle_value, throttle_value);

//    printf("input: throttle %ld, steer %ld\n", throttle_delta, steer_delta);
//    printf("mapped: throttle %d, steer %d\n", throttle_value, steer_value);
//    printf("steer: proportional %d, direction %d\n", proportional_steer, turn_direction);

    int target_speed_outside = abs(throttle_value);
    int target_direction_outside = throttle_value < 0 ? 1 : 0;
    int target_speed_inside = target_speed_outside - abs(proportional_steer);
    int target_direction_inside = target_direction_outside;
    if (target_speed_inside < 0) {
        target_direction_inside = !target_direction_inside;
        target_speed_inside = abs(target_speed_inside);
    }

    if (turn_direction) {
        left_speed = target_speed_inside;
        left_direction = target_direction_inside;
        right_speed = target_speed_outside;
        right_direction = target_direction_outside;
    } else {
        left_speed = target_speed_outside;
        left_direction = target_direction_outside;
        right_speed = target_speed_inside;
        right_direction = target_direction_inside;
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
    pwm_config_set_wrap(&left_config, 12499); // 125 Mhz / 10 / 12499 = 1000.08 hz

    uint left_slice_num = pwm_gpio_to_slice_num(GPIO_PIN_LEFT_PWM);
    pwm_init(left_slice_num, &left_config, false);

    uint right_slice_num = pwm_gpio_to_slice_num(GPIO_PIN_RIGHT_PWM);
    pwm_init(right_slice_num, &left_config, false);


    pwm_set_enabled(pwm_gpio_to_slice_num(GPIO_PIN_LEFT_PWM), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(GPIO_PIN_RIGHT_PWM), true);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for(;;) {
        mixer();
//        printf("targets: left speed %lu, direction %d, right speed %lu / direction %d\n",
//               left_speed, left_direction,
//               right_speed, right_direction);

        int left_pwm = map(left_speed, 0, 127, 0, 12499);
        int right_pwm = map(right_speed, 0, 127, 0, 12499);
        gpio_put(GPIO_PIN_RIGHT_DIR, right_direction);
        gpio_put(GPIO_PIN_LEFT_DIR, left_direction);
        pwm_set_gpio_level(GPIO_PIN_LEFT_PWM, left_pwm);
        pwm_set_gpio_level(GPIO_PIN_RIGHT_PWM, right_pwm);

//        printf("control: left pwm %d, direction %d, right pwm %d / direction %d\n",
//               left_pwm, left_direction,
//               right_pwm, right_direction);

        sleep_ms(20);
    }
#pragma clang diagnostic pop
}