#include "actuator_control.h"
#include <stdio.h>
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "ACTUATOR";

// --- PIN ASSIGNMENTS ---
#define PIN_SERVO_ELEVATOR  15
#define PIN_SERVO_RUDDER    16
#define PIN_SERVO_LEFT_AILERON   14
#define PIN_SERVO_RIGHT_AILERON   17
#define PIN_MOTOR_ESC    10

// --- TIMING CONFIGURATION ---
// Servos usually use 500us to 2500us
#define SERVO_MIN_US     500   
#define SERVO_MAX_US     2500  
#define SERVO_NEUTRAL_US 1500  

// ESCs (Motors) usually use 1000us (Off) to 2000us (Max)
#define ESC_MIN_US       1000
#define ESC_MAX_US       2000

// Global handles for comparators
static mcpwm_cmpr_handle_t cmpr_pitch = NULL;
static mcpwm_cmpr_handle_t cmpr_yaw = NULL;
static mcpwm_cmpr_handle_t cmpr_left_aileron = NULL;
static mcpwm_cmpr_handle_t cmpr_right_aileron = NULL;
static mcpwm_cmpr_handle_t cmpr_throttle = NULL;

// Helper function to map values
static uint32_t map_value(long x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper to set up a single channel
static void setup_mcpwm_channel(mcpwm_timer_handle_t timer, int gpio_num, mcpwm_cmpr_handle_t *cmpr_handle_out, uint32_t initial_us) {
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmpr_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, cmpr_handle_out));

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_config = { .gen_gpio_num = gpio_num };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*cmpr_handle_out, initial_us));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmpr_handle_out, MCPWM_GEN_ACTION_LOW)));
}

void actuators_init(void) {
    ESP_LOGI(TAG, "Initializing 50Hz MCPWM Timer for Actuators...");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1us tick
        .period_ticks = 20000,    // 20ms period (50Hz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Setup all 4 channels on the same timer
    setup_mcpwm_channel(timer, PIN_SERVO_ELEVATOR, &cmpr_pitch, SERVO_NEUTRAL_US);
    setup_mcpwm_channel(timer, PIN_SERVO_RUDDER, &cmpr_yaw, SERVO_NEUTRAL_US);

    setup_mcpwm_channel(timer, PIN_SERVO_LEFT_AILERON, &cmpr_left_aileron, SERVO_NEUTRAL_US);
    setup_mcpwm_channel(timer, PIN_SERVO_RIGHT_AILERON, &cmpr_right_aileron, SERVO_NEUTRAL_US);
    
    // Safety: Initialize throttle to lowest setting (1000us) so the ESC doesn't spin the motor
    setup_mcpwm_channel(timer, PIN_MOTOR_ESC, &cmpr_throttle, ESC_MIN_US);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Actuators Initialized Successfully.");
}

void actuator_set_pitch(int16_t pitch_raw) {
    uint32_t pulse = map_value(pitch_raw, -32768, 32767, SERVO_MIN_US, SERVO_MAX_US);
    mcpwm_comparator_set_compare_value(cmpr_pitch, pulse);
}

void actuator_set_yaw(int16_t yaw_raw) {
    uint32_t pulse = map_value(yaw_raw, -32768, 32767, SERVO_MIN_US, SERVO_MAX_US);
    mcpwm_comparator_set_compare_value(cmpr_yaw, pulse);
}

void actuator_set_roll(int16_t roll_raw) {
    // Left aileron moves normally
    uint32_t pulse_left = map_value(roll_raw, -32768, 32767, SERVO_MIN_US, SERVO_MAX_US);
    
    // Right aileron moves in the opposite direction (Differential)
    // We invert the output range: min input maps to max pulse
    uint32_t pulse_right = map_value(roll_raw, -32768, 32767, SERVO_MAX_US, SERVO_MIN_US);
    
    mcpwm_comparator_set_compare_value(cmpr_left_aileron, pulse_left);
    mcpwm_comparator_set_compare_value(cmpr_right_aileron, pulse_right);
}

void actuator_set_throttle(uint8_t throttle_raw) {
    uint32_t pulse = map_value(throttle_raw, 0, 255, ESC_MIN_US, ESC_MAX_US);
    mcpwm_comparator_set_compare_value(cmpr_throttle, pulse);
}