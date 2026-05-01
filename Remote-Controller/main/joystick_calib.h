#ifndef JOYSTICK_CALIB_H
#define JOYSTICK_CALIB_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

#define JOY_PITCH_CHAN      ADC_CHANNEL_9
#define JOY_YAW_CHAN        ADC_CHANNEL_4
#define JOY_ROLL_CHAN       ADC_CHANNEL_8
#define JOY_THROTTLE_CHAN   ADC_CHANNEL_5

typedef struct {
    int pitch_min, pitch_center, pitch_max;
    int yaw_min,   yaw_center,   yaw_max;
    int roll_min,  roll_center,  roll_max;
    int throttle_min, throttle_center, throttle_max;
    bool is_calibrated;
} joystick_calib_t;

// Expose the calibration data struct so main.c can use it for mapping
extern joystick_calib_t joy_calib;

// Pass in the ADC handle, whether to force a new calibration, and a pointer to your set_rgb function
void init_joystick_calibration(adc_oneshot_unit_handle_t adc_handle, bool force_calibration, void (*led_callback)(uint32_t, uint32_t, uint32_t));

#endif