#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdint.h>

// Initialize the MCPWM timer, operators, and generators for all actuators
void actuators_init(void);

// Setters for flight surfaces (Inputs: -32768 to 32767)
void actuator_set_pitch(int16_t pitch_raw);
void actuator_set_yaw(int16_t yaw_raw);
void actuator_set_roll(int16_t roll_raw);

// Setter for the motor ESC (Input: 0 to 255)
void actuator_set_throttle(uint8_t throttle_raw);

#endif // ACTUATOR_CONTROL_H