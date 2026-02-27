#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rc_protocol.h" // Your shared structs

// This magic block tells the C++ compiler to expose these as standard C functions
#ifdef __cplusplus
extern "C" {
#endif

// Initialize the SX1280 radio over SPI
bool radio_init(void);

// For the Remote-Controller: Send command, wait for telemetry reply
bool radio_send_and_receive(control_packet_t* cmd, telemetry_packet_t* telem_reply);

// For the Flight-Controller: Listen for command, instantly send telemetry reply
bool radio_listen_and_reply(control_packet_t* cmd_out, telemetry_packet_t* telem_in);

#ifdef __cplusplus
}
#endif