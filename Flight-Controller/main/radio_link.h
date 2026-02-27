#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

bool radio_init(void);
bool radio_receive_command(control_packet_t* cmd);
bool radio_transmit_telemetry(telemetry_packet_t* telem);

#ifdef __cplusplus
}
#endif