#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int spi_sck;
    int spi_miso;
    int spi_mosi;
    int radio_csn;
    int radio_irq;
    int radio_ce_rst; 
    int radio_busy;   
} radio_config_t;

bool radio_init(radio_config_t* config);

// For the Flight-Controller
bool radio_receive_command(control_packet_t* cmd);
bool radio_transmit_telemetry(telemetry_packet_t* telem);

// For the Remote-Controller
bool radio_transmit_command(control_packet_t* cmd);
bool radio_receive_telemetry(telemetry_packet_t* telem);

#ifdef __cplusplus
}
#endif