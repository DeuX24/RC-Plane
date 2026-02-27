#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Struct to pass pin definitions from the specific board
typedef struct {
    int spi_sck;
    int spi_miso;
    int spi_mosi;
    int radio_csn;
    int radio_irq;
    int radio_ce_rst; // CE pin for nRF24, RST pin for SX1281
    int radio_busy;   // Only used by SX1281 (can be set to -1 for nRF24)
} radio_config_t;

// Pass the config struct into the init function
bool radio_init(radio_config_t* config);

bool radio_receive_command(control_packet_t* cmd);
bool radio_transmit_telemetry(telemetry_packet_t* telem);

#ifdef __cplusplus
}
#endif