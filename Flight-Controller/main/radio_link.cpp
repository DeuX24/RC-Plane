#include "radio_link.h"
#include <RadioLib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Define your SX1280 pins here
#define RADIO_NSS   10
#define RADIO_DIO1  11
#define RADIO_RST   12
#define RADIO_BUSY  13

// Instantiate the radio object
SX1280 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);

extern "C" bool radio_init(void) {
    // 1. Initialize ESP32 SPI bus here (spi_bus_initialize...)
    
    // 2. Initialize RadioLib LoRa/FLRC modulation
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        // Set your frequency, bandwidth, and TX power here
        radio.setFrequency(2450.0);
        radio.setOutputPower(10);
        return true;
    }
    return false;
}

extern "C" bool radio_send_and_receive(control_packet_t* cmd, telemetry_packet_t* telem_reply) {
    // Transmit the 10-byte control packet
    int state = radio.transmit((uint8_t*)cmd, sizeof(control_packet_t));
    
    if (state == RADIOLIB_ERR_NONE) {
        // Immediately switch to receive mode with a short timeout (e.g., 50ms)
        state = radio.receive((uint8_t*)telem_reply, sizeof(telemetry_packet_t));
        if (state == RADIOLIB_ERR_NONE) {
            return true; // We sent a command and got telemetry back!
        }
    }
    return false;
}