#include "radio_link.h"
#include <RadioLib.h>
#include "EspHal.h"
#include "esp_log.h"

static const char *TAG = "RADIO_LINK";

// ==========================================
// 1. CHOOSE YOUR RADIO MODULE HERE
// ==========================================
#define USE_NRF24
//#define USE_SX1281
// ==========================================

// --- SHARED SPI PINS ---
#define SPI_SCK  12
#define SPI_MISO 13
#define SPI_MOSI 11
#define RADIO_CSN 10
#define RADIO_IRQ 2

EspHal* hal = new EspHal(SPI_SCK, SPI_MISO, SPI_MOSI, SPI2_HOST);

#if defined(USE_NRF24)
    #define RADIO_CE 3
    nRF24 radio = new Module(hal, RADIO_CSN, RADIO_IRQ, RADIO_CE);
    uint8_t radio_address[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};

#elif defined(USE_SX1281)
    #define RADIO_RST  3
    #define RADIO_BUSY 4
    SX1281 radio = new Module(hal, RADIO_CSN, RADIO_IRQ, RADIO_RST, RADIO_BUSY);
#endif

extern "C" bool radio_init(void) {
    int state = RADIOLIB_ERR_NONE;

#if defined(USE_NRF24)
    state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        radio.setReceivePipe(0, radio_address);
        radio.setTransmitPipe(radio_address);
    }
#elif defined(USE_SX1281)
    // Initialize SX1281 in LoRa mode for maximum 2.4GHz range
    state = radio.begin(2450.0, 812.5, 7, 5, 10, 0); 
#endif

    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Radio Initialized Successfully!");
        return true;
    } else {
        ESP_LOGE(TAG, "Radio Initialization Failed, code %d", state);
        return false;
    }
}

extern "C" bool radio_receive_command(control_packet_t* cmd) {
    int state = radio.receive((uint8_t*)cmd, sizeof(control_packet_t));
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" bool radio_transmit_telemetry(telemetry_packet_t* telem) {
    int state = radio.transmit((uint8_t*)telem, sizeof(telemetry_packet_t));
    return (state == RADIOLIB_ERR_NONE);
}