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

// Create global pointers, but don't define the pins yet
EspHal* hal = nullptr;

#if defined(USE_NRF24)
    nRF24* radio = nullptr;
    uint8_t radio_address[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};
#elif defined(USE_SX1281)
    SX1281* radio = nullptr;
#endif

extern "C" bool radio_init(radio_config_t* config) {
    int state = RADIOLIB_ERR_NONE;

    // 1. Dynamically create the HAL using the pins passed from main.c
    hal = new EspHal(config->spi_sck, config->spi_miso, config->spi_mosi, SPI2_HOST);

    // 2. Dynamically create the Radio module
#if defined(USE_NRF24)
    Module* mod = new Module(hal, config->radio_csn, config->radio_irq, config->radio_ce_rst);
    radio = new nRF24(mod);
    
    state = radio->begin();
    if (state == RADIOLIB_ERR_NONE) {
        radio->setReceivePipe(0, radio_address);
        radio->setTransmitPipe(radio_address);
    }
#elif defined(USE_SX1281)
    Module* mod = new Module(hal, config->radio_csn, config->radio_irq, config->radio_ce_rst, config->radio_busy);
    radio = new SX1281(mod);
    
    // Initialize SX1281 in LoRa mode for maximum 2.4GHz range
    state = radio->begin(2450.0, 812.5, 7, 5, 10, 0); 
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
    if (!radio) return false; // Safety check
    int state = radio->receive((uint8_t*)cmd, sizeof(control_packet_t));
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" bool radio_transmit_telemetry(telemetry_packet_t* telem) {
    if (!radio) return false; // Safety check
    int state = radio->transmit((uint8_t*)telem, sizeof(telemetry_packet_t));
    return (state == RADIOLIB_ERR_NONE);
}