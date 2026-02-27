#include "radio_link.h"
#include <RadioLib.h>
#include "EspHal.h"
#include "esp_log.h"

static const char *TAG = "RADIO_DEBUG";

// ==========================================
// 1. CHOOSE YOUR RADIO MODULE HERE
// ==========================================
#define USE_NRF24
// ==========================================

EspHal* hal = nullptr;

#if defined(USE_NRF24)
    nRF24* radio = nullptr;
    uint8_t radio_address[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};
#endif

extern "C" bool radio_init(radio_config_t* config) {
    int state = RADIOLIB_ERR_NONE;

    hal = new EspHal(config->spi_sck, config->spi_miso, config->spi_mosi, SPI2_HOST);

#if defined(USE_NRF24)
    // Module* mod = new Module(hal, config->radio_csn, config->radio_irq, config->radio_ce_rst);
    Module* mod = new Module(hal, config->radio_csn, RADIOLIB_NC, config->radio_ce_rst);
    radio = new nRF24(mod);
    
    state = radio->begin();
    if (state == RADIOLIB_ERR_NONE) {
        // --- STABILIZATION SETTINGS FOR PA+LNA ---
        radio->setBitRate(250.0f);      // 250kbps = Best range/stability
        radio->setOutputPower(-12);    // Lower power to prevent blinding at close range

        radio->setAutoAck(false);
        
        radio->setReceivePipe(0, radio_address);
        radio->setTransmitPipe(radio_address);
        ESP_LOGI(TAG, "nRF24 Configured: 250kbps, -12dBm Power.");
    }
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
    if (!radio) return false;
    int state = radio->receive((uint8_t*)cmd, sizeof(control_packet_t));
    
    // Only log if something other than "No packet" (timeout) happens
    if (state != RADIOLIB_ERR_NONE && state != RADIOLIB_ERR_RX_TIMEOUT) {
        ESP_LOGW(TAG, "Receive Command Failed, code %d", state);
    }
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" bool radio_transmit_command(control_packet_t* cmd) {
    if (!radio) return false;
    int state = radio->transmit((uint8_t*)cmd, sizeof(control_packet_t), 0);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Transmit Command Failed! Code: %d", state);
    }
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" bool radio_transmit_telemetry(telemetry_packet_t* telem) {
    if (!radio) return false;
    int state = radio->transmit((uint8_t*)telem, sizeof(telemetry_packet_t), 0);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Transmit Telemetry Failed! Code: %d", state);
    }
    return (state == RADIOLIB_ERR_NONE);
}

extern "C" bool radio_receive_telemetry(telemetry_packet_t* telem) {
    if (!radio) return false;
    int state = radio->receive((uint8_t*)telem, sizeof(telemetry_packet_t));
    
    if (state != RADIOLIB_ERR_NONE && state != RADIOLIB_ERR_RX_TIMEOUT) {
        ESP_LOGW(TAG, "Receive Telemetry Failed, code %d", state);
    }
    return (state == RADIOLIB_ERR_NONE);
}