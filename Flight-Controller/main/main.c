#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "rc_protocol.h"
#include "actuator_control.h"
#include "udp_logger.h"

static const char *TAG = "PLANE_MAIN";

// Helper: Maps 9.0V - 13.0V to 0-255 for a 3S LiPo, change this if you use a different battery configuration.
uint8_t pack_3s_voltage(float voltage) {
    if (voltage < 9.0f) voltage = 9.0f;
    if (voltage > 13.0f) voltage = 13.0f;
    return (uint8_t)((voltage - 9.0f) * 63.75f);
}

// Callback when data is received over ESP-NOW
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(control_packet_t) && data[0] == 0xA5) {
        control_packet_t packet;
        memcpy(&packet, data, sizeof(packet));

        // Validate Checksum (XOR bytes 1-8)
        uint8_t calc = 0;
        for (int i = 1; i < 9; i++) calc ^= data[i];
        
        if (calc == packet.checksum) {
            // Apply values to actuators
            actuator_set_pitch(packet.pitch);
            actuator_set_yaw(packet.yaw);
            actuator_set_roll(packet.roll);
            actuator_set_throttle(packet.status == 1 ? packet.throttle : 0);

            // Register Remote as a peer if not already registered (Required for send)
            if (!esp_now_is_peer_exist(recv_info->src_addr)) {
                esp_now_peer_info_t peer_info = {};
                memcpy(peer_info.peer_addr, recv_info->src_addr, 6);
                peer_info.channel = 0; // Use current Wi-Fi channel
                peer_info.encrypt = false;
                esp_now_add_peer(&peer_info);
            }

            // Construct telemetry
            telemetry_packet_t telem = {
                .header = 0x5A,
                .voltage = 12.6f, 
                .pitch = (float)packet.pitch / 364.0f,
                .roll = (float)packet.roll / 364.0f,
                .yaw = (float)packet.yaw / 364.0f,
                .link_quality = 100
            };
            esp_now_send(recv_info->src_addr, (uint8_t *)&telem, sizeof(telem));
        }
    }
}

// --- INITIALIZATION FUNCTION ---
void init_all() {

    // 1. Initialize NVS (Wi-Fi requires this to store calibration data)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize the Wi-Fi stack (Required for ESP-NOW)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)); // Lock to channel 1
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Force radio to stay awake 100%

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    // Initialize Servos and Motor
    actuators_init();
}



void app_main(void)
{
    // init_wifi_and_udp_logger("Hos Therese IoT", "S7jodi4n", "192.168.2.96", 3333);

    // Initialize all hardware peripherals
    init_all();

    ESP_LOGI(TAG, "Plane Receiver Online via ESP-NOW. Listening for control packets...");


    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Passive loop
}