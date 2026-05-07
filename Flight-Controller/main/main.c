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
#include "led_strip.h"

static const char *TAG = "PLANE_MAIN";

#define RGB_LED_GPIO 48

float current_voltage = 12.6f; // Global to store the latest voltage reading
double total_mah_consumed = 0.0; 
double total_mwh_consumed = 0.0;

static led_strip_handle_t led_strip;

// --- LINK STATE ---
static bool link_established = false;
static uint32_t last_packet_time = 0;


void set_rgb(uint32_t r, uint32_t g, uint32_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

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

            link_established = true;
            last_packet_time = esp_log_timestamp();

            set_rgb(0, 50, 0); // Solid Green

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
                .voltage = current_voltage,
                .pitch = (float)packet.pitch / 364.0f,
                .roll = (float)packet.roll / 364.0f,
                .yaw = (float)packet.yaw / 364.0f,
                .link_quality = 100
            };
            esp_now_send(recv_info->src_addr, (uint8_t *)&telem, sizeof(telem));
        }
    }
}

void init_rgb() {
    // Init RGB LED (WS2812)
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    set_rgb(0, 0, 50); // Start with BLUE (Waiting/Booting)
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

    init_rgb();

    // Initialize all hardware peripherals
    init_all();

    set_rgb(50, 40, 0); // Amber (Ready/Idle)

    ESP_LOGI(TAG, "Plane Receiver Online via ESP-NOW. Listening for control packets...");

    // Setup the precise timing variables for FreeRTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms exact period

    const double hours_elapsed = 0.1 / 3600.0; // 100ms loop delay

    while(1) { 

        // Wait for exactly the remainder of the 100ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t now = esp_log_timestamp();

        // --- FAILSAFE CHECK ---
        // If the link was active, but we haven't received a packet in 1 seconds
        if (link_established && (now - last_packet_time > 1000)) 
        {
            link_established = false;
            ESP_LOGW(TAG, "CRITICAL: Remote link lost! Entering failsafe.");
            
            set_rgb(50, 0, 50); // Solid Purple (Link Lost)
            
            // Set actuators to a safe state
            actuator_set_throttle(0); // Cut motor
            actuator_set_pitch(0); // Level elevator
            actuator_set_roll(0);  // Level ailerons
        }
    }

}