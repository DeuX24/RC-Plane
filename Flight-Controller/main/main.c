#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "rc_protocol.h"
#include "actuator_control.h"
#include "udp_logger.h"
#include "radio_link.h"

static const char *TAG = "PLANE_MAIN";

// Helper: Maps 9.0V - 13.0V to 0-255 for a 3S LiPo, change this if you use a different battery configuration.
uint8_t pack_3s_voltage(float voltage) {
    if (voltage < 9.0f) voltage = 9.0f;
    if (voltage > 13.0f) voltage = 13.0f;
    return (uint8_t)((voltage - 9.0f) * 63.75f);
}

void app_main(void)
{
    init_wifi_and_udp_logger("Hos Therese IoT", "S7jodi4n", "192.168.2.96", 3333);

    actuators_init();

    ESP_LOGI(TAG, "Plane Receiver Booting...");

    if (!radio_init()) {
        ESP_LOGE(TAG, "Radio failed to initialize!");
        while(1) { vTaskDelay(10); } // Halt
    }

    ESP_LOGI(TAG, "Radio Online. Listening for control packets...");

    control_packet_t cmd_in = {0};
    telemetry_packet_t telem_out = {0};

    while (1) {
        // 1. Actively listen for an incoming command
        if (radio_receive_command(&cmd_in)) {
            
            if (cmd_in.header == 0xA5) {
                // 2. Apply the commands we just received
                if (cmd_in.status == 1) { // Armed
                    actuator_set_throttle(cmd_in.throttle);
                    actuator_set_pitch(cmd_in.pitch);
                    actuator_set_roll(cmd_in.roll);
                    actuator_set_yaw(cmd_in.yaw);
                } else {
                    actuator_set_throttle(0); // Disarmed failsafe
                }

                // 3. Update and send the Telemetry reply immediately
                telem_out.header = 0x5A;
                telem_out.voltage = 12.4; // Replace with actual ADC read later
                telem_out.link_quality = 100;
                telem_out.pitch = cmd_in.pitch / 256.0f * 90.0f; // Mock telemetry
                telem_out.roll = cmd_in.roll / 256.0f * 90.0f;
                telem_out.yaw = cmd_in.yaw / 256.0f * 90.0f;

                radio_transmit_telemetry(&telem_out);
            }
        }

        // Small yield to prevent watchdog resets
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}