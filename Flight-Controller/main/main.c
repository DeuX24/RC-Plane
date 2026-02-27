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

#include "led_strip.h"

static const char *TAG = "PLANE_MAIN";

// --- RGB LED Setup ---
static led_strip_handle_t led_strip;

void init_rgb(void) {
    led_strip_config_t strip_config = { .strip_gpio_num = 48, .max_leds = 1 };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}

void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// Helper: Maps 9.0V - 13.0V to 0-255 for a 3S LiPo, change this if you use a different battery configuration.
uint8_t pack_3s_voltage(float voltage) {
    if (voltage < 9.0f) voltage = 9.0f;
    if (voltage > 13.0f) voltage = 13.0f;
    return (uint8_t)((voltage - 9.0f) * 63.75f);
}

void app_main(void)
{
    init_rgb();
    set_rgb(50, 0, 0); // RED = Booting

    init_wifi_and_udp_logger("Hos Therese IoT", "S7jodi4n", "192.168.2.96", 3333);

    actuators_init();

    radio_config_t fc_pins = {
        .spi_sck = 12,
        .spi_miso = 13,
        .spi_mosi = 11,
        .radio_csn = 10,
        .radio_irq = 2,
        .radio_ce_rst = 3,
        .radio_busy = 4  // Ignored by the wrapper if using nRF24, but good to set
    };

    ESP_LOGI(TAG, "Plane Receiver Booting...");

    if (!radio_init(&fc_pins)) {
        set_rgb(100, 0, 0); // BRIGHT RED = Radio Hardware Failure
        ESP_LOGE(TAG, "Radio failed to initialize!");
        while(1) { vTaskDelay(10); } // Halt
    }

    ESP_LOGI(TAG, "Radio Online. Listening for control packets...");

    set_rgb(50, 0, 50); // PURPLE = Waiting for Remote

    control_packet_t cmd_in = {0};
    telemetry_packet_t telem_out = {0};

    uint32_t last_packet_time = 0;

    while (1) {
        uint32_t now = esp_log_timestamp();

        // 1. Actively listen for an incoming command
        if (radio_receive_command(&cmd_in)) {
            
            if (cmd_in.header == 0xA5) {
                last_packet_time = now; // Update our "Last Seen" timestamp
                set_rgb(0, 50, 0); // GREEN = Receiving Data!
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
                telem_out.pitch = (int8_t)((cmd_in.pitch / 32768.0f) * 90.0f); // Map -32768 to 32767 range to -90 to 90 degrees. Mock telemetry
                telem_out.roll  = (int8_t)((cmd_in.roll / 32768.0f) * 90.0f);
                telem_out.yaw   = (int8_t)((cmd_in.yaw / 32768.0f) * 90.0f);

                radio_transmit_telemetry(&telem_out);
            }
        }

        // --- THE TIMEOUT FAILSAFE ---
        // If we haven't heard from the remote in 1000ms (1 second)
        if (now - last_packet_time > 1000) {
            set_rgb(50, 0, 50);        // PURPLE = Link Lost / Searching
            actuator_set_throttle(0);   // EMERGENCY: Cut motor if remote disconnects
            
            // Optional: Center the control surfaces for a glide
            // actuator_set_pitch(127); 
            // actuator_set_roll(127);
        }

        // Small yield to prevent watchdog resets
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}