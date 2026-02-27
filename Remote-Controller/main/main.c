#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "rc_protocol.h"
#include "radio_link.h"

static const char *TAG = "REMOTE_MAIN";

// --- RGB LED Setup (Assuming standard ESP32-S3 WS2812 pin 48) ---
static led_strip_handle_t led_strip;
void init_rgb(void) {
    led_strip_config_t strip_config = { .strip_gpio_num = 48, .max_leds = 1, };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000, };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}
void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void app_main(void) {
    init_rgb();
    set_rgb(50, 0, 0); // RED = Booting

    ESP_LOGI(TAG, "Remote Controller Booting...");

    // Configure the USB Serial port to read your C# GUI
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&usb_config);

    // Define the specific wiring for the Remote Controller
    radio_config_t rc_pins = {
        .spi_sck = 12,    // Change these to your actual Remote wiring!
        .spi_miso = 13,
        .spi_mosi = 11,
        .radio_csn = 10,
        .radio_irq = 2,
        .radio_ce_rst = 3,
        .radio_busy = 4   
    };

    if (!radio_init(&rc_pins)) {
        ESP_LOGE(TAG, "Radio failed to initialize!");
        while(1) { vTaskDelay(10); } 
    }

    ESP_LOGI(TAG, "Radio Online. Starting Heartbeat Loop...");
    set_rgb(50, 0, 50); // PURPLE = Searching for Plane

    control_packet_t last_command = { .header = 0xA5, .status = 0 };
    telemetry_packet_t telem_in = { 0 };
    
    uint32_t last_usb_time = 0;
    uint32_t last_telem_time = 0;

    while (1) {
        uint32_t now = esp_log_timestamp();

        // 1. Read PC Commands (Non-blocking)
        uint8_t header;
        if (usb_serial_jtag_read_bytes(&header, 1, 0) == 1 && header == 0xA5) {
            uint8_t usb_buffer[10];
            usb_buffer[0] = 0xA5;
            if (usb_serial_jtag_read_bytes(&usb_buffer[1], 9, pdMS_TO_TICKS(5)) == 9) {
                memcpy(&last_command, usb_buffer, sizeof(control_packet_t));
                last_usb_time = now;
            }
        }

        // 2. PC Failsafe: Disarm if GUI crashes or unplugs
        if (now - last_usb_time > 500) {
            last_command.status = 0; 
            last_command.throttle = 0;
        }

        // 3. Transmit the Command (Heartbeat)
        if (radio_transmit_command(&last_command)) {
            
            // 4. Instantly check for the Plane's telemetry reply
            if (radio_receive_telemetry(&telem_in)) {
                if (telem_in.header == 0x5A) {
                    last_telem_time = now;
                    set_rgb(0, 50, 0); // GREEN = Link Established!
                    
                    // Forward telemetry back to the C# GUI
                    usb_serial_jtag_write_bytes((const char*)&telem_in, sizeof(telemetry_packet_t), pdMS_TO_TICKS(10));
                }
            }
        }

        // 5. Plane Failsafe: Light turns Purple if Plane drops out of sky
        if (now - last_telem_time > 1000) {
            set_rgb(50, 0, 50); // PURPLE = Link Lost
        }

        // Run the heartbeat loop at 50Hz (every 20ms)
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}