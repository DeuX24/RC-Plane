#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "led_strip.h"
#include "rc_protocol.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "MAIN_LOOP";

// --- RC STATE ---
static control_packet_t last_command = { .header = 0xA5, .status = 0, .checksum = 0 };
static bool link_established = false;
static uint32_t last_recv_time = 0;

// --- GPIO CONFIG ---
#define RED_LED_GPIO      21  // Standard LED
#define RGB_LED_GPIO      48  // Common WS2812 pin on S3 DevKits
#define STATUS_LED_COUNT  1
#define PITCH_CENTER      1748  // 2048 - 300
#define ROLL_CENTER       1728  // 2048 - 320
#define DEADZONE          50

// --- GLOBAL HANDLES ---
adc_oneshot_unit_handle_t adc1_handle; // Fixed: Now global so all functions can see it
static led_strip_handle_t led_strip;

void set_rgb(uint32_t r, uint32_t g, uint32_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}


// Callback for incoming telemetry from the plane
void on_telem_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) 
{
    if (len == sizeof(telemetry_packet_t) && data[0] == 0x5A) 
    {
        // Mark link as active and save the time
        link_established = true;
        last_recv_time = esp_log_timestamp();
        
        // Pass to PC
        usb_serial_jtag_write_bytes((const char*)data, len, 10);
        set_rgb(0, 50, 0); // Solid GREEN = Connected
    }
}

// Dedicated function to handle the discovery and timeout logic
void handle_channel_discovery(void) {
    uint32_t now = esp_log_timestamp();

    // 1. Air Failsafe: Plane lost for 2 seconds
    if (link_established && (now - last_recv_time > 2000)) {
        link_established = false;
        set_rgb(50, 0, 50); // PURPLE
    }

    // 2. Heartbeat & Discovery Timer (Every 200ms)
    if (now - last_scan_time > 200) { 
        if (!link_established) {
            current_channel++;
            if (current_channel > 13) current_channel = 1; 
            esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
        }
        last_scan_time = now;
        
        // PC FAILSAFE
        // If the USB hasn't sent a command in 500ms, force a disarm to prevent flyaways
        if (now - last_usb_time > 500) {
            last_command.status = 0;
            last_command.throttle = 0;
            
            // Recompute checksum for the failsafe packet
            uint8_t *p = (uint8_t*)&last_command;
            last_command.checksum = 0;
            for (int i = 1; i < 9; i++) last_command.checksum ^= p[i];
        }
        
        // Send the last known command (or the disarmed command if PC timed out)
        esp_now_send(BROADCAST_MAC, (uint8_t*)&last_command, sizeof(control_packet_t));
    }
}

// --- INITIALIZATION FUNCTION ---
void init_all() {

    // 1. Minimal Wi-Fi Setup for ESP-NOW
    esp_log_level_set("*", ESP_LOG_NONE); // Clean USB stream
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // 2. Init ESP-NOW
    esp_now_init();
    esp_now_register_recv_cb(on_telem_recv);

    // 3. Add Plane as a Peer (Using Broadcast for now)
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, BROADCAST_MAC, 6);
    peer_info.channel = 0; 
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);

    // 4. Init Red LED
    gpio_reset_pin(RED_LED_GPIO);
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);

    // 5. Init RGB LED (WS2812)
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    set_rgb(0, 0, 50); // Start with BLUE (Waiting)

    // 6. Init USB
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usb_config);


}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    // Initialize everything (GPIOs, USB, RMT for WS2812, etc.)
    init_all();

    uint8_t usb_rx_buffer[10];

    while (1) 
    {
        // Run the discovery logic
        handle_channel_discovery();

        // PC to Plane Forwarding
        uint8_t header;
        if (usb_serial_jtag_read_bytes(&header, 1, pdMS_TO_TICKS(10)) == 1 && header == 0xA5) 
        {
            uint8_t usb_rx_buffer[10];
            usb_rx_buffer[0] = 0xA5;
            if (usb_serial_jtag_read_bytes(&usb_rx_buffer[1], 9, pdMS_TO_TICKS(10)) == 9) 
            {
                // Save the valid packet to our global state
                memcpy(&last_command, usb_rx_buffer, sizeof(control_packet_t));
                last_usb_time = esp_log_timestamp(); // Reset the PC failsafe timer

                // Forward directly to Plane on current channel
                esp_now_send(BROADCAST_MAC, (uint8_t*)&last_command, sizeof(control_packet_t));
            }
        }
    }
}