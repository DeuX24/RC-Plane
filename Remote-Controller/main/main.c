#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "rc_protocol.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN_LOOP";

// --- SCANNING STATE ---
static uint8_t current_channel = 1;
static bool link_established = false;
static uint32_t last_recv_time = 0;
static uint32_t last_scan_time = 0;

// --- RC STATE ---
static control_packet_t last_command = { .header = 0xA5, .status = 0, .checksum = 0 };
static uint32_t last_usb_time = 0;

// --- GPIO CONFIG ---
#define RED_LED_GPIO      21  // Standard LED
#define RGB_LED_GPIO      48  // Common WS2812 pin on S3 DevKits
#define STATUS_LED_COUNT  1

// --- USB TELEMETRY FORMAT ---
#pragma pack(push, 1)
typedef struct {
    uint8_t header;       // 1 byte (0x5A)
    float voltage;        // 4 bytes
    float pitch;          // 4 bytes
    float roll;           // 4 bytes
    float yaw;            // 4 bytes
    uint8_t link_quality; // 1 byte
} usb_telemetry_t;        // Total: 18 bytes
#pragma pack(pop)

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

    // 1. Handle Timeout: If we haven't heard from the plane in 2 seconds, start searching
    if (link_established && (now - last_recv_time > 2000)) {
        link_established = false;
        set_rgb(50, 0, 50); // PURPLE = Searching
    }

    // 2. Handle Scanning: Cycle channels if link is down
    if (!link_established) {
        if (now - last_scan_time > 200) { // Try a new channel every 200ms
            current_channel++;
            if (current_channel > 13) current_channel = 1; // Standard Sweden range
            
            esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
            last_scan_time = now;
            
            // Construct a "Ping" packet to trigger a response from the plane
            control_packet_t ping = { 
                .header = 0xA5, 
                .status = 0 // Disarmed ping
            };
            
            // Simple XOR Checksum for the ping
            uint8_t *p = (uint8_t*)&ping;
            ping.checksum = 0;
            for (int i = 1; i < 9; i++) ping.checksum ^= p[i];
            
            esp_now_send(BROADCAST_MAC, (uint8_t*)&ping, sizeof(ping));
        }
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
            usb_rx_buffer[0] = 0xA5;
            if (usb_serial_jtag_read_bytes(&usb_rx_buffer[1], 9, pdMS_TO_TICKS(10)) == 9) 
            {
                // Forward directly to Plane on current channel
                esp_now_send(BROADCAST_MAC, usb_rx_buffer, sizeof(control_packet_t));
            }
        }
    }
}