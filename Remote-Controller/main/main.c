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

int map_joystick(int raw_val, int center_val) {
    int diff = raw_val - center_val;
    
    // 1. Deadzone check
    if (abs(diff) < DEADZONE) {
        return 0; 
    }

    // 2. Map to -100 to 100 percentage
    if (raw_val < center_val) {
        // Negative side (Stick pulling down/left)
        // Maps the range [0 to center] -> [-100 to 0]
        return (diff * 100) / center_val;
    } else {
        // Positive side (Stick pushing up/right)
        // Maps the range [center to 4095] -> [0 to 100]
        return (diff * 100) / (4095 - center_val);
    }
}

// Callback for incoming telemetry from the plane
void on_telem_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) 
{
    // Make sure we received exactly the 6-byte struct from the Plane
    if (len == sizeof(telemetry_packet_t) && data[0] == 0x5A) 
    {
        // Only print the MAC if the link was previously down
        if (!link_established) {
            ESP_LOGI("MAC_SNIPER", "Plane Found! MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
                     recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], 
                     recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        }
        link_established = true;
        last_recv_time = esp_log_timestamp();
        
        // Cast the raw air data to our 6-byte struct
        // telemetry_packet_t* telem_in = (telemetry_packet_t*)data;
        
        set_rgb(0, 50, 0); // Solid GREEN = Connected
    }
}

// Callback to tell us if our transmission was successful
// Updated for ESP-IDF v5.x API
void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGE(TAG, "TX Delivery Fail"); 
        // Optional: Flash the LED red briefly if a packet drops
    }
}

void init_joystick() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    
    // GPIO 5 (X) and GPIO 6 (Y)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));
}

void init_comms() {
    // 1. Minimal Wi-Fi Setup for ESP-NOW
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // ---> ADD THIS LINE TO FIX PACKET LOSS <---
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // 2. Init ESP-NOW
    esp_now_init();
    esp_now_register_recv_cb(on_telem_recv);
    esp_now_register_send_cb(on_data_sent);

    // 3. Add Plane as a Peer (Using Broadcast for now)
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, BROADCAST_MAC, 6);
    peer_info.channel = 1; 
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);
}

// --- INITIALIZATION FUNCTION ---
void init_all() {

    init_comms();
    init_joystick();

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
}

void app_main(void)
{
    // Initialize everything (GPIOs, RMT for WS2812, etc.)
    init_all();

    int x_raw, y_raw;

    // EMA state variables
    float pitch_smoothed = PITCH_CENTER; 
    float roll_smoothed = ROLL_CENTER;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Update Rate (Standard for RC)

    int print_counter = 0; // Add a dedicated counter

    while (1) 
    {
        // Wait for the next cycle exactly 20ms after the last one started
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // set_rgb(50, 50, 0);

        uint32_t now = esp_log_timestamp();

        // 1. Air Failsafe: Plane lost for 2 seconds
        if (link_established && (now - last_recv_time > 2000)) 
        {
            link_established = false;
            set_rgb(50, 0, 50); // PURPLE (Link Lost)
        }

        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &x_raw);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &y_raw);

        // Apply EMA Filter (0.2 = 20% new reading, 80% old history)
        pitch_smoothed = (0.2 * x_raw) + (0.8 * pitch_smoothed);
        roll_smoothed = (0.2 * y_raw) + (0.8 * roll_smoothed);

        // Map the values to a clean -100 to +100 range
        last_command.pitch = map_joystick((int)pitch_smoothed, PITCH_CENTER);
        last_command.roll  = map_joystick((int)roll_smoothed, ROLL_CENTER);

        uint8_t *p = (uint8_t*)&last_command;
        last_command.checksum = 0;
        for (int i = 0; i < (sizeof(control_packet_t) - 1); i++) 
        {
            last_command.checksum ^= p[i];
        }

/*         // Log for debugging (Only every 10th frame to avoid flooding)
        if (print_counter++ >= 100) 
        {
            ESP_LOGI(TAG, "Pitch: %d, Roll: %d", last_command.pitch, last_command.roll);
            //ESP_LOGI(TAG, "x_raw: %d, y_raw: %d", x_raw, y_raw);
            set_rgb(0, 50, 0); // GREEN = Ready to Send
            print_counter = 0; // Reset counter after logging
        } */

        esp_err_t result = esp_now_send(BROADCAST_MAC, (uint8_t*)&last_command, sizeof(control_packet_t));
        ESP_LOGI(TAG, "Sent control packet with Pitch: %d, Roll: %d", last_command.pitch, last_command.roll);
        
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Error sending: %s", esp_err_to_name(result));
        }
    }
}