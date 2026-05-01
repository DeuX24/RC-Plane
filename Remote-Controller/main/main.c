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
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "remote_ota.h"
#include "joystick_calib.h"

static const char *TAG = "MAIN_LOOP";

// --- RC STATE ---
static control_packet_t last_command = { .header = 0xA5, .status = 0, .checksum = 0 };
static bool link_established = false;
static uint32_t last_recv_time = 0;

// --- GPIO CONFIG ---
#define RED_LED_GPIO      21  // Standard LED
#define RGB_LED_GPIO      48  // Common WS2812 pin on S3 DevKits
#define STATUS_LED_COUNT  1
#define DEADZONE          50

#define BUTTON_PIN GPIO_NUM_14

// --- GLOBAL HANDLES ---
adc_oneshot_unit_handle_t adc1_handle;
static led_strip_handle_t led_strip;

void set_rgb(uint32_t r, uint32_t g, uint32_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

#pragma region JOYSTICK

// Helper function to constrain raw values
int constrain(int val, int min_val, int max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

int map_joystick(int raw_val, int min_val, int center_val, int max_val) {
    raw_val = constrain(raw_val, min_val, max_val);
    int diff = raw_val - center_val;
    
    if (abs(diff) < DEADZONE) return 0; 

    if (raw_val < center_val) {
        int active_range = center_val - min_val - DEADZONE;
        int active_diff = abs(diff) - DEADZONE;
        // Check to prevent division by zero
        if (active_range <= 0) return 0; 
        return -(active_diff * 32768) / active_range;
    } else {
        int active_range = max_val - center_val - DEADZONE;
        int active_diff = diff - DEADZONE;
        if (active_range <= 0) return 0;
        return (active_diff * 32767) / active_range;
    }
}

uint8_t map_throttle_uint8(int raw_val, int min_val, int center_val, int max_val) {
    raw_val = constrain(raw_val, min_val, max_val);
    int diff = raw_val - center_val;

    if (diff <= DEADZONE) return 0; 

    int active_diff = diff - DEADZONE;
    int active_range = max_val - center_val - DEADZONE;
    if (active_range <= 0) return 0;

    return (uint8_t)((active_diff * 255) / active_range);
}

#pragma endregion

#pragma region TRANSMISSION

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
void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGE(TAG, "TX Delivery Fail"); 
        // Optional: Flash the LED red briefly if a packet drops
    }
}

#pragma endregion

#pragma region INITIALIZATION

void configure_status_button() {
    gpio_config_t io_conf = {}; // Zero-initialize the config structure
    
    // Disable interrupts for standard polling
    io_conf.intr_type = GPIO_INTR_DISABLE; 
    
    // Set as Input mode
    io_conf.mode = GPIO_MODE_INPUT; 
    
    // Bit mask of the pins you want to set (you can OR multiple pins together)
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN); 
    
    // Disable pull-down mode
    io_conf.pull_down_en = 0; 
    
    // Enable pull-up mode
    io_conf.pull_up_en = 1; 
    
    // Apply the configuration
    gpio_config(&io_conf);
}

void init_joysticks() {
    // Initialize ADC Unit 1 (Handles all 4 joystick axes)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Shared configuration 
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_PITCH_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_YAW_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_ROLL_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_THROTTLE_CHAN, &config));
}

void init_esp_now() {
    // 1. Minimal Wi-Fi Setup for ESP-NOW (Core network stack is already running)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // 2. Init ESP-NOW
    esp_now_init();
    esp_now_register_recv_cb(on_telem_recv);
    esp_now_register_send_cb(on_data_sent);

    // 3. Add Plane as a Peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, BROADCAST_MAC, 6);
    peer_info.channel = 1; 
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);
}

// --- INITIALIZATION FUNCTION ---
void init_essentials() {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Configure the button so we can read it immediately
    configure_status_button();
    
    // Configure the ADCs for the joysticks
    init_joysticks();

    // Init Red LED
    gpio_reset_pin(RED_LED_GPIO);
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);

    // Init RGB LED (WS2812)
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    set_rgb(0, 0, 50); // Start with BLUE (Waiting)
}

#pragma endregion

void app_main(void)
{
    // Initialize everything (GPIOs, RMT for WS2812, etc.)
    init_essentials();

    bool force_calib = false;

    // Check the button state immediately on boot (Active LOW)
    if (gpio_get_level(BUTTON_PIN) == 0) {
        
        set_rgb(50, 50, 0); // YELLOW = Button detected, waiting for decision
        ESP_LOGI("BOOT", "Button held. Determining mode...");
        
        int hold_time = 0;
        while (gpio_get_level(BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            hold_time += 100;
        }

        if (hold_time > 3000) {
            // Held for > 3 seconds -> Enter Home OTA Mode
            set_rgb(0, 50, 50); // CYAN = OTA Mode
            start_home_ota_mode();
            
            // Trap the processor here so it doesn't run the flight code
            while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } 
        } 
        else {
            // Held for < 3 seconds -> Force Calibration
            force_calib = true;
        }
    }

    // Run the calibration logic (loads from flash OR runs the loop if forced)
    init_joystick_calibration(adc1_handle, force_calib, set_rgb);

    // Only initialize ESP-NOW after we have safely bypassed OTA mode
    init_esp_now();
    
    set_rgb(0, 50, 0); // Solid Green = Ready to fly

    int pitch_raw, yaw_raw;
    int roll_raw, throttle_raw;

    // EMA state variables
    float pitch_smoothed = joy_calib.pitch_center; 
    float yaw_smoothed = joy_calib.yaw_center;
    float roll_smoothed = joy_calib.roll_center;
    float throttle_smoothed = joy_calib.throttle_min; // Start at min for safety

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Update Rate (Standard for RC)

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
 
        adc_oneshot_read(adc1_handle, JOY_YAW_CHAN, &yaw_raw);
        adc_oneshot_read(adc1_handle, JOY_THROTTLE_CHAN, &throttle_raw);
        adc_oneshot_read(adc1_handle, JOY_ROLL_CHAN, &roll_raw);
        adc_oneshot_read(adc1_handle, JOY_PITCH_CHAN, &pitch_raw);

        // Apply EMA Filter (0.2 = 20% new reading, 80% old history)
        pitch_smoothed = (0.2 * pitch_raw) + (0.8 * pitch_smoothed);
        yaw_smoothed = (0.2 * yaw_raw) + (0.8 * yaw_smoothed);
        roll_smoothed = (0.2 * roll_raw) + (0.8 * roll_smoothed);
        throttle_smoothed = (0.2 * throttle_raw) + (0.8 * throttle_smoothed);

        // Map the values using the live NVS calibration data
        last_command.status = !gpio_get_level(BUTTON_PIN); 
        
        last_command.pitch = map_joystick((int)pitch_smoothed, joy_calib.pitch_min, joy_calib.pitch_center, joy_calib.pitch_max);
        last_command.yaw   = map_joystick((int)yaw_smoothed, joy_calib.yaw_min, joy_calib.yaw_center, joy_calib.yaw_max);
        last_command.roll  = map_joystick((int)roll_smoothed, joy_calib.roll_min, joy_calib.roll_center, joy_calib.roll_max); 
        
        last_command.throttle = map_throttle_uint8((int)throttle_smoothed, joy_calib.throttle_min, joy_calib.throttle_center, joy_calib.throttle_max);

        uint8_t *p = (uint8_t*)&last_command;
        last_command.checksum = 0;
        for (int i = 1; i < (sizeof(control_packet_t) - 1); i++) 
        {
            last_command.checksum ^= p[i];
        }

        esp_err_t result = esp_now_send(BROADCAST_MAC, (uint8_t*)&last_command, sizeof(control_packet_t));
        // ESP_LOGI(TAG, "Sent control packet with Pitch: %d, Yaw: %d", last_command.pitch, last_command.yaw);
        
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Error sending: %s", esp_err_to_name(result));
        }
    }
}