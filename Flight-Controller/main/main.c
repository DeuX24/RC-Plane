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

#include "i2cdev.h"
#include "ina3221.h"

static const char *TAG = "PLANE_MAIN";

#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_9

ina3221_t ina_dev;
float current_voltage = 12.6f; // Global to store the latest voltage reading
double total_mah_consumed = 0.0; 
double total_mwh_consumed = 0.0;

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

    // Initialize INA3221 for voltage monitoring
    ESP_ERROR_CHECK(i2cdev_init());
    
    // Address INA3221_I2C_ADDR_GND assumes the A0 pin is tied to Ground. Adjust if tied to VS/SDA/SCL.
    if (ina3221_init_desc(&ina_dev, INA3221_I2C_ADDR_GND, 0, I2C_SDA_PIN, I2C_SCL_PIN) == ESP_OK) 
    {
        ina3221_reset(&ina_dev);
        
        ina_dev.shunt[INA3221_CHANNEL_2] = 100; 
        ina_dev.shunt[INA3221_CHANNEL_3] = 100; 

        // Set the hardware averaging to 16 samples
        ina3221_set_average(&ina_dev, INA3221_AVG_16);

        // (Optional) Explicitly set the conversion time to 1.1ms (this is usually default)
        ina3221_set_shunt_conversion_time(&ina_dev, INA3221_CT_1100);
        ina3221_set_bus_conversion_time(&ina_dev, INA3221_CT_1100);

        ina3221_enable_channel(&ina_dev, true, true, true);
        
        ESP_LOGI(TAG, "INA3221 Initialized: Hardware Averaging ON (16x)");
    } 
    else 
    {
        ESP_LOGE(TAG, "Failed to initialize INA3221");
    }
}



void app_main(void)
{
    // init_wifi_and_udp_logger("Hos Therese IoT", "S7jodi4n", "192.168.2.96", 3333);

    // Initialize all hardware peripherals
    init_all();

    ESP_LOGI(TAG, "Plane Receiver Online via ESP-NOW. Listening for control packets...");

    // Setup the precise timing variables for FreeRTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms exact period

    const double hours_elapsed = 0.1 / 3600.0; // 100ms loop delay

    while(1) { 
        float voltage_ch1 = 0.0f, voltage_ch2 = 0.0f, voltage_ch3 = 0.0f;
        float shunt_mv = 0.0f;     
        float current_ma_ch2 = 0.0f; 
        float current_ma_ch3 = 0.0f; 

        // --- CHANNEL 1 (Voltage Only) ---
        if (ina3221_get_bus_voltage(&ina_dev, INA3221_CHANNEL_1, &voltage_ch1) == ESP_OK) {
            current_voltage = voltage_ch1; // Update global for telemetry (Main Battery)

            ESP_LOGI(TAG, "CH1 | Voltage: %.2f V", voltage_ch1);
        }

        // --- CHANNEL 2 ---
        if (ina3221_get_bus_voltage(&ina_dev, INA3221_CHANNEL_2, &voltage_ch2) == ESP_OK &&
            ina3221_get_shunt_value(&ina_dev, INA3221_CHANNEL_2, &shunt_mv, &current_ma_ch2) == ESP_OK) {
            
            // Math uses CH2's specific voltage
            double power_mw_ch2 = (double)voltage_ch2 * (double)current_ma_ch2;    

            total_mah_consumed += ((double)current_ma_ch2 * hours_elapsed);
            total_mwh_consumed += (power_mw_ch2 * hours_elapsed);

            ESP_LOGI(TAG, "CH2 | Voltage: %.2f V | Current: %.2f mA | Power: %.2f mW", 
                     voltage_ch2, current_ma_ch2, power_mw_ch2);
        }

        // --- CHANNEL 3 ---
        if (ina3221_get_bus_voltage(&ina_dev, INA3221_CHANNEL_3, &voltage_ch3) == ESP_OK &&
            ina3221_get_shunt_value(&ina_dev, INA3221_CHANNEL_3, &shunt_mv, &current_ma_ch3) == ESP_OK) {
            
            // Math uses CH3's specific voltage
            double power_mw_ch3 = (double)voltage_ch3 * (double)current_ma_ch3;
            
            total_mah_consumed += ((double)current_ma_ch3 * hours_elapsed);
            total_mwh_consumed += (power_mw_ch3 * hours_elapsed);

            ESP_LOGI(TAG, "CH3 | Voltage: %.2f V | Current: %.2f mA | Power: %.2f mW", 
                     voltage_ch3, current_ma_ch3, power_mw_ch3);
        }

        // Wait for exactly the remainder of the 100ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}