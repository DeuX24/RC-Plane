#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "mirf.h"
#include "rc_protocol.h"
#include "esp_log.h"

static const char *TAG = "MAIN_LOOP";

// --- GPIO CONFIG ---
#define RED_LED_GPIO      21  // Standard LED
#define RGB_LED_GPIO      48  // Common WS2812 pin on S3 DevKits
#define STATUS_LED_COUNT  1

// --- USB TELEMETRY FORMAT ---
#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    uint8_t header; 
    float voltage;  
    float pitch;
    float roll;
    float yaw;
    uint8_t link_quality; 
} usb_telemetry_t;
#pragma pack(pop)

static led_strip_handle_t led_strip;

void set_rgb(uint32_t r, uint32_t g, uint32_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// --- INITIALIZATION FUNCTION ---
void hardware_init(NRF24_t *dev) {
    // 1. Init Red LED
    gpio_reset_pin(RED_LED_GPIO);
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RED_LED_GPIO, 1); // Power is ON

    // 2. Init RGB LED (WS2812)
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    set_rgb(0, 0, 50); // Start with BLUE (Waiting)

    // 3. Init USB
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usb_config);

    // 4. Init NRF24 Radio
    Nrf24_init(dev);
    Nrf24_config(dev, 90, 10); 
    Nrf24_setTADDR(dev, (uint8_t*)NRF24_ADDRESS);
    Nrf24_SetOutputRF_PWR(dev, 3);   
    Nrf24_SetSpeedDataRates(dev, 2); 

    // Hardware Check
    if (Nrf24_getStatus(dev) == 0xFF) {
        set_rgb(255, 0, 0); // Solid RED RGB = Hardware Fail
        ESP_LOGE(TAG, "NRF24 Hardware Fail! Check wiring.");
        while(1) {
            gpio_set_level(RED_LED_GPIO, !gpio_get_level(RED_LED_GPIO));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Enable Auto-ACK Hardware Features
    uint8_t feature_val = 0x06; 
    Nrf24_writeRegister(dev, 0x1D, &feature_val, 1); // 0x1D = FEATURE (Enable EN_ACK_PAY & EN_DPL)
    
    uint8_t dynpd_val = 0x01;
    Nrf24_writeRegister(dev, 0x1C, &dynpd_val, 1);   // 0x1C = DYNPD (Enable Dynamic Payload on Pipe 0)
}

void app_main(void)
{
    // Initialize all hardware peripherals
    NRF24_t dev;
    hardware_init(&dev);

    ESP_LOGI(TAG, "Ground Station Online. Ready to fly.");

    uint8_t usb_rx_buffer[10];
    uint8_t telem_buffer[10];

    while (1)
    {
        uint8_t header;
        // Wait for PC control packet
        if (usb_serial_jtag_read_bytes(&header, 1, portMAX_DELAY) == 1 && header == 0xA5)
        {
            usb_rx_buffer[0] = 0xA5;
            if (usb_serial_jtag_read_bytes(&usb_rx_buffer[1], 9, pdMS_TO_TICKS(10)) == 9)
            {
                uint8_t calc = 0;
                for (int i = 1; i < 9; i++) calc ^= usb_rx_buffer[i];

                if (calc == usb_rx_buffer[9])
                {
                    // Transmit Control Packet to Plane
                    Nrf24_send(&dev, usb_rx_buffer);
                    
                    // Wait for the hardware to finish the transaction
                    if (Nrf24_isSend(&dev, 10)) {
                        
                        // Because of Enhanced ShockBurst, if the Plane attached telemetry 
                        // to the ACK, it is ALREADY sitting in our RX FIFO right now.
                        if (Nrf24_dataReady(&dev)) {
                            Nrf24_getData(&dev, telem_buffer);
                            
                            if (telem_buffer[0] == 0x5A) {
                                float voltage = 9.0f + (telem_buffer[1] / 63.75f);
                                uint8_t link_quality = telem_buffer[2];
                                int8_t pitch = (int8_t)telem_buffer[3];
                                int8_t roll  = (int8_t)telem_buffer[4];
                                int8_t yaw   = (int8_t)telem_buffer[5];
                                
                                ESP_LOGI(TAG, "Telem -> 3S: %.2fV | Link: %d%% | P: %d, R: %d, Y: %d", 
                                         voltage, link_quality, pitch, roll, yaw);
                                
                                // Send to PC over USB
                                usb_telemetry_t pc_telem = {
                                    .header = 0x5A,
                                    .voltage = voltage,
                                    .pitch = (float)pitch,
                                    .roll = (float)roll,
                                    .yaw = (float)yaw,
                                    .link_quality = link_quality
                                };
                                usb_serial_jtag_write_bytes((const char*)&pc_telem, sizeof(pc_telem), 10);

                                set_rgb(0, 50, 0); // GREEN = Full Bidirectional Link
                            }
                        } else {
                            set_rgb(50, 20, 0); // ORANGE = Sent successfully, but no telem attached
                        }
                    } else {
                        set_rgb(50, 0, 0); // RED = Transmission totally failed
                    }
                }
            }
        }
    }
}