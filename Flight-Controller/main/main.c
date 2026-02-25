#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mirf.h"
#include "rc_protocol.h"
#include "actuator_control.h"

static const char *TAG = "PLANE_MAIN";

// Helper: Maps 9.0V - 13.0V to 0-255 for a 3S LiPo, change this if you use a different battery configuration.
uint8_t pack_3s_voltage(float voltage) {
    if (voltage < 9.0f) voltage = 9.0f;
    if (voltage > 13.0f) voltage = 13.0f;
    return (uint8_t)((voltage - 9.0f) * 63.75f);
}

// --- INITIALIZATION FUNCTION ---
void hardware_init(NRF24_t *dev) {
    // 1. Initialize Servos and Motor
    actuators_init();

    // 2. Initialize NRF24
    Nrf24_init(dev);
    
    // Fixed 10-byte payload for both TX and RX to match the Control Packet
    Nrf24_config(dev, 90, 10); 
    
    // Set the shared Receive Address (Hardware will auto-transmit ACKs back to this address)
    Nrf24_setRADDR(dev, (uint8_t*)NRF24_ADDRESS); 
    
    Nrf24_SetOutputRF_PWR(dev, 3);
    Nrf24_SetSpeedDataRates(dev, 2);

    // Hardware Check
    if (Nrf24_getStatus(dev) == 0xFF) {
        ESP_LOGE(TAG, "NRF24 Hardware Fail! Check wiring.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(100)); }
    }

    // 3. Enable Advanced NRF24 Features for Auto-ACK Payloads
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

    ESP_LOGI(TAG, "Plane Receiver Online. Listening for control packets...");

    // Buffers sized to our shared 10-byte payload limit
    uint8_t rx_buffer[10];
    uint8_t tx_buffer[10] = {0};

    // 4. Pre-load the FIRST telemetry packet into the TX FIFO
    // This ensures the hardware has something to attach to the very first Acknowledgment
    telemetry_packet_t telem = {
        .header = 0x5A,
        .voltage = pack_3s_voltage(12.6f), // Dummy start voltage (Fully charged 3S)
        .link_quality = 100,               // Dummy start link quality
        .pitch = 0, 
        .roll = 0, 
        .yaw = 0
    };
    memcpy(tx_buffer, &telem, sizeof(telemetry_packet_t));
    Nrf24_writeAckPayload(&dev, 0, tx_buffer, 10);

    // 5. Main Flight Loop
    while (1)
    {
        // Check if the radio has received a Control Packet
        if (Nrf24_dataReady(&dev))
        {
            // Reading the RX FIFO here automatically triggers the radio to blast
            // the previously pre-loaded TX FIFO data back to the Ground Station!
            Nrf24_getData(&dev, rx_buffer); 
            
            // Validate the Header
            if (rx_buffer[0] == 0xA5) 
            {
                uint8_t calc = 0;
                for (int i = 1; i < sizeof(control_packet_t) - 1; i++) {
                    calc ^= rx_buffer[i];
                }

                // Validate the Checksum
                if (calc == rx_buffer[sizeof(control_packet_t) - 1])
                {
                    control_packet_t packet;
                    memcpy(&packet, rx_buffer, sizeof(packet));

                    // --- APPLY CONTROL TO ACTUATORS ---
                    if (packet.status == 1) { 
                        actuator_set_throttle(packet.throttle);
                    } else {
                        actuator_set_throttle(0); // Disarmed safety
                    }
                    actuator_set_pitch(packet.pitch);
                    actuator_set_yaw(packet.yaw);
                    actuator_set_roll(packet.roll);

                    // --- CONSTRUCT NEXT TELEMETRY PACKET ---
                    // Build the telemetry for the *next* round-trip
                    telemetry_packet_t next_telem = {
                        .header = 0x5A,
                        .voltage = pack_3s_voltage(11.4f), // TODO: Replace with actual ADC read
                        .link_quality = 100,               // TODO: Calculate based on dropped packets
                        .pitch = (int8_t)(packet.pitch / 364),
                        .roll = (int8_t)(packet.roll / 364),
                        .yaw = (int8_t)(packet.yaw / 364)
                    };
                    
                    // --- PRE-LOAD THE ACK PAYLOAD ---
                    // Push this new data into the TX FIFO so it is waiting for the next incoming transmission
                    memset(tx_buffer, 0, 10); // Clear buffer
                    memcpy(tx_buffer, &next_telem, sizeof(telemetry_packet_t));
                    Nrf24_writeAckPayload(&dev, 0, tx_buffer, 10);
                }
                else
                {
                    ESP_LOGW(TAG, "Control Packet Checksum Failed!");
                }
            }
        }
        
        // Yield for 1 tick (~1ms) so the FreeRTOS watchdog doesn't crash the ESP32
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}