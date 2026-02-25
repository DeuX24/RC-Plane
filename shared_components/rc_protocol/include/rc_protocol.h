#ifndef RC_PROTOCOL_H
#define RC_PROTOCOL_H

#include <stdint.h>

// PC -> S3 (Controller) -> S3 (Plane) - 10 Bytes Total
#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
  uint8_t header;   // 0xA5
  uint8_t throttle; // 0-255
  int16_t yaw;      // -32768 to 32767 (Matches C# 'short')
  int16_t pitch;
  int16_t roll;
  uint8_t status;   // 0 = Disarmed, 1 = Armed
  uint8_t checksum; // XOR of all previous bytes
} control_packet_t;
#pragma pack(pop)

// S3 (Plane) -> S3 (Controller) - 6 Bytes Total
#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
  uint8_t header;       // 0x5A
  uint8_t voltage;      // 0-255 (Mapped from 9.0V to 13.0V)
  uint8_t link_quality; // 0-100 (%)
  int8_t pitch;         // -90 to +90 degrees
  int8_t roll;          // -90 to +90 degrees
  int8_t yaw;           // -90 to +90 degrees
} telemetry_packet_t;
#pragma pack(pop)

// --- NRF24 RADIO SETTINGS ---
#define NRF24_CHANNEL      76
#define NRF24_PAYLOAD_SIZE 10
#define NRF24_DATA_RATE    2    // 0 = 1Mbps, 1 = 2Mbps, 2 = 250Kbps
#define NRF24_PA_LEVEL     3    // 3 = PA_MAX

// Shared TX/RX Address (5 bytes)
static const uint8_t NRF24_ADDRESS[5] = {0x54, 0x53, 0x44, 0x53, 0xE7};

#endif