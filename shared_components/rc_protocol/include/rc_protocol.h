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

// ESP-NOW uses MAC addresses for pairing. 
// You can use the broadcast address for testing:
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#endif