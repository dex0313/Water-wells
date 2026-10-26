#include "protocol.h"

// ============================================================
// CRC-16/Modbus
// Poly 0xA001 (reflected 0x8005), init 0xFFFF, no final XOR.
// Computed over the whole MeshPacket except the trailing crc field
// (57 bytes for a fixed-size frame).
// ============================================================

uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}
