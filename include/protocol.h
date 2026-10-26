#pragma once
// ============================================================
// protocol.h — Wire format v1 (frozen) + additive v1 extensions
// ============================================================
// This header is the single source of truth for the over-the-air
// packet layout. It must stay host-compilable (no Arduino.h) so
// that native unit tests can verify the wire format (see test/).
//
// HARD RULES (do not break, see revision notes 0.1):
//   - MeshPacket layout and fixed length of 59 bytes must never change.
//   - PKT_DATA=1 / PKT_CMD=2 / PKT_ACK=3 type values must never change.
//   - DataPayload layout must never change.
//   - CMD payload layout [0]=cmd [1]=relay_index [2]=value must never
//     change; new fields are appended at the END only (e.g. [3]=seq).
//   - CRC-16/Modbus over the whole frame minus the trailing CRC field.
//   - New packet types may be added (old receivers ignore them).
// ============================================================

#include <cstdint>
#include <cstddef>

// ------------------------------------------------------------
// Packet types
// ------------------------------------------------------------

#define PKT_DATA   1
#define PKT_CMD    2
#define PKT_ACK    3
// Additive extension (repeater diagnostics, uptime, counters).
// Old firmware silently ignores unknown types (verified: the
// dispatch chain is "if DATA ... else if ACK ...").
#define PKT_STATUS 4

// Highest known packet type — used by the frame parser to drop
// garbage early. Update when adding new types.
#define PKT_TYPE_MAX PKT_STATUS

// ------------------------------------------------------------
// Protocol limits
// ------------------------------------------------------------

// Maximum number of relays/sensors per node (protocol limit,
// bounded by the fixed-size voltage[] array in DataPayload).
#define MAX_RELAYS_PER_NODE  4
#define MAX_SENSORS_PER_NODE 4

// Fixed over-the-air frame length in bytes.
#define MESH_PKT_LEN 59

// ------------------------------------------------------------
// Frame layout
// ------------------------------------------------------------

#pragma pack(push, 1)
struct MeshPacket {
    uint8_t  version;       // Always 1 for wire format v1
    uint8_t  type;          // PKT_DATA / PKT_CMD / PKT_ACK / PKT_STATUS
    uint16_t source;        // Sender address (BASE_ID = 0)
    uint16_t destination;   // Receiver address (0xFFFF = broadcast)
    uint16_t packet_id;     // Per-sender sequence number (dedup key)
    uint8_t  payload_size;  // Number of valid bytes in payload[]
    uint8_t  payload[48];
    uint16_t crc;           // CRC-16/Modbus over all preceding bytes
};
#pragma pack(pop)

// Data payload: BME280 + N relays + N voltage sensors.
// num_relays/num_sensors tell the receiver how many entries are valid.
// relay_states and motor_states are bitmasks (bit 0 = channel 0, etc.)
// voltage[] contains RMS voltage for each ZMPT101B channel.
#pragma pack(push, 1)
struct DataPayload {
    float t;                              // BME280 temperature
    float h;                              // BME280 humidity
    float p;                              // BME280 pressure
    uint8_t num_relays;                   // Number of relays on this node
    uint8_t num_sensors;                  // Number of ZMPT101B sensors
    uint8_t relay_states;                 // Bitmask: bit N = relay N state
    uint8_t motor_states;                 // Bitmask: bit N = motor N state
    float voltage[MAX_SENSORS_PER_NODE];  // RMS voltage per channel
};
#pragma pack(pop)

// ACK payload (sent from NODE back to BASE).
// Wire format v1.1 (additive): seq appended at the END.
// Old BASE firmware checks payload_size >= 4 and reads bytes 0..3 —
// it accepts 5-byte ACKs without noticing the extra byte.
#pragma pack(push, 1)
struct AckPayload {
    uint8_t cmd;          // Original command type (1 = relay)
    uint8_t relay_index;  // Which relay was addressed
    uint8_t result;       // Result: 1 = success, 0 = failure (NAK)
    uint8_t state;        // Current state of that relay after command
    uint8_t seq;          // v1.1: echo of CMD payload[3], 0 = not set
};
#pragma pack(pop)

// STATUS payload (additive, PKT_STATUS=4): repeater/node diagnostics.
#pragma pack(push, 1)
struct StatusPayload {
    uint32_t uptime_s;      // Seconds since boot
    uint32_t boot_count;    // NVS boot counter
    uint32_t fwd;           // Frames forwarded (repeater)
    uint32_t rx_dup;        // Duplicate frames dropped by dedup
    uint32_t fwd_drop;      // Forwarded frames dropped (queue/rate limit)
    uint32_t rx_crc_err;    // Frames that passed header but failed CRC
    uint32_t resync_bytes;  // Bytes skipped by frame parser resync
};
#pragma pack(pop)

// ------------------------------------------------------------
// Compile-time guarantees for the frozen wire format
// ------------------------------------------------------------

static_assert(sizeof(MeshPacket) == 59,  "wire format v1 changed");
static_assert(sizeof(DataPayload) == 32, "wire format v1 changed");
static_assert(sizeof(AckPayload)  >= 4,  "wire format v1 changed");
static_assert(sizeof(StatusPayload) <= sizeof(((MeshPacket*)0)->payload),
              "StatusPayload does not fit into MeshPacket payload");

// ------------------------------------------------------------
// CRC-16/Modbus (poly 0xA001 reflected, init 0xFFFF, no final xor)
// ------------------------------------------------------------

uint16_t crc16(const uint8_t* data, uint16_t len);
