#pragma once
#include <cstdint>

// ============================================================
// Per-Node Tracking (Heartbeat + Online Detection)
// ============================================================

struct NodeInfo {
    uint32_t last_seen_ms;    // millis() when last packet was received
    bool     ever_seen;       // Whether this node has ever been seen
    bool     is_online;       // Current online state (tracks transitions)
    uint8_t  num_relays;      // Number of relays reported by node (0 = unknown)
    uint8_t  num_sensors;     // Number of voltage sensors reported by node
};

// ============================================================
// Pending Command (ACK Retransmission Queue)
// ============================================================

struct PendingCommand {
    uint16_t node_id;         // Target node address
    uint8_t  cmd;             // Command type (1 = relay)
    uint8_t  relay_index;     // Which relay (0..N-1)
    uint8_t  value;           // Command value (0 or 1)
    uint8_t  retries_left;    // Remaining retry attempts
    uint32_t sent_at_ms;      // millis() when last sent
    bool     active;          // Whether this slot is in use
};

// Maximum concurrent pending commands
#define MAX_PENDING_COMMANDS 8

// ============================================================
// Core LoRa Functions
// ============================================================

void loraInit();
void loraLoop();

void sendData(float t, float h, float p,
              uint8_t num_relays, uint8_t num_sensors,
              uint8_t relay_states, uint8_t motor_states,
              const float voltages[]);
void sendCommand(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value);

// ============================================================
// ACK Functions (BASE side)
// ============================================================

void sendCommandWithAck(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value);
void loraProcessPendingAcks();

// ============================================================
// Heartbeat / Online Functions (BASE side)
// ============================================================

bool loraIsNodeOnline(uint16_t node_id);
void loraUpdateNodeSeen(uint16_t node_id);
void loraCheckHeartbeat();
void loraPublishAllNodeStatuses();