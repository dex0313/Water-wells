#pragma once
#include <cstdint>
#include "ack_match.h"

// ============================================================
// lora_manager.h — public LoRa API
// ============================================================
// The implementation is split by role (R0 / P1-8):
//   lora_phy.cpp       UART/AUX/modes + non-blocking TX queue
//   lora_rx.cpp        frame parser, dedup, dispatch
//   lora_base.cpp      ROLE_BASE handlers (data, ACK, status)
//   lora_node.cpp      ROLE_NODE handlers (commands, NAK)
//   lora_repeater.cpp  ROLE_REPEATER forwarder + heartbeat
// The public API below is unchanged from the original firmware
// (plus additive helpers marked with the revision note they implement).
// ============================================================

// ------------------------------------------------------------
// Per-Node Tracking (Heartbeat + Online Detection)
// ------------------------------------------------------------

struct NodeInfo {
    uint32_t last_seen_ms;    // millis() when last packet was received
    bool     ever_seen;       // Whether this node has ever been seen
    bool     is_online;       // Current online state (tracks transitions)
    uint8_t  num_relays;      // Number of relays reported by node (0 = unknown)
    uint8_t  num_sensors;     // Number of voltage sensors reported by node
};

// PendingCommand is defined in ack_match.h (kept pure for host tests)

// Maximum concurrent pending commands
#define MAX_PENDING_COMMANDS 8

// ------------------------------------------------------------
// Core LoRa Functions
// ------------------------------------------------------------

void loraInit();
void loraLoop();

void sendData(float t, float h, float p,
              uint8_t num_relays, uint8_t num_sensors,
              uint8_t relay_states, uint8_t motor_states,
              const float voltages[]);
void sendCommand(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value);

// ------------------------------------------------------------
// ACK Functions (BASE side)
// ------------------------------------------------------------

void sendCommandWithAck(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value);
void loraProcessPendingAcks();

// ------------------------------------------------------------
// Heartbeat / Online Functions (BASE side)
// ------------------------------------------------------------

bool loraIsNodeOnline(uint16_t node_id);
void loraUpdateNodeSeen(uint16_t node_id);
void loraCheckHeartbeat();
void loraPublishAllNodeStatuses();

// ------------------------------------------------------------
// Additive helpers (P1-1 / P0-4)
// ------------------------------------------------------------

// Reset the per-node discovery flags so every node re-publishes its
// HA discovery on its next packet (called after MQTT reconnect and
// on homeassistant/status = online).
void loraResetDiscovery();

// Called by mqtt_manager after a successful MQTT connect or when
// Home Assistant announces "online" on homeassistant/status:
// resets discovery flags, re-publishes discovery for nodes with a
// known profile and refreshes all online statuses (P1-1.3/P1-1.4).
void loraOnMqttConnected();

// Node profile access for the MQTT command parser (P0-4).
bool    loraNodeEverSeen(uint16_t node_id);
uint8_t loraNodeNumRelays(uint16_t node_id);

// ------------------------------------------------------------
// REPEATER role (R8.2)
// ------------------------------------------------------------

// Periodic PKT_STATUS sender; call from the repeater main loop.
void repeaterStatusLoop();
