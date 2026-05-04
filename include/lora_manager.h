#pragma once
#include <cstdint>

// ============================================================
// Per-Node Tracking (Heartbeat + Online Detection)
// ============================================================

struct NodeInfo {
    uint32_t last_seen_ms;    // millis() when last packet was received
    bool     ever_seen;       // Whether this node has ever been seen
    bool     is_online;       // Current online state (tracks transitions)
};

// ============================================================
// Pending Command (ACK Retransmission Queue)
// ============================================================

struct PendingCommand {
    uint16_t node_id;         // Target node address
    uint8_t  cmd;             // Command type (1 = relay)
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

void sendData(float t, float h, float p, uint8_t relay, float voltage, uint8_t motor);
void sendCommand(uint16_t node, uint8_t cmd, uint8_t value);

// ============================================================
// ACK Functions (BASE side)
// ============================================================

/**
 * Send a command to a node and wait for ACK with automatic retries.
 * Non-blocking: stores the command and processes ACK/retries in loraLoop().
 *
 * @param node   Target node address
 * @param cmd    Command type (1 = relay)
 * @param value  Command value (0 = off, 1 = on)
 */
void sendCommandWithAck(uint16_t node, uint8_t cmd, uint8_t value);

/**
 * Process pending ACK timeouts and retries.
 * Must be called periodically from the main loop.
 */
void loraProcessPendingAcks();

// ============================================================
// Heartbeat / Online Functions (BASE side)
// ============================================================

/**
 * Check if a node is currently considered online.
 * A node is online if it has been heard from within HEARTBEAT_OFFLINE_TIMEOUT.
 *
 * @param node_id  The node address
 * @return         true if online, false if offline or never seen
 */
bool loraIsNodeOnline(uint16_t node_id);

/**
 * Update last_seen timestamp for a node.
 * Called when any packet is received from the node.
 *
 * @param node_id  The node address
 */
void loraUpdateNodeSeen(uint16_t node_id);

/**
 * Check all tracked nodes for online/offline status transitions.
 * Publishes state changes via MQTT only when status actually changes.
 * Must be called periodically from the main loop.
 */
void loraCheckHeartbeat();

/**
 * Re-publish online/offline status for all ever-seen nodes.
 * Should be called after MQTT reconnect to ensure HA has current state.
 */
void loraPublishAllNodeStatuses();