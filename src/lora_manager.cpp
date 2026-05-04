#include <HardwareSerial.h>
#include <Arduino.h>
#include "lora_manager.h"
#include "mqtt_manager.h"
#include "config.h"
#include "sensor_manager.h"

HardwareSerial loraSerial(2);

// ============================================================
// Packet structures
// ============================================================

#pragma pack(push,1)
struct MeshPacket {
    uint8_t version;
    uint8_t type;
    uint16_t source;
    uint16_t destination;
    uint16_t packet_id;
    uint8_t payload_size;
    uint8_t payload[48];
    uint16_t crc;
};
#pragma pack(pop)

#pragma pack(push,1)
struct DataPayload {
    float t;
    float h;
    float p;
    uint8_t relay;
    float voltage;   // ZMPT101B RMS voltage (replaces old uint8_t motor)
    uint8_t motor;   // Derived motor state: 1 = voltage above threshold, 0 = below
};
#pragma pack(pop)

// ACK payload structure (sent from NODE back to BASE)
#pragma pack(push,1)
struct AckPayload {
    uint8_t cmd;       // Original command that was acknowledged (1 = relay)
    uint8_t result;    // Result: 1 = success, 0 = failure
    uint8_t state;     // Current state after command (relay: 0 or 1)
};
#pragma pack(pop)

// ============================================================
// CRC-16/Modbus
// ============================================================

static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

// ============================================================
// LoRa E22 helpers
// ============================================================

static bool waitAux(uint32_t timeout = 1000) {
    uint32_t start = millis();
    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - start > timeout) {
            Serial.println("[LoRa] AUX timeout");
            return false;
        }
        delay(1);
    }
    return true;
}

static void setMode(uint8_t mode) {
    switch (mode) {
        case LORA_MODE_NORMAL:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, LOW);
            break;

        case LORA_MODE_WAKEUP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, LOW);
            break;

        case LORA_MODE_POWER:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, HIGH);
            break;

        case LORA_MODE_SLEEP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, HIGH);
            break;

        default:
            return;
    }

    delay(50);
    waitAux();
}

// ============================================================
// Packet TX
// ============================================================

static uint16_t counter = 0;

static void sendPacket(MeshPacket& pkt) {
    pkt.packet_id = counter++;
    pkt.crc = crc16(reinterpret_cast<uint8_t*>(&pkt),
                     sizeof(MeshPacket) - sizeof(uint16_t));

    if (!waitAux()) {
        Serial.println("[LoRa] Not ready (before TX)");
        return;
    }

    loraSerial.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(MeshPacket));
    loraSerial.flush();

    if (!waitAux()) {
        Serial.println("[LoRa] TX not completed");
        return;
    }

    Serial.printf("[LoRa] TX OK (id=%d, type=%d, dst=%d)\n",
                  pkt.packet_id, pkt.type, pkt.destination);
}

// ============================================================
// Per-Node Tracking (Heartbeat)
// ============================================================

static NodeInfo nodeInfo[256] = {};  // Track up to 256 node IDs

void loraUpdateNodeSeen(uint16_t node_id) {
    if (node_id < 256) {
        bool first_seen = !nodeInfo[node_id].ever_seen;
        bool was_offline = !first_seen && !nodeInfo[node_id].is_online;

        nodeInfo[node_id].last_seen_ms = millis();
        nodeInfo[node_id].ever_seen = true;
        nodeInfo[node_id].is_online = true;

        // Publish "1" on any transition to online
        // (first time seen, or returning from offline)
        if (first_seen || was_offline) {
            char topic[64];
            snprintf(topic, sizeof(topic), "lora/node_%d/online", node_id);
            mqttPublish(topic, "1", true);
            if (first_seen) {
                Serial.printf("[Heartbeat] Node %d -> FIRST SEEN (online)\n", node_id);
            } else {
                Serial.printf("[Heartbeat] Node %d -> ONLINE (packet received)\n", node_id);
            }
        }
    }
}

bool loraIsNodeOnline(uint16_t node_id) {
    if (node_id >= 256 || !nodeInfo[node_id].ever_seen) {
        return false;
    }
    return (millis() - nodeInfo[node_id].last_seen_ms) < HEARTBEAT_OFFLINE_TIMEOUT;
}

// ============================================================
// Pending ACK Commands (BASE side)
// ============================================================

static PendingCommand pendingCmds[MAX_PENDING_COMMANDS] = {};
static unsigned long lastAckCheck = 0;

void sendCommandWithAck(uint16_t node, uint8_t cmd, uint8_t value) {
#ifdef ROLE_BASE
    // Find a free slot in the pending commands array
    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (!pendingCmds[i].active) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        Serial.println("[ACK] No free pending slot! Dropping command.");
        // Report failure to MQTT
        char topic[64];
        snprintf(topic, sizeof(topic), "lora/node_%d/cmd_status", node);
        mqttPublish(topic, "failed:no_slot", true);
        return;
    }

    // Store the pending command
    pendingCmds[slot].node_id = node;
    pendingCmds[slot].cmd = cmd;
    pendingCmds[slot].value = value;
    pendingCmds[slot].retries_left = ACK_MAX_RETRIES;
    pendingCmds[slot].active = true;
    pendingCmds[slot].sent_at_ms = millis();

    // Send the command immediately
    sendCommand(node, cmd, value);

    Serial.printf("[ACK] Command queued: node=%d cmd=%d val=%d (slot %d, %d retries left)\n",
                  node, cmd, value, slot, ACK_MAX_RETRIES);
#endif
}

void loraProcessPendingAcks() {
#ifdef ROLE_BASE
    if (millis() - lastAckCheck < ACK_CHECK_INTERVAL) return;
    lastAckCheck = millis();

    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (!pendingCmds[i].active) continue;

        PendingCommand& pc = pendingCmds[i];
        uint32_t elapsed = millis() - pc.sent_at_ms;

        // Check if timeout has elapsed
        if (elapsed < ACK_TIMEOUT_MS) continue;

        if (pc.retries_left > 0) {
            // Retry: resend the command
            pc.retries_left--;
            pc.sent_at_ms = millis();

            Serial.printf("[ACK] Timeout! Retrying node=%d (retries left=%d)\n",
                          pc.node_id, pc.retries_left);

            sendCommand(pc.node_id, pc.cmd, pc.value);
        } else {
            // All retries exhausted — report failure
            Serial.printf("[ACK] FAILED: node=%d cmd=%d — all retries exhausted\n",
                          pc.node_id, pc.cmd);

            char topic[64];
            snprintf(topic, sizeof(topic), "lora/node_%d/cmd_status", pc.node_id);
            mqttPublish(topic, "failed:no_ack", true);

            pc.active = false;  // Free the slot
        }
    }
#endif
}

// ============================================================
// Init
// ============================================================

void loraInit() {

    pinMode(LORA_AUX, INPUT);
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);

    setMode(LORA_MODE_NORMAL);

    loraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);

    delay(100);
    waitAux();

    Serial.println("LoRa init done");

    // Initialize node tracking
    // IMPORTANT: is_online = false on startup ensures that after BASE
    // restart, all nodes are considered OFFLINE until they send data.
    // This prevents stale retained "1" MQTT messages from keeping
    // nodes appearing online when they are not.
    for (uint16_t i = 0; i < 256; i++) {
        nodeInfo[i].last_seen_ms = 0;
        nodeInfo[i].ever_seen = false;
        nodeInfo[i].is_online = false;
    }

    // Initialize pending commands
    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
        pendingCmds[i].active = false;
    }
}

// ============================================================
// Send data (from node to base)
// ============================================================

void sendData(float t, float h, float p, uint8_t relay, float voltage, uint8_t motor) {
#ifndef ROLE_BASE
    MeshPacket pkt{};
    pkt.version = 1;
    pkt.type    = PKT_DATA;
    pkt.source  = NODE_ID;
    pkt.destination = BASE_ID;

    DataPayload payload;
    payload.t     = t;
    payload.h     = h;
    payload.p     = p;
    payload.relay = relay;
    payload.voltage = voltage;
    payload.motor = motor;

    memcpy(pkt.payload, &payload, sizeof(payload));
    pkt.payload_size = sizeof(payload);

    sendPacket(pkt);
#endif
}

// ============================================================
// Send command (from base to node)
// ============================================================

void sendCommand(uint16_t node, uint8_t cmd, uint8_t value) {
#ifdef ROLE_BASE
    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_CMD;
    pkt.source      = BASE_ID;
    pkt.destination = node;

    pkt.payload[0] = cmd;
    pkt.payload[1] = value;
    pkt.payload_size = 2;

    sendPacket(pkt);
#endif
}

// ============================================================
// Send ACK (from node back to base)
// ============================================================

static void sendAck(uint16_t destination, uint8_t cmd, uint8_t result, uint8_t state) {
#ifndef ROLE_BASE
    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_ACK;
    pkt.source      = NODE_ID;
    pkt.destination = destination;

    AckPayload ack;
    ack.cmd    = cmd;
    ack.result = result;
    ack.state  = state;

    memcpy(pkt.payload, &ack, sizeof(ack));
    pkt.payload_size = sizeof(ack);

    sendPacket(pkt);

    Serial.printf("[NODE] ACK sent: cmd=%d result=%d state=%d\n", cmd, result, state);
#endif
}

// ============================================================
// RX loop with packet boundary detection
// ============================================================

// Fixed-size receive buffer
static uint8_t rxBuffer[sizeof(MeshPacket)];
static uint16_t rxIndex = 0;
static unsigned long rxStartTime = 0;
static constexpr uint32_t RX_TIMEOUT_MS = 500; // timeout to reset stale partial packet

void loraLoop() {
    while (loraSerial.available()) {

        // Reset buffer if we started receiving but timed out
        if (rxIndex > 0 && (millis() - rxStartTime > RX_TIMEOUT_MS)) {
            Serial.println("[LoRa] RX timeout - resetting buffer");
            rxIndex = 0;
        }

        rxBuffer[rxIndex++] = loraSerial.read();

        if (rxIndex == 1) {
            rxStartTime = millis();
        }

        if (rxIndex >= sizeof(MeshPacket)) {
            rxIndex = 0; // always reset after consuming a full frame

            MeshPacket pkt;
            memcpy(&pkt, rxBuffer, sizeof(pkt));

            // Verify CRC
            uint16_t computed = crc16(
                reinterpret_cast<uint8_t*>(&pkt),
                sizeof(MeshPacket) - sizeof(uint16_t));

            if (computed != pkt.crc) {
                Serial.printf("[LoRa] CRC error: expected 0x%04X got 0x%04X\n",
                              pkt.crc, computed);
                continue;
            }

            // Validate version
            if (pkt.version != 1) {
                Serial.printf("[LoRa] Unknown version: %d\n", pkt.version);
                continue;
            }

            // ---- BASE: receive sensor data and ACKs from nodes ----
#ifdef ROLE_BASE

            if (pkt.type == PKT_DATA) {
                // Update node heartbeat
                loraUpdateNodeSeen(pkt.source);

                Serial.printf("[LoRa] RX data from node %d\n", pkt.source);

                // Send discovery if this is a new node
                static bool discovered[256] = {false};

                if (pkt.source < 256 && !discovered[pkt.source]) {
                    publishDiscovery(pkt.source);
                    discovered[pkt.source] = true;
                } else if (pkt.source >= 256) {
                    Serial.printf("[LoRa] Node ID %d exceeds discovery array\n",
                                  pkt.source);
                }

                if (pkt.payload_size >= sizeof(DataPayload)) {
                    DataPayload data;
                    memcpy(&data, pkt.payload, sizeof(data));

                    char topic[64], buf[32];

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/temperature", pkt.source);
                    dtostrf(data.t, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/humidity", pkt.source);
                    dtostrf(data.h, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/pressure", pkt.source);
                    dtostrf(data.p, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/relay", pkt.source);
                    snprintf(buf, sizeof(buf), "%d", data.relay);
                    mqttPublish(topic, buf, true);

                    // Voltage from ZMPT101B
                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/voltage", pkt.source);
                    dtostrf(data.voltage, 1, 1, buf);
                    mqttPublish(topic, buf, true);

                    // Motor binary state (derived from voltage)
                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/motor", pkt.source);
                    snprintf(buf, sizeof(buf), "%d", data.motor);
                    mqttPublish(topic, buf, true);

                    // Note: online status transition is handled by
                    // loraUpdateNodeSeen() called above — it publishes
                    // "1" when a node transitions from offline to online.
                    // First-time nodes are published online below.
                } else {
                    Serial.printf("[LoRa] Payload too small: %d bytes\n",
                                  pkt.payload_size);
                }
            }

            // ---- BASE: receive ACK from node ----
            else if (pkt.type == PKT_ACK) {
                Serial.printf("[LoRa] RX ACK from node %d\n", pkt.source);

                // Update node heartbeat (also handles online transition publish)
                loraUpdateNodeSeen(pkt.source);

                if (pkt.payload_size >= sizeof(AckPayload)) {
                    AckPayload ack;
                    memcpy(&ack, pkt.payload, sizeof(ack));

                    Serial.printf("[ACK] cmd=%d result=%d state=%d\n",
                                  ack.cmd, ack.result, ack.state);

                    // Find matching pending command and clear it
                    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
                        if (pendingCmds[i].active &&
                            pendingCmds[i].node_id == pkt.source &&
                            pendingCmds[i].cmd == ack.cmd) {

                            Serial.printf("[ACK] Matched pending command in slot %d\n", i);

                            // Publish success status
                            char topic[64];
                            snprintf(topic, sizeof(topic),
                                     "lora/node_%d/cmd_status", pkt.source);
                            mqttPublish(topic, "ok", true);

                            // If ACK reports actual state, publish it immediately
                            if (ack.cmd == 1) {  // relay command
                                snprintf(topic, sizeof(topic),
                                         "lora/node_%d/relay", pkt.source);
                                char buf[8];
                                snprintf(buf, sizeof(buf), "%d", ack.state);
                                mqttPublish(topic, buf, true);
                            }

                            pendingCmds[i].active = false;  // Free the slot
                            break;
                        }
                    }
                }
            }

#else
            // ---- NODE: receive commands from base ----
            if (pkt.type == PKT_CMD && pkt.destination == NODE_ID) {
                if (pkt.payload[0] == 1 && pkt.payload_size >= 2) {
                    uint8_t value = pkt.payload[1];

                    if (value == 1) {
                        digitalWrite(RELAY_PIN, RELAY_ON);
                        Serial.println("[NODE] Relay ON");
                    } else {
                        digitalWrite(RELAY_PIN, RELAY_OFF);
                        Serial.println("[NODE] Relay OFF");
                    }

                    // Read current state after execution
                    uint8_t relay_state = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;

                    // NEW: Send ACK back to BASE
                    sendAck(pkt.source, 1, 1, relay_state);

                    // Also send back full sensor data (same as before)
                    float t = 0, h = 0, p = 0;
                    float voltage = 0;
                    uint8_t motor_state = 0;

                    if (sensorAvailable()) {
                        t = bme.readTemperature();
                        h = bme.readHumidity();
                        p = bme.readPressure() / 100.0F;
                    }
                    voltage = readZMPT101B();
                    motor_state = (voltage > MOTOR_VOLTAGE_THRESHOLD) ? 1 : 0;

                    sendData(t, h, p, relay_state, voltage, motor_state);
                }
            }

#endif
        }
    }
}

// ============================================================
// Heartbeat Check — called periodically from system_manager
// ============================================================

void loraCheckHeartbeat() {
#ifdef ROLE_BASE
    for (uint16_t nodeId = 0; nodeId < 256; nodeId++) {
        if (!nodeInfo[nodeId].ever_seen) continue;

        bool now_online = (millis() - nodeInfo[nodeId].last_seen_ms) < HEARTBEAT_OFFLINE_TIMEOUT;

        // Only publish on state TRANSITION (avoids MQTT spam)
        if (now_online != nodeInfo[nodeId].is_online) {
            nodeInfo[nodeId].is_online = now_online;

            char topic[64];
            snprintf(topic, sizeof(topic), "lora/node_%d/online", nodeId);

            if (now_online) {
                mqttPublish(topic, "1", true);
                Serial.printf("[Heartbeat] Node %d -> ONLINE\n", nodeId);
            } else {
                mqttPublish(topic, "0", true);
                Serial.printf("[Heartbeat] Node %d -> OFFLINE (last seen %lu ms ago)\n",
                              nodeId, millis() - nodeInfo[nodeId].last_seen_ms);
            }
        }
    }
#endif
}

// ============================================================
// Re-publish all node statuses (called after MQTT reconnect)
// ============================================================

void loraPublishAllNodeStatuses() {
#ifdef ROLE_BASE
    for (uint16_t nodeId = 0; nodeId < 256; nodeId++) {
        if (!nodeInfo[nodeId].ever_seen) continue;

        char topic[64];
        snprintf(topic, sizeof(topic), "lora/node_%d/online", nodeId);

        if (nodeInfo[nodeId].is_online) {
            mqttPublish(topic, "1", true);
        } else {
            mqttPublish(topic, "0", true);
        }
    }
    Serial.println("[Heartbeat] Re-published all node online statuses");
#endif
}