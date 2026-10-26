#if defined(ROLE_BASE)  // P1-8: whole-file role guard (BASE handlers)

// ============================================================
// lora_base.cpp — BASE role: data/ACK/status handling (R0 split)
// ============================================================
// Only built for ROLE_BASE (see the whole-file guard above).
// ============================================================

#include <Arduino.h>
#include <math.h>
#include "lora_manager.h"
#include "lora_phy.h"
#include "lora_base_rx.h"
#include "mqtt_manager.h"
#include "config.h"
#include "dedup.h"
#include "ack_match.h"

// ============================================================
// Per-Node Tracking (Heartbeat)
// ============================================================

static NodeInfo nodeInfo[256] = {};
static bool discovered[256] = {};        // P1-1.2: module scope
static bool statsDiscovered[256] = {};   // PKT_STATUS entities (R8.2)

static uint32_t lastStatsPublish = 0;

void loraUpdateNodeSeen(uint16_t node_id) {
    if (node_id < 256) {
        bool first_seen = !nodeInfo[node_id].ever_seen;
        bool was_offline = !first_seen && !nodeInfo[node_id].is_online;

        nodeInfo[node_id].last_seen_ms = millis();
        nodeInfo[node_id].ever_seen = true;
        nodeInfo[node_id].is_online = true;

        // Publish "1" on any transition to online
        if (first_seen || was_offline) {
            char topic[64];
            snprintf(topic, sizeof(topic), "lora/node_%u/online", (unsigned)node_id);
            mqttPublish(topic, "1", true);
            if (first_seen) {
                Serial.printf("[Heartbeat] Node %u -> FIRST SEEN (online)\n", (unsigned)node_id);
            } else {
                Serial.printf("[Heartbeat] Node %u -> ONLINE (packet received)\n", (unsigned)node_id);
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

bool loraNodeEverSeen(uint16_t node_id) {
    if (node_id >= 256) return false;
    return nodeInfo[node_id].ever_seen;
}

uint8_t loraNodeNumRelays(uint16_t node_id) {
    if (node_id >= 256) return 0;
    return nodeInfo[node_id].num_relays;
}

// ============================================================
// Command status / relay publishing helpers
// ============================================================

static void publishCmdStatus(uint16_t node, const char* status) {
    char topic[64];
    snprintf(topic, sizeof(topic), "lora/node_%u/cmd_status", (unsigned)node);
    // P0-3.6: cmd_status is NOT retained — a stale "ok" must not
    // survive reboots; "pending" is published at command send instead.
    mqttPublish(topic, status, false);
}

static void publishRelayState(uint16_t node, uint8_t relay_index, uint8_t state) {
    char topic[64], buf[8];
    snprintf(topic, sizeof(topic), "lora/node_%u/relay_%u",
             (unsigned)node, (unsigned)(relay_index + 1));
    snprintf(buf, sizeof(buf), "%u", (unsigned)state);
    mqttPublish(topic, buf, true);   // telemetry stays retained
}

// ============================================================
// Pending ACK Commands (BASE side)
// ============================================================

static PendingCommand pendingCmds[MAX_PENDING_COMMANDS] = {};
static unsigned long lastAckCheck = 0;

void sendCommand(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value, uint8_t seq) {
    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_CMD;
    pkt.source      = BASE_ID;
    pkt.destination = node;

    // Wire format v1 layout: [0]=cmd [1]=relay_index [2]=value.
    // v1.1 additive: [3]=seq (old nodes read only bytes 0..2).
    pkt.payload[0] = cmd;
    pkt.payload[1] = relay_index;
    pkt.payload[2] = value;
    pkt.payload[3] = seq;
    pkt.payload_size = 4;

    // R3: non-blocking enqueue; each (re)transmission carries a NEW
    // packet_id so repeater dedup cannot swallow retries (P1-10).
    txEnqueue(pkt, TXP_CMD, 0, true);
}

void sendCommandWithAck(uint16_t node, uint8_t cmd, uint8_t relay_index, uint8_t value) {
    // P0-3.2: preemption — an active command for the same (node,
    // relay) is REPLACED so the last command always wins and no
    // duplicates accumulate on one relay.
    int8_t slot = ackPickSlot(pendingCmds, MAX_PENDING_COMMANDS,
                              node, cmd, relay_index);

    if (slot < 0) {
        Serial.println("[ACK] No free pending slot! Dropping command.");
        publishCmdStatus(node, "failed:no_slot");
        return;
    }

    // P0-3: command sequence numbers 1..255 (0 = "not set", used by
    // legacy paths only).
    static uint8_t seqCounter = 0;
    seqCounter = (seqCounter >= 255) ? 1 : (uint8_t)(seqCounter + 1);

    PendingCommand& pc = pendingCmds[slot];
    pc.node_id = node;
    pc.cmd = cmd;
    pc.relay_index = relay_index;
    pc.value = value;
    pc.seq = seqCounter;
    // P0-4: a node that was never seen on air gets a single retry —
    // saves the channel when the address is stale.
    pc.retries_left = loraNodeEverSeen(node) ? ACK_MAX_RETRIES : 1;
    pc.active = true;

    Serial.printf("[ACK] Command queued: node=%u cmd=%u relay=%u val=%u seq=%u (slot %d)\n",
                  (unsigned)node, (unsigned)cmd, (unsigned)relay_index,
                  (unsigned)value, (unsigned)seqCounter, slot);

    sendCommand(node, cmd, relay_index, value, pc.seq);
    // P0-3.5: the timer starts AFTER the send call returns. With the
    // non-blocking TX queue (R3) this is immediate.
    pc.sent_at_ms = millis();

    // P0-3.6: mark the new command as pending so a retained "ok" from
    // an earlier command is cleared right away.
    publishCmdStatus(node, "pending");
}

void loraProcessPendingAcks() {
    if (millis() - lastAckCheck < ACK_CHECK_INTERVAL) return;
    lastAckCheck = millis();

    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (!pendingCmds[i].active) continue;

        PendingCommand& pc = pendingCmds[i];
        uint32_t elapsed = millis() - pc.sent_at_ms;

        if (elapsed < ACK_TIMEOUT_MS) continue;

        if (pc.retries_left > 0) {
            pc.retries_left--;
            Serial.printf("[ACK] Timeout! Retrying node=%u relay=%u seq=%u (retries left=%u)\n",
                          (unsigned)pc.node_id, (unsigned)pc.relay_index,
                          (unsigned)pc.seq, (unsigned)pc.retries_left);

            // Retry with a NEW packet_id (P1-10); correlation via seq.
            sendCommand(pc.node_id, pc.cmd, pc.relay_index, pc.value, pc.seq);
            pc.sent_at_ms = millis();   // P0-3.5: after the send returns
        } else {
            Serial.printf("[ACK] FAILED: node=%u cmd=%u relay=%u — all retries exhausted\n",
                          (unsigned)pc.node_id, (unsigned)pc.cmd, (unsigned)pc.relay_index);

            publishCmdStatus(pc.node_id, "failed:no_ack");
            pc.active = false;
        }
    }
}

// ============================================================
// ACK handling (P0-3)
// ============================================================

static void handleAck(const MeshPacket& pkt) {
    Serial.printf("[LoRa] RX ACK from node %u\n", (unsigned)pkt.source);
    loraUpdateNodeSeen(pkt.source);

    if (pkt.payload_size < 4) {  // legacy AckPayload minimum
        Serial.printf("[LoRa] ACK payload too small: %u\n", (unsigned)pkt.payload_size);
        return;
    }

    AckPayload ack{};
    size_t copy_len = (pkt.payload_size < sizeof(ack)) ? pkt.payload_size : sizeof(ack);
    memcpy(&ack, pkt.payload, copy_len);

    AckFrame frame;
    frame.cmd = ack.cmd;
    frame.relay_index = ack.relay_index;
    frame.result = ack.result;
    frame.state = ack.state;
    // v1.1 nodes echo seq (5-byte payload); legacy nodes do not.
    frame.has_seq = (pkt.payload_size >= sizeof(AckPayload));
    frame.seq = ack.seq;

    Serial.printf("[ACK] cmd=%u relay=%u result=%u state=%u seq=%u%s\n",
                  (unsigned)ack.cmd, (unsigned)ack.relay_index,
                  (unsigned)ack.result, (unsigned)ack.state, (unsigned)ack.seq,
                  frame.has_seq ? "" : " (legacy)");

    for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
        PendingCommand& pc = pendingCmds[i];
        if (!pc.active || pc.node_id != pkt.source) continue;

        AckOutcome outcome = ackEvaluate(pc, frame);
        if (outcome == ACK_IGNORED) continue;

        Serial.printf("[ACK] Matched pending command in slot %u\n", i);

        switch (outcome) {
            case ACK_OK:
                // ack.result == 1 && ack.state == pc.value
                publishCmdStatus(pkt.source, "ok");
                publishRelayState(pkt.source, ack.relay_index, ack.state);
                break;

            case ACK_NAK:
                // ack.result == 0: relay state NOT published (P0-3.4)
                publishCmdStatus(pkt.source, "failed:nak");
                break;

            case ACK_STATE_MISMATCH:
                // ack.result == 1 but the relay is somewhere else:
                // report the REAL state (P0-3.4).
                publishCmdStatus(pkt.source, "failed:state_mismatch");
                publishRelayState(pkt.source, ack.relay_index, ack.state);
                break;

            default:
                break;
        }

        pc.active = false;
        break;  // preemption guarantees a single match per relay
    }
}

// ============================================================
// DATA handling (P0-5 / P0-1 / P2-2 / P1-1)
// ============================================================

static void publishFloatTopic(uint16_t node, const char* name, float value,
                              float lo, float hi) {
    // P0-1 / P2-2: NaN and out-of-range values are not published —
    // the HA entity stays "unknown" instead of showing garbage.
    if (isnan(value) || !isfinite(value) || value < lo || value > hi) {
        Serial.printf("[LoRa] Drop %s (out of range or NaN)\n", name);
        return;
    }
    char topic[64], buf[32];
    snprintf(topic, sizeof(topic), "lora/node_%u/%s", (unsigned)node, name);
    dtostrf(value, 1, 2, buf);
    mqttPublish(topic, buf, true);
}

static void handleData(const MeshPacket& pkt) {
    loraUpdateNodeSeen(pkt.source);
    Serial.printf("[LoRa] RX data from node %u\n", (unsigned)pkt.source);

    // P0-5: validate BEFORE discovery so a corrupted first packet
    // cannot memorize a wrong profile. Short payloads are dropped —
    // the old "discovery with defaults 1/1" fallback is removed.
    if (pkt.payload_size < sizeof(DataPayload)) {
        Serial.printf("[LoRa] DATA payload too small: %u (need %u)\n",
                      (unsigned)pkt.payload_size, (unsigned)sizeof(DataPayload));
        return;
    }

    DataPayload data;
    memcpy(&data, pkt.payload, sizeof(data));

    if (data.num_relays > MAX_RELAYS_PER_NODE ||
        data.num_sensors > MAX_SENSORS_PER_NODE) {
        Serial.printf("[LoRa] Invalid profile: relays=%u sensors=%u — packet dropped\n",
                      (unsigned)data.num_relays, (unsigned)data.num_sensors);
        return;
    }

    // P1-1/P2-6: if the node profile changed, re-run discovery so
    // new entities appear and stale ones are cleaned up.
    if (discovered[pkt.source] &&
        (nodeInfo[pkt.source].num_relays != data.num_relays ||
         nodeInfo[pkt.source].num_sensors != data.num_sensors)) {
        Serial.printf("[LoRa] Node %u profile changed (%u/%u -> %u/%u) — re-discovery\n",
                      (unsigned)pkt.source,
                      (unsigned)nodeInfo[pkt.source].num_relays,
                      (unsigned)nodeInfo[pkt.source].num_sensors,
                      (unsigned)data.num_relays, (unsigned)data.num_sensors);
        discovered[pkt.source] = false;
    }

    // Discovery on first packet; flags are set only on success (P1-1.1)
    if (!discovered[pkt.source]) {
        nodeInfo[pkt.source].num_relays = data.num_relays;
        nodeInfo[pkt.source].num_sensors = data.num_sensors;

        if (publishDiscovery(pkt.source, data.num_relays, data.num_sensors)) {
            discovered[pkt.source] = true;
        } else {
            Serial.println("[MQTT] Discovery deferred (MQTT offline) — will retry");
        }
    }

    // BME280 (P0-1/P2-2): NaN or implausible values are skipped.
    publishFloatTopic(pkt.source, "temperature", data.t, -60.0f, 100.0f);
    publishFloatTopic(pkt.source, "humidity",    data.h,   0.0f, 100.0f);
    publishFloatTopic(pkt.source, "pressure",    data.p, 300.0f, 1100.0f);

    char topic[64], buf[32];

    // Relays (individual topics with bitmask)
    for (uint8_t i = 0; i < data.num_relays; i++) {
        snprintf(topic, sizeof(topic), "lora/node_%u/relay_%u",
                 (unsigned)pkt.source, (unsigned)(i + 1));
        snprintf(buf, sizeof(buf), "%u", (unsigned)((data.relay_states >> i) & 1));
        mqttPublish(topic, buf, true);
    }

    // Voltages (P0-5): isfinite and 0..500 V only.
    for (uint8_t i = 0; i < data.num_sensors; i++) {
        float v = data.voltage[i];
        if (!isfinite(v) || v < 0.0f || v > 500.0f) {
            Serial.printf("[LoRa] Drop voltage_%u (out of range or NaN)\n",
                          (unsigned)(i + 1));
            continue;
        }
        snprintf(topic, sizeof(topic), "lora/node_%u/voltage_%u",
                 (unsigned)pkt.source, (unsigned)(i + 1));
        dtostrf(v, 1, 1, buf);
        mqttPublish(topic, buf, true);
    }

    // Motors (individual binary states derived from voltage)
    for (uint8_t i = 0; i < data.num_sensors; i++) {
        snprintf(topic, sizeof(topic), "lora/node_%u/motor_%u",
                 (unsigned)pkt.source, (unsigned)(i + 1));
        snprintf(buf, sizeof(buf), "%u", (unsigned)((data.motor_states >> i) & 1));
        mqttPublish(topic, buf, true);
    }
}

// ============================================================
// STATUS handling (R8.2) — repeater/node diagnostics
// ============================================================

static void handleStatus(const MeshPacket& pkt) {
    if (pkt.payload_size < sizeof(StatusPayload)) {
        Serial.printf("[LoRa] STATUS payload too small: %u\n", (unsigned)pkt.payload_size);
        return;
    }

    loraUpdateNodeSeen(pkt.source);
    Serial.printf("[LoRa] RX status from node %u\n", (unsigned)pkt.source);

    StatusPayload st;
    memcpy(&st, pkt.payload, sizeof(st));

    if (!statsDiscovered[pkt.source]) {
        if (publishStatusDiscovery(pkt.source)) {
            statsDiscovered[pkt.source] = true;
        }
    }

    char topic[64], buf[16];
    struct { const char* name; uint32_t value; } fields[] = {
        { "uptime_s",     st.uptime_s },
        { "boot_count",   st.boot_count },
        { "fwd",          st.fwd },
        { "rx_dup",       st.rx_dup },
        { "fwd_drop",     st.fwd_drop },
        { "rx_crc_err",   st.rx_crc_err },
        { "resync_bytes", st.resync_bytes },
    };
    for (uint8_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        snprintf(topic, sizeof(topic), "lora/node_%u/stats/%s",
                 (unsigned)pkt.source, fields[i].name);
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)fields[i].value);
        mqttPublish(topic, buf, true);
    }
}

// ============================================================
// Frame entry point (R2: destination filter)
// ============================================================

void loraBaseHandlePacket(const MeshPacket& pkt) {
    // R2: accept only frames addressed to BASE (or broadcast).
    // With a repeater in the middle, foreign traffic shows up too.
    if (pkt.destination != BASE_ID && pkt.destination != 0xFFFF) {
        return;
    }

    // nodeInfo[]/discovered[] are sized 256; a (CRC-valid) frame with
    // a source above the address space must not index them.
    if (pkt.source >= 256) {
        Serial.printf("[LoRa] Source %u out of range — frame dropped\n",
                      (unsigned)pkt.source);
        return;
    }

    switch (pkt.type) {
        case PKT_DATA:   handleData(pkt);  break;
        case PKT_ACK:    handleAck(pkt);   break;
        case PKT_STATUS: handleStatus(pkt); break;
        default:         break;  // unknown type: ignore silently
    }
}

// ============================================================
// Discovery reset / re-publish (P1-1)
// ============================================================

void loraResetDiscovery() {
    memset(discovered, 0, sizeof(discovered));
    memset(statsDiscovered, 0, sizeof(statsDiscovered));
}

// For nodes with a known profile, publish discovery immediately after
// an MQTT (re)connect instead of waiting for the next packet (P1-1.3).
static void loraPublishKnownDiscovery() {
    for (uint16_t id = 0; id < 256; id++) {
        if (!nodeInfo[id].ever_seen) continue;
        if (nodeInfo[id].num_relays == 0 && nodeInfo[id].num_sensors == 0) {
            continue;  // profile not known yet (or stats-only node)
        }
        if (publishDiscovery(id, nodeInfo[id].num_relays, nodeInfo[id].num_sensors)) {
            discovered[id] = true;
        }
    }
}

void loraPublishAllNodeStatuses() {
    for (uint16_t nodeId = 0; nodeId < 256; nodeId++) {
        if (!nodeInfo[nodeId].ever_seen) continue;

        char topic[64];
        snprintf(topic, sizeof(topic), "lora/node_%u/online", (unsigned)nodeId);

        if (nodeInfo[nodeId].is_online) {
            mqttPublish(topic, "1", true);
        } else {
            mqttPublish(topic, "0", true);
        }
    }
    Serial.println("[Heartbeat] Re-published all node online statuses");
}

// Called by mqtt_manager after a successful connect / HA restart.
void loraOnMqttConnected() {
    loraResetDiscovery();
    loraPublishKnownDiscovery();
    loraPublishAllNodeStatuses();
}

// ============================================================
// LoRa statistics publishing (P1-3) — every 5 minutes
// ============================================================

#define LORA_STATS_PUBLISH_INTERVAL_MS 300000UL

static void publishStat(const char* name, uint32_t value) {
    char topic[64], buf[16];
    snprintf(topic, sizeof(topic), "lora/base/stats/%s", name);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
    mqttPublish(topic, buf, true);
}

static void maybePublishLoraStats() {
    if (millis() - lastStatsPublish < LORA_STATS_PUBLISH_INTERVAL_MS) return;
    lastStatsPublish = millis();

    uint32_t tx_ok = 0, tx_drop = 0;
    loraTxCounters(tx_ok, tx_drop);

    publishStat("rx_ok",         gLoraStats.rx_ok);
    publishStat("rx_crc_err",    gLoraStats.rx_crc_err);
    publishStat("resync_bytes",  gLoraStats.resync_bytes);
    publishStat("rx_dup",        gLoraStats.rx_dup);
    publishStat("tx_ok",         tx_ok);
    publishStat("tx_drop",       tx_drop);
    publishStat("dedup_evicted", dedupEvictedLive());
    Serial.println("[LoRa] Stats published to lora/base/stats/*");
}

// ============================================================
// Heartbeat Check — called periodically from system_manager
// ============================================================

void loraCheckHeartbeat() {
    for (uint16_t nodeId = 0; nodeId < 256; nodeId++) {
        if (!nodeInfo[nodeId].ever_seen) continue;

        bool now_online = (millis() - nodeInfo[nodeId].last_seen_ms) < HEARTBEAT_OFFLINE_TIMEOUT;

        if (now_online != nodeInfo[nodeId].is_online) {
            nodeInfo[nodeId].is_online = now_online;

            char topic[64];
            snprintf(topic, sizeof(topic), "lora/node_%u/online", (unsigned)nodeId);

            if (now_online) {
                mqttPublish(topic, "1", true);
                Serial.printf("[Heartbeat] Node %u -> ONLINE\n", (unsigned)nodeId);
            } else {
                mqttPublish(topic, "0", true);
                Serial.printf("[Heartbeat] Node %u -> OFFLINE (last seen %lu ms ago)\n",
                              (unsigned)nodeId,
                              (unsigned long)(millis() - nodeInfo[nodeId].last_seen_ms));
            }
        }
    }

    maybePublishLoraStats();
}

// ============================================================
// Init (called from loraInit)
// ============================================================

void loraBaseInit() {
    memset(nodeInfo, 0, sizeof(nodeInfo));
    memset(pendingCmds, 0, sizeof(pendingCmds));
    lastAckCheck = 0;
    lastStatsPublish = millis();   // first stats publish after 5 min
    loraResetDiscovery();
}

#endif  // ROLE_BASE
