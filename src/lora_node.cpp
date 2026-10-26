#if defined(ROLE_NODE)  // P1-8: whole-file role guard (NODE handlers)

// ============================================================
// lora_node.cpp — NODE role: command handling (R0 split)
// ============================================================
// Only built for ROLE_NODE (see the whole-file guard above).
// ============================================================

#include <Arduino.h>
#include "lora_manager.h"
#include "lora_phy.h"
#include "lora_node_rx.h"
#include "config.h"
#include "sensor_manager.h"

// ============================================================
// Command handler (P0-3 / P0-6 / P1-6)
// ============================================================
//
// RX path requirements:
//   - the ACK goes out immediately through the non-blocking TX
//     queue (R3) — never blocks the RX loop;
//   - the full sensor report is DEFERRED by
//     POST_CMD_REPORT_DELAY_MS (P1-6) so the contactor has time to
//     settle and HA does not receive a stale motor=0;
//   - unknown commands and malformed payloads are answered with a
//     NAK instead of silence (P0-3).
// ============================================================

void loraNodeHandlePacket(const MeshPacket& pkt) {
    if (pkt.type != PKT_CMD) {
        return;  // nodes only accept commands
    }

    // R2: only accept commands originated by the BASE.
    if (pkt.source != BASE_ID) {
        Serial.printf("[NODE] CMD from non-BASE source %u ignored\n",
                      (unsigned)pkt.source);
        return;
    }

    // R2: dedup already applied centrally; a retried command carries
    // a new packet_id and is executed again (commands are idempotent,
    // relaySet skips the NVS write when nothing changes).

    if (pkt.destination != NODE_ID) {
        return;  // existing address filter
    }

    // Legacy minimum: payload_size < 3 cannot carry cmd/relay/value.
    // P0-3: answer with NAK instead of silence.
    if (pkt.payload_size < 3) {
        Serial.printf("[NODE] CMD too short (%u) — NAK\n",
                      (unsigned)pkt.payload_size);
        sendAck(pkt.source, pkt.payload_size >= 1 ? pkt.payload[0] : 0,
                0, 0, 0, 0);
        return;
    }

    uint8_t cmd        = pkt.payload[0];
    uint8_t relay_idx  = pkt.payload[1];
    uint8_t value      = pkt.payload[2];
    // v1.1: seq echo for ACK matching (0 = legacy command without seq)
    uint8_t seq        = (pkt.payload_size >= 4) ? pkt.payload[3] : 0;

    if (cmd != 1) {
        Serial.printf("[NODE] Unknown command %u — NAK\n", (unsigned)cmd);
        sendAck(pkt.source, cmd, relay_idx, 0, 0, seq);
        return;
    }

    if (relay_idx >= NODE_NUM_RELAYS) {
        Serial.printf("[NODE] Invalid relay index: %u (max %u)\n",
                      (unsigned)relay_idx, (unsigned)(NODE_NUM_RELAYS - 1));
        sendAck(pkt.source, cmd, relay_idx, 0, 0, seq);
        return;
    }

    // P0-6: single mutation point — writes the pin, updates the
    // shadow, saves NVS only when the bit actually changes.
    relaySet(relay_idx, value == 1);
    Serial.printf("[NODE] Relay %u %s\n",
                  (unsigned)(relay_idx + 1), value ? "ON" : "OFF");

    // ACK immediately with the resulting state (idempotent commands
    // are still ACKed — P0-3).
    uint8_t new_state = relayGet(relay_idx);
    sendAck(pkt.source, cmd, relay_idx, 1, new_state, seq);

    // P1-6: full report later — repeated commands just re-schedule.
    scheduleReport(POST_CMD_REPORT_DELAY_MS);
}

#endif  // ROLE_NODE
