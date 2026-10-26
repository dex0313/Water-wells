#if defined(ROLE_REPEATER)  // P1-8: whole-file role guard (repeater forwarder)

// ============================================================
// lora_repeater.cpp — REPEATER role (R5 / R7 / R8)
// ============================================================
// Only built for ROLE_REPEATER (see the whole-file guard above).
//
// The repeater receives validated frames (header + CRC checked by
// the frame parser) and re-transmits the SAME frame byte-for-byte
// (same source, packet_id, CRC) after a small random jitter, so
// nodes outside BASE coverage still reach it.
// ============================================================

#include <Arduino.h>
#include <esp_system.h>
#include "lora_manager.h"
#include "lora_phy.h"
#include "lora_repeater.h"
#include "config.h"
#include "persistence.h"
#include "dedup.h"

// ------------------------------------------------------------
// Rate limiting (R7): max FWD_MAX_PER_MIN forwarded frames per
// rolling minute — protection against retransmission storms.
// ------------------------------------------------------------
static uint32_t fwdWindowStart = 0;
static uint16_t fwdWindowCount = 0;

static bool fwdRateAllow(uint32_t now) {
    if (now - fwdWindowStart >= 60000UL) {
        fwdWindowStart = now;
        fwdWindowCount = 0;
    }
    if (fwdWindowCount >= FWD_MAX_PER_MIN) {
        return false;
    }
    fwdWindowCount++;
    return true;
}

// ------------------------------------------------------------
// Local handling of frames addressed to the repeater itself
// ------------------------------------------------------------

static void handleLocal(const MeshPacket& pkt) {
    if (pkt.type == PKT_CMD) {
        // The repeater has no relays; answer with NAK (P0-3 style).
        uint8_t cmd = (pkt.payload_size >= 1) ? pkt.payload[0] : 0;
        uint8_t relay_idx = (pkt.payload_size >= 2) ? pkt.payload[1] : 0;
        uint8_t seq = (pkt.payload_size >= 4) ? pkt.payload[3] : 0;
        Serial.printf("[REPEATER] CMD %u addressed to me — NAK\n", (unsigned)cmd);
        sendAck(pkt.source, cmd, relay_idx, 0, 0, seq);
        return;
    }
    // DATA/STATUS/ACK addressed to me: nothing to do locally.
    Serial.printf("[REPEATER] Frame type %u addressed to me — local\n",
                  (unsigned)pkt.type);
}

// ------------------------------------------------------------
// Forwarding decision (R7)
// ------------------------------------------------------------

void loraRepeaterHandlePacket(const MeshPacket& pkt) {
    // Own echo: never forward our own transmissions back.
    if (pkt.source == NODE_ID) {
        return;
    }

    // NOTE (R2/R4/R7): dedup by (source, packet_id) is applied
    // centrally in lora_rx.cpp for ALL roles, including the
    // repeater — CMD/ACK frames are deduplicated as well.

    // Frames addressed to the repeater itself are not forwarded.
    if (pkt.destination == NODE_ID) {
        handleLocal(pkt);
        return;
    }

    // Unknown types are not relayed (muscle memory of the parser
    // already filters type > PKT_TYPE_MAX; this also guards the
    // lower bound).
    if (pkt.type < PKT_DATA || pkt.type > PKT_TYPE_MAX) {
        return;
    }

    // Random jitter before forwarding: avoids colliding with the
    // direct transmission the addressee may still be receiving and
    // with other repeaters (half-duplex channel).
    TxPrio prio = (pkt.type == PKT_ACK) ? TXP_ACK
                : (pkt.type == PKT_CMD) ? TXP_CMD
                : TXP_FWD;
    uint32_t jitter = 100 + esp_random() % (FWD_JITTER_MAX_MS - 100);

    if (!fwdRateAllow(millis())) {
        Serial.println("[REPEATER] Forward rate limit — frame dropped");
        gLoraStats.fwd_drop++;
        return;
    }

    // assign_new_id=false: the frame goes out byte-for-byte.
    if (txEnqueue(pkt, prio, jitter, /*assign_new_id=*/false)) {
        gLoraStats.fwd++;
    } else {
        gLoraStats.fwd_drop++;
    }
}

// ------------------------------------------------------------
// Heartbeat (R8)
// ------------------------------------------------------------
// 1) Every SENSOR_INTERVAL (with P1-5 jitter) the repeater sends a
//    normal PKT_DATA with num_relays=0 / num_sensors=0 and BME280
//    values (or NaN) — old BASE builds display it as an online
//    node with temperature/humidity/pressure.
// 2) Every REPEATER_STATUS_INTERVAL_MS it additionally sends the
//    additive PKT_STATUS with uptime and counters — old BASE
//    silently ignores the unknown type.
// ------------------------------------------------------------

static uint32_t lastStatusSent = 0;

void repeaterStatusLoop() {
    if (millis() - lastStatusSent < REPEATER_STATUS_INTERVAL_MS) return;
    lastStatusSent = millis();

    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_STATUS;
    pkt.source      = NODE_ID;
    pkt.destination = BASE_ID;

    StatusPayload st{};
    st.uptime_s     = millis() / 1000UL;
    st.boot_count   = nvsBootCount();
    st.fwd          = gLoraStats.fwd;
    st.rx_dup       = gLoraStats.rx_dup;
    st.fwd_drop     = gLoraStats.fwd_drop;
    st.rx_crc_err   = gLoraStats.rx_crc_err;
    st.resync_bytes = gLoraStats.resync_bytes;

    memcpy(pkt.payload, &st, sizeof(st));
    pkt.payload_size = sizeof(st);

    txEnqueue(pkt, TXP_DATA, 0, true);
    Serial.println("[REPEATER] Status packet sent");
}

#endif  // ROLE_REPEATER
