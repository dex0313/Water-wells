// ============================================================
// lora_rx.cpp — RX loop, dedup, role dispatch (R0 / R2 / R4 / R7)
// ============================================================
// Whole-file role guard (P1-8): compiled for every role.
// ============================================================

#include <Arduino.h>
#include "lora_manager.h"
#include "lora_phy.h"
#include "config.h"
#include "frame_parser.h"
#include "dedup.h"

#if defined(ROLE_BASE)
#include "lora_base_rx.h"      // internal: loraBaseHandlePacket()
#elif defined(ROLE_NODE)
#include "lora_node_rx.h"      // internal: loraNodeHandlePacket()
#elif defined(ROLE_REPEATER)
#include "lora_repeater.h"     // internal: loraRepeaterHandlePacket()
#endif

static FrameParser s_parser;

// ============================================================
// Init — PHY + per-role state
// ============================================================

void loraInit() {
    loraPhyInit();
#if defined(ROLE_BASE)
    loraBaseInit();
#endif
}

// ============================================================
// RX loop — non-blocking, P1-3 sliding window, R2 dedup
// ============================================================

void loraLoop() {
    // Drive the non-blocking TX state machine first (R3).
    txPoll();

    // Read at most 256 bytes per iteration, then pop all complete
    // frames the sliding window can extract.
    uint8_t chunk[256];
    size_t n = 0;
    while (n < sizeof(chunk) && loraSerial.available() > 0) {
        chunk[n++] = (uint8_t)loraSerial.read();
    }

    for (size_t i = 0; i < n; i++) {
        s_parser.push(chunk[i]);
        loraNoteRx();

        // Extract frames as soon as they complete. Popping after
        // every push keeps the sliding buffer small: even a burst of
        // back-to-back frames (ACK + DATA) never overflows it.
        MeshPacket pkt;
        while (s_parser.pop(pkt)) {
            // Merge parser statistics into the shared stats block.
            const LoraRxStats& rs = s_parser.stats();
            gLoraStats.rx_ok = rs.rx_ok;
            gLoraStats.rx_crc_err = rs.rx_crc_err;
            gLoraStats.resync_bytes = rs.resync_bytes;

            // R2/R4: every role deduplicates by (source, packet_id).
            // Each frame can arrive twice (directly + via the
            // repeater).
            if (dedupCheckAndInsert(pkt.source, pkt.packet_id, millis())) {
                gLoraStats.rx_dup++;
                continue;
            }

            // Frames already passed header + CRC validation in the
            // parser, so garbage is never dispatched or forwarded (R7).
#if defined(ROLE_BASE)
            loraBaseHandlePacket(pkt);
#elif defined(ROLE_NODE)
            loraNodeHandlePacket(pkt);
#elif defined(ROLE_REPEATER)
            loraRepeaterHandlePacket(pkt);
#endif
        }
    }
}
