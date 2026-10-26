#include "ack_match.h"

// ============================================================
// ACK matching logic (P0-3)
// ============================================================

AckOutcome ackEvaluate(const PendingCommand& pc, const AckFrame& ack) {
    if (!pc.active) {
        return ACK_IGNORED;
    }

    bool match = (pc.cmd == ack.cmd) && (pc.relay_index == ack.relay_index);
    if (!match) {
        return ACK_IGNORED;
    }

    if (ack.has_seq) {
        // v1.1: the node echoes the command sequence number.
        if (pc.seq != ack.seq) {
            return ACK_IGNORED;  // late ACK of an older command
        }
    } else {
        // Legacy node (no seq echo): NAK closes the entry, ACK only
        // when the node reports exactly the requested state.
        if (!((ack.result == 0) || (ack.state == pc.value))) {
            return ACK_IGNORED;
        }
    }

    if (ack.result == 0) {
        return ACK_NAK;
    }
    if (ack.state == pc.value) {
        return ACK_OK;
    }
    return ACK_STATE_MISMATCH;
}

int8_t ackPickSlot(const PendingCommand* slots, uint8_t n,
                   uint16_t node, uint8_t cmd, uint8_t relay_index) {
    // Preemption: last command per (node, relay) wins.
    for (uint8_t i = 0; i < n; i++) {
        if (slots[i].active &&
            slots[i].node_id == node &&
            slots[i].cmd == cmd &&
            slots[i].relay_index == relay_index) {
            return (int8_t)i;
        }
    }
    // First free slot.
    for (uint8_t i = 0; i < n; i++) {
        if (!slots[i].active) {
            return (int8_t)i;
        }
    }
    return -1;
}
