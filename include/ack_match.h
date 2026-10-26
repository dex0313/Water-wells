#pragma once
// ============================================================
// ack_match.h — ACK-to-pending-command matching (pure logic)
// ============================================================
// Host-compilable (no Arduino.h), covered by native tests (T-4).
//
// Fixes revision note P0-3:
//   - ack.result was ignored: a negative ACK was published as "ok";
//   - matching was only by (node, cmd, relay): a late ACK of an old
//     command closed a newer one — fixed with the additive seq byte;
//   - several pending entries could exist for the same relay —
//     fixed by preemption on the BASE side.
// ============================================================

#include <cstdint>

// View of a pending command slot (BASE side).
// This is the real PendingCommand type used by the BASE firmware;
// it lives here so the matching logic stays host-testable.
struct PendingCommand {
    uint16_t node_id;         // Target node address
    uint8_t  cmd;             // Command type (1 = relay)
    uint8_t  relay_index;     // Which relay (0..N-1)
    uint8_t  value;           // Command value (0 or 1)
    uint8_t  seq;             // Command sequence (P0-3): 1..255, 0 = not set
    uint8_t  retries_left;    // Remaining retry attempts
    uint32_t sent_at_ms;      // millis() when last sent
    bool     active;          // Whether this slot is in use
};

// View of a received ACK payload.
struct AckFrame {
    uint8_t cmd;
    uint8_t relay_index;
    uint8_t result;    // 1 = success, 0 = NAK
    uint8_t state;     // relay state reported by the node
    bool    has_seq;   // true when payload_size >= 5 (v1.1 node)
    uint8_t seq;       // echo of the command seq (valid only if has_seq)
};

enum AckOutcome : uint8_t {
    ACK_IGNORED = 0,     // no match — keep the pending entry
    ACK_OK,              // success, relay state matches the command
    ACK_NAK,             // node reported failure (result == 0)
    ACK_STATE_MISMATCH   // success flag but the relay is in another state
};

// Evaluates an ACK against one pending command.
// The caller checks pkt.source == pc.node_id before calling.
// Matching rules:
//   - base match: cmd and relay_index are equal;
//   - v1.1 node (has_seq): additionally requires seq equality, so a
//     late ACK of an older command cannot close a newer one;
//   - legacy node (!has_seq): falls back to
//       (ack.result == 0) || (ack.state == pc.value)
//     i.e. a NAK always closes the entry, an ACK only when the node
//     confirms the requested state.
AckOutcome ackEvaluate(const PendingCommand& pc, const AckFrame& ack);

// Slot selection for a new command (P0-3.2 preemption):
//   1. an ACTIVE entry with the same (node, cmd, relay_index) is
//      replaced — the last command wins;
//   2. otherwise the first free slot;
//   3. -1 when everything is busy.
int8_t ackPickSlot(const PendingCommand* slots, uint8_t n,
                   uint16_t node, uint8_t cmd, uint8_t relay_index);
