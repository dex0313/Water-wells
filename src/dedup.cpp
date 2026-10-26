#include "dedup.h"
#include <cstring>

// ============================================================
// Deduplication cache (R4)
// ============================================================
// Ring buffer of DEDUP_SIZE entries, linear search on lookup.
// An entry is active while (uint32_t)(now - ts) < DEDUP_TTL_MS —
// the unsigned subtraction makes the comparison correct across the
// millis() wrap at 2^32.
//
// The timestamp is NOT refreshed on a repeated hit: the TTL anchors
// to the first sighting so that a node rebooting with a fresh
// packet_id sequence (R1) cannot keep colliding with a stale key
// forever. Frames with a bad CRC/version never reach this module —
// they are dropped by the frame parser before dedup.
// ============================================================

struct DedupEntry {
    uint16_t src;
    uint16_t id;
    uint32_t ts;
    bool     used;
};

static DedupEntry s_entries[DEDUP_SIZE];
static size_t     s_next = 0;      // ring cursor
static uint32_t   s_evicted_live = 0;

static bool entryActive(const DedupEntry& e, uint32_t now_ms) {
    return e.used && (uint32_t)(now_ms - e.ts) < DEDUP_TTL_MS;
}

bool dedupCheckAndInsert(uint16_t src, uint16_t id, uint32_t now_ms) {
    // First pass: look for an active duplicate.
    for (size_t i = 0; i < DEDUP_SIZE; i++) {
        DedupEntry& e = s_entries[i];
        if (entryActive(e, now_ms) && e.src == src && e.id == id) {
            // Timestamp intentionally NOT updated (see header comment).
            return true;
        }
    }

    // Not found: insert into the next ring slot.
    DedupEntry& slot = s_entries[s_next];
    if (slot.used && entryActive(slot, now_ms)) {
        s_evicted_live++;  // live entry pushed out — cache too small
    }
    slot.src = src;
    slot.id  = id;
    slot.ts  = now_ms;
    slot.used = true;
    s_next = (s_next + 1) % DEDUP_SIZE;
    return false;
}

void dedupReset() {
    memset(s_entries, 0, sizeof(s_entries));
    s_next = 0;
    s_evicted_live = 0;
}

uint32_t dedupEvictedLive() {
    return s_evicted_live;
}
