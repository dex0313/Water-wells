#pragma once
// ============================================================
// dedup.h — (source, packet_id) deduplication cache (pure logic)
// ============================================================
// Host-compilable (no Arduino.h): time is passed as a parameter so
// the module can be unit-tested on the host, including the millis()
// wrap-around at 2^32 (revision note T-3).
//
// Used by ALL roles (R2/R4):
//   - the repeater must not forward the same frame twice;
//   - BASE and nodes receive each frame twice (directly and via the
//     repeater) and must process it only once.
// ============================================================

#include <cstdint>

#define DEDUP_SIZE    64
#define DEDUP_TTL_MS  45000UL

// Returns true if (src, id) was already seen within TTL.
// Otherwise inserts the pair and returns false.
bool dedupCheckAndInsert(uint16_t src, uint16_t id, uint32_t now_ms);

// Clears the cache (used by tests; not required at runtime).
void dedupReset();

// Diagnostics: how many LIVE entries were evicted by the ring before
// their TTL expired. Must stay 0 in normal operation; a non-zero
// value means DEDUP_SIZE is too small for the traffic (R4).
uint32_t dedupEvictedLive();
