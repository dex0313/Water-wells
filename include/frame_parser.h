#pragma once
// ============================================================
// frame_parser.h — Sliding-window frame extractor (pure logic)
// ============================================================
// Host-compilable (no Arduino.h) so it is covered by native tests.
//
// Replaces the old fixed-offset rxBuffer/rxIndex/RX_TIMEOUT_MS
// scheme (revision note P1-3): the radio link is half-duplex and
// noisy (pumps/motors), so a single lost or extra byte used to
// corrupt every following frame in a burst. The sliding window
// re-synchronizes by shifting one byte at a time until a frame
// with a valid header and CRC is found.
//
// The over-the-air format is UNCHANGED: fixed 59-byte frames.
// ============================================================

#include <cstdint>
#include <cstddef>
#include "protocol.h"

// Receive-side statistics shared with LoraStats (P1-3).
struct LoraRxStats {
    uint32_t rx_ok;         // Frames extracted with valid CRC
    uint32_t rx_crc_err;    // Candidates with valid header but bad CRC
    uint32_t resync_bytes;  // Bytes skipped while sliding/resyncing
};

class FrameParser {
public:
    void push(uint8_t b);
    // Returns true when a valid frame was extracted; call in a loop.
    bool pop(MeshPacket& out);
    void reset();
    const LoraRxStats& stats() const { return st_; }
    size_t buffered() const { return len_; }

private:
    void consume(size_t n);

    // Two frame lengths: enough for one full frame plus noise tail.
    // Never overflows: push() drops the oldest byte when full.
    uint8_t buf_[MESH_PKT_LEN * 2];
    size_t  len_ = 0;
    LoraRxStats st_{};
};
