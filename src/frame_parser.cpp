#include "frame_parser.h"
#include <cstring>

// ============================================================
// FrameParser — sliding window with re-synchronization (P1-3)
// ============================================================
//
// push() appends a received byte. pop() tries to extract a frame:
//   - while at least MESH_PKT_LEN bytes are buffered, validate the
//     candidate frame header (version, type range, payload_size)
//     and CRC-16/Modbus;
//   - on success consume exactly MESH_PKT_LEN bytes and return it;
//   - otherwise slide the window by ONE byte and retry. A candidate
//     that passed the header check but failed the CRC is counted in
//     rx_crc_err, every skipped byte is counted in resync_bytes.
//
// A separate RX timeout is not needed: stale tails are discarded by
// the slide. Side effect: tolerance to an extra RSSI byte appended
// to the frame, should an E22 firmware ever produce one.
// ============================================================

void FrameParser::consume(size_t n) {
    // memmove handles the shrinking tail; len_ >= n always holds here.
    if (n < len_) {
        memmove(buf_, buf_ + n, len_ - n);
    }
    len_ -= n;
}

void FrameParser::push(uint8_t b) {
    if (len_ == sizeof(buf_)) {
        consume(1);  // never overflow — drop the oldest byte
    }
    buf_[len_++] = b;
}

void FrameParser::reset() {
    len_ = 0;
}

bool FrameParser::pop(MeshPacket& out) {
    while (len_ >= MESH_PKT_LEN) {
        MeshPacket c;
        memcpy(&c, buf_, MESH_PKT_LEN);

        // Header plausibility first, so rx_crc_err only counts
        // candidates that look like our frames.
        bool header_ok = (c.version == 1) &&
                         (c.type >= PKT_DATA && c.type <= PKT_TYPE_MAX) &&
                         (c.payload_size <= sizeof(c.payload));

        if (header_ok &&
            crc16(buf_, MESH_PKT_LEN - sizeof(uint16_t)) == c.crc) {
            out = c;
            consume(MESH_PKT_LEN);
            st_.rx_ok++;
            return true;
        }

        if (header_ok) {
            st_.rx_crc_err++;
        }
        consume(1);        // slide by one byte and retry
        st_.resync_bytes++;
    }
    return false;
}
