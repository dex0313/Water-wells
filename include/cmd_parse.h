#pragma once
// ============================================================
// cmd_parse.h — MQTT command topic/payload parsing (pure logic)
// ============================================================
// Host-compilable (no Arduino.h), covered by native tests (T-5).
//
// Replaces the old String-based parsing in mqttCallback that had
// dangerous fallbacks (revision note P0-4):
//   - any payload other than "1" was treated as OFF (including
//     garbage, "ON", empty string);
//   - cmd_0 and non-numeric suffixes became relay 1;
//   - the node id was not validated.
// ============================================================

#include <cstdint>
#include <cstddef>

// Result of parsing "lora/node_<id>/cmd_<relay>".
enum CmdParseResult : uint8_t {
    CMD_OK = 0,       // node and relay_idx are valid
    CMD_ERR_NO_MATCH, // topic is not a command topic at all
    CMD_ERR_INVALID   // matched shape but id/relay out of range or junk
};

// Parses "lora/node_<id>/cmd_<relay_num>".
// On CMD_OK fills node (1..255) and relay_idx (0..MAX_RELAYS_PER_NODE-1).
// On CMD_ERR_INVALID fills node when the id itself was readable
// (so the caller can publish cmd_status for it), relay_idx = 0.
// On CMD_ERR_NO_MATCH outputs are untouched.
CmdParseResult parseCmdTopic(const char* topic, uint16_t& node, uint8_t& relay_idx);

// Strict ON/OFF payload parsing. Accepts "1"/"on"/"ON"/"On" and
// "0"/"off"/"OFF"/"Off" only. Anything else (empty, "2", "true",
// overlong) returns false — it is NOT silently an OFF command.
bool parseOnOff(const uint8_t* p, size_t len, uint8_t& value);
