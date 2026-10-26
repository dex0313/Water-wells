#include "cmd_parse.h"
#include "protocol.h"
#include <cstdio>
#include <cstring>
#include <cctype>

// ============================================================
// Command topic / payload parsing (P0-4)
// ============================================================

CmdParseResult parseCmdTopic(const char* topic, uint16_t& node, uint8_t& relay_idx) {
    unsigned n = 0, r = 0;
    int used = 0;

    // %n captures how many characters the two numbers consumed, so a
    // trailing suffix like "cmd_1x" is rejected instead of parsed as 1.
    if (sscanf(topic, "lora/node_%u/cmd_%u%n", &n, &r, &used) != 2) {
        return CMD_ERR_NO_MATCH;
    }
    if (topic[used] != '\0') {
        // The node id itself was readable — report it for cmd_status.
        node = (uint16_t)n;
        relay_idx = 0;
        return CMD_ERR_INVALID;
    }
    if (n < 1 || n > 255) {
        node = (uint16_t)n;
        relay_idx = 0;
        return CMD_ERR_INVALID;
    }
    if (r < 1 || r > MAX_RELAYS_PER_NODE) {
        node = (uint16_t)n;
        relay_idx = 0;
        return CMD_ERR_INVALID;
    }

    node = (uint16_t)n;
    relay_idx = (uint8_t)(r - 1);
    return CMD_OK;
}

bool parseOnOff(const uint8_t* p, size_t len, uint8_t& value) {
    char b[8];
    if (len == 0 || len >= sizeof(b)) {
        return false;  // empty or overlong — NOT an OFF command
    }
    for (size_t i = 0; i < len; i++) {
        b[i] = (char)tolower(p[i]);
    }
    b[len] = '\0';

    if (!strcmp(b, "1") || !strcmp(b, "on"))  { value = 1; return true; }
    if (!strcmp(b, "0") || !strcmp(b, "off")) { value = 0; return true; }
    return false;
}
