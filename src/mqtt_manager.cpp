// ============================================================
// mqtt_manager.cpp — BASE role: WiFi/MQTT/HA plumbing
// ============================================================
// Whole-file role guard (P1-8): NODE/REPEATER builds get inert
// stubs and never pull in PubSubClient (P1-8 dependency cleanup).
// ============================================================

#if defined(ROLE_BASE)
#include <WiFi.h>
#include <PubSubClient.h>
#endif

#include "mqtt_manager.h"
#include "config.h"
#include "cmd_parse.h"
#include "lora_manager.h"
#include "wifi_manager.h"

#if defined(ROLE_BASE)
#include "secret.h"   // P2-4: secrets only inside BASE sources
#endif

#if defined(ROLE_BASE)

static WiFiClient espClient;
static PubSubClient client(espClient);
static unsigned long lastAttempt = 0;
static bool baseDiscoverySent = false;

// ============================================================
// Deferred publication queue (P1-2.5)
// ============================================================
// Discovery bursts (~10-30 messages) must not run inside the LoRa
// RX handler. Publications are enqueued here and drained 1-2 per
// mqttLoop() pass, keeping the handler fast and the RX buffer safe.
//
// P1-1.7: every caller enqueues retained HA discovery configs, and
// a burst that only partially fits is RETRIED as a whole (discovery
// flags stay false until every message is queued). To keep such a
// retry from duplicating the messages already sitting in the queue,
// pubEnqueue() skips an exact (topic+payload) match that is still
// pending. A retry therefore only adds the missing pieces and the
// burst completes incrementally instead of churning the queue under
// overload. Same topic with a DIFFERENT payload (e.g. an empty
// HA_DISCOVERY_CLEANUP payload replacing a stale config after a
// profile shrink) is always enqueued — MQTT delivers both in order
// and the newer payload wins.
// ============================================================

#define PUB_QUEUE_SIZE 40

struct PubItem {
    char*    topic;
    char*    payload;
    uint16_t topic_len;     // cached strlen for the duplicate scan
    uint16_t payload_len;
    bool     retained;
};

static PubItem s_pubQueue[PUB_QUEUE_SIZE];
static uint8_t s_pubQHead = 0;   // next to publish
static uint8_t s_pubQTail = 0;   // next free slot
static uint8_t s_pubQCount = 0;

static bool pubEnqueue(const char* topic, const char* payload, bool retained) {
    if (s_pubQCount >= PUB_QUEUE_SIZE) {
        Serial.println("[MQTT] Publish queue full");
        return false;
    }

    const uint16_t tlen = (uint16_t)strlen(topic);
    const uint16_t plen = (uint16_t)strlen(payload);

    // P1-1.7: exact duplicate still pending? Then it WILL be
    // published — re-queueing it would only waste a slot (and, on a
    // burst retry, re-add every already-queued message).
    for (uint8_t i = 0; i < s_pubQCount; i++) {
        const PubItem& q = s_pubQueue[(uint8_t)((s_pubQHead + i) % PUB_QUEUE_SIZE)];
        if (q.topic_len == tlen && q.payload_len == plen &&
            memcmp(q.topic, topic, tlen) == 0 &&
            memcmp(q.payload, payload, plen) == 0) {
            return true;
        }
    }

    PubItem& it = s_pubQueue[s_pubQTail];
    it.topic = strdup(topic);
    it.payload = strdup(payload);
    if (!it.topic || !it.payload) {
        free(it.topic);
        free(it.payload);
        it.topic = it.payload = nullptr;
        it.topic_len = it.payload_len = 0;
        return false;
    }
    it.topic_len = tlen;
    it.payload_len = plen;
    it.retained = retained;
    s_pubQTail = (uint8_t)((s_pubQTail + 1) % PUB_QUEUE_SIZE);
    s_pubQCount++;
    return true;
}

static void pubDropHead() {
    PubItem& it = s_pubQueue[s_pubQHead];
    free(it.topic);
    free(it.payload);
    it.topic = it.payload = nullptr;
    s_pubQHead = (uint8_t)((s_pubQHead + 1) % PUB_QUEUE_SIZE);
    s_pubQCount--;
}

// Drop everything queued (used right after reconnect: discovery is
// re-created from scratch).
static void pubFlush() {
    while (s_pubQCount > 0) {
        pubDropHead();
    }
}

static void pubDrain() {
    if (s_pubQCount == 0) return;

    // P1-2.5: 1-2 messages per pass keeps each loop iteration short.
    uint8_t budget = 2;
    while (budget-- > 0 && s_pubQCount > 0) {
        PubItem& it = s_pubQueue[s_pubQHead];
        if (client.publish(it.topic, it.payload, it.retained)) {
            pubDropHead();
        } else {
            // Broker buffer full — retry on the next pass.
            break;
        }
    }
}

// ============================================================
// JSON build helper with truncation detection (P1-1.5)
// ============================================================

#include <stdarg.h>

static bool buildJson(char* buf, size_t size, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, size, fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= size) {
        Serial.println("[MQTT] discovery truncated");
        return false;
    }
    return true;
}

// ============================================================
// Availability fragments (P1-1.5)
// ============================================================
// Node entities depend on BOTH the base status and the node online
// topic; base entities only on the base status. The `online` binary
// sensor itself carries no availability (it IS the signal).
// ============================================================

static bool nodeAvailabilityJson(char* buf, size_t size, uint16_t node_id) {
    return buildJson(buf, size,
        "\"availability\":["
        "{\"topic\":\"lora/base/status\"},"
        "{\"topic\":\"lora/node_%u/online\",\"payload_available\":\"1\",\"payload_not_available\":\"0\"}],"
        "\"availability_mode\":\"all\",",
        (unsigned)node_id);
}

// ============================================================
// Base discovery (own BME280 + RSSI entities)
// ============================================================

static bool sendBaseDiscovery() {
    if (baseDiscoverySent) return true;

    // P1-1.5: base entities depend only on lora/base/status.
    const char* base_avail =
        "\"availability\":[{\"topic\":\"lora/base/status\"}],";
    bool ok = true;

    char payload[300];

    if (buildJson(payload, sizeof(payload),
        "{\"name\":\"LoRa Base Temperature\","
        "\"state_topic\":\"lora/base/temperature\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "%s"
        "\"unique_id\":\"lora_base_temp\","
        "\"device\":"
        "{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\","
        "\"model\":\"ESP32 LoRa Gateway\","
        "\"manufacturer\":\"DIY\"}}",
        base_avail)) {
        ok &= pubEnqueue("homeassistant/sensor/lora_base_temp/config", payload, true);
    } else {
        ok = false;
    }

    if (buildJson(payload, sizeof(payload),
        "{\"name\":\"LoRa Base Humidity\","
        "\"state_topic\":\"lora/base/humidity\","
        "\"unit_of_measurement\":\"%%\","
        "\"device_class\":\"humidity\","
        "%s"
        "\"unique_id\":\"lora_base_humidity\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        base_avail)) {
        ok &= pubEnqueue("homeassistant/sensor/lora_base_humidity/config", payload, true);
    } else {
        ok = false;
    }

    if (buildJson(payload, sizeof(payload),
        "{\"name\":\"LoRa Base Pressure\","
        "\"state_topic\":\"lora/base/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "%s"
        "\"unique_id\":\"lora_base_pressure\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        base_avail)) {
        ok &= pubEnqueue("homeassistant/sensor/lora_base_pressure/config", payload, true);
    } else {
        ok = false;
    }

    if (buildJson(payload, sizeof(payload),
        "{\"name\":\"LoRa Base WiFi RSSI\","
        "\"state_topic\":\"lora/base/wifi_rssi\","
        "\"unit_of_measurement\":\"dBm\","
        "%s"
        "\"unique_id\":\"lora_base_wifi_rssi\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        base_avail)) {
        ok &= pubEnqueue("homeassistant/sensor/lora_base_wifi_rssi/config", payload, true);
    } else {
        ok = false;
    }

    if (ok) {
        baseDiscoverySent = true;
        Serial.println("[MQTT] Base discovery sent");
    }
    return ok;
}

// ============================================================
// Dynamic Node Discovery (P1-1)
// ============================================================

bool publishDiscovery(uint16_t node_id, uint8_t num_relays, uint8_t num_sensors) {
    char topic[96];
    char payload[1000];   // P1-1.5: relay discovery with availability ~600B
    char avail[200];

    char device_id[32];
    snprintf(device_id, sizeof(device_id), "node_%u", (unsigned)node_id);

    char device_name[64];
    snprintf(device_name, sizeof(device_name), "LoRa Node %u", (unsigned)node_id);

    // Common device JSON fragment used in all entities
    char device_json[200];
    snprintf(device_json, sizeof(device_json),
        "{\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"model\":\"ESP32 LoRa Node\","
        "\"manufacturer\":\"DIY\"}",
        device_id, device_name);

    if (!nodeAvailabilityJson(avail, sizeof(avail), node_id)) return false;

    bool ok = true;

    // --- TEMPERATURE ---
    if (buildJson(topic, sizeof(topic),
            "homeassistant/sensor/%s_temperature/config", device_id) &&
        buildJson(payload, sizeof(payload),
        "{\"name\":\"%s Temperature\","
        "\"state_topic\":\"lora/%s/temperature\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "%s"
        "\"unique_id\":\"%s_temp\","
        "\"device\":%s}",
        device_name, device_id, avail, device_id, device_json)) {
        ok &= pubEnqueue(topic, payload, true);
    } else {
        ok = false;
    }

    // --- HUMIDITY ---
    if (buildJson(topic, sizeof(topic),
            "homeassistant/sensor/%s_humidity/config", device_id) &&
        buildJson(payload, sizeof(payload),
        "{\"name\":\"%s Humidity\","
        "\"state_topic\":\"lora/%s/humidity\","
        "\"unit_of_measurement\":\"%%\","
        "\"device_class\":\"humidity\","
        "%s"
        "\"unique_id\":\"%s_hum\","
        "\"device\":%s}",
        device_name, device_id, avail, device_id, device_json)) {
        ok &= pubEnqueue(topic, payload, true);
    } else {
        ok = false;
    }

    // --- PRESSURE ---
    if (buildJson(topic, sizeof(topic),
            "homeassistant/sensor/%s_pressure/config", device_id) &&
        buildJson(payload, sizeof(payload),
        "{\"name\":\"%s Pressure\","
        "\"state_topic\":\"lora/%s/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "%s"
        "\"unique_id\":\"%s_press\","
        "\"device\":%s}",
        device_name, device_id, avail, device_id, device_json)) {
        ok &= pubEnqueue(topic, payload, true);
    } else {
        ok = false;
    }

    // --- RELAY SWITCHES (dynamic count) ---
    for (uint8_t i = 0; i < num_relays; i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/switch/%s_relay_%u/config", device_id, i + 1) &&
            buildJson(payload, sizeof(payload),
            "{\"name\":\"%s Relay %u\","
            "\"command_topic\":\"lora/%s/cmd_%u\","
            "\"state_topic\":\"lora/%s/relay_%u\","
            "\"payload_on\":\"1\","
            "\"payload_off\":\"0\","
            "%s"
            "\"unique_id\":\"%s_relay_%u\","
            "\"device\":%s}",
            device_name, i + 1,
            device_id, i + 1,
            device_id, i + 1,
            avail,
            device_id, i + 1,
            device_json)) {
            ok &= pubEnqueue(topic, payload, true);
        } else {
            ok = false;
        }
    }

    // --- VOLTAGE SENSORS (dynamic count) ---
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/sensor/%s_voltage_%u/config", device_id, i + 1) &&
            buildJson(payload, sizeof(payload),
            "{\"name\":\"%s Voltage %u\","
            "\"state_topic\":\"lora/%s/voltage_%u\","
            "\"unit_of_measurement\":\"V\","
            "\"device_class\":\"voltage\","
            "%s"
            "\"unique_id\":\"%s_voltage_%u\","
            "\"device\":%s}",
            device_name, i + 1,
            device_id, i + 1,
            avail,
            device_id, i + 1,
            device_json)) {
            ok &= pubEnqueue(topic, payload, true);
        } else {
            ok = false;
        }
    }

    // --- MOTOR BINARY SENSORS (dynamic count) ---
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/binary_sensor/%s_motor_%u/config", device_id, i + 1) &&
            buildJson(payload, sizeof(payload),
            "{\"name\":\"%s Motor %u\","
            "\"state_topic\":\"lora/%s/motor_%u\","
            "\"payload_on\":\"1\","
            "\"payload_off\":\"0\","
            "\"device_class\":\"power\","
            "%s"
            "\"unique_id\":\"%s_motor_%u\","
            "\"device\":%s}",
            device_name, i + 1,
            device_id, i + 1,
            avail,
            device_id, i + 1,
            device_json)) {
            ok &= pubEnqueue(topic, payload, true);
        } else {
            ok = false;
        }
    }

    // --- ONLINE STATUS (no availability — it IS the signal) ---
    if (buildJson(topic, sizeof(topic),
            "homeassistant/binary_sensor/%s_online/config", device_id) &&
        buildJson(payload, sizeof(payload),
        "{\"name\":\"%s Online\","
        "\"state_topic\":\"lora/%s/online\","
        "\"payload_on\":\"1\","
        "\"payload_off\":\"0\","
        "\"device_class\":\"connectivity\","
        "\"unique_id\":\"%s_online\","
        "\"device\":%s}",
        device_name, device_id, device_id, device_json)) {
        ok &= pubEnqueue(topic, payload, true);
    } else {
        ok = false;
    }

    // --- COMMAND STATUS ---
    if (buildJson(topic, sizeof(topic),
            "homeassistant/sensor/%s_cmd_status/config", device_id) &&
        buildJson(payload, sizeof(payload),
        "{\"name\":\"%s Command Status\","
        "\"state_topic\":\"lora/%s/cmd_status\","
        "\"icon\":\"mdi:check-circle\","
        "%s"
        "\"unique_id\":\"%s_cmd_status\","
        "\"device\":%s}",
        device_name, device_id, avail, device_id, device_json)) {
        ok &= pubEnqueue(topic, payload, true);
    } else {
        ok = false;
    }

    // --- STALE ENTITY CLEANUP (P1-1.6) ---
    // Entities above the node profile get an empty retained payload,
    // removing HA leftovers after a profile shrinks. Owner question
    // was open — gated by HA_DISCOVERY_CLEANUP in config.h.
#if HA_DISCOVERY_CLEANUP
    for (uint8_t i = num_relays; i < MAX_RELAYS_PER_NODE; i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/switch/%s_relay_%u/config", device_id, i + 1)) {
            ok &= pubEnqueue(topic, "", true);
        }
    }
    for (uint8_t i = num_sensors; i < MAX_SENSORS_PER_NODE; i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/sensor/%s_voltage_%u/config", device_id, i + 1)) {
            ok &= pubEnqueue(topic, "", true);
        }
        if (buildJson(topic, sizeof(topic),
                "homeassistant/binary_sensor/%s_motor_%u/config", device_id, i + 1)) {
            ok &= pubEnqueue(topic, "", true);
        }
    }
#endif

    if (ok) {
        Serial.printf("[MQTT] Discovery queued for %s (%u relays, %u sensors)\n",
                      device_id, (unsigned)num_relays, (unsigned)num_sensors);
    }
    return ok;
}

// ============================================================
// PKT_STATUS diagnostic entities (R8.2)
// ============================================================

bool publishStatusDiscovery(uint16_t node_id) {
    char topic[96];
    char payload[700];
    char avail[200];

    char device_id[32];
    snprintf(device_id, sizeof(device_id), "node_%u", (unsigned)node_id);

    char device_name[64];
    snprintf(device_name, sizeof(device_name), "LoRa Node %u", (unsigned)node_id);

    char device_json[200];
    snprintf(device_json, sizeof(device_json),
        "{\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"model\":\"ESP32 LoRa Node\","
        "\"manufacturer\":\"DIY\"}",
        device_id, device_name);

    if (!nodeAvailabilityJson(avail, sizeof(avail), node_id)) return false;

    struct { const char* field; const char* title; const char* unit_json; } fields[] = {
        { "uptime_s",     "Uptime",        "\"unit_of_measurement\":\"s\"," },
        { "boot_count",   "Boot Count",    "" },
        { "fwd",          "Forwarded",     "" },
        { "rx_dup",       "Rx Duplicates", "" },
        { "fwd_drop",     "Fwd Dropped",   "" },
        { "rx_crc_err",   "CRC Errors",    "" },
        { "resync_bytes", "Resync Bytes",  "" },
    };

    bool ok = true;
    for (uint8_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        if (buildJson(topic, sizeof(topic),
                "homeassistant/sensor/%s_stats_%s/config",
                device_id, fields[i].field) &&
            buildJson(payload, sizeof(payload),
            "{\"name\":\"%s %s\","
            "\"state_topic\":\"lora/%s/stats/%s\","
            "%s"
            "%s"
            "\"entity_category\":\"diagnostic\","
            "\"unique_id\":\"%s_stats_%s\","
            "\"device\":%s}",
            device_name, fields[i].title,
            device_id, fields[i].field,
            fields[i].unit_json,
            avail,
            device_id, fields[i].field,
            device_json)) {
            ok &= pubEnqueue(topic, payload, true);
        } else {
            ok = false;
        }
    }
    return ok;
}

// ============================================================
// MQTT Callback — P0-4 parsing + P1-1.4 HA status routing
// ============================================================

static void publishCmdStatusFromMqtt(uint16_t node, const char* status) {
    char topic[64];
    snprintf(topic, sizeof(topic), "lora/node_%u/cmd_status", (unsigned)node);
    mqttPublish(topic, status, false);
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // P1-1.4: Home Assistant asks everyone to republish discovery
    // after its own restart.
    if (strcmp(topic, "homeassistant/status") == 0) {
        if (length == 6 && memcmp(payload, "online", 6) == 0) {
            Serial.println("[MQTT] Home Assistant online — republishing discovery");
            resetDiscovery();
            sendBaseDiscovery();
            loraOnMqttConnected();
        }
        return;
    }

    Serial.printf("[MQTT] cmd: %s (%u bytes)\n", topic, length);

    uint16_t node = 0;
    uint8_t  relay_idx = 0;
    uint8_t  value = 0;

    CmdParseResult pr = parseCmdTopic(topic, node, relay_idx);
    if (pr == CMD_ERR_NO_MATCH) {
        Serial.printf("[MQTT] Not a command topic: %s\n", topic);
        return;
    }

    // P0-4: strict payload parsing — anything but 1/0/on/off is a
    // bad request, NOT silently an OFF command.
    if (!parseOnOff(payload, length, value)) {
        Serial.printf("[MQTT] Bad payload — nothing sent to the air\n");
        if (node != 0) {
            publishCmdStatusFromMqtt(node, "failed:bad_request");
        }
        return;
    }

    if (pr == CMD_ERR_INVALID) {
        Serial.printf("[MQTT] Bad command topic (node=%u) — nothing sent to the air\n",
                      (unsigned)node);
        if (node != 0) {
            publishCmdStatusFromMqtt(node, "failed:bad_request");
        }
        return;
    }

    // P0-4: reject relays beyond the node's reported profile.
    if (loraNodeEverSeen(node) && relay_idx >= loraNodeNumRelays(node)) {
        Serial.printf("[MQTT] Relay %u out of profile for node %u\n",
                      (unsigned)(relay_idx + 1), (unsigned)node);
        publishCmdStatusFromMqtt(node, "failed:bad_relay");
        return;
    }

    sendCommandWithAck(node, 1, relay_idx, value);
    Serial.printf("[MQTT] Send command (with ACK) to node %u relay %u: %u\n",
                  (unsigned)node, (unsigned)(relay_idx + 1), (unsigned)value);
}

// ============================================================
// Init / Connect / Loop
// ============================================================

void mqttInit() {
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(mqttCallback);
    client.setKeepAlive(60);
    client.setBufferSize(MQTT_BUFFER_SIZE);
    // P1-2.3: bound the blocking of PubSubClient socket reads —
    // client.connect()/loop() may no longer stall the RX path for
    // many seconds.
    client.setSocketTimeout(3);
    Serial.println("[MQTT] Init done");
}

static void mqttConnect() {
    Serial.println("[MQTT] Connecting...");

    if (client.connect(
            MQTT_CLIENT_ID,
            MQTT_USER,
            MQTT_PASS,
            "lora/base/status",
            0,
            true,
            "offline")) {

        Serial.println("[MQTT] Connected");
        client.publish("lora/base/status", "online", true);

        // Subscribe to relay command topics.
        // IMPORTANT: MQTT spec requires '+' wildcard to occupy an ENTIRE
        // topic level. "lora/+/cmd_+" is INVALID — Mosquitto rejects it
        // as "Invalid subscription string". We must subscribe to each
        // cmd_N topic individually.
        for (uint8_t i = 1; i <= MAX_RELAYS_PER_NODE; i++) {
            char sub_topic[32];
            snprintf(sub_topic, sizeof(sub_topic), "lora/+/cmd_%d", i);
            client.subscribe(sub_topic);
        }
        Serial.printf("[MQTT] Subscribed to lora/+/cmd_1..%d\n", MAX_RELAYS_PER_NODE);

        // P1-1.4: HA restart announcements.
        client.subscribe("homeassistant/status");

        // P1-1.3: fresh discovery after reconnect.
        pubFlush();
        sendBaseDiscovery();
        loraOnMqttConnected();
    } else {
        Serial.printf("[MQTT] Connect failed, rc=%d\n", client.state());
    }
}

void mqttLoop() {
    if (!wifiConnected())
        return;

    if (!client.connected()) {
        if (millis() - lastAttempt > MQTT_RETRY_INTERVAL) {
            lastAttempt = millis();
            mqttConnect();
        }
    } else {
        client.loop();
        pubDrain();
    }
}

bool mqttConnected() {
    return client.connected();
}

bool mqttPublish(const char* topic, const char* payload, bool retained) {
    if (!client.connected()) {
        return false;   // P1-1.1: silent loss made discovery sticky
    }
    return client.publish(topic, payload, retained);
}

void resetDiscovery() {
    baseDiscoverySent = false;
}

#else  // !ROLE_BASE — inert stubs keep the public API linkable

void mqttInit() {}
void mqttLoop() {}
bool mqttConnected() { return false; }
bool mqttPublish(const char*, const char*, bool) { return false; }
bool publishDiscovery(uint16_t, uint8_t, uint8_t) { return false; }
bool publishStatusDiscovery(uint16_t) { return false; }
void resetDiscovery() {}

#endif // ROLE_BASE
