#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "wifi_manager.h"
#include "lora_manager.h"
#include "sensor_manager.h"

static WiFiClient espClient;
static PubSubClient client(espClient);
static unsigned long lastAttempt = 0;
static bool baseDiscoverySent = false;

// Forward declarations
void mqttPublish(const char* topic, const char* payload, bool retained = false);

// ============================================================
// Internal helpers
// ============================================================

static void sendBaseDiscovery() {
    if (baseDiscoverySent) return;

    const char* device =
        "{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\","
        "\"model\":\"ESP32 LoRa Gateway\","
        "\"manufacturer\":\"DIY\"}";

    // Temperature
    mqttPublish(
        "homeassistant/sensor/lora_base_temp/config",
        "{\"name\":\"LoRa Base Temperature\","
        "\"state_topic\":\"lora/base/temperature\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "\"unique_id\":\"lora_base_temp\","
        "\"device\":"
        "{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\","
        "\"model\":\"ESP32 LoRa Gateway\","
        "\"manufacturer\":\"DIY\"}"
        "}",
        true);

    // Humidity
    mqttPublish(
        "homeassistant/sensor/lora_base_humidity/config",
        "{\"name\":\"LoRa Base Humidity\","
        "\"state_topic\":\"lora/base/humidity\","
        "\"unit_of_measurement\":\"%\","
        "\"device_class\":\"humidity\","
        "\"unique_id\":\"lora_base_humidity\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        true);

    // Pressure
    mqttPublish(
        "homeassistant/sensor/lora_base_pressure/config",
        "{\"name\":\"LoRa Base Pressure\","
        "\"state_topic\":\"lora/base/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "\"unique_id\":\"lora_base_pressure\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        true);

    // WiFi RSSI
    mqttPublish(
        "homeassistant/sensor/lora_base_wifi_rssi/config",
        "{\"name\":\"LoRa Base WiFi RSSI\","
        "\"state_topic\":\"lora/base/wifi_rssi\","
        "\"unit_of_measurement\":\"dBm\","
        "\"unique_id\":\"lora_base_wifi_rssi\","
        "\"device\":{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\"}}",
        true);

    baseDiscoverySent = true;
    Serial.println("[MQTT] Base discovery sent");
}

void publishDiscovery(uint16_t node_id) {
#ifdef ROLE_BASE
    char topic[128];
    char payload[512];

    char device_id[32];
    snprintf(device_id, sizeof(device_id), "node_%d", node_id);

    char device_name[64];
    snprintf(device_name, sizeof(device_name), "LoRa Node %d", node_id);

    // --- TEMPERATURE ---
    snprintf(topic, sizeof(topic),
        "homeassistant/sensor/%s_temperature/config", device_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s Temperature\","
        "\"state_topic\":\"lora/%s/temperature\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "\"unique_id\":\"%s_temp\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ESP32 LoRa Node\","
            "\"manufacturer\":\"DIY\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );
    mqttPublish(topic, payload, true);

    // --- HUMIDITY ---
    snprintf(topic, sizeof(topic),
        "homeassistant/sensor/%s_humidity/config", device_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s Humidity\","
        "\"state_topic\":\"lora/%s/humidity\","
        "\"unit_of_measurement\":\"%%\","
        "\"device_class\":\"humidity\","
        "\"unique_id\":\"%s_hum\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ESP32 LoRa Node\","
            "\"manufacturer\":\"DIY\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );
    mqttPublish(topic, payload, true);

    // --- PRESSURE ---
    snprintf(topic, sizeof(topic),
        "homeassistant/sensor/%s_pressure/config", device_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s Pressure\","
        "\"state_topic\":\"lora/%s/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "\"unique_id\":\"%s_press\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ESP32 LoRa Node\","
            "\"manufacturer\":\"DIY\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );
    mqttPublish(topic, payload, true);

    // --- MOTOR SENSOR ---
    snprintf(topic, sizeof(topic),
        "homeassistant/binary_sensor/%s_motor/config", device_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s Motor\","
        "\"state_topic\":\"lora/%s/motor\","
        "\"payload_on\":\"1\","
        "\"payload_off\":\"0\","
        "\"device_class\":\"power\","
        "\"unique_id\":\"%s_motor\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ESP32 LoRa Node\","
            "\"manufacturer\":\"DIY\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );
    mqttPublish(topic, payload, true);

    // --- RELAY SWITCH ---
    snprintf(topic, sizeof(topic),
        "homeassistant/switch/%s_relay/config", device_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s Relay\","
        "\"command_topic\":\"lora/%s/cmd\","
        "\"state_topic\":\"lora/%s/relay\","
        "\"payload_on\":\"1\","
        "\"payload_off\":\"0\","
        "\"unique_id\":\"%s_relay\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ESP32 LoRa Node\","
            "\"manufacturer\":\"DIY\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_id, device_name
    );
    mqttPublish(topic, payload, true);

    Serial.printf("[MQTT] Discovery published for %s\n", device_id);
#endif
}

// ============================================================
// MQTT Callback (FIXED: no double processing)
// ============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
#ifdef ROLE_BASE
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    String t(topic);
    Serial.printf("[MQTT] cmd: %s = %s\n", topic, msg.c_str());

    // topic format: lora/node_<id>/cmd
    if (t.startsWith("lora/node_") && t.endsWith("/cmd")) {
        // Extract node ID between "lora/node_" and "/cmd"
        int idStart = 10; // after "lora/node_"
        int idEnd = t.indexOf("/cmd", idStart);
        if (idEnd > idStart) {
            int node = t.substring(idStart, idEnd).toInt();
            uint8_t value = (msg == "1") ? 1 : 0;
            sendCommand(node, 1, value);
            Serial.printf("[MQTT] Send command to node %d: %d\n", node, value);
        }
    }
#endif
}

// ============================================================
// Init / Connect / Loop
// ============================================================

void mqttInit() {
#ifdef ROLE_BASE
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(mqttCallback);
    client.setKeepAlive(60);
    client.setBufferSize(MQTT_BUFFER_SIZE);
    Serial.println("[MQTT] Init done");
#endif
}

static void mqttConnect() {
#ifdef ROLE_BASE
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
        client.subscribe("lora/+/cmd");

        // Re-send discovery on every reconnection
        // so that Home Assistant re-registers devices
        sendBaseDiscovery();
    } else {
        Serial.printf("[MQTT] Connect failed, rc=%d\n", client.state());
    }
#endif
}

void mqttLoop() {
#ifdef ROLE_BASE
    if (!wifiConnected())
        return;

    if (!client.connected()) {
        if (millis() - lastAttempt > MQTT_RETRY_INTERVAL) {
            lastAttempt = millis();
            mqttConnect();
        }
    } else {
        client.loop();
    }
#endif
}

bool mqttConnected() {
#ifdef ROLE_BASE
    return client.connected();
#else
    return false;
#endif
}

void mqttPublish(const char* topic, const char* payload, bool retained) {
#ifdef ROLE_BASE
    if (client.connected()) {
        client.publish(topic, payload, retained);
    }
#endif
}

void resetDiscovery() {
    baseDiscoverySent = false;
}
