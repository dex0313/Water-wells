#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "wifi_manager.h"
#include "lora_manager.h"

static WiFiClient espClient;
static PubSubClient client(espClient);
static unsigned long lastAttempt = 0;
static bool discoverySent = false;

void mqttPublish(const char* topic, const char* payload, bool retained) {
#ifdef ROLE_BASE
    if (client.connected()) {
        client.publish(topic, payload, retained);
    }
#endif
}

void sendDiscovery() {

    if (discoverySent) return;

    const char* device =
        "{\"identifiers\":[\"lora_base\"],"
        "\"name\":\"LoRa Base\","
        "\"model\":\"ESP32 LoRa Gateway\"}";

    mqttPublish(
        "homeassistant/sensor/lora_base_temp/config",
        "{\"name\":\"LoRa Base Temperature\","
        "\"state_topic\":\"lora/base/temperature\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "\"unique_id\":\"lora_base_temp\","
        "\"device\":"
        "{\"identifiers\":[\"lora_base\"]}"
        "}",
        true);

    mqttPublish(
        "homeassistant/sensor/lora_base_humidity/config",
        "{\"name\":\"LoRa Base Humidity\","
        "\"state_topic\":\"lora/base/humidity\","
        "\"unit_of_measurement\":\"%\","
        "\"device_class\":\"humidity\","
        "\"unique_id\":\"lora_base_humidity\","
        "\"device\":{\"identifiers\":[\"lora_base\"]}}",
        true);

    mqttPublish(
        "homeassistant/sensor/lora_base_pressure/config",
        "{\"name\":\"LoRa Base Pressure\","
        "\"state_topic\":\"lora/base/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "\"unique_id\":\"lora_base_pressure\","
        "\"device\":{\"identifiers\":[\"lora_base\"]}}",
        true);

    mqttPublish(
        "homeassistant/sensor/lora_base_wifi_rssi/config",
        "{\"name\":\"LoRa Base WiFi RSSI\","
        "\"state_topic\":\"lora/base/wifi_rssi\","
        "\"unit_of_measurement\":\"dBm\","
        "\"unique_id\":\"lora_base_wifi_rssi\","
        "\"device\":{\"identifiers\":[\"lora_base\"]}}",
        true);

    discoverySent = true;
}

void publishDiscovery(uint16_t node_id) {

#ifdef ROLE_BASE

    char topic[128];
    char payload[512];

    char device_id[32];
    sprintf(device_id, "node_%d", node_id);

    char device_name[64];
    sprintf(device_name, "LoRa Node %d", node_id);

    // --- TEMPERATURE ---
    sprintf(topic, "homeassistant/sensor/%s_temperature/config", device_id);

    sprintf(payload,
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
    sprintf(topic, "homeassistant/sensor/%s_humidity/config", device_id);

    sprintf(payload,
        "{"
        "\"name\":\"%s Humidity\","
        "\"state_topic\":\"lora/%s/humidity\","
        "\"unit_of_measurement\":\"%%\","
        "\"device_class\":\"humidity\","
        "\"unique_id\":\"%s_hum\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );

    mqttPublish(topic, payload, true);

    // --- PRESSURE ---
    sprintf(topic, "homeassistant/sensor/%s_pressure/config", device_id);

    sprintf(payload,
        "{"
        "\"name\":\"%s Pressure\","
        "\"state_topic\":\"lora/%s/pressure\","
        "\"unit_of_measurement\":\"hPa\","
        "\"device_class\":\"pressure\","
        "\"unique_id\":\"%s_press\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );

    mqttPublish(topic, payload, true);

    // --- MOTOR SENSOR ---
    sprintf(topic, "homeassistant/binary_sensor/%s_motor/config", device_id);

    sprintf(payload,
        "{"
        "\"name\":\"%s Motor\","
        "\"state_topic\":\"lora/%s/motor\","
        "\"payload_on\":\"1\","
        "\"payload_off\":\"0\","
        "\"device_class\":\"power\","
        "\"unique_id\":\"%s_motor\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_name
    );

    mqttPublish(topic, payload, true);

    // --- RELAY SWITCH ---
    sprintf(topic, "homeassistant/switch/%s_relay/config", device_id);

    sprintf(payload,
        "{"
        "\"name\":\"%s Relay\","
        "\"command_topic\":\"lora/%s/cmd\","
        "\"state_topic\":\"lora/%s/relay\","
        "\"payload_on\":\"1\","
        "\"payload_off\":\"0\","
        "\"unique_id\":\"%s_relay\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\""
        "}"
        "}",
        device_name, device_id, device_id, device_id, device_id, device_name
    );

    mqttPublish(topic, payload, true);

    Serial.printf("Discovery published for node %d\n", node_id);

#endif
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

    String msg;
    for (int i = 0; i < length; i++) msg += (char)payload[i];

    if (String(topic).startsWith("lora/node_")) {
        int node = atoi(topic + 10);
        sendCommand(node, 1, msg.toInt());
    }
    client.subscribe("lora/+/cmd");

#ifdef ROLE_BASE

    String t = String(topic);

    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.printf("MQTT cmd: %s = %s\n", topic, msg.c_str());

    // topic: lora/node_1/cmd
    if (t.startsWith("lora/node_")) {

        int node = t.substring(10, t.indexOf("/cmd")).toInt();

        uint8_t value = (msg == "1") ? 1 : 0;

        sendCommand(node, 1, value);

        Serial.printf("Send command to node %d: %d\n", node, value);
    }

#endif
}

void mqttInit() {
#ifdef ROLE_BASE
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(mqttCallback);

    client.setKeepAlive(60);      // вместо 15 секунд
    client.setBufferSize(512);    // для Home Assistant discovery
#endif
}

static void mqttConnect() {

    if (client.connect(
            MQTT_CLIENT_ID,
            MQTT_USER,
            MQTT_PASS,
            "lora/base/status",
            0,
            true,
            "offline")) {

        client.publish("lora/base/status", "online", true);
        client.subscribe("lora/+/cmd");

        sendDiscovery();
    }
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
        client.loop();   // ВАЖНО: всегда вызываем
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