#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "wifi_manager.h"

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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // пока только лог
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