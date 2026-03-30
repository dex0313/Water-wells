#include <Wire.h>
#include <Adafruit_BME280.h>
#include "config.h"
#include "mqtt_manager.h"
#include "lora_manager.h"
#include "sensor_manager.h"
#include <WiFi.h>

Adafruit_BME280 bme;
static unsigned long lastPublish = 0;
static bool sensorOk = false;

bool sensorAvailable() {
    return sensorOk;
}

void sensorInit() {
    if (!bme.begin(0x76)) {
        Serial.println("[SENSOR] ERROR: BME280 not found at 0x76!");
        sensorOk = false;
        return;
    }
    sensorOk = true;
    Serial.println("[SENSOR] BME280 init OK");

#ifdef ROLE_NODE
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pinMode(MOTOR_PIN, INPUT);
    Serial.println("[SENSOR] Relay + Motor pins init OK");
#endif
}

void sensorLoop() {
    if (millis() - lastPublish < SENSOR_INTERVAL) return;
    lastPublish = millis();

    if (!sensorOk) {
        Serial.println("[SENSOR] Skip - BME280 not available");
        return;
    }

    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;

#ifdef ROLE_BASE
    char buffer[32];

    dtostrf(t, 1, 2, buffer);
    mqttPublish("lora/base/temperature", buffer, true);

    dtostrf(h, 1, 2, buffer);
    mqttPublish("lora/base/humidity", buffer, true);

    dtostrf(p, 1, 2, buffer);
    mqttPublish("lora/base/pressure", buffer, true);

    // WiFi RSSI
    snprintf(buffer, sizeof(buffer), "%d", WiFi.RSSI());
    mqttPublish("lora/base/wifi_rssi", buffer, true);

#else
    uint8_t relay_state = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;
    uint8_t motor_state = (analogRead(MOTOR_PIN) > MOTOR_THRESHOLD) ? 1 : 0;

    Serial.printf("[NODE] Sending: T=%.1f H=%.1f P=%.1f R=%d M=%d\n",
                  t, h, p, relay_state, motor_state);
    sendData(t, h, p, relay_state, motor_state);
#endif
}