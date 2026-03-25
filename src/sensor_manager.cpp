#include <Wire.h>
#include <Adafruit_BME280.h>
#include "mqtt_manager.h"
#include "lora_manager.h"

#ifdef ROLE_NODE
    #define RELAY_PIN 15
    #define MOTOR_PIN 2
    #define RELAY_ON  LOW
    #define RELAY_OFF HIGH
#endif

static Adafruit_BME280 bme;
static unsigned long lastPublish = 0;
#define SENSOR_INTERVAL 60000

void sensorInit() {
    bme.begin(0x76);
    
#ifdef ROLE_NODE
    digitalWrite(RELAY_PIN, HIGH);
    pinMode(RELAY_PIN, OUTPUT);
#endif
}

void sensorLoop() {

    if (millis() - lastPublish < SENSOR_INTERVAL) return;
    lastPublish = millis();

    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0;

#ifdef ROLE_BASE

    char buffer[32];

    dtostrf(bme.readTemperature(), 1, 2, buffer);
    mqttPublish("lora/base/temperature", buffer, true);

    dtostrf(bme.readHumidity(), 1, 2, buffer);
    mqttPublish("lora/base/humidity", buffer, true);

    dtostrf(bme.readPressure() / 100.0F, 1, 2, buffer);
    mqttPublish("lora/base/pressure", buffer, true);
#else

    uint8_t relay = digitalRead(RELAY_PIN);
    uint8_t motor = analogRead(MOTOR_PIN) > 2000;
    Serial.println("NODE: sending data");
    sendData(t,h,p,relay,motor);
    
#endif
}