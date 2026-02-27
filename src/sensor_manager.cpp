#include <Wire.h>
#include <Adafruit_BME280.h>
#include "mqtt_manager.h"

static Adafruit_BME280 bme;
static unsigned long lastPublish = 0;
#define SENSOR_INTERVAL 60000

void sensorInit() {
#ifdef ROLE_BASE
    bme.begin(0x76);
#endif
}

void sensorLoop() {

#ifdef ROLE_BASE

    if (millis() - lastPublish < SENSOR_INTERVAL)
        return;

    lastPublish = millis();

    char buffer[32];

    dtostrf(bme.readTemperature(), 1, 2, buffer);
    mqttPublish("lora/base/temperature", buffer, true);

    dtostrf(bme.readHumidity(), 1, 2, buffer);
    mqttPublish("lora/base/humidity", buffer, true);

    dtostrf(bme.readPressure() / 100.0F, 1, 2, buffer);
    mqttPublish("lora/base/pressure", buffer, true);

#endif
}