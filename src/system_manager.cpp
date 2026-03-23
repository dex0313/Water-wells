#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "lora_manager.h"

void baseLoop() {
    wifiLoop();
    mqttLoop();
   // systemLoop();
    loraLoop();
    sensorLoop();
}

void nodeLoop() {
    loraLoop();
    sensorLoop();
}