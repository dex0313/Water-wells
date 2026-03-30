#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "lora_manager.h"


void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Water Wells System ===");
    loraInit();
    sensorInit();

#ifdef ROLE_BASE
    Serial.println("Role: BASE");
    wifiInit();
    mqttInit();
#else
    Serial.println("Role: NODE");
#endif
}

void loop() {

#ifdef ROLE_BASE
    baseLoop();
#else
    nodeLoop();
#endif
}