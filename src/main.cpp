#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "lora_manager.h"


void setup() {
    Serial.begin(115200);

    loraInit();
    sensorInit();

#ifdef ROLE_BASE
    wifiInit();
    mqttInit();
#endif
}

void loop() {

#ifdef ROLE_BASE
    baseLoop();
#else
    nodeLoop();
#endif

}