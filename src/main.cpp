#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"

void setup() {
    Serial.begin(115200);

#ifdef ROLE_BASE
    wifiInit();
    mqttInit();
    sensorInit();
#endif
}

void loop() {

#ifdef ROLE_BASE
    wifiLoop();
    mqttLoop();
    sensorLoop();
    systemLoop();
#endif

}