#include <WiFi.h>
#include "mqtt_manager.h"

static unsigned long lastHeartbeat = 0;
#define HEARTBEAT_INTERVAL 30000

void systemLoop() {

#ifdef ROLE_BASE

    if (millis() - lastHeartbeat < HEARTBEAT_INTERVAL)
        return;

    lastHeartbeat = millis();

    char buffer[32];

    sprintf(buffer, "%lu", millis() / 1000);
    mqttPublish("lora/base/uptime", buffer, true);

    sprintf(buffer, "%u", ESP.getFreeHeap());
    mqttPublish("lora/base/heap", buffer, true);

    sprintf(buffer, "%d", WiFi.RSSI());
    mqttPublish("lora/base/wifi_rssi", buffer, true);

#endif
}