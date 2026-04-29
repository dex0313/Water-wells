#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "lora_manager.h"
#include "config.h"

static unsigned long lastHeartbeatCheck = 0;

void baseLoop() {
    wifiLoop();
    mqttLoop();
   // systemLoop();
    loraLoop();
    sensorLoop();

    // NEW: Process pending ACK timeouts and retries
    loraProcessPendingAcks();

    // NEW: Periodically check node online/offline status
    if (millis() - lastHeartbeatCheck >= HEARTBEAT_CHECK_INTERVAL) {
        lastHeartbeatCheck = millis();
        loraCheckHeartbeat();
    }

}

void nodeLoop() {
    loraLoop();
    sensorLoop();
}