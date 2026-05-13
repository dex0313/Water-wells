#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "lora_manager.h"
#include "config.h"
#include "persistence.h"

static unsigned long lastHeartbeatCheck = 0;

void baseLoop() {
    wdtReset();            // Feed the watchdog

    wifiLoop();
    mqttLoop();
    loraLoop();
    sensorLoop();

    // Process pending ACK timeouts and retries
    loraProcessPendingAcks();

    // Periodically check node online/offline status
    if (millis() - lastHeartbeatCheck >= HEARTBEAT_CHECK_INTERVAL) {
        lastHeartbeatCheck = millis();
        loraCheckHeartbeat();
    }
}

void nodeLoop() {
    wdtReset();            // Feed the watchdog

    loraLoop();
    sensorLoop();
}