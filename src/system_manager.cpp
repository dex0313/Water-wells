#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "lora_manager.h"
#include "config.h"
#include "persistence.h"

#if defined(ROLE_BASE)
static unsigned long lastHeartbeatCheck = 0;
#endif

// ============================================================
// ROLE_BASE
// ============================================================

#if defined(ROLE_BASE)

void baseLoop() {
    wdtReset();            // Feed the watchdog

    wifiLoop();
    mqttLoop();
    loraLoop();
    sensorLoop();

    // Process pending ACK timeouts and retries
    loraProcessPendingAcks();

    // Periodically check node online/offline status
    // (also publishes the LoRa statistics every 5 min, P1-3)
    if (millis() - lastHeartbeatCheck >= HEARTBEAT_CHECK_INTERVAL) {
        lastHeartbeatCheck = millis();
        loraCheckHeartbeat();
    }
}

// ============================================================
// ROLE_NODE
// ============================================================

#elif defined(ROLE_NODE)

void nodeLoop() {
    wdtReset();            // Feed the watchdog

    loraLoop();            // includes txPoll() — non-blocking TX (R3)
    sensorLoop();
}

// ============================================================
// ROLE_REPEATER
// ============================================================

#elif defined(ROLE_REPEATER)

void repeaterLoop() {
    wdtReset();            // Feed the watchdog

    loraLoop();            // RX parse + dedup + forward + txPoll()
    sensorLoop();          // R8.1 heartbeat (PKT_DATA, empty profile)
    repeaterStatusLoop();  // R8.2 diagnostics (PKT_STATUS)
}

#endif
