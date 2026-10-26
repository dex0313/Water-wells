// ============================================================
// wifi_manager.cpp — BASE role WiFi station management (P2-1)
// ============================================================

#include "wifi_manager.h"
#include "config.h"

#if defined(ROLE_BASE)
#include <WiFi.h>
#include "secret.h"

static unsigned long lastAttempt = 0;
static unsigned long disconnectTime = 0;
static bool wasConnected = false;

void wifiInit() {
    // P2-1: persistent(false) BEFORE mode — writing flash-stored
    // config on every begin() slows association and can corrupt it.
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);

    // P2-1: modem sleep delays incoming MQTT commands; auto
    // reconnect lets the stack finish an ongoing association.
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);

// #if WIFI_STATIC_IP
//     WiFi.config(WIFI_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_DNS);
// #endif

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // P2-1: arm the reboot timeout right away — previously the
    // "reboot after 5 min without WiFi" never fired when the very
    // first association failed.
    disconnectTime = millis();

    Serial.println("WiFi init done");
}

void wifiLoop() {
    wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED) {
        if (!wasConnected) {
            wasConnected = true;
            disconnectTime = 0;
            Serial.print("WiFi connected. IP: ");
            Serial.println(WiFi.localIP());
        }
        return;
    }

    if (wasConnected) {
        wasConnected = false;
        disconnectTime = millis();
        Serial.println("WiFi disconnected");
    }

    // P2-1: never tear down an association in progress. The stack
    // reconnects itself (autoReconnect); nudge it periodically only
    // when it is idle.
    if (status != WL_IDLE_STATUS &&
        millis() - lastAttempt > WIFI_RETRY_INTERVAL) {
        lastAttempt = millis();
        WiFi.reconnect();
    }

    // Reboot timeout kept (P2-1).
    if (disconnectTime > 0 &&
        millis() - disconnectTime > WIFI_RESTART_TIMEOUT) {
        Serial.println("[WiFi] Restart timeout - rebooting");
        ESP.restart();
    }
}

bool wifiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

#else

void wifiInit() {}
void wifiLoop() {}
bool wifiConnected() { return false; }

#endif // ROLE_BASE
