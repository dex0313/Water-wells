#include "wifi_manager.h"
#include "config.h"

static unsigned long lastAttempt = 0;
static unsigned long disconnectTime = 0;
static bool wasConnected = false;

void wifiInit() {

#ifdef ROLE_BASE
    WiFi.mode(WIFI_STA);

#if WIFI_STATIC_IP
    WiFi.config(WIFI_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_DNS);
#endif

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif
}

void wifiLoop() {

#ifdef ROLE_BASE

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

    if (millis() - lastAttempt > WIFI_RETRY_INTERVAL) {
        lastAttempt = millis();
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    if (disconnectTime > 0 &&
        millis() - disconnectTime > WIFI_RESTART_TIMEOUT) {
        ESP.restart();
    }

#endif
}

bool wifiConnected() {
#ifdef ROLE_BASE
    return WiFi.status() == WL_CONNECTED;
#else
    return false;
#endif
}