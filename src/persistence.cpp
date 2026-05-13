#include <Arduino.h>
#include "config.h"
#include "persistence.h"
#include <esp_task_wdt.h>
#include <Preferences.h>

// ============================================================
// Watchdog Timer Implementation
// ============================================================
//
// ESP32 has a built-in Task Watchdog Timer (TWDT).
// We subscribe the current task to the TWDT. If wdtReset()
// is not called within WDT_TIMEOUT_S, the TWDT triggers a
// system panic and reboots the device.
//
// This protects against:
//   - Infinite loops (e.g. I2C bus lockup with BME280)
//   - UART blocking (LoRa module freeze)
//   - Any software hang where the main loop stops running
//
// The TWDT monitors the idle task as well, so even if our
// task is stuck but the system is alive, it can still detect
// some issues. Setting panic=true ensures a full reboot.
// ============================================================

void wdtInit() {
    Serial.printf("[WDT] Initializing watchdog, timeout=%ds\n", WDT_TIMEOUT_S);

    esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_S, true);
    if (err == ESP_OK) {
        Serial.println("[WDT] Watchdog initialized OK");
    } else {
        Serial.printf("[WDT] WARNING: esp_task_wdt_init failed: 0x%x\n", err);
    }

    // Subscribe the current task (Arduino loop task) to the watchdog
    err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) {
        Serial.println("[WDT] Task subscribed to watchdog");
    } else {
        Serial.printf("[WDT] WARNING: esp_task_wdt_add failed: 0x%x\n", err);
        // ESP_ERR_INVALID_STATE means already subscribed — not a real error
        if (err == ESP_ERR_INVALID_STATE) {
            Serial.println("[WDT] (already subscribed — this is OK on soft reboot)");
        }
    }
}

void wdtReset() {
    esp_task_wdt_reset();
}

// ============================================================
// NVS Relay State Persistence
// ============================================================
//
// Uses ESP32 Preferences library (built on top of NVS flash).
// Key design decisions:
//
// 1. We open NVS in RW mode only when writing, then close.
//    This minimizes flash wear — NVS has ~100K write cycles
//    per key. At 1 write per relay toggle, this is effectively
//    unlimited for normal operation.
//
// 2. We store the entire relay bitmask as a single uint8_t.
//    For up to 8 relays, this is sufficient.
//
// 3. On boot, we read the saved mask and apply each bit to
//    the corresponding relay pin BEFORE any other code runs.
//
// 4. A boot counter is maintained separately for diagnostics.
//    It can be published to Home Assistant as a sensor.
// ============================================================

static Preferences prefs;

void nvsSaveRelayStates(uint8_t relay_mask) {
    prefs.begin(NVS_NAMESPACE, false);  // false = read-write
    prefs.putUChar(NVS_KEY_RELAYS, relay_mask);
    prefs.end();

    Serial.printf("[NVS] Relay states saved: 0x%02X\n", relay_mask);
}

uint8_t nvsLoadRelayStates() {
    prefs.begin(NVS_NAMESPACE, true);  // true = read-only
    uint8_t mask = prefs.getUChar(NVS_KEY_RELAYS, 0);
    prefs.end();

    Serial.printf("[NVS] Relay states loaded: 0x%02X\n", mask);
    return mask;
}

uint32_t nvsIncrementBootCount() {
    prefs.begin(NVS_NAMESPACE, false);  // read-write
    uint32_t count = prefs.getUInt(NVS_KEY_BOOT_CNT, 0);
    count++;
    prefs.putUInt(NVS_KEY_BOOT_CNT, count);
    prefs.end();

    Serial.printf("[NVS] Boot count: %lu\n", (unsigned long)count);
    return count;
}