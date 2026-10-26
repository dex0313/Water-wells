#include <Arduino.h>
#include "config.h"
#include "persistence.h"
#include <esp_task_wdt.h>
#include <esp_arduino_version.h>
#include <Preferences.h>

// ============================================================
// Watchdog Timer Implementation (P1-9: core 2.x AND 3.x)
// ============================================================
// ESP32 has a built-in Task Watchdog Timer (TWDT).
// We subscribe the current task to the TWDT. If wdtReset()
// is not called within WDT_TIMEOUT_S, the TWDT triggers a
// system panic and reboots the device.
//
// esp_task_wdt_init(timeout, panic) is the Arduino-core 2.x API;
// core 3.x replaced it with esp_task_wdt_reconfigure()/init() over
// esp_task_wdt_config_t. Both variants are handled here.
// ============================================================

void wdtInit() {
    Serial.printf("[WDT] Initializing watchdog, timeout=%ds\n", WDT_TIMEOUT_S);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms     = (uint32_t)WDT_TIMEOUT_S * 1000U;
    cfg.idle_core_mask = 0;          // watch only our loop task
    cfg.trigger_panic  = true;

    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        // TWDT was not initialized yet (possible on core 3.x) — do it.
        err = esp_task_wdt_init(&cfg);
    }
    if (err == ESP_OK) {
        Serial.println("[WDT] Watchdog initialized OK (core 3.x API)");
    } else {
        Serial.printf("[WDT] WARNING: esp_task_wdt reconfigure/init failed: 0x%x\n", err);
    }
#else
    esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_S, true);
    if (err == ESP_OK) {
        Serial.println("[WDT] Watchdog initialized OK (core 2.x API)");
    } else {
        Serial.printf("[WDT] WARNING: esp_task_wdt_init failed: 0x%x\n", err);
    }
#endif

    // Subscribe the current task (Arduino loop task) to the watchdog
    err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) {
        Serial.println("[WDT] Task subscribed to watchdog");
    } else {
        // ESP_ERR_INVALID_STATE means already subscribed — not a real error
        if (err == ESP_ERR_INVALID_STATE) {
            Serial.println("[WDT] (already subscribed — this is OK on soft reboot)");
        } else {
            Serial.printf("[WDT] WARNING: esp_task_wdt_add failed: 0x%x\n", err);
        }
    }
}

void wdtReset() {
    esp_task_wdt_reset();
}

// ============================================================
// NVS Relay State Persistence
// ============================================================
// Uses ESP32 Preferences library (built on top of NVS flash).
//
// 1. RW mode only when writing, then closed — minimizes flash wear.
// 2. The whole relay bitmask is stored as a single uint8_t.
// 3. On boot the mask is read and applied to the pins by
//    relaysInit() (P0-6) BEFORE anything else runs.
// 4. A boot counter is maintained for diagnostics (PKT_STATUS, R8).
//
// Namespace/keys are FROZEN — see config.h (0.1 rule 3).
// ============================================================

static Preferences prefs;
static uint32_t s_bootCount = 0;

void nvsSaveRelayStates(uint8_t relay_mask) {
    // P2-7: check the result of prefs.begin().
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS] ERROR: cannot open namespace for writing");
        return;
    }
    prefs.putUChar(NVS_KEY_RELAYS, relay_mask);
    prefs.end();

    Serial.printf("[NVS] Relay states saved: 0x%02X\n", relay_mask);
}

uint8_t nvsLoadRelayStates() {
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        // P2-7: missing namespace is normal on the very first boot —
        // the safe default is all relays OFF.
        Serial.println("[NVS] No stored relay states (first boot?) — default OFF");
        return 0;
    }
    uint8_t mask = prefs.getUChar(NVS_KEY_RELAYS, 0);
    prefs.end();

    Serial.printf("[NVS] Relay states loaded: 0x%02X\n", mask);
    return mask;
}

uint32_t nvsIncrementBootCount() {
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS] ERROR: cannot open namespace (boot counter)");
        return s_bootCount;
    }
    uint32_t count = prefs.getUInt(NVS_KEY_BOOT_CNT, 0);
    count++;
    prefs.putUInt(NVS_KEY_BOOT_CNT, count);
    prefs.end();

    s_bootCount = count;
    Serial.printf("[NVS] Boot count: %lu\n", (unsigned long)count);
    return count;
}

uint32_t nvsBootCount() {
    // Value of this boot's counter (used by PKT_STATUS, R8.2).
    return s_bootCount;
}
