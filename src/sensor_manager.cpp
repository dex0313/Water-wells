// ============================================================
// sensor_manager.cpp — BME280 / ZMPT101B / relays / reporting
// ============================================================
// Role layout (P1-8): role branches are whole blocks, the mixed
// #ifdef style inside functions is gone.
//   ROLE_NODE      relays (P0-6) + ZMPT (P1-7) + full reports
//   ROLE_REPEATER  BME optional + heartbeat reports (R8.1)
//   ROLE_BASE      local BME publishing (skip when missing)
//
// FR-2: FEATURE_SENSORS=0 (config.h) compiles out ALL local sensor
// code (BME280 + ZMPT101B) for relay-only wells. Relays (P0-6)
// keep working unchanged; reports carry t/h/p = NaN and
// num_sensors = 0.
// ============================================================

#include <Arduino.h>   // FR-2: not transitively included without sensors
#include <math.h>
#include "config.h"
#include "mqtt_manager.h"
#include "lora_manager.h"
#include "sensor_manager.h"
#include "persistence.h"

#if FEATURE_SENSORS
#include <Wire.h>
#include <Adafruit_BME280.h>
#endif

#if defined(ROLE_BASE)
#include <WiFi.h>
#endif
#if defined(ROLE_NODE) || defined(ROLE_REPEATER)
#include <esp_system.h>
#endif
#if defined(ROLE_NODE)
#include "driver/gpio.h"
#endif

#if FEATURE_SENSORS
Adafruit_BME280 bme;

static bool sensorOk = false;
static bool isBmp280 = false;         // BMP280 chip: no humidity (P2-7)
static uint32_t lastBmeAttempt = 0;

bool sensorAvailable() {
    return sensorOk;
}
#else
bool sensorAvailable() {
    return false;                     // FR-2: local sensors compiled out
}
#endif

// ============================================================
// BME280 detection (P0-1 / P2-7)
// ============================================================
// The result of bme.begin() is NOT fatal (P0-1): relays and ADC are
// initialized before this and the node keeps working without
// temperature/humidity/pressure. sensorLoop retries every
// BME_RETRY_INTERVAL_MS while the sensor is missing.
// FR-2: the whole detection block is compiled only with sensors.
// ============================================================

#if FEATURE_SENSORS

static uint8_t readChipId(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0xD0);                       // BME/BMP chip-id register
    if (Wire.endTransmission(false) != 0) return 0x00;
    if (Wire.requestFrom((int)addr, 1) != 1) return 0x00;
    return Wire.read();
}

static bool tryInitBmeAt(uint8_t addr) {
    uint8_t chip = readChipId(addr);
    isBmp280 = (chip == 0x58);              // BMP280

    if (chip == 0x00) {
        return false;                       // nothing answered
    }
    if (chip != 0x60 && !isBmp280) {
        Serial.printf("[SENSOR] Unknown chip id 0x%02X at 0x%02X\n", chip, addr);
        return false;
    }
    if (isBmp280) {
        Serial.println("[SENSOR] BMP280 detected: humidity unavailable");
    }

    if (!bme.begin(addr)) {
        // The Adafruit_BME280 library (2.2.4) hard-rejects BMP280 chips
        // (init() checks _sensorID == 0x60 only). Everything stays
        // unavailable and we retry — the log below tells the owner why.
        if (isBmp280) {
            Serial.println("[SENSOR] BMP280 present but Adafruit_BME280 2.2.4 "
                           "does not support it (chip 0x58 != 0x60)");
        }
        return false;
    }
    Serial.printf("[SENSOR] BME280 init OK at 0x%02X\n", addr);
    return true;
}

static void tryInitBme() {
    lastBmeAttempt = millis();
    Wire.begin();
    // P2-7: probe both common addresses.
    if (tryInitBmeAt(0x76) || tryInitBmeAt(0x77)) {
        sensorOk = true;
    } else {
        sensorOk = false;
        isBmp280 = false;
        Serial.println("[SENSOR] ERROR: BME280 not found on 0x76/0x77!");
    }
}

#endif // FEATURE_SENSORS (BME280 detection, FR-2)

// ============================================================
// ROLE_NODE — relays (P0-6) and ZMPT101B (P1-7)
// ============================================================

#if defined(ROLE_NODE)

static uint8_t relayShadow = 0;

void relaysInit() {
    // Called from setup() right after Serial.begin() (+10 ms UART
    // settle, so the restore log below is actually visible) and
    // BEFORE wdtInit()/loraInit(). Still the first code that touches
    // the relay pins: order per pin is preload the output latch with
    // the TARGET level, only then enable the output driver. No
    // "safety OFF" pulse ever reaches a running pump.
    uint8_t saved = nvsLoadRelayStates() & (uint8_t)((1u << NODE_NUM_RELAYS) - 1);
    for (uint8_t i = 0; i < NODE_NUM_RELAYS; i++) {
        uint8_t level = (saved & (1u << i)) ? RELAY_ON : RELAY_OFF;
        // gpio_set_level writes the output latch even while the pad
        // is still an input; it behaves the same on core 2.x/3.x.
        gpio_set_level((gpio_num_t)RELAY_PINS[i], level);
        pinMode(RELAY_PINS[i], OUTPUT);
    }
    relayShadow = saved;
    Serial.printf("[NVS] Relay states restored: 0x%02X\n", saved);
}

void relaySet(uint8_t idx, bool on) {
    if (idx >= NODE_NUM_RELAYS) return;

    uint8_t bit = (uint8_t)(1u << idx);
    uint8_t new_mask = on ? (uint8_t)(relayShadow | bit)
                          : (uint8_t)(relayShadow & ~bit);

    digitalWrite(RELAY_PINS[idx], on ? RELAY_ON : RELAY_OFF);

    // NVS write only on a real change — saves flash wear on
    // idempotent repeated commands (P0-3).
    if (new_mask != relayShadow) {
        relayShadow = new_mask;
        nvsSaveRelayStates(relayShadow);
    }
}

uint8_t relayGet(uint8_t idx) {
    if (idx >= NODE_NUM_RELAYS) return 0;
    return (relayShadow >> idx) & 1;
}

uint8_t readRelayStates() {
    return relayShadow;
}

#if FEATURE_SENSORS

// ------------------------------------------------------------
// ZMPT101B RMS (P1-7)
// ------------------------------------------------------------
// Single pass over one window: mean and variance in the same loop.
// The DC component is removed mathematically (variance of the
// signal), so the old per-channel DC offset calibration arrays and
// the calibration cycle are gone entirely.
//
// The window is ZMPT101B_SAMPLE_MS = 100 ms = five full 50 Hz
// periods, immune to grid frequency drift.
//
// Blocking: 100 ms per channel, up to 400 ms for 4 channels —
// acceptable since the UART RX buffer is 1024 bytes (P1-2).
//
// NOTE: GPIO 25 is ADC2 — it does NOT work while WiFi is active.
// Fine for NODE/REPEATER (no WiFi); a BASE with ZMPT must use an
// ADC1 pin (GPIO 32..39).
// ------------------------------------------------------------

float readZMPT101B(uint8_t channel) {
    if (channel >= NODE_NUM_SENSORS) return 0.0f;

    const uint8_t pin = ZMPT_PINS[channel];
    const uint32_t window_us = (uint32_t)ZMPT101B_SAMPLE_MS * 1000UL;
    uint32_t n = 0;
    uint64_t sum = 0, sumsq = 0;
    const uint32_t t0 = micros();

    while ((uint32_t)(micros() - t0) < window_us) {
        uint32_t r = analogRead(pin);
        sum += r;
        sumsq += (uint64_t)r * r;
        n++;
    }
    if (n < 100) return 0.0f;

    double mean = (double)sum / n;
    double var  = (double)sumsq / n - mean * mean;
    if (var < 0) var = 0;

    float v = (float)(sqrt(var) * 3.3 / 4095.0) * ZMPT_CALIBRATIONS[channel];
    if (v < ZMPT101B_ZERO_THRESHOLD) v = 0.0f;

#ifdef ZMPT_DEBUG
    // P1-7: serial spam removed from the hot path, enable only when
    // calibrating.
    Serial.printf("[ZMPT101B] Ch%u: n=%lu voltage=%.1f V\n",
                  (unsigned)channel, (unsigned long)n, v);
#endif
    return v;
}

#endif // FEATURE_SENSORS (ZMPT101B, FR-2)

#endif // ROLE_NODE

// ============================================================
// Reporting (NODE / REPEATER) — P1-5 / P1-6 / R8.1
// ============================================================

#if defined(ROLE_NODE) || defined(ROLE_REPEATER)

static uint32_t nextReportAt = 0;
static bool     deferredPending = false;
static uint32_t deferredReportAt = 0;

// P1-5: interval jitter +-10% so nodes powered up together do not
// collide on air forever.
static uint32_t jitteredInterval() {
    int32_t j = (int32_t)(esp_random() % (SENSOR_INTERVAL / 5)) - (int32_t)(SENSOR_INTERVAL / 10);
    return (uint32_t)((int32_t)SENSOR_INTERVAL + j);
}

void scheduleReport(uint32_t delay_ms) {
    deferredReportAt = millis() + delay_ms;
    deferredPending = true;
}

// Single report implementation used by the periodic loop, the
// deferred post-command report (P1-6) and the repeater heartbeat.
void collectAndSendReport() {
#if defined(ROLE_NODE)
    // P0-2: BME280 failure must not kill the node — report with NaN.
    float t = NAN, h = NAN, p = NAN;
#if FEATURE_SENSORS
    if (sensorOk) {
        t = bme.readTemperature();
        h = isBmp280 ? NAN : bme.readHumidity();
        p = bme.readPressure() / 100.0F;
    }
#endif

    uint8_t relay_states = readRelayStates();

#if FEATURE_SENSORS
    float voltages[NODE_NUM_SENSORS];
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        voltages[i] = readZMPT101B(i);
    }

    // Motor bits derived from the SAME voltage samples (P2-7: the
    // duplicate readMotorStates() is gone).
    uint8_t motor_states = 0;
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        if (voltages[i] > MOTOR_THRESHOLDS[i]) {
            motor_states |= (uint8_t)(1 << i);
        }
    }
#else
    // FR-2: relay-only well — the report carries relays only
    // (num_sensors = 0, motor_states = 0, telemetry = NaN). The
    // placeholder keeps the single sendData() call below unchanged;
    // sendData() never reads the array when num_sensors is 0.
    static const float voltages[MAX_SENSORS_PER_NODE] = {0};
    uint8_t motor_states = 0;
#endif

    Serial.printf("[NODE] Sending: T=%.1f H=%.1f P=%.1f R=0x%02X M=0x%02X",
                  t, h, p, relay_states, motor_states);
#if FEATURE_SENSORS
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        Serial.printf(" V%d=%.0f", i + 1, voltages[i]);
    }
#endif
    Serial.println();

    sendData(t, h, p, NODE_NUM_RELAYS, NODE_NUM_SENSORS,
             relay_states, motor_states, voltages);

#elif defined(ROLE_REPEATER)
    // R8.1: heartbeat as a normal DATA packet with an empty profile —
    // old BASE builds show the repeater as an online node with the
    // BME entities (or unknown values when BME is absent/NaN).
    float t = NAN, h = NAN, p = NAN;
#if FEATURE_SENSORS
    if (sensorOk) {
        t = bme.readTemperature();
        h = isBmp280 ? NAN : bme.readHumidity();
        p = bme.readPressure() / 100.0F;
    }
#endif
    float voltages[MAX_SENSORS_PER_NODE] = {0};
    sendData(t, h, p, 0, 0, 0, 0, voltages);
#endif
}

#endif // ROLE_NODE || ROLE_REPEATER

// ============================================================
// Init
// ============================================================

void sensorInit() {
#if defined(ROLE_NODE) && FEATURE_SENSORS
    // P0-1: ADC/ZMPT initialization is unconditional and happens
    // BEFORE the BME280 attempt (a missing BME used to abort here
    // and leave the node without pump control).
    analogSetAttenuation(ADC_11db);        // Full range 0-3.3V
    analogReadResolution(12);              // 12-bit (0-4095)
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        pinMode(ZMPT_PINS[i], INPUT);
    }
    // NOTE: relay pins were already configured by relaysInit() as the
    // first code that touches the relay pins in setup() (P0-6/PR-1).
#endif

#if FEATURE_SENSORS
    // P0-1: failure is not fatal.
    tryInitBme();
#endif

#if defined(ROLE_NODE) || defined(ROLE_REPEATER)
    // P1-5: first report is randomized 5..30 s after boot so nodes
    // restored by a common power outage do not transmit in sync.
    // FR-2: kept for relay-only wells — relays still report state.
    nextReportAt = millis() + FIRST_REPORT_MIN_MS +
                   esp_random() % (FIRST_REPORT_MAX_MS - FIRST_REPORT_MIN_MS);
#endif

#if defined(ROLE_NODE)
#if FEATURE_SENSORS
    Serial.printf("[SENSOR] %d relays + %d ZMPT101B pins init OK\n",
                  NODE_NUM_RELAYS, NODE_NUM_SENSORS);
#else
    Serial.printf("[SENSOR] Relay-only build (FEATURE_SENSORS=0): "
                  "%d relays, sensors compiled out\n", NODE_NUM_RELAYS);
#endif
#endif
}

// ============================================================
// Loop
// ============================================================

void sensorLoop() {
#if defined(ROLE_BASE)

#if FEATURE_SENSORS
    // BASE: local BME publishing, skip when the sensor is missing
    // (unchanged behavior, P0-1). FR-2: compiled out without sensors.
    static uint32_t nextBasePublish = 0;
    if ((int32_t)(millis() - nextBasePublish) < 0) return;
    nextBasePublish = millis() + SENSOR_INTERVAL;

    if (!sensorOk) {
        Serial.println("[SENSOR] Skip - BME280 not available");
        return;
    }

    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;

    char buffer[32];

    dtostrf(t, 1, 2, buffer);
    mqttPublish("lora/base/temperature", buffer, true);

    dtostrf(h, 1, 2, buffer);
    mqttPublish("lora/base/humidity", buffer, true);

    dtostrf(p, 1, 2, buffer);
    mqttPublish("lora/base/pressure", buffer, true);

    // WiFi RSSI
    snprintf(buffer, sizeof(buffer), "%d", WiFi.RSSI());
    mqttPublish("lora/base/wifi_rssi", buffer, true);
#endif // FEATURE_SENSORS

#else

#if FEATURE_SENSORS
    // P0-1: retry the BME280 every 10 minutes while it is missing.
    if (!sensorOk && (millis() - lastBmeAttempt) >= BME_RETRY_INTERVAL_MS) {
        Serial.println("[SENSOR] Retrying BME280 init...");
        tryInitBme();
    }
#endif

    // P1-6: deferred report after a relay command.
    if (deferredPending && (int32_t)(millis() - deferredReportAt) >= 0) {
        deferredPending = false;
        collectAndSendReport();
        // Shift the periodic report so the two do not pile up.
        nextReportAt = millis() + jitteredInterval();
        return;
    }

    // P1-5: periodic report with wrap-safe signed compare.
    if ((int32_t)(millis() - nextReportAt) >= 0) {
        collectAndSendReport();
        nextReportAt = millis() + jitteredInterval();
    }

#endif
}
