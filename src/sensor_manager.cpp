#include <Wire.h>
#include <Adafruit_BME280.h>
#include "config.h"
#include "mqtt_manager.h"
#include "lora_manager.h"
#include "sensor_manager.h"
#include "persistence.h"
#include <WiFi.h>

Adafruit_BME280 bme;
static unsigned long lastPublish = 0;
static bool sensorOk = false;

// Per-channel DC offset calibration for ZMPT101B
static float zmptDcOffset[NODE_NUM_SENSORS] = {};
static bool zmptOffsetCalibrated[NODE_NUM_SENSORS] = {};

bool sensorAvailable() {
    return sensorOk;
}

void sensorInit() {
    if (!bme.begin(0x76)) {
        Serial.println("[SENSOR] ERROR: BME280 not found at 0x76!");
        sensorOk = false;
        return;
    }
    sensorOk = true;
    Serial.println("[SENSOR] BME280 init OK");

#ifdef ROLE_NODE
    // Initialize all relay pins (first set to OFF as safe default)
    for (uint8_t i = 0; i < NODE_NUM_RELAYS; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        digitalWrite(RELAY_PINS[i], RELAY_OFF);
    }

    // Restore relay states from NVS (saved before last reboot/power-loss)
    // This ensures pumps/valves return to their previous state automatically.
    restoreRelayStates();

    // Configure ADC for ZMPT101B
    analogSetAttenuation(ADC_11db);        // Full range 0-3.3V
    analogReadResolution(12);              // 12-bit (0-4095)

    // Initialize all ZMPT101B pins
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        pinMode(ZMPT_PINS[i], INPUT);
        zmptDcOffset[i] = 2048.0;  // Default midpoint
        zmptOffsetCalibrated[i] = false;
    }

    Serial.printf("[SENSOR] %d relays + %d ZMPT101B pins init OK\n",
                  NODE_NUM_RELAYS, NODE_NUM_SENSORS);
#endif
}

// ============================================================
// ZMPT101B Voltage Sensor — RMS Reading (per-channel)
// ============================================================
//
// The ZMPT101B outputs an AC sine wave centered at VCC/2 (~1.65V).
// To measure actual mains voltage we:
//   1. Sample the ADC for ~40ms (2 full AC cycles at 50Hz)
//   2. Subtract the DC offset (calibrated per-channel on first read)
//   3. Calculate RMS of the AC component
//   4. Apply per-channel calibration factor to convert to real voltage
//
// IMPORTANT: GPIO 25 is ADC2 on ESP32. ADC2 does NOT work when
// WiFi is active. This is fine for NODE role (no WiFi), but
// would fail on BASE. If you ever need ZMPT101B on BASE,
// move it to an ADC1 pin (GPIO 32-39).
// ============================================================

float readZMPT101B(uint8_t channel) {
    if (channel >= NODE_NUM_SENSORS) return 0.0;

    uint8_t pin = ZMPT_PINS[channel];

    // Calibrate DC offset on first reading for this channel
    if (!zmptOffsetCalibrated[channel]) {
        float offset_sum = 0;
        for (int i = 0; i < 100; i++) {
            offset_sum += analogRead(pin);
            delayMicroseconds(200);
        }
        zmptDcOffset[channel] = offset_sum / 100.0;
        zmptOffsetCalibrated[channel] = true;
        Serial.printf("[ZMPT101B] Ch%d DC offset calibrated: %.1f\n",
                      channel, zmptDcOffset[channel]);
    }

    int sample_count = 0;
    float sum_squared = 0.0;

    unsigned long start_time = micros();

    // Sample for the configured window (default 40ms = 2 cycles at 50Hz)
    while (micros() - start_time < (uint32_t)ZMPT101B_SAMPLE_MS * 1000) {
        int raw = analogRead(pin);

        // Subtract DC offset to isolate the AC component
        float shifted = (float)raw - zmptDcOffset[channel];

        // Accumulate squared values for RMS calculation
        sum_squared += shifted * shifted;
        sample_count++;
    }

    if (sample_count == 0) return 0.0;

    // Calculate RMS of AC component (in ADC units)
    float rms_adc = sqrt(sum_squared / (float)sample_count);

    // Convert ADC RMS to voltage RMS
    float v_rms_raw = rms_adc * 3.3 / 4095.0;

    // Apply per-channel calibration factor
    float voltage = v_rms_raw * ZMPT_CALIBRATIONS[channel];

    // Apply noise floor threshold
    if (voltage < ZMPT101B_ZERO_THRESHOLD) {
        voltage = 0.0;
    }

    Serial.printf("[ZMPT101B] Ch%d: raw_rms=%.1f ADC, voltage=%.1f V\n",
                  channel, rms_adc, voltage);
    return voltage;
}

// ============================================================
// Restore relay states from NVS after boot
// ============================================================
// Reads the saved bitmask from flash and applies each bit
// to the corresponding relay pin. This is called once during
// sensorInit() so that relays return to their pre-reboot state.
// ============================================================

void restoreRelayStates() {
    uint8_t saved = nvsLoadRelayStates();

    // Clamp: only bits 0..NODE_NUM_RELAYS-1 are valid
    uint8_t valid_mask = (1 << NODE_NUM_RELAYS) - 1;
    saved &= valid_mask;

    for (uint8_t i = 0; i < NODE_NUM_RELAYS; i++) {
        if (saved & (1 << i)) {
            digitalWrite(RELAY_PINS[i], RELAY_ON);
            Serial.printf("[NVS] Relay %d restored -> ON\n", i + 1);
        } else {
            digitalWrite(RELAY_PINS[i], RELAY_OFF);
            // OFF is already the default from init, but log for clarity
        }
    }

    Serial.printf("[NVS] Relay states restored: 0x%02X\n", saved);
}

// ============================================================
// Read all relay states as bitmask
// ============================================================

uint8_t readRelayStates() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NODE_NUM_RELAYS; i++) {
        if (digitalRead(RELAY_PINS[i]) == RELAY_ON) {
            mask |= (1 << i);
        }
    }
    return mask;
}

// ============================================================
// Read all motor states as bitmask (derived from ZMPT voltage)
// ============================================================

uint8_t readMotorStates() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        float v = readZMPT101B(i);
        if (v > MOTOR_THRESHOLDS[i]) {
            mask |= (1 << i);
        }
    }
    return mask;
}

// ============================================================
// Sensor loop — periodic data publishing
// ============================================================

void sensorLoop() {
    if (millis() - lastPublish < SENSOR_INTERVAL) return;
    lastPublish = millis();

    if (!sensorOk) {
        Serial.println("[SENSOR] Skip - BME280 not available");
        return;
    }

    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;

#ifdef ROLE_BASE
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

#else
    // Read all relay states as bitmask
    uint8_t relay_states = readRelayStates();

    // Read all ZMPT101B voltages
    float voltages[NODE_NUM_SENSORS];
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        voltages[i] = readZMPT101B(i);
    }

    // Derive motor states from voltages
    uint8_t motor_states = 0;
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        if (voltages[i] > MOTOR_THRESHOLDS[i]) {
            motor_states |= (1 << i);
        }
    }

    // Debug output
    Serial.printf("[NODE] Sending: T=%.1f H=%.1f P=%.1f R=0x%02X M=0x%02X",
                  t, h, p, relay_states, motor_states);
    for (uint8_t i = 0; i < NODE_NUM_SENSORS; i++) {
        Serial.printf(" V%d=%.0f", i + 1, voltages[i]);
    }
    Serial.println();

    sendData(t, h, p, NODE_NUM_RELAYS, NODE_NUM_SENSORS,
             relay_states, motor_states, voltages);
#endif
}