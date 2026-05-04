#include <Wire.h>
#include <Adafruit_BME280.h>
#include "config.h"
#include "mqtt_manager.h"
#include "lora_manager.h"
#include "sensor_manager.h"
#include <WiFi.h>

Adafruit_BME280 bme;
static unsigned long lastPublish = 0;
static bool sensorOk = false;

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
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);
// Configure ADC for ZMPT101B
    analogSetAttenuation(ADC_11db);        // Full range 0-3.3V
    analogReadResolution(12);              // 12-bit (0-4095)
    pinMode(ZMPT101B_PIN, INPUT);

    Serial.println("[SENSOR] Relay + ZMPT101B pins init OK");
#endif
}

// ============================================================
// ZMPT101B Voltage Sensor — RMS Reading
// ============================================================
//
// The ZMPT101B outputs an AC sine wave centered at VCC/2 (~1.65V).
// To measure actual mains voltage we:
//   1. Sample the ADC for ~40ms (2 full AC cycles at 50Hz)
//   2. Subtract the DC offset (midpoint ~2048 for 12-bit ADC)
//   3. Calculate RMS of the AC component
//   4. Apply calibration factor to convert to real voltage
//
// IMPORTANT: GPIO 25 is ADC2 on ESP32. ADC2 does NOT work when
// WiFi is active. This is fine for NODE role (no WiFi), but
// would fail on BASE. If you ever need ZMPT101B on BASE,
// move it to an ADC1 pin (GPIO 32-39).
// ============================================================

float readZMPT101B() {
    int sample_count = 0;
    float sum_squared = 0.0;

    // Approximate midpoint of ADC range (VCC/2 for 12-bit ADC)
    // We measure the actual midpoint during first reading for better accuracy
    static float dc_offset = 2048.0;
    static bool offset_calibrated = false;

    // Calibrate DC offset: with no AC input (or at any time),
    // average several readings to find the center
    if (!offset_calibrated) {
        float offset_sum = 0;
        for (int i = 0; i < 100; i++) {
            offset_sum += analogRead(ZMPT101B_PIN);
            delayMicroseconds(200);
        }
        dc_offset = offset_sum / 100.0;
        offset_calibrated = true;
        Serial.printf("[ZMPT101B] DC offset calibrated: %.1f\n", dc_offset);
    }

    unsigned long start_time = micros();

    // Sample for the configured window (default 40ms = 2 cycles at 50Hz)
    while (micros() - start_time < (uint32_t)ZMPT101B_SAMPLE_MS * 1000) {
        int raw = analogRead(ZMPT101B_PIN);

        // Subtract DC offset to isolate the AC component
        float shifted = (float)raw - dc_offset;

        // Accumulate squared values for RMS calculation
        sum_squared += shifted * shifted;
        sample_count++;
    }

    if (sample_count == 0) return 0.0;

    // Calculate RMS of AC component (in ADC units)
    float rms_adc = sqrt(sum_squared / (float)sample_count);

    // Convert ADC RMS to voltage RMS
    // ESP32 12-bit ADC: 0-4095 corresponds to 0-3.3V (approx)
    float v_rms_raw = rms_adc * 3.3 / 4095.0;

    // Log raw value for calibration (before applying calibration factor)
    Serial.printf("[ZMPT101B] raw_rms=%.2f ADC, v_rms_raw=%.3f V\n", rms_adc, v_rms_raw);

    // Apply calibration factor
    float voltage = v_rms_raw * ZMPT101B_CALIBRATION;

    // Apply noise floor threshold
    if (voltage < ZMPT101B_ZERO_THRESHOLD) {
        voltage = 0.0;
    }

    Serial.printf("[ZMPT101B] Voltage=%.1f V\n", voltage);
    return voltage;
}

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
    uint8_t relay_state = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;
    float voltage = readZMPT101B();
    uint8_t motor_state = (voltage > MOTOR_VOLTAGE_THRESHOLD) ? 1 : 0;

    Serial.printf("[NODE] Sending: T=%.1f H=%.1f P=%.1f R=%d V=%.1f M=%d\n",
                  t, h, p, relay_state, voltage, motor_state);
    sendData(t, h, p, relay_state, voltage, motor_state);
#endif
}