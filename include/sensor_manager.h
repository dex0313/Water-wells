#pragma once
#include "config.h"   // FEATURE_SENSORS must be known here (FR-2)

// FR-2: with FEATURE_SENSORS=0 (config.h) the BME280/ZMPT101B code
// is compiled out; the relay API below keeps working unchanged.
#if FEATURE_SENSORS
#include <Adafruit_BME280.h>

extern Adafruit_BME280 bme;
#endif

void sensorInit();
void sensorLoop();
bool sensorAvailable();

#if defined(ROLE_NODE)
// ------------------------------------------------------------
// Relay control (P0-6)
// ------------------------------------------------------------

/**
 * Initialize relay pins from the NVS-saved mask.
 * MUST run before wdtInit() and loraInit(): the output latch is
 * preloaded with the target level BEFORE the output driver is
 * enabled, so a running pump never sees an OFF pulse during reboot.
 * (setup() calls Serial.begin() first — see main.cpp — but the
 * relay pins are still the first pins touched, PR-1/P0-6.)
 */
void relaysInit();

/**
 * Single mutation point for a relay: writes the pin, updates the
 * shadow state and saves NVS only when the bit actually changed.
 */
void relaySet(uint8_t idx, bool on);

/**
 * Shadow state of one relay (0/1) without touching the hardware.
 */
uint8_t relayGet(uint8_t idx);

/**
 * Read all relay states as a bitmask (from the shadow, not from pins).
 * Bit 0 = relay 0, bit 1 = relay 1, etc.
 */
uint8_t readRelayStates();

/**
 * Read RMS voltage from a specific ZMPT101B channel (P1-7 method).
 * Compiled only with FEATURE_SENSORS=1 (FR-2).
 * @param channel  Index (0..NODE_NUM_SENSORS-1) into ZMPT_PINS array
 * @return         Calibrated RMS voltage in Volts
 */
float readZMPT101B(uint8_t channel);
#endif // ROLE_NODE

// ------------------------------------------------------------
// Reporting (NODE / REPEATER)
// ------------------------------------------------------------

/**
 * Collect one full report (BME + relays + voltages + motors) and
 * send it over LoRa. Used by the periodic report loop AND by the
 * deferred post-command report (P1-6) — a single implementation.
 * With FEATURE_SENSORS=0 (FR-2) the report carries relays only:
 * t/h/p = NaN, num_sensors = 0, motor_states = 0.
 */
void collectAndSendReport();

/**
 * Request a full report delay_ms milliseconds from now (P1-6).
 * Repeated requests re-schedule (merge) the pending report.
 */
void scheduleReport(uint32_t delay_ms);
