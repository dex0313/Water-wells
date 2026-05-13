#pragma once
#include <Adafruit_BME280.h>

extern Adafruit_BME280 bme;

void sensorInit();
void sensorLoop();
bool sensorAvailable();

/**
 * Read RMS voltage from a specific ZMPT101B channel.
 * @param channel  Index (0..NODE_NUM_SENSORS-1) into ZMPT_PINS array
 * @return         Calibrated RMS voltage in Volts
 */
float readZMPT101B(uint8_t channel);

/**
 * Read all relay states as a bitmask.
 * Bit 0 = relay 0, bit 1 = relay 1, etc.
 */
uint8_t readRelayStates();

/**
 * Read all motor states as a bitmask (derived from ZMPT voltages).
 * Bit 0 = sensor 0, bit 1 = sensor 1, etc.
 */
uint8_t readMotorStates();