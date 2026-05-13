#pragma once
#include <cstdint>

// ============================================================
// Watchdog Timer
// ============================================================

/**
 * Initialize the ESP32 task watchdog timer.
 * If wdtReset() is not called within WDT_TIMEOUT_S seconds,
 * the ESP32 will automatically reboot.
 * Must be called once in setup().
 */
void wdtInit();

/**
 * Reset (feed) the watchdog timer.
 * Must be called regularly from the main loop.
 * If not called within WDT_TIMEOUT_S, the device reboots.
 */
void wdtReset();

// ============================================================
// NVS Relay State Persistence
// ============================================================

/**
 * Save relay states bitmask to NVS (flash).
 * Call this every time a relay changes state.
 *
 * @param relay_mask  Bitmask of relay states (bit 0 = relay 0, etc.)
 */
void nvsSaveRelayStates(uint8_t relay_mask);

/**
 * Load relay states bitmask from NVS (flash).
 * Call once during boot to restore relay states after reboot.
 *
 * @return  Saved relay bitmask, or 0 if no valid data found
 */
uint8_t nvsLoadRelayStates();

/**
 * Increment and return the boot counter from NVS.
 * Useful for diagnostic purposes — can be published to HA.
 *
 * @return  Number of times the device has booted
 */
uint32_t nvsIncrementBootCount();
