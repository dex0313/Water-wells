#pragma once
#include <IPAddress.h>
#include "secret.h"

// ============================================================
// Node Identity
// ============================================================

#define NODE_ID 15
#define BASE_ID 0

// ============================================================
// WiFi
// ============================================================

// #define WIFI_STATIC_IP  true
// #define WIFI_IP         IPAddress(192,168,88,13)
// #define WIFI_GATEWAY    IPAddress(192,168,88,1)
// #define WIFI_SUBNET     IPAddress(255,255,0,0)
// #define WIFI_DNS        IPAddress(8,8,8,8)

#define WIFI_RETRY_INTERVAL  5000
#define WIFI_RESTART_TIMEOUT 300000

// ============================================================
// MQTT
// ============================================================

#define MQTT_CLIENT_ID "lora_base"
#define MQTT_RETRY_INTERVAL 5000
#define MQTT_BUFFER_SIZE    1024

// ============================================================
// Node Profile — Configure per-node hardware here
// ============================================================
// Change NODE_NUM_RELAYS and NODE_NUM_SENSORS for each node.
// Pins and calibration arrays must match the count above.
//
// Example for 1-relay node (original):
//   #define NODE_NUM_RELAYS  1
//   #define NODE_NUM_SENSORS 1
//
// Example for 4-relay + 4-ZMPT node:
//   #define NODE_NUM_RELAYS  4
//   #define NODE_NUM_SENSORS 4
// ============================================================

#define NODE_NUM_RELAYS  1
#define NODE_NUM_SENSORS 1

// Relay pins — must have exactly NODE_NUM_RELAYS entries
static const uint8_t RELAY_PINS[NODE_NUM_RELAYS] = {26};

// Relay active level (all relays share the same logic)
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

// ZMPT101B ADC pins — must have exactly NODE_NUM_SENSORS entries
// NOTE: GPIO 25 = ADC2 (works only without WiFi, fine for NODE)
//       GPIO 32,33,34 = ADC1 (always works, more stable)
static const uint8_t ZMPT_PINS[NODE_NUM_SENSORS] = {25};

// Per-channel ZMPT101B calibration factors
// How to calibrate: apply known voltage, read raw_rms from serial,
// then CALIBRATION = actual_voltage / v_rms_raw
static const float ZMPT_CALIBRATIONS[NODE_NUM_SENSORS] = {
    470.35
};

// Per-channel motor ON thresholds (voltage above this = motor running)
static const float MOTOR_THRESHOLDS[NODE_NUM_SENSORS] = {
    100.0
};

// ZMPT101B sampling parameters
#define ZMPT101B_SAMPLE_MS      40     // Sampling window (2 cycles at 50Hz)
#define ZMPT101B_ZERO_THRESHOLD 5.0    // Noise floor — below this = 0V

// ============================================================
// Sensor timing
// ============================================================

#define SENSOR_INTERVAL 180000  // 3 minutes  

// ============================================================
// LoRa protocol
// ============================================================

#define PKT_DATA 1
#define PKT_CMD  2
#define PKT_ACK  3

// Maximum number of relays/sensors per node (protocol limit)
#define MAX_RELAYS_PER_NODE  4
#define MAX_SENSORS_PER_NODE 4

// LoRa pins
#define LORA_RX 16
#define LORA_TX 17
#define LORA_AUX 4
#define LORA_M0 18
#define LORA_M1 5

// LoRa modes
#define LORA_MODE_NORMAL 0
#define LORA_MODE_SLEEP 1
#define LORA_MODE_WAKEUP 2
#define LORA_MODE_POWER 3

// ============================================================
// ACK (Acknowledgment) Configuration
// ============================================================

#define ACK_TIMEOUT_MS      2000
#define ACK_MAX_RETRIES     3
#define ACK_RETRY_DELAY_MS  1000
#define ACK_CHECK_INTERVAL  500

// ============================================================
// Heartbeat / Online Detection Configuration
// ============================================================

#define HEARTBEAT_CHECK_INTERVAL  60000
#define HEARTBEAT_OFFLINE_TIMEOUT 300000  // 5 minutes

// ============================================================
// Watchdog Timer
// ============================================================
// If the main loop does not call wdtReset() within this time,
// the ESP32 will automatically reboot. This protects against
// firmware hangs, infinite loops, and I2C/UART freezes.
// The timeout must be longer than the longest blocking operation
// (e.g. ZMPT101B sampling = 40ms per channel, so 160ms for 4).
// ============================================================

#define WDT_TIMEOUT_S  30    // Watchdog timeout in seconds

// ============================================================
// NVS (Non-Volatile Storage) — Relay State Persistence
// ============================================================
// When enabled, relay states are saved to flash on every change
// and restored on boot. This prevents pumps from staying off
// after a power outage or unexpected reboot.
// ============================================================

#define NVS_NAMESPACE      "ww_relay"     // NVS namespace (max 15 chars)
#define NVS_KEY_RELAYS     "states"       // Key for relay bitmask
#define NVS_KEY_BOOT_CNT   "boots"        // Key for boot counter (diagnostic)