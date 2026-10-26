#pragma once
// ============================================================
// roles.h — build-time role model (P1-8)
// ============================================================
// Exactly one role macro must be defined by platformio.ini:
//   ROLE_BASE      MQTT gateway (WiFi + MQTT + LoRa)
//   ROLE_NODE      well node (relays + ZMPT101B + BME280)
//   ROLE_REPEATER  LoRa repeater (no relays, BME280 optional)
// A node acting as a repeater (e.g. a substation) is built with
//   ROLE_NODE + FEATURE_REPEATER.
// ============================================================

#if (defined(ROLE_BASE) + defined(ROLE_NODE) + defined(ROLE_REPEATER)) != 1
#error "Define exactly one of ROLE_BASE / ROLE_NODE / ROLE_REPEATER"
#endif

// Repeating can also be enabled on a node (e.g. substation acting as repeater)
#if defined(ROLE_REPEATER) && !defined(FEATURE_REPEATER)
#define FEATURE_REPEATER
#endif

// Kept for compatibility with older code that branches on a bool.
#ifdef ROLE_BASE
  #define IS_BASE true
#else
  #define IS_BASE false
#endif
