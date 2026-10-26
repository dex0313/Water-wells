#pragma once
#include <cstdint>

// ============================================================
// mqtt_manager.h — BASE role MQTT / Home Assistant integration
// ============================================================

void mqttInit();
void mqttLoop();
bool mqttConnected();

/**
 * Publish immediately (only when connected).
 * @return true when the broker accepted the message.
 * P1-1.1: the return value must be checked where delivery matters.
 */
bool mqttPublish(const char* topic, const char* payload, bool retained = false);

/**
 * Publish Home Assistant MQTT discovery for a node (P1-1).
 * Publications go through the deferred queue (P1-2.5) so the LoRa
 * handler is never blocked; the result reports whether everything
 * was queued — discovery flags must be set only on true.
 * P1-1.7: a burst that only partially fits is retried as a whole;
 * messages already pending in the queue are recognized and skipped,
 * so the retry enqueues only the missing pieces (no duplicates).
 *
 * @param node_id      Node address
 * @param num_relays   Number of relay entities to create
 * @param num_sensors  Number of voltage/motor entities to create
 * @return true when all discovery messages were queued successfully
 */
bool publishDiscovery(uint16_t node_id, uint8_t num_relays, uint8_t num_sensors);

/**
 * Publish HA discovery for PKT_STATUS diagnostic entities (R8.2).
 * @return true when all messages were queued successfully.
 */
bool publishStatusDiscovery(uint16_t node_id);

/**
 * Forget that the base discovery was sent (forces re-publish on the
 * next call to the internal sendBaseDiscovery after reconnect).
 */
void resetDiscovery();
