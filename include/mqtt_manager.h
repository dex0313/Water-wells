#pragma once

void mqttInit();
void mqttLoop();
bool mqttConnected();
void mqttPublish(const char* topic, const char* payload, bool retained = false);

/**
 * Publish Home Assistant MQTT discovery for a node.
 * Dynamically creates entities based on num_relays and num_sensors.
 *
 * @param node_id      Node address
 * @param num_relays   Number of relay entities to create
 * @param num_sensors  Number of voltage/motor entities to create
 */
void publishDiscovery(uint16_t node_id, uint8_t num_relays, uint8_t num_sensors);
void resetDiscovery();
