#pragma once

void mqttInit();
void mqttLoop();
bool mqttConnected();
void mqttPublish(const char* topic, const char* payload, bool retained = false);
void publishDiscovery(uint16_t node_id);
void resetDiscovery();
