#pragma once

void mqttInit();
void mqttLoop();
bool mqttConnected();
void mqttPublish(const char* topic, const char* payload, bool retained = false);