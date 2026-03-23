#pragma once

void loraInit();
void loraLoop();

void sendData(float t, float h, float p, uint8_t relay, uint8_t motor);
void sendCommand(uint16_t node, uint8_t cmd, uint8_t value);