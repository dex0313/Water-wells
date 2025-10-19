#include <Arduino.h>
#include "../shared/LoRaHelper.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);
  printHeader("STATION", "PROD");

  if (!initLoRa()) {
    Serial.println("LoRa init failed!");
    while (1);
  }
}

void loop() {
  String msg = receiveMessage();
  if (msg.length()) {
    Serial.printf("📥 Данные от ноды: %s\n", msg.c_str());
    // Здесь можно добавить запись в БД, MQTT, или управление реле
  }
}
